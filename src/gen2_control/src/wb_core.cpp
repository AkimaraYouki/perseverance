#include "gen2_control/wb_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gen2_control
{
namespace
{
double clip(double x, double lo, double hi) {return std::min(std::max(x, lo), hi);}
double sgn(double x) {return (x > 0) - (x < 0);}
double lpf_a(double hz) {return hz > 0 ? 1.0 - std::exp(-2.0 * M_PI * hz * kDt) : 1.0;}
double interp(double x, const std::vector<double> & xs, const std::vector<double> & ys)
{   // numpy.interp: clamps outside the grid
  if (x <= xs.front()) {return ys.front();}
  if (x >= xs.back()) {return ys.back();}
  const auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const std::size_t i = static_cast<std::size_t>(it - xs.begin());
  const double u = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + u * (ys[i] - ys[i - 1]);
}
}  // namespace

WBCore::WBCore(const Params & p, const LqrTable & lqr)
: P_(p), lqr_(lqr)
{
  if (lqr_.l.size() < 2 || lqr_.l.size() != lqr_.K.size()) {throw std::runtime_error("bad LQR table");}
  reset();
}

void WBCore::reset()
{
  x_err_ = t_un_ = t_ld_ = g_vf_ = g_i_ = g_ref_ = vf_ = rf_ = th_bias_ = v_prev_ = roll_i_ = yaw_i_ = g_acc_ = 0.0;
  v_i_ = 0.0;
  lift_on_ = false;
  wf_[0] = wf_[1] = db_e_[0] = db_e_[1] = 0.0;
  phase_ = kDrive; t_phase_ = t_takeoff_ = 0.0; h0_ = P_.idle_h; soft_ = false;
}

double WBCore::lqr_torque(double l, double x_err, double v_err, double th, double thd) const
{
  double K[4];
  for (int j = 0; j < 4; ++j) {
    std::vector<double> col(lqr_.K.size());
    for (std::size_t i = 0; i < col.size(); ++i) {col[i] = lqr_.K[i][j];}
    K[j] = interp(l, lqr_.l, col);
  }
  return -(K[0] * x_err + K[1] * v_err + K[2] * th + K[3] * thd);
}

Output WBCore::step(const Frame & f, double vx, double wz, double h_ref, double h_mid, bool jump)
{
  const Params & P = P_;
  Output o;
  // --- sense / state (est = sensors, no synthetic noise)
  const double pitch = std::asin(clip(f.g_b[0], -1.0, 1.0));
  const double roll = std::asin(clip(f.g_b[1], -1.0, 1.0));
  const double gx = f.w_b[0], gy = f.w_b[1], gz = f.w_b[2];
  double th = pitch + f.th_kin;
  const bool on0 = f.tau_hip[0] >= P.contact_tau_min, on1 = f.tau_hip[1] >= P.contact_tau_min;
  const int cnt = on0 + on1;
  const double v_raw = P.r_wheel * (f.w_wheel_abs[0] * on0 + f.w_wheel_abs[1] * on1) / std::max(cnt, 1);
  if (cnt > 0) {vf_ += lpf_a(P.v_lpf_hz) * (v_raw - vf_);}
  const double thd = gy, l_p = f.l_pend, v_now = vf_, wz_now = gz;
  // --- balance point adaptation
  const double acc = (v_now - v_prev_) / kDt;
  v_prev_ = v_now;
  const bool ad = P.bal_adapt > 0 && phase_ == kDrive && !lift_on_ && std::fabs(acc) < 0.3 && std::fabs(thd) < 0.3 &&
    std::fabs(v_now) < 0.05 && std::fabs(vx) < 0.02 &&
    (P.bal_adapt_wz_max <= 0 || (std::fabs(wz) < P.bal_adapt_wz_max && std::fabs(gz) < P.bal_adapt_wz_max)) &&
    (P.bal_adapt_x_max <= 0 || std::fabs(x_err_) < P.bal_adapt_x_max);
  const double lim = P.bal_adapt_max_deg * M_PI / 180.0;
  if (ad) {th_bias_ = clip(th_bias_ + P.bal_adapt * kDt * (th - th_bias_), -lim, lim);}
  th -= th_bias_;
  const double w_max = kWheelMax * f.motor_scale;
  rf_ += lpf_a(P.roll_rate_lpf_hz) * (-gx - rf_);
  double leg_kp = P.vmc_kp, leg_kd = P.vmc_kd, ffF = 0.0;
  // --- jump state machine: every transition is judged on the phase at the start of this step (wbctrl.py)
  const double t = f.t;
  const int ph0 = phase_;
  const double tp = t - t_phase_;
  const bool blocked = jump && ph0 == kDrive && (v_now < P.jump_min_v || v_now > P.jump_max_v || std::fabs(wz_now) > P.jump_max_wz);
  o.jump_blocked = blocked;
  jump = jump && !blocked;
  const bool m0 = ph0 == kDrive && !lift_on_ && jump;
  const bool m1 = ph0 == kRetract && tp >= P.t_retract;
  const bool m2 = ph0 == kExtract && (std::min(f.h[0], f.h[1]) >= kHMax - 0.008 || tp >= 0.25);
  const bool m3 = ph0 == kFly && tp >= P.t_tuck;
  const bool m4 = ph0 == kDescend &&
    ((tp > 0.04 && std::max(std::fabs(f.tau_hip[0]), std::fabs(f.tau_hip[1])) > P.contact_tau) || t - t_takeoff_ >= P.t_fly_max);
  const bool m5 = ph0 == kLand && tp >= P.land_s;
  if (m0) {h0_ = 0.5 * (f.h[0] + f.h[1]);}
  if (m2) {t_takeoff_ = t;}
  if (m3) {soft_ = true;} else if (m5) {soft_ = false;}
  phase_ = m0 ? kRetract : m1 ? kExtract : m2 ? kFly : m3 ? kDescend : m4 ? kLand : m5 ? kDrive : ph0;
  if (m0 || m1 || m2 || m3 || m4 || m5) {t_phase_ = t;}
  const int ph = phase_;
  const bool ctl = ph == kDrive || ph == kRetract || ph == kExtract || ph == kLand;   // LQR drives the wheels
  // --- speed limit / brake / LQR (state committed only while ctl)
  const double vm = std::min(P.vmax_kmh / 3.6, P.vmax_motor_frac * w_max * P.r_wheel);
  double g_vf = g_vf_ + lpf_a(P.speed_lpf_hz) * (v_now - g_vf_);
  const double e = std::fabs(g_vf) - vm;
  double g_i = clip(g_i_ + P.brake_ki * e * kDt, 0.0, vm);
  double v_lim = std::max(0.0, vm - (P.brake_kp * std::max(e, 0.0) + g_i));
  if (P.turn_slow) {v_lim = std::min(v_lim, std::max(0.0, vm - kHalfTrack * std::fabs(wz)));}
  const double tgt = clip(vx, -v_lim, v_lim);
  double g_ref, g_acc = 0.0;
  if (P.jerk_max > 0) {
    // S-curve: the acceleration itself ramps (the body lean follows the acceleration, so no step in lean)
    const double a_want = clip(std::sqrt(2.0 * P.jerk_max * std::fabs(tgt - g_ref_)) * sgn(tgt - g_ref_), -P.accel_max, P.accel_max);
    g_acc = g_acc_ + clip(a_want - g_acc_, -P.jerk_max * kDt, P.jerk_max * kDt);
    g_ref = g_ref_ + g_acc * kDt;
    if ((tgt - g_ref) * (tgt - g_ref_) < 0) {g_ref = tgt; g_acc = 0.0;}   // no overshoot of the target
  } else {
    g_ref = g_ref_ + clip(tgt - g_ref_, -P.accel_max * kDt, P.accel_max * kDt);
  }
  double v_ref = g_ref;
  const bool turning = P.x_turn_wz > 0 && std::fabs(wz) > P.x_turn_wz;
  double x_err = std::fabs(vx) > v_lim + 1e-3 || (!P.x_hold_moving && std::fabs(vx) > 0.02) || turning ? 0.0 : x_err_;
  bool guarded = std::fabs(vx) > v_lim + 1e-3;          // brake / speed limit active
  if (P.speed_guard < 1.0) {
    const double ww = std::max(std::fabs(f.w_wheel_joint[0]), std::fabs(f.w_wheel_joint[1])) / w_max;
    if (ww > P.speed_guard) {
      const double cut = std::min(1.0, (ww - P.speed_guard) / (1.0 - P.speed_guard));
      v_ref = v_now * vx >= 0 ? v_now * (1.0 - 0.6 * cut) : vx;
      x_err = 0.0;
      guarded = true;
    }
  }
  // slope compensation: only with a steady command (ramp done), driving, on the ground, no guard; fades otherwise
  double v_i = v_i_;
  if (P.v_ki > 0) {
    const bool steady = ph == kDrive && !lift_on_ && !guarded && std::fabs(vx) > 0.02 && std::fabs(g_ref - tgt) < 0.02;
    if (steady) {
      v_i = clip(v_i + P.v_ki * (v_ref - v_now) * kDt, -P.v_boost_max, P.v_boost_max);
    } else if (std::fabs(vx) <= 0.02) {
      v_i = 0.0;                                   // stick released: no lingering boost (it slowed the stop)
    } else {
      v_i *= 1.0 - std::min(1.0, 4.0 * kDt);
    }
    v_ref = clip(v_ref + v_i, -vm, vm);
  }
  if (P.x_leak_moving > 0 && std::fabs(vx) > 0.02 && std::fabs(vx) <= v_lim + 1e-3) {
    x_err = x_err_ * (1.0 - P.x_leak_moving * kDt);   // driving: forget a big catch-up in 1/x_leak s
  }
  x_err = clip(x_err + (v_now - v_ref) * kDt, -0.3, 0.3);
  double th_ref = ph == kRetract ? P.retract_lean * M_PI / 180.0 : 0.0;   // lean forward while crouching
  if (P.lean_ff > 0 && ph == kDrive) {th_ref += P.lean_ff * std::atan(g_acc / 9.81);}   // lean for the reference accel
  const double tau_w = lqr_torque(l_p, x_err, v_now - v_ref, th - th_ref, thd);
  if (P.turn_limit && ctl) {
    const double v_cmd = std::min(std::fabs(vx), vm);
    const double wz_lim = std::max(0.5, (P.wheel_margin * w_max * P.r_wheel - v_cmd) / kHalfTrack);
    wz = clip(wz, -wz_lim, wz_lim);
  }
  if (P.yaw_ki > 0) {yaw_i_ = ctl ? clip(yaw_i_ + P.yaw_ki * (wz - wz_now) * kDt, -P.yaw_i_max, P.yaw_i_max) : 0.0;}
  const double tau_y = P.yaw_kd * (wz - wz_now) + (P.yaw_ki > 0 ? yaw_i_ : 0.0);
  double act[4] = {0, 0, 0, 0};
  if (ctl) {
    act[2] = clip((0.5 * tau_w - tau_y) / P.wheel_tau_max, -1.0, 1.0);
    act[3] = clip((0.5 * tau_w + tau_y) / P.wheel_tau_max, -1.0, 1.0);
    g_vf_ = g_vf; g_i_ = g_i; g_ref_ = g_ref; g_acc_ = g_acc; x_err_ = x_err; v_i_ = v_i;
  }
  o.v_ref = v_ref; o.v_lim = v_lim;
  // --- lift / landing detection
  const double tmax = std::max(f.tau_hip[0], f.tau_hip[1]), tmin = std::min(f.tau_hip[0], f.tau_hip[1]);
  const bool unl = tmax < P.contact_tau_min;
  const bool ldd = tmax >= P.contact_tau_min && f.sf > P.land_sf_min;
  const bool a_ = ph == kDrive && !lift_on_, b_ = ph == kDrive && lift_on_;
  if (a_) {t_un_ = unl ? t_un_ + kDt : 0.0;}
  const bool up = a_ && t_un_ >= P.lift_detect_s;
  if (b_) {t_ld_ = ldd ? t_ld_ + kDt : 0.0;} else if (up) {t_ld_ = 0.0;}
  const bool down = b_ && t_ld_ >= P.land_detect_s;
  lift_on_ = (lift_on_ || up) && !down;
  if (down) {t_un_ = 0.0; x_err_ = 0.0; g_i_ = 0.0; g_ref_ = v_now; g_vf_ = v_now; roll_i_ = 0.0; g_acc_ = 0.0; v_i_ = 0.0;}
  const bool lifted = ph == kDrive && lift_on_;
  if (lifted) {
    for (int k = 0; k < 2; ++k) {
      act[2 + k] = clip(-P.lift_wheel_kd * f.w_wheel_joint[k] / P.wheel_tau_max, -1.0, 1.0);
      act[k] = (P.idle_h - h_ref) / 0.12;
    }
    roll_i_ = 0.0; x_err_ = 0.0; g_i_ = g_ref_ = g_vf_ = 0.0; g_acc_ = 0.0; v_i_ = 0.0;   // yaw_i kept (wbctrl: integrates in DRIVE even when lifted)
  }
  const bool drv = ph == kDrive && !lift_on_;
  const double roll_ref = clip(std::atan(P.turn_lean * g_ref_ * wz / 9.81), -20.0 * M_PI / 180.0, 20.0 * M_PI / 180.0);
  const double rl = roll - roll_ref;
  const bool airborne = tmin < P.contact_tau_min;
  // roll PI (lqr_vmc.RollPI array version)
  const double track = 0.198;
  const double er = track * std::sin(rl);
  const bool freeze = airborne || std::fabs(rl * 180.0 / M_PI) > P.roll_freeze_deg;
  double ri = freeze ? roll_i_ : roll_i_ + P.roll_ki * er * kDt;
  ri = clip(ri * (1.0 - P.roll_leak * kDt), -P.level_max, P.level_max);
  if (drv) {roll_i_ = ri;}
  const double dlt = clip(roll_i_ + P.roll_kp * er + P.roll_kd * track * rf_, -P.level_max, P.level_max);
  const double hc = h_mid < 0 ? P.idle_h : h_mid;
  if (drv) {
    act[0] = (clip(hc + 0.5 * dlt, kHMin, kHMax) - h_ref) / 0.12;
    act[1] = (clip(hc - 0.5 * dlt, kHMin, kHMax) - h_ref) / 0.12;
  }
  if (drv || ph == kLand) {ffF = 0.5 * P.m_pend * 9.81;}
  const bool jmp = ph == kRetract || ph == kExtract || ph == kFly || ph == kDescend;
  if (jmp) {roll_i_ = f.h[0] - f.h[1];}   // roll PI restarts from the current leg difference
  // --- jump phases: leg targets, wheel pitch PD in the air, leg gains
  if (ph != kDrive) {
    double tgt_h = ph == kExtract ? kHMax : ph == kFly ? kHMin : P.h_land;
    if (ph == kRetract) {tgt_h = h0_ + (kHMin - h0_) * std::min(1.0, tp / (0.75 * P.t_retract));}   // tp: before this step's switch
    act[0] = act[1] = (tgt_h - h_ref) / 0.12;
    const bool pd = ph == kFly || ph == kDescend || (P.air_pd_extract && ph == kExtract);
    if (pd) {
      if (P.air_ctrl) {
        const double ref = ph == kExtract ? P.extract_pitch : ph == kDescend ? P.land_pitch : P.air_pitch;
        const double u = P.air_kp * (pitch - ref * M_PI / 180.0) + P.air_kd * gy;   // [N.m], same for both wheels
        for (int k = 0; k < 2; ++k) {
          double uw = u;
          if (P.spin_match_k > 0 && ph == kDescend) {uw = P.spin_match_k * (vf_ / P.r_wheel - f.w_wheel_abs[k]);}
          if (P.air_w_max > 0 && sgn(uw) * f.w_wheel_joint[k] > P.air_w_max) {uw = 0.0;}
          act[2 + k] = clip(uw / P.air_tau, -1.0, 1.0);
        }
      } else {
        act[2] = act[3] = 0.0;
      }
    }
    if (soft_) {leg_kp = P.land_kp; leg_kd = P.land_kd;} else if (ph != kLand) {leg_kp = P.leg_kp; leg_kd = P.leg_kd;}
  }
  // --- wheel friction compensation (not while lifted)
  if (P.fric_comp_nm > 0 || P.fric_comp_static_nm > 0 || P.fric_comp_cmd_nm > 0) {
    for (int k = 0; k < 2; ++k) {
      const double sv = std::tanh(f.w_wheel_joint[k] / P.fric_comp_w);
      const double tcmd = act[2 + k] * P.wheel_tau_max;
      double comp = P.fric_comp_nm * sv + P.fric_comp_static_nm * (1.0 - std::fabs(sv)) * std::tanh(tcmd / 0.05);
      if (P.fric_comp_cmd_nm > 0) {comp += P.fric_comp_cmd_nm * std::tanh(tcmd / P.fric_comp_cmd_w);}
      if (!lifted) {act[2 + k] = clip(act[2 + k] + comp / P.wheel_tau_max, -1.0, 1.0);}
    }
  }
  o.tau_lqr = tau_w; o.tau_yaw = tau_y;
  for (int k = 0; k < 2; ++k) {o.wheel_pre_lpf[k] = act[2 + k] * P.wheel_tau_max;}
  // --- wheel torque LPF, then drive deadband compensation (sigma-delta)
  if (P.wheel_lpf_hz > 0) {
    const double a = lpf_a(P.wheel_lpf_hz);
    for (int k = 0; k < 2; ++k) {wf_[k] += a * (act[2 + k] - wf_[k]); act[2 + k] = wf_[k];}
  }
  if (P.db_comp == 1 || P.db_comp == 2) {
    const double db = P.db_comp_nm / P.wheel_tau_max;
    for (int k = 0; k < 2; ++k) {
      if (P.db_comp == 1) {
        const double av = std::fabs(act[2 + k]);
        if (av > P.db_comp_eps / P.wheel_tau_max) {act[2 + k] = sgn(act[2 + k]) * std::max(av, db);}
      } else {
        const double v = act[2 + k] + db_e_[k];
        const double av = std::fabs(v);
        const double out = av >= db ? v : (av >= 0.5 * db ? sgn(v) * db : 0.0);
        db_e_[k] = clip(v - out, -db, db);
        act[2 + k] = clip(out, -1.0, 1.0);
      }
    }
  }
  for (int k = 0; k < 4; ++k) {o.act[k] = act[k];}
  o.leg_kp = leg_kp; o.leg_kd = leg_kd; o.ff_force = ffF;
  o.th = th; o.thd = thd; o.v = v_now; o.pitch = pitch; o.roll = roll; o.x_err = x_err_;
  o.th_bias = th_bias_; o.lifted = lifted; o.phase = ph;
  return o;
}

}  // namespace gen2_control
