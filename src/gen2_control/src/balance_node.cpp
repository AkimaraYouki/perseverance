// balance_node: runs wb_core (port of wbctrl.py) on the robot at 200 Hz.
//
// Modes (services balance/stand, balance/balance, balance/disarm):
//   disarmed  no CAN TX (read-only)
//   stand     hips hold the IDLE leg height (ramped from the current height), wheels 0 A.
//             For the first hand-held tests; needs only the leg table.
//   balance   wb_core: wheel LQR + leg VMC. Needs the model tables (LQR K(l), mass/COM kinematics).
//   fault     any guard tripped: 0 A to all drives, TX off, stays until balance/disarm.
// Guards (stand and balance): operator heartbeat (balance/heartbeat, 0.5 s), IMU age, motor feedback
// age / drive error / temperature, hip zero resolved (60 deg wrap), tilt limit.
// Inputs: /imu/data (body frame), cmd_vel (vx, wz; 0.5 s watchdog -> 0).  Output state: controller/state.
// This process owns the CAN bus for commanding (MotorBus::claim_commander).
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gen2_control/wb_core.hpp"
#include "gen2_hardware/cubemars_mit.hpp"
#include "gen2_hardware/cubemars_servo.hpp"
#include "gen2_hardware/motor_bus.hpp"
#include "gen2_hardware/motor_params.hpp"
#include "gen2_msgs/msg/controller_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using gen2_hardware::MotorBus;
using gen2_hardware::MotorFeedback;
using gen2_control::Frame;
using gen2_control::Output;

namespace
{
int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
double interp(double x, const std::vector<double> & xs, const std::vector<double> & ys)
{
  if (x <= xs.front()) {return ys.front();}
  if (x >= xs.back()) {return ys.back();}
  const std::size_t i = static_cast<std::size_t>(std::upper_bound(xs.begin(), xs.end(), x) - xs.begin());
  const double u = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + u * (ys[i] - ys[i - 1]);
}
enum class Mode {kDisarmed, kStand, kBalance, kFault};
const char * mode_name(Mode m)
{
  return m == Mode::kDisarmed ? "disarmed" : m == Mode::kStand ? "stand" : m == Mode::kBalance ? "balance" : "fault";
}
}  // namespace

class BalanceNode : public rclcpp::Node
{
public:
  BalanceNode()
  : Node("balance")
  {
    const std::string ifname = declare_parameter("can_interface", std::string("can0"));
    auto motors = gen2_hardware::declare_motor_params(*this);
    for (const char * n : {"wheel_l", "wheel_r", "leg_l", "leg_r"}) {
      bool found = false;
      for (std::size_t i = 0; i < motors.size(); ++i) {
        if (motors[i].name == n) {idx_[n] = i; found = true;}
      }
      if (!found) {throw std::runtime_error(std::string("motors.yaml has no '") + n + "'");}
    }
    // leg table (scripts/make_leg_table.py)
    leg_th_ = declare_parameter("leg_table.theta_rad", std::vector<double>{});
    leg_h_ = declare_parameter("leg_table.h_m", std::vector<double>{});
    leg_dh_ = declare_parameter("leg_table.dh_dtheta", std::vector<double>{});
    theta0_ = declare_parameter("leg_table.theta0_deg", 45.002) * M_PI / 180.0;
    if (leg_th_.size() < 10 || leg_th_.size() != leg_h_.size() || leg_th_.size() != leg_dh_.size()) {
      throw std::runtime_error("leg_table missing (config/leg_table.yaml)");
    }
    // model tables (desktop: sim/model/balance_tables.yaml). Optional: without them only 'stand'.
    model_ok_ = load_model();
    // safety / loop
    rate_hz_ = declare_parameter("rate_hz", 200.0);
    max_tilt_ = declare_parameter("max_tilt_deg", 45.0) * M_PI / 180.0;
    start_tilt_ = declare_parameter("start_tilt_deg", 15.0) * M_PI / 180.0;
    imu_timeout_ = declare_parameter("imu_timeout_s", 0.05);
    motor_timeout_ = declare_parameter("motor_timeout_s", 0.05);
    hb_timeout_ = declare_parameter("heartbeat_timeout_s", 0.5);
    cmd_timeout_ = declare_parameter("cmd_timeout_s", 0.5);
    stand_ramp_s_ = declare_parameter("stand_ramp_s", 2.0);
    stand_ff_ = declare_parameter("stand_feedforward", true);
    // hips: "servo_pos" = drive position-speed loop (PID inside the drive, stable; stiff, no VMC
    // compliance / feed-forward), "current_pd" = host PD -> current (wbctrl VMC, needs >= 500 Hz and
    // a clean velocity: shook at 200 Hz on 2026-10-07)
    hip_mode_ = declare_parameter("hip_mode", std::string("servo_pos"));
    // "mit" = AK 3.0 MIT frames: damping kd inside the drive (kHz), stiffness kp(M*-M) + feed-forward
    //   computed here and sent as t_ff; p_des unused (drive frame has the 60 deg power-up wrap)
    if (hip_mode_ != "servo_pos" && hip_mode_ != "current_pd" && hip_mode_ != "mit") {
      throw std::runtime_error("hip_mode: servo_pos|current_pd|mit");
    }
    mit_kt_drive_ = declare_parameter("mit_kt_drive", 0.5994);   // Kt the drive uses for t -> current (measured 2026-10-07)
    osc_flips_ = static_cast<int>(declare_parameter("osc_flips", 6));
    osc_window_s_ = declare_parameter("osc_window_s", 0.3);
    osc_vel_ = declare_parameter("osc_vel_rad_s", 1.0);
    sat_frac_ = declare_parameter("sat_frac", 0.9);
    sat_time_s_ = declare_parameter("sat_time_s", 0.1);
    hip_speed_ = declare_parameter("hip_speed_rad_s", 2.0);
    hip_accel_ = declare_parameter("hip_accel_rad_s2", 20.0);
    rt_prio_ = static_cast<int>(declare_parameter("rt_priority", 80));
    // controller params (wbctrl TUNE names), defaults = Params{}
    gen2_control::Params P;
    P.r_wheel = declare_parameter("leg_table.r_wheel", P.r_wheel);
    P.idle_h = declare_parameter("tune.idle_h", P.idle_h);
    P.vmc_kp = declare_parameter("tune.vmc_kp", P.vmc_kp);
    P.vmc_kd = declare_parameter("tune.vmc_kd", P.vmc_kd);
    P.vmax_kmh = declare_parameter("tune.vmax_kmh", P.vmax_kmh);
    P.fric_comp_nm = declare_parameter("tune.fric_comp_nm", P.fric_comp_nm);
    P.db_comp = static_cast<int>(declare_parameter("tune.db_comp", static_cast<int64_t>(P.db_comp)));
    P.db_comp_nm = declare_parameter("tune.db_comp_nm", P.db_comp_nm);
    P.wheel_lpf_hz = declare_parameter("tune.wheel_lpf_hz", P.wheel_lpf_hz);
    P.bal_adapt = declare_parameter("tune.bal_adapt", P.bal_adapt);
    P.contact_tau_min = declare_parameter("tune.contact_tau_min", P.contact_tau_min);
    P.m_pend = model_ok_ ? m_pend_ : P.m_pend;
    params_ = P;
    if (model_ok_) {core_ = std::make_unique<gen2_control::WBCore>(P, lqr_);}

    bus_ = std::make_unique<MotorBus>(ifname, motors);
    std::string err;
    if (!bus_->claim_commander(err)) {throw std::runtime_error(err);}
    bus_->enable_tx(false);
    bus_->start();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(imu_m_);
          imu_ = *m;
          imu_ns_ = now_ns();
        });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(cmd_m_);
          vx_ = m->linear.x; wz_ = m->angular.z; cmd_ns_ = now_ns();
        });
    hb_sub_ = create_subscription<std_msgs::msg::Empty>("balance/heartbeat", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {hb_ns_ = now_ns();});
    auto srv = [this](const char * name, Mode m) {
        return create_service<std_srvs::srv::Trigger>(name,
                 [this, m](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr rs) {
                   rs->message = request(m);
                   rs->success = rs->message.rfind("ok", 0) == 0;
                 });
      };
    srv_stand_ = srv("balance/stand", Mode::kStand);
    srv_bal_ = srv("balance/balance", Mode::kBalance);
    srv_dis_ = srv("balance/disarm", Mode::kDisarmed);
    pub_ = create_publisher<gen2_msgs::msg::ControllerState>("controller/state", 10);
    pub_timer_ = create_wall_timer(10ms, [this] {publish();});
    th_ = std::thread([this] {loop();});
    RCLCPP_INFO(get_logger(), "balance_node ready (model %s): modes disarmed/stand%s",
      model_ok_ ? "loaded" : "MISSING", model_ok_ ? "/balance" : " only");
  }

  ~BalanceNode() override
  {
    run_ = false;
    if (th_.joinable()) {th_.join();}
    zero_all();
  }

private:
  // ------------------------------------------------------------------ model tables
  bool load_model()
  {
    const auto l = declare_parameter("model.lqr_l", std::vector<double>{});
    const auto k = declare_parameter("model.lqr_k", std::vector<double>{});        // row-major n x 4
    m_body_ = declare_parameter("model.body_mass", 0.0);
    body_c_ = declare_parameter("model.body_com_xz", std::vector<double>{});
    m_leg_ = declare_parameter("model.leg_mass", 0.0);
    kin_th_ = declare_parameter("model.theta_rad", std::vector<double>{});
    wheel_xz_ = declare_parameter("model.wheel_xz", std::vector<double>{});       // 2 per theta
    leg_com_xz_ = declare_parameter("model.leg_com_xz", std::vector<double>{});   // 2 per theta
    dphi_ = declare_parameter("model.dphi_shank_dtheta", std::vector<double>{});
    const std::size_t n = kin_th_.size();
    if (l.size() < 2 || k.size() != 4 * l.size() || body_c_.size() != 2 || !(m_body_ > 0) || !(m_leg_ > 0) ||
      n < 10 || wheel_xz_.size() != 2 * n || leg_com_xz_.size() != 2 * n || dphi_.size() != n)
    {
      return false;
    }
    lqr_.l = l;
    for (std::size_t i = 0; i < l.size(); ++i) {lqr_.K.push_back({k[4 * i], k[4 * i + 1], k[4 * i + 2], k[4 * i + 3]});}
    for (std::size_t i = 0; i < n; ++i) {
      wx_.push_back(wheel_xz_[2 * i]); wz_tab_.push_back(wheel_xz_[2 * i + 1]);
      cx_.push_back(leg_com_xz_[2 * i]); cz_.push_back(leg_com_xz_[2 * i + 1]);
    }
    m_pend_ = m_body_ + 2.0 * m_leg_;
    return true;
  }

  // th_kin, l_pend from the nominal masses and both leg angles (body frame, x fwd, z up)
  void pendulum(const double th[2], double & th_kin, double & l_pend) const
  {
    double cx = m_body_ * body_c_[0], cz = m_body_ * body_c_[1], wxs = 0, wzs = 0;
    for (int k = 0; k < 2; ++k) {
      cx += m_leg_ * interp(th[k], kin_th_, cx_);
      cz += m_leg_ * interp(th[k], kin_th_, cz_);
      wxs += 0.5 * interp(th[k], kin_th_, wx_);
      wzs += 0.5 * interp(th[k], kin_th_, wz_tab_);
    }
    const double dx = cx / m_pend_ - wxs, dz = cz / m_pend_ - wzs;
    th_kin = std::atan2(dx, dz);
    l_pend = std::hypot(dx, dz);
  }

  // ------------------------------------------------------------------ mode requests
  std::string request(Mode m)
  {
    std::lock_guard<std::mutex> lk(mode_m_);
    if (m == Mode::kDisarmed) {
      want_ = Mode::kDisarmed;
      return "ok: disarm";
    }
    if (m == Mode::kBalance && !model_ok_) {return "refused: model tables missing (sim/model/balance_tables.yaml)";}
    if (mode_ == Mode::kFault) {return "refused: in fault (" + fault_ + "), disarm first";}
    const std::string why = arm_check();
    if (!why.empty()) {last_refusal_ = why; return "refused: " + why;}
    want_ = m;
    return std::string("ok: ") + mode_name(m);
  }

  std::string arm_check()
  {
    const int64_t t = now_ns();
    if ((t - hb_ns_.load()) * 1e-9 > hb_timeout_) {return "no operator heartbeat (balance/heartbeat)";}
    {
      std::lock_guard<std::mutex> lk(imu_m_);
      if ((t - imu_ns_) * 1e-9 > imu_timeout_) {return "IMU not fresh";}
    }
    for (const auto & kv : idx_) {
      const MotorFeedback fb = bus_->feedback(kv.second);
      const auto & c = bus_->motors()[kv.second];
      if (!fb.valid || (t - fb.mono_ns) * 1e-9 > motor_timeout_) {return kv.first + ": feedback not fresh";}
      if (fb.status.error) {return kv.first + ": drive fault";}
      if (!fb.zero_ok()) {return kv.first + ": joint zero not resolved (hip_cli find)";}
      if (fb.status.temperature_c > c.max_temperature_c) {return kv.first + ": over-temperature";}
    }
    if (std::fabs(pitch_) > start_tilt_ || std::fabs(roll_) > start_tilt_) {
      return "tilt " + std::to_string(static_cast<int>(pitch_ * 57.3)) + "/" +
             std::to_string(static_cast<int>(roll_ * 57.3)) + " deg: hold the body upright";
    }
    return "";
  }

  // ------------------------------------------------------------------ loop
  void set_rt()
  {
    sched_param sp{};
    sp.sched_priority = rt_prio_;
    if (rt_prio_ > 0 && pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(get_logger(), "SCHED_FIFO %d not permitted (add rtprio to /etc/security/limits.d) — running normal", rt_prio_);
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {RCLCPP_WARN(get_logger(), "mlockall failed");}
  }

  void zero_all()
  {
    if (!bus_ || !bus_->tx_enabled()) {return;}
    std::string err;
    for (int r = 0; r < 10; ++r) {
      for (const auto & kv : idx_) {bus_->send(gen2_hardware::cubemars::encode_current(bus_->motors()[kv.second].can_id, 0.0), err);}
      std::this_thread::sleep_for(2ms);
    }
  }

  void enter(Mode m, const std::string & why = "")
  {
    if (m == Mode::kFault || m == Mode::kDisarmed) {
      zero_all();
      bus_->enable_tx(false);
    } else {
      bus_->enable_tx(true);
    }
    if (m == Mode::kFault) {
      fault_ = why;
      RCLCPP_ERROR(get_logger(), "FAULT: %s", why.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "mode -> %s", mode_name(m));
    }
    if (m == Mode::kStand || m == Mode::kBalance) {
      t_mode_ = 0.0;
      for (int k = 0; k < 2; ++k) {M_start_[k] = M_[k]; osc_t_[k].clear(); osc_sign_[k] = 0; sat_s_[k] = 0.0;}
      if (core_) {core_->reset();}
    }
    mode_ = m;
  }

  void loop()
  {
    set_rt();
    const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate_hz_));
    auto next = std::chrono::steady_clock::now();
    int64_t prev = now_ns();
    while (run_ && rclcpp::ok()) {
      next += period;
      std::this_thread::sleep_until(next);
      const int64_t t = now_ns();
      const double dt_ms = (t - prev) * 1e-6;
      prev = t;
      dt_max_ms_ = std::max(dt_max_ms_.load(), dt_ms);
      if (dt_ms > 2e3 / rate_hz_) {++overruns_;}
      step(t);
    }
  }

  void step(int64_t t)
  {
    // ---- sense
    sensor_msgs::msg::Imu imu;
    int64_t imu_ns;
    {
      std::lock_guard<std::mutex> lk(imu_m_);
      imu = imu_;
      imu_ns = imu_ns_;
    }
    imu_age_ms_ = (t - imu_ns) * 1e-6;
    const auto & q = imu.orientation;
    Frame f;
    // projected gravity (pointing down) in the body frame: -R^T e_z, R from q_world_body
    f.g_b[0] = -2.0 * (q.x * q.z - q.w * q.y);
    f.g_b[1] = -2.0 * (q.y * q.z + q.w * q.x);
    f.g_b[2] = -(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    f.w_b[0] = imu.angular_velocity.x; f.w_b[1] = imu.angular_velocity.y; f.w_b[2] = imu.angular_velocity.z;
    const auto & a = imu.linear_acceleration;
    f.sf = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) / 9.80665;
    pitch_ = std::asin(std::clamp(f.g_b[0], -1.0, 1.0));
    roll_ = std::asin(std::clamp(f.g_b[1], -1.0, 1.0));
    std::string bad;
    MotorFeedback fb[4];
    const char * names[4] = {"leg_l", "leg_r", "wheel_l", "wheel_r"};
    for (int i = 0; i < 4; ++i) {
      const std::size_t k = idx_.at(names[i]);
      fb[i] = bus_->feedback(k);
      const auto & c = bus_->motors()[k];
      if (bad.empty()) {
        if (!fb[i].valid || (t - fb[i].mono_ns) * 1e-9 > motor_timeout_) {bad = std::string(names[i]) + " feedback stale";}
        else if (fb[i].status.error) {bad = std::string(names[i]) + " drive fault " + std::to_string(fb[i].status.error);}
        else if (fb[i].status.temperature_c > c.max_temperature_c) {bad = std::string(names[i]) + " over-temperature";}
        else if (i < 2 && !fb[i].zero_ok()) {bad = std::string(names[i]) + " zero not resolved";}
      }
    }
    double th[2], Md[2];
    double * M = M_;
    for (int k = 0; k < 2; ++k) {
      const auto & c = bus_->motors()[idx_.at(names[k])];
      M[k] = c.raw_to_joint_pos(fb[k].raw_unwrapped());
      Md[k] = c.erpm_to_joint_vel(fb[k].status.speed_erpm);
      th[k] = theta0_ + M[k];
      h_[k] = interp(th[k], leg_th_, leg_h_);
      f.h[k] = h_[k];
      f.tau_hip[k] = c.current_to_joint_torque(fb[k].status.current_a);
      const auto & cw = bus_->motors()[idx_.at(names[2 + k])];
      f.w_wheel_joint[k] = cw.erpm_to_joint_vel(fb[2 + k].status.speed_erpm);
      // wheel rotation in the world = joint rate + body pitch rate + shank rotation from the leg motion
      f.w_wheel_abs[k] = f.w_wheel_joint[k] + f.w_b[1] + (model_ok_ ? interp(th[k], kin_th_, dphi_) * Md[k] : 0.0);
    }
    if (model_ok_) {pendulum(th, f.th_kin, f.l_pend);}
    f.t = t * 1e-9;
    th_kin_ = f.th_kin;
    l_pend_ = f.l_pend;
    double vx = 0, wz = 0;
    {
      std::lock_guard<std::mutex> lk(cmd_m_);
      if ((t - cmd_ns_) * 1e-9 < cmd_timeout_) {vx = vx_; wz = wz_;}
    }
    vx_cmd_ = vx; wz_cmd_ = wz;

    // ---- mode changes and guards
    {
      std::lock_guard<std::mutex> lk(mode_m_);
      if (want_ != mode_ && !(mode_ == Mode::kFault && want_ != Mode::kDisarmed)) {enter(want_);}
      if (mode_ == Mode::kStand || mode_ == Mode::kBalance) {
        std::string why = bad;
        // oscillation / saturation guard on the hips
        for (int k = 0; k < 2 && why.empty(); ++k) {
          const auto & c = bus_->motors()[idx_.at(names[k])];
          const double w = c.erpm_to_joint_vel(fb[k].status.speed_erpm);
          if (std::fabs(w) > osc_vel_) {
            const int sg = w > 0 ? 1 : -1;
            if (sg != osc_sign_[k]) {
              osc_t_[k].push_back(t * 1e-9);
              osc_sign_[k] = sg;
            }
          }
          while (!osc_t_[k].empty() && t * 1e-9 - osc_t_[k].front() > osc_window_s_) {osc_t_[k].erase(osc_t_[k].begin());}
          if (static_cast<int>(osc_t_[k].size()) >= osc_flips_) {why = std::string(names[k]) + " oscillation detected";}
          const bool sat = std::fabs(fb[k].status.current_a) > sat_frac_ * c.current_limit_a;
          sat_s_[k] = sat ? sat_s_[k] + 1.0 / rate_hz_ : 0.0;
          if (sat_s_[k] > sat_time_s_) {why = std::string(names[k]) + " current saturated";}
        }
        if (why.empty() && imu_age_ms_ > imu_timeout_ * 1e3) {why = "IMU stale";}
        if (why.empty() && (t - hb_ns_.load()) * 1e-9 > hb_timeout_) {why = "operator heartbeat lost";}
        if (why.empty() && (std::fabs(pitch_) > max_tilt_ || std::fabs(roll_) > max_tilt_)) {why = "tilt limit (fell)";}
        if (!why.empty()) {enter(Mode::kFault, why); want_ = Mode::kFault;}
      }
    }
    if (mode_ != Mode::kStand && mode_ != Mode::kBalance) {return;}
    t_mode_ += 1.0 / rate_hz_;

    // ---- control
    double h_tgt[2], M_tgt[2], ffF = 0.0, kp = params_.vmc_kp, kd = params_.vmc_kd, wheel_tau[2] = {0, 0};
    const double M_idle = interp(params_.idle_h, leg_h_, leg_th_) - theta0_;
    if (mode_ == Mode::kStand) {
      // ramp in joint angle from where the leg really is (it may sit beyond the leg table, e.g. at the
      // retracted stop -12 deg < -7 deg: a height ramp would start with a step there)
      const double u = std::min(1.0, t_mode_ / stand_ramp_s_);
      for (int k = 0; k < 2; ++k) {
        M_tgt[k] = M_start_[k] + u * (M_idle - M_start_[k]);
        h_tgt[k] = interp(theta0_ + M_tgt[k], leg_th_, leg_h_);
      }
      ffF = stand_ff_ ? 0.5 * (model_ok_ ? m_pend_ : params_.m_pend) * 9.81 : 0.0;
    } else {
      const Output o = core_->step(f, vx, wz, params_.idle_h);
      last_ = o;
      for (int k = 0; k < 2; ++k) {
        h_tgt[k] = params_.idle_h + 0.12 * o.act[k];
        M_tgt[k] = interp(std::clamp(h_tgt[k], leg_h_.front(), leg_h_.back()), leg_h_, leg_th_) - theta0_;
        wheel_tau[k] = o.act[2 + k] * params_.wheel_tau_max;
      }
      ffF = o.ff_force; kp = o.leg_kp; kd = o.leg_kd;
    }
    std::string err;
    for (int k = 0; k < 2; ++k) {
      const auto & c = bus_->motors()[idx_.at(names[k])];
      h_tgt_[k] = h_tgt[k];
      if (hip_mode_ == "servo_pos") {
        // absolute drive degrees = reported drive position + joint error (same frame as MotorTester)
        const double tgt_raw = fb[k].status.position_deg +
          (M_tgt[k] - M[k]) * 180.0 / M_PI * c.raw_deg_per_output_rev / 360.0 / c.direction;
        const double epr = c.pole_pairs * c.gear_ratio * 60.0 / (2.0 * M_PI);   // ERPM per joint rad/s
        bus_->send(gen2_hardware::cubemars::encode_pos_spd(c.can_id, tgt_raw, hip_speed_ * epr, hip_accel_ * epr), err);
        hip_tau_[k] = f.tau_hip[k];                        // measured (logging)
        hip_cur_[k] = fb[k].status.current_a;
      } else if (hip_mode_ == "mit") {
        // drive: t = t_ff + kd_drive * (0 - w_drive); it converts with its own Kt (0.6) -> scale by kt_drive / kt
        const double kt = std::isfinite(c.kt_nm_per_a) ? c.kt_nm_per_a : mit_kt_drive_;
        const double sc = mit_kt_drive_ / kt;
        double tau = kp * (M_tgt[k] - M[k]) + ffF * interp(th[k], leg_th_, leg_dh_);
        const double tmax = c.current_limit_a * kt;
        tau = std::clamp(tau, -tmax, tmax);
        gen2_hardware::mit::Command mc;
        mc.t = tau * c.direction * sc;
        mc.kd = kd * sc;
        bus_->send(gen2_hardware::mit::encode(gen2_hardware::mit::Proto::kV3, c.can_id, c.mit_ranges, mc), err);
        hip_tau_[k] = tau; hip_cur_[k] = fb[k].status.current_a;
      } else {
        const double tau = kp * (M_tgt[k] - M[k]) - kd * Md[k] + ffF * interp(th[k], leg_th_, leg_dh_);
        const double cur = c.joint_torque_to_current(tau);
        hip_tau_[k] = tau; hip_cur_[k] = cur;
        bus_->send(gen2_hardware::cubemars::encode_current(c.can_id, cur), err);
      }
      const auto & cw = bus_->motors()[idx_.at(names[2 + k])];
      const double wc = cw.joint_torque_to_current(wheel_tau[k]);
      wheel_tau_[k] = wheel_tau[k]; wheel_cur_[k] = wc;
      bus_->send(gen2_hardware::cubemars::encode_current(cw.can_id, wc), err);
    }
  }

  void publish()
  {
    gen2_msgs::msg::ControllerState s;
    s.header.stamp = now();
    {
      std::lock_guard<std::mutex> lk(mode_m_);
      s.mode = mode_name(mode_);
      s.fault = mode_ == Mode::kFault ? fault_ : last_refusal_;
    }
    s.pitch = pitch_; s.roll = roll_;
    s.th = last_.th; s.thd = last_.thd; s.th_kin = th_kin_; s.th_bias = last_.th_bias; s.l_pend = l_pend_;
    s.v = last_.v; s.v_ref = last_.v_ref; s.x_err = last_.x_err; s.lifted = last_.lifted;
    for (int k = 0; k < 4; ++k) {s.act[k] = last_.act[k];}
    for (int k = 0; k < 2; ++k) {
      s.h[k] = h_[k]; s.h_target[k] = h_tgt_[k]; s.hip_tau_cmd[k] = hip_tau_[k]; s.hip_cur_cmd[k] = hip_cur_[k];
      s.wheel_tau_cmd[k] = wheel_tau_[k]; s.wheel_cur_cmd[k] = wheel_cur_[k];
    }
    s.vx_cmd = vx_cmd_; s.wz_cmd = wz_cmd_;
    s.imu_age_ms = imu_age_ms_;
    s.loop_dt_max_ms = dt_max_ms_.exchange(0.0);
    s.overruns = overruns_;
    pub_->publish(s);
  }

  // config
  std::map<std::string, std::size_t> idx_;
  std::vector<double> leg_th_, leg_h_, leg_dh_;
  double theta0_;
  bool model_ok_ = false;
  gen2_control::LqrTable lqr_;
  double m_body_ = 0, m_leg_ = 0, m_pend_ = 0;
  std::vector<double> body_c_, kin_th_, wheel_xz_, leg_com_xz_, dphi_, wx_, wz_tab_, cx_, cz_;
  double rate_hz_, max_tilt_, start_tilt_, imu_timeout_, motor_timeout_, hb_timeout_, cmd_timeout_, stand_ramp_s_;
  bool stand_ff_;
  std::string hip_mode_;
  double hip_speed_, hip_accel_, mit_kt_drive_, osc_window_s_, osc_vel_, sat_frac_, sat_time_s_;
  int osc_flips_;
  int osc_sign_[2] = {0, 0};
  std::vector<double> osc_t_[2];
  double sat_s_[2] = {0, 0};
  int rt_prio_;
  gen2_control::Params params_;
  std::unique_ptr<gen2_control::WBCore> core_;
  std::unique_ptr<MotorBus> bus_;
  // inputs
  std::mutex imu_m_, cmd_m_, mode_m_;
  sensor_msgs::msg::Imu imu_;
  int64_t imu_ns_ = 0, cmd_ns_ = 0;
  double vx_ = 0, wz_ = 0;
  std::atomic<int64_t> hb_ns_{0};
  // state (written by the loop thread, read by publish — plain doubles, debug only)
  Mode mode_ = Mode::kDisarmed, want_ = Mode::kDisarmed;
  std::string fault_, last_refusal_;
  double t_mode_ = 0, M_start_[2] = {0, 0}, M_[2] = {0, 0}, h_[2] = {0, 0}, h_tgt_[2] = {0, 0};
  double hip_tau_[2] = {0, 0}, hip_cur_[2] = {0, 0}, wheel_tau_[2] = {0, 0}, wheel_cur_[2] = {0, 0};
  double pitch_ = 0, roll_ = 0, th_kin_ = 0, l_pend_ = 0, imu_age_ms_ = 1e9, vx_cmd_ = 0, wz_cmd_ = 0;
  Output last_;
  std::atomic<double> dt_max_ms_{0.0};
  std::atomic<uint32_t> overruns_{0};
  std::atomic<bool> run_{true};
  std::thread th_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hb_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stand_, srv_bal_, srv_dis_;
  rclcpp::Publisher<gen2_msgs::msg::ControllerState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::executors::MultiThreadedExecutor ex;
    auto node = std::make_shared<BalanceNode>();
    ex.add_node(node);
    ex.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("balance"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
