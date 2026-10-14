// Wheel-leg balance controller core = C++ port of sim/isaaclab/scripts/wbctrl.py (WBController.step),
// one robot, DRIVE + jump state machine (2026-10-12; no automatic edge trigger: the jump is requested by the caller),
// bump_slow off, jump_retry off, no sensor noise / delay models.
// Pure computation, no ROS: the node builds a Frame from the hardware each 5 ms and applies the output.
// Units and signs as wbctrl.py:
//   pitch = asin(g_x) (+ = nose down), roll = asin(g_y) (+ = left side low)
//   wheel torque / speed: + = rolling forward; hip torque: + = extending (supports the body)
//   act[0:2] = (leg target h - h_ref) / 0.12, act[2:4] = wheel torque / wheel_tau_max
#pragma once

#include <array>
#include <vector>

namespace gen2_control
{

constexpr double kDt = 0.005;
constexpr double kHMin = 0.1225, kHMax = 0.2425;
constexpr double kHalfTrack = 0.094;
constexpr double kWheelMax = 18.85;      // rad/s, nominal motor limit the controller believes

// jump phases (wbctrl.py DRIVE, RETRACT, EXTRACT, FLY, DESCEND, LAND)
enum Phase : int {kDrive = 0, kRetract = 1, kExtract = 2, kFly = 3, kDescend = 4, kLand = 5};

struct Params      // climb_test.py TUNE defaults (2026-10-07)
{
  double r_wheel = 0.070;
  double contact_tau_min = 0.8, v_lpf_hz = 10.0;
  double bal_adapt = 0.3, bal_adapt_max_deg = 8.0;
  double roll_rate_lpf_hz = 8.0;
  double vmc_kp = 60.0, vmc_kd = 1.0;
  double vmax_kmh = 3.5, vmax_motor_frac = 0.75;
  double speed_lpf_hz = 4.0, brake_kp = 1.0, brake_ki = 5.5, accel_max = 1.5, speed_guard = 0.8;
  // slope compensation (robot 2026-10-14; 0 = off = wbctrl): steady command + slower than commanded (uphill) ->
  // the speed reference the LQR tracks is raised by v_ki * integral(v_ref - v), at most +-v_boost_max
  double v_ki = 0.0, v_boost_max = 0.4;
  // landing wheels (wbctrl desktop/jump-rl b072017; 0 = off): spin_match_k: in DESCEND each wheel's absolute speed is driven to
  // vf/R (body speed held from take-off), tau = k (vf/R - w_abs) instead of the air pitch PD; air_w_max: in the air no torque in
  // the direction that would push |wheel joint speed| beyond this
  double spin_match_k = 0.0, air_w_max = 0.0;
  // after in-place turns (robot 2026-10-14; 0 = off = wbctrl): no balance-point learning while |wz cmd| or |yaw rate| >=
  // bal_adapt_wz_max (turning leans the body -> wrong th_bias -> creep after the turn); |wz cmd| > x_turn_wz counts as
  // moving for the position hold (x_err 0 -> it holds where the turn ended instead of creeping back to where it began)
  double bal_adapt_wz_max = 0.0, x_turn_wz = 0.0;
  // bias walk (robot 2026-10-14; 0 = off): learn the balance point only while |x_err| < bal_adapt_x_max — otherwise the lean the
  // position hold makes is learned as "upright", th_bias walks to its limit and the robot lurches back every 30-60 s
  double bal_adapt_x_max = 0.0;
  bool turn_slow = true, turn_limit = true;
  double wheel_margin = 0.85, yaw_kd = 0.5, wheel_tau_max = 7.0;
  double yaw_ki = 0.0, yaw_i_max = 0.5;   // yaw-rate integral (desktop 2026-10-08: beats wheel stiction in place)
  bool x_hold_moving = true;          // false: position hold only while the speed command is ~0 (robot 2026-10-09)
  double x_leak_moving = 0.0;         // > 0 [1/s]: while driving x_err leaks instead of being 0 (desktop 2026-10-09)
  double lean_ff = 0.0;               // lean feed-forward: th_ref = lean_ff * atan(g_acc / g) (desktop 2026-10-09)
  double jerk_max = 0.0;              // > 0: S-curve speed reference [m/s^3]; 0 = plain accel_max ramp (wbctrl)
  double land_sf_min = 0.4, lift_detect_s = 0.3, land_detect_s = 0.02, lift_wheel_kd = 0.05;
  double turn_lean = 1.0, roll_kp = 1.5, roll_ki = 15.0, roll_kd = 0.3, roll_leak = 0.5;
  double level_max = 0.10, roll_freeze_deg = 20.0, idle_h = 0.1825;
  double fric_comp_nm = 0.11, fric_comp_w = 0.5, fric_comp_static_nm = 0.0;
  double fric_comp_cmd_nm = 0.0, fric_comp_cmd_w = 0.05;
  double wheel_lpf_hz = 20.0;
  int db_comp = 2;                  // 0 off, 1 boost, 2 sigma
  double db_comp_nm = 0.65, db_comp_eps = 0.05;
  double m_pend = 4.02;             // nominal mass without wheels [kg] (export 7)
  // --- jump state machine (climb_test.py TUNE 2026-10-12, docs: comms/to-robot/attachments/jump_state_machine.md)
  double jump_min_v = 0.2, jump_max_wz = 1.0;        // a request is ignored below this speed / above this yaw rate
  double jump_max_v = 1e9;                           // ... and above this speed (robot: tune.jump_max_v 0.4; wbctrl has none)
  double t_retract = 0.25, retract_lean = 3.0;       // crouch time [s], lean while crouching [deg]
  double t_tuck = 0.12, t_fly_max = 0.45;            // legs tucked after take-off [s], forced landing after [s]
  double contact_tau = 2.0;                          // landing: max |hip torque| above this [N·m] (after 0.04 s)
  double land_s = 0.30, h_land = 0.1825;             // landing phase time [s], leg height to land on [m]
  double leg_kp = 60.0, leg_kd = 1.5, land_kp = 20.0, land_kd = 1.0;   // leg gains in the jump / soft landing
  bool air_ctrl = true, air_pd_extract = true;        // wheel pitch PD in the air (and already in EXTRACT)
  double air_pitch = -5.0, land_pitch = 0.0, extract_pitch = 0.0;   // [deg] targets FLY / DESCEND / EXTRACT
  double air_kp = 30.0, air_kd = 5.0, air_tau = 7.0;  // [N·m/rad], [N·m·s/rad], clip [N·m] per wheel
};

struct LqrTable    // K(l) for x = [x_err, v_err, th, thd]; tau = -K x (sum of both wheels)
{
  std::vector<double> l;
  std::vector<std::array<double, 4>> K;
};

struct Frame
{
  double t = 0.0;
  double g_b[3] = {0, 0, 1};        // projected gravity, body frame
  double w_b[3] = {0, 0, 0};        // gyro, body frame [rad/s]
  double h[2] = {0.1825, 0.1825};   // leg joint values [m] (L, R)
  double tau_hip[2] = {0, 0};       // + = extending
  double w_wheel_joint[2] = {0, 0}; // + = forward
  double w_wheel_abs[2] = {0, 0};   // wheel rotation incl. leg link rotation
  double th_kin = 0.0, l_pend = 0.25;
  double sf = 1.0;                  // IMU specific force [g]
  double motor_scale = 1.0;
};

struct Output
{
  double act[4] = {0, 0, 0, 0};
  double leg_kp = 0.0, leg_kd = 0.0, ff_force = 0.0;
  // debug
  double th = 0, thd = 0, v = 0, pitch = 0, roll = 0, v_ref = 0, v_lim = 0, x_err = 0, th_bias = 0;
  double tau_lqr = 0, tau_yaw = 0, wheel_pre_lpf[2] = {0, 0};   // wheel torque before the LPF / deadband comp
  bool lifted = false;
  int phase = kDrive;               // jump phase after this step
  bool jump_blocked = false;        // a jump was requested but refused (too slow / turning)
};

class WBCore
{
public:
  WBCore(const Params & p, const LqrTable & lqr);
  void reset();
  // vx [m/s], wz [rad/s] commands, h_ref (= idle_h in auto), h_mid (< 0 -> idle_h),
  // jump: operator request this step (pad Y edge) — honoured only in DRIVE, not lifted, jump_min_v <= v <= jump_max_v, |gz| <= jump_max_wz
  Output step(const Frame & f, double vx, double wz, double h_ref, double h_mid = -1.0, bool jump = false);
  int phase() const {return phase_;}
  const Params & params() const {return P_;}
  void set_vmax_kmh(double v) {P_.vmax_kmh = v;}     // runtime speed cap (balance_node live param, disarmed only)

private:
  double lqr_torque(double l, double x_err, double v_err, double th, double thd) const;
  Params P_;
  LqrTable lqr_;
  // state (names as wbctrl.py)
  double v_i_ = 0.0;
  double x_err_, t_un_, t_ld_, g_vf_, g_i_, g_ref_, vf_, rf_, th_bias_, v_prev_, roll_i_, yaw_i_ = 0.0, g_acc_ = 0.0;
  bool lift_on_;
  int phase_ = kDrive;
  double t_phase_ = 0.0, t_takeoff_ = 0.0, h0_ = 0.1825;
  bool soft_ = false;
  double wf_[2], db_e_[2];
};

}  // namespace gen2_control
