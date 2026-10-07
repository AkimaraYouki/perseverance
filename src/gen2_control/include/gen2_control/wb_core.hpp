// Wheel-leg balance controller core = C++ port of sim/isaaclab/scripts/wbctrl.py (WBController.step),
// one robot, DRIVE phase only (no jump state machine, bump_slow off, no sensor noise / delay models).
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

struct Params      // climb_test.py TUNE defaults (2026-10-07)
{
  double r_wheel = 0.070;
  double contact_tau_min = 0.8, v_lpf_hz = 10.0;
  double bal_adapt = 0.3, bal_adapt_max_deg = 8.0;
  double roll_rate_lpf_hz = 8.0;
  double vmc_kp = 60.0, vmc_kd = 1.0;
  double vmax_kmh = 3.5, vmax_motor_frac = 0.75;
  double speed_lpf_hz = 4.0, brake_kp = 1.0, brake_ki = 5.5, accel_max = 1.5, speed_guard = 0.8;
  bool turn_slow = true, turn_limit = true;
  double wheel_margin = 0.85, yaw_kd = 0.5, wheel_tau_max = 7.0;
  double land_sf_min = 0.4, lift_detect_s = 0.3, land_detect_s = 0.02, lift_wheel_kd = 0.05;
  double turn_lean = 1.0, roll_kp = 1.5, roll_ki = 15.0, roll_kd = 0.3, roll_leak = 0.5;
  double level_max = 0.10, roll_freeze_deg = 20.0, idle_h = 0.1825;
  double fric_comp_nm = 0.11, fric_comp_w = 0.5, fric_comp_static_nm = 0.0;
  double fric_comp_cmd_nm = 0.0, fric_comp_cmd_w = 0.05;
  double wheel_lpf_hz = 20.0;
  int db_comp = 2;                  // 0 off, 1 boost, 2 sigma
  double db_comp_nm = 0.65, db_comp_eps = 0.05;
  double m_pend = 4.02;             // nominal mass without wheels [kg] (export 7)
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
  bool lifted = false;
};

class WBCore
{
public:
  WBCore(const Params & p, const LqrTable & lqr);
  void reset();
  // vx [m/s], wz [rad/s] commands, h_ref (= idle_h in auto), h_mid (< 0 -> idle_h)
  Output step(const Frame & f, double vx, double wz, double h_ref, double h_mid = -1.0);
  const Params & params() const {return P_;}

private:
  double lqr_torque(double l, double x_err, double v_err, double th, double thd) const;
  Params P_;
  LqrTable lqr_;
  // state (names as wbctrl.py)
  double x_err_, t_un_, t_ld_, g_vf_, g_i_, g_ref_, vf_, rf_, th_bias_, v_prev_, roll_i_;
  bool lift_on_;
  double wf_[2], db_e_[2];
};

}  // namespace gen2_control
