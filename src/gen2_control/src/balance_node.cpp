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
#include <array>
#include <cstdio>
#include <ctime>
#include <filesystem>
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
#include "std_msgs/msg/float64.hpp"
#include "gen2_sensors/imu_shm.hpp"
#include "gen2_msgs/msg/motor_state_array.hpp"
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

// One 200 Hz control step (stand/balance), written to CSV by a non-RT thread (desktop request 2026-10-08)
struct StepRec
{
  uint64_t step; int64_t t_ns; int session; int mode;
  double dt_ms, imu_age_ms, pitch, roll, gx, gy, gz, th, thd, th_kin, th_bias, x_err, v, v_ref, vx, wz;
  double h[2], M[2], Md[2], h_tgt[2], hip_tau[2], hip_cur_fb[2], hip_age_ms[2];
  double w_wheel[2], tau_lqr, tau_yaw, wheel_pre_lpf[2], wheel_tau[2], wheel_tau_fb[2], wheel_age_ms[2];
};
constexpr std::size_t kRing = 8192;

// RBJ biquad notch (direct form I)
struct Notch
{
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  void design(double f0, double q, double fs)
  {
    const double w = 2.0 * M_PI * f0 / fs, al = std::sin(w) / (2.0 * q), a0 = 1.0 + al;
    b0 = 1.0 / a0; b1 = -2.0 * std::cos(w) / a0; b2 = 1.0 / a0; a1 = -2.0 * std::cos(w) / a0; a2 = (1.0 - al) / a0;
  }
  double step(double x)
  {
    const double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
};   // 41 s at 200 Hz
constexpr double kHMinSit = gen2_control::kHMin + 0.003;   // just above the lowest table height
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
    // operator link lost while balancing: stop (vx, wz = 0) -> after link_sit_after_s sit down and keep
    // balancing low -> disarm after link_disarm_s (desktop 2026-10-08: 0 A at once tips the robot over)
    link_stop_ = declare_parameter("link_loss_stop", true);
    frozen_s_ = declare_parameter("wheel_frozen_s", 0.15);
    // optional notch on the sent wheel torque (pitch chatter through the gear backlash, 14-18 Hz); 0 = off
    {
      const double f0 = declare_parameter("wheel_notch_hz", 0.0), q = declare_parameter("wheel_notch_q", 2.0);
      notch_on_ = f0 > 0.0;
      if (notch_on_) {for (auto & n : notch_) {n.design(f0, q, rate_hz_);}}
    }
    frozen_cmd_nm_ = declare_parameter("wheel_frozen_cmd_nm", 0.15);
    link_sit_after_s_ = declare_parameter("link_sit_after_s", 2.0);
    link_disarm_s_ = declare_parameter("link_disarm_s", 30.0);
    cmd_timeout_ = declare_parameter("cmd_timeout_s", 0.5);
    stand_ramp_s_ = declare_parameter("stand_ramp_s", 2.0);
    stand_ff_ = declare_parameter("stand_feedforward", true);
    // hips: "servo_pos" = drive position-speed loop (PID inside the drive, stable; stiff, no VMC
    // compliance / feed-forward), "current_pd" = host PD -> current (wbctrl VMC, needs >= 500 Hz and
    // a clean velocity: shook at 200 Hz on 2026-10-07)
    hip_mode_ = declare_parameter("hip_mode", std::string("servo_pos"));
    // "mit" = AK 3.0 MIT frames: damping kd inside the drive (kHz), stiffness kp(M*-M) + feed-forward
    //   computed here and sent as t_ff; p_des unused (drive frame has the 60 deg power-up wrap)
    if (hip_mode_ != "servo_pos" && hip_mode_ != "current_pd" && hip_mode_ != "mit" && hip_mode_ != "mit_pos") {
      throw std::runtime_error("hip_mode: servo_pos|current_pd|mit");
    }
    mit_kt_drive_ = declare_parameter("mit_kt_drive", 0.5994);
    mit_kp_scale_ = declare_parameter("mit_kp_scale", 1.0);   // mit_pos: drive kp correction (probe hinted ~0.56 effective)   // Kt the drive uses for t -> current (measured 2026-10-07)
    osc_flips_ = static_cast<int>(declare_parameter("osc_flips", 6));
    osc_window_s_ = declare_parameter("osc_window_s", 0.3);
    osc_vel_ = declare_parameter("osc_vel_rad_s", 1.0);
    sat_frac_ = declare_parameter("sat_frac", 0.9);
    sat_time_s_ = declare_parameter("sat_time_s", 0.1);
    sit_sat_a_ = declare_parameter("sit_sat_current_a", 5.0);
    hip_speed_ = declare_parameter("hip_speed_rad_s", 2.0);
    hip_accel_ = declare_parameter("hip_accel_rad_s2", 20.0);
    rt_prio_ = static_cast<int>(declare_parameter("rt_priority", 80));
    use_shm_ = declare_parameter("imu_shm", true);
    // balance-point trim [deg] added to th_kin: + = the controller balances with the body leaning BACK
    // by this much (use when the robot drifts forward: the real COM is ahead of the model)
    th_trim_ = declare_parameter("th_trim_deg", 0.0) * M_PI / 180.0;
    // controller params (wbctrl TUNE names), defaults = Params{}
    gen2_control::Params P;
    P.r_wheel = declare_parameter("leg_table.r_wheel", P.r_wheel);
    P.idle_h = declare_parameter("tune.idle_h", P.idle_h);
    P.vmc_kp = declare_parameter("tune.vmc_kp", P.vmc_kp);
    P.vmc_kd = declare_parameter("tune.vmc_kd", P.vmc_kd);
    P.vmax_kmh = declare_parameter("tune.vmax_kmh", P.vmax_kmh);
    P.accel_max = declare_parameter("tune.accel_max", P.accel_max);
    P.x_hold_moving = declare_parameter("tune.x_hold_moving", P.x_hold_moving);
    P.fric_comp_nm = declare_parameter("tune.fric_comp_nm", P.fric_comp_nm);
    P.db_comp = static_cast<int>(declare_parameter("tune.db_comp", static_cast<int64_t>(P.db_comp)));
    P.db_comp_nm = declare_parameter("tune.db_comp_nm", P.db_comp_nm);
    P.wheel_lpf_hz = declare_parameter("tune.wheel_lpf_hz", P.wheel_lpf_hz);
    P.bal_adapt = declare_parameter("tune.bal_adapt", P.bal_adapt);
    P.contact_tau_min = declare_parameter("tune.contact_tau_min", P.contact_tau_min);
    P.roll_kp = declare_parameter("tune.roll_kp", P.roll_kp);
    P.roll_ki = declare_parameter("tune.roll_ki", P.roll_ki);
    P.roll_kd = declare_parameter("tune.roll_kd", P.roll_kd);
    P.turn_lean = declare_parameter("tune.turn_lean", P.turn_lean);
    P.yaw_kd = declare_parameter("tune.yaw_kd", P.yaw_kd);
    P.yaw_ki = declare_parameter("tune.yaw_ki", P.yaw_ki);
    P.yaw_i_max = declare_parameter("tune.yaw_i_max", P.yaw_i_max);
    P.m_pend = model_ok_ ? m_pend_ : P.m_pend;
    // wheels in MIT (legacy AK45-10 firmware): no 0.5 A deadband -> no sigma-delta compensation
    for (const auto & c : motors) {
      if ((c.name == "wheel_l" || c.name == "wheel_r") && c.protocol == "mit_legacy" && P.db_comp != 0) {
        P.db_comp = 0;
        RCLCPP_INFO(get_logger(), "wheels in MIT: deadband compensation off");
      }
    }
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
    // hip zero: adopt the monitor's resolved wrap when this process starts inside the ambiguous band
    // (legs hanging at the end stop) — only if both see the same drive position (same drive boot)
    ms_sub_ = create_subscription<gen2_msgs::msg::MotorStateArray>("motors/state", rclcpp::SensorDataQoS(),
        [this](gen2_msgs::msg::MotorStateArray::ConstSharedPtr msg) {
          for (const auto & s : msg->motors) {
            const auto it = idx_.find(s.name);
            if (it == idx_.end() || s.stale || s.zero_state != 1) {continue;}
            const MotorFeedback fb = bus_->feedback(it->second);
            if (fb.valid && !fb.zero_ok() && std::fabs(fb.status.position_deg - s.raw_position_deg) < 1.0) {
              bus_->adopt_unwrap(it->second, s.unwrap_deg);
              RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s: adopted the monitor's resolved zero (unwrap %+.0f deg)",
                s.name.c_str(), s.unwrap_deg);
            }
          }
        });
    hb_sub_ = create_subscription<std_msgs::msg::Empty>("balance/heartbeat", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {hb_ns_ = now_ns();});
    // body height command [m] (balance mode): clamped to the leg table, followed at height_rate_m_s
    h_rate_ = declare_parameter("height_rate_m_s", 0.05);
    h_sub_ = create_subscription<std_msgs::msg::Float64>("balance/height", 10,
        [this](std_msgs::msg::Float64::ConstSharedPtr m) {
          if (std::isfinite(m->data)) {h_cmd_ = std::clamp(m->data, gen2_control::kHMin + 0.01, gen2_control::kHMax - 0.01);}
        });
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
    // START: (home if a hip zero is ambiguous) -> stand -> balance as soon as the body is upright
    srv_start_ = create_service<std_srvs::srv::Trigger>("balance/start",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr, std_srvs::srv::Trigger::Response::SharedPtr rs) {
          rs->message = request(Mode::kStand, true);
          rs->success = rs->message.rfind("ok", 0) == 0;
        });
    home_tau_ = declare_parameter("home_tau_nm", 0.4);
    home_kd_ = declare_parameter("home_kd", 0.3);
    home_timeout_s_ = declare_parameter("home_timeout_s", 3.0);
    auto_tilt_ = declare_parameter("start_balance_tilt_deg", 5.0) * M_PI / 180.0;
    auto_wait_s_ = declare_parameter("start_balance_wait_s", 10.0);
    // sit: lower the body slowly to the lowest leg height (balance keeps running), then disarm
    srv_sit_ = create_service<std_srvs::srv::Trigger>("balance/sit",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr, std_srvs::srv::Trigger::Response::SharedPtr rs) {
          std::lock_guard<std::mutex> lk(mode_m_);
          if (mode_ != Mode::kStand && mode_ != Mode::kBalance) {rs->success = false; rs->message = "not standing"; return;}
          sit_ = true; sit_t_ = 0.0; sit_hold_ = false;
          rs->success = true; rs->message = "ok: sitting down";
        });
    sit_s_ = declare_parameter("sit_s", 3.0);
    pub_ = create_publisher<gen2_msgs::msg::ControllerState>("controller/state", 10);
    pub_timer_ = create_wall_timer(10ms, [this] {publish();});
    step_log_ = declare_parameter("step_log", true);
    step_log_dir_ = declare_parameter("step_log_dir", std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") + "/gen2_ws/logs/steps");
    if (step_log_) {log_th_ = std::thread([this] {log_writer();});}
    th_ = std::thread([this] {loop();});
    RCLCPP_INFO(get_logger(), "balance_node ready (model %s): modes disarmed/stand%s",
      model_ok_ ? "loaded" : "MISSING", model_ok_ ? "/balance" : " only");
  }

  ~BalanceNode() override
  {
    run_ = false;
    if (th_.joinable()) {th_.join();}
    log_run_ = false;
    if (log_th_.joinable()) {log_th_.join();}
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
  std::string request(Mode m, bool start = false)
  {
    in_request_ = true;
    struct Done {std::atomic<bool> & f; ~Done() {f = false;}} done{in_request_};
    bool wake = false;
    {
      std::lock_guard<std::mutex> lk(mode_m_);
      if (m == Mode::kDisarmed) {
        want_ = Mode::kDisarmed;
        return "ok: disarm";
      }
      if (m == Mode::kBalance && !model_ok_) {return "refused: model tables missing (sim/model/balance_tables.yaml)";}
      if (mode_ == Mode::kFault) {return "refused: in fault (" + fault_ + "), disarm first";}
      // START while already up must not drop to stand (wheels off -> the robot falls, seen 2026-10-08)
      if (start && (mode_ == Mode::kBalance || want_ == Mode::kBalance || (mode_ == Mode::kStand && start_))) {
        return "ok: already running";
      }
      wake = mode_ == Mode::kDisarmed;
    }
    // MIT wheels: wake them without holding the mode lock (the 200 Hz loop takes it every step)
    if (wake) {
      const std::string w = wake_mit();
      if (!w.empty()) {std::lock_guard<std::mutex> lk(mode_m_); last_refusal_ = w; return "refused: " + w;}
    }
    std::string why;
    {
      std::lock_guard<std::mutex> lk(mode_m_);
      why = arm_check(start);
      if (why.empty() && start && hip_mode_.rfind("mit", 0) != 0 && !hips_zero_ok()) {why = "home needs hip_mode mit / mit_pos";}
      if (why.empty()) {
        want_ = m;
        last_refusal_.clear();
        if (start) {
          start_ = true; homing_ = !hips_zero_ok(); start_t_ = 0.0;
          return homing_ ? "ok: start (home -> stand -> balance)" : "ok: start (stand -> balance)";
        }
        return std::string("ok: ") + mode_name(m);
      }
      last_refusal_ = why;
    }
    if (wake) {zero_all(); bus_->enable_tx(false);}    // undo the wake (zero_all sleeps: not under the lock)
    return "refused: " + why;
  }

  bool hips_zero_ok() const
  {
    for (const char * n : {"leg_l", "leg_r"}) {if (!bus_->feedback(idx_.at(n)).zero_ok()) {return false;}}
    return true;
  }

  std::string arm_check(bool start = false)
  {
    const int64_t t = now_ns();
    if ((t - hb_ns_.load()) * 1e-9 > hb_timeout_) {return "no operator heartbeat (balance/heartbeat)";}
    if (imu_age_ms_ > imu_timeout_ * 1e3) {return "IMU not fresh";}
    for (const auto & kv : idx_) {
      const MotorFeedback fb = bus_->feedback(kv.second);
      const auto & c = bus_->motors()[kv.second];
      if (!fb.valid || (t - fb.mono_ns) * 1e-9 > motor_timeout_) {return kv.first + ": feedback not fresh";}
      if (fb.status.error) {return kv.first + ": drive fault";}
      if (!fb.zero_ok() && !(start && (kv.first == "leg_l" || kv.first == "leg_r"))) {return kv.first + ": joint zero not resolved (press START to home)";}
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

  // zero torque frame for any protocol
  gen2_hardware::cubemars::Frame zero_frame(const gen2_hardware::MotorConfig & c) const
  {
    if (c.protocol == "mit_legacy") {
      return gen2_hardware::mit::encode(gen2_hardware::mit::Proto::kLegacy, c.can_id, c.mit_ranges, {});
    }
    return gen2_hardware::cubemars::encode_current(c.can_id, 0.0);
  }

  void zero_all()
  {
    if (!bus_ || !bus_->tx_enabled()) {return;}
    std::string err;
    for (int r = 0; r < 10; ++r) {
      for (const auto & kv : idx_) {bus_->send(zero_frame(bus_->motors()[kv.second]), err);}
      std::this_thread::sleep_for(2ms);
    }
    for (const auto & kv : idx_) {     // MIT wheels: leave motor mode
      const auto & c = bus_->motors()[kv.second];
      if (c.protocol == "mit_legacy") {bus_->send(gen2_hardware::mit::exit(c.can_id), err);}
    }
  }

  // MIT wheels reply only to commands: enter motor mode (zero gains) and wait for fresh feedback
  std::string wake_mit()
  {
    std::vector<std::size_t> ids;
    for (const auto & kv : idx_) {
      if (bus_->motors()[kv.second].protocol == "mit_legacy") {ids.push_back(kv.second);}
    }
    if (ids.empty()) {return "";}
    bus_->enable_tx(true);
    std::string err;
    for (auto i : ids) {bus_->send(gen2_hardware::mit::enter(bus_->motors()[i].can_id), err);}
    for (int k = 0; k < 20; ++k) {
      std::this_thread::sleep_for(5ms);
      bool all = true;
      for (auto i : ids) {
        bus_->send(zero_frame(bus_->motors()[i]), err);
        const auto fb = bus_->feedback(i);
        all = all && fb.valid && (now_ns() - fb.mono_ns) * 1e-9 < 0.02;
      }
      if (all) {return "";}
    }
    std::string who;
    for (auto i : ids) {
      const auto fb = bus_->feedback(i);
      if (!fb.valid || (now_ns() - fb.mono_ns) * 1e-9 >= 0.02) {who += bus_->motors()[i].name + " ";}
    }
    for (auto i : ids) {bus_->send(gen2_hardware::mit::exit(bus_->motors()[i].can_id), err);}
    if (mode_ == Mode::kDisarmed || mode_ == Mode::kFault) {bus_->enable_tx(false);}
    return "MIT wheel(s) not answering: " + who;
  }

  void enter(Mode m, const std::string & why = "")
  {
    link_lost_ = false; sit_hold_ = false;
    if (m != Mode::kStand) {start_ = false; homing_ = false;}
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
    if ((m == Mode::kStand || m == Mode::kBalance) && (mode_ == Mode::kDisarmed || mode_ == Mode::kFault)) {++log_session_;}
    if (m == Mode::kStand || m == Mode::kBalance) {
      t_mode_ = 0.0;
      for (auto & n : notch_) {n.x1 = n.x2 = n.y1 = n.y2 = 0.0;}
      for (int k = 0; k < 2; ++k) {M_start_[k] = M_[k]; osc_t_[k].clear(); osc_sign_[k] = 0; sat_s_[k] = 0.0;}
      sit_ = false;
      h_set_ = params_.idle_h; h_cmd_ = -1.0;
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
    // preferred: shared memory written by iahrs_node (no DDS, stamp = serial line arrival)
    if (use_shm_ && !shm_.is_open() && t - shm_try_ns_ > 1000000000LL) {
      shm_try_ns_ = t;
      if (shm_.open()) {RCLCPP_INFO(get_logger(), "IMU via shared memory %s", gen2_sensors::kImuShmName);}
    }
    gen2_sensors::ImuSample sm;
    if (shm_.read(sm) && (t - sm.rx_mono_ns) * 1e-9 < imu_timeout_) {
      imu.orientation.w = sm.q[0]; imu.orientation.x = sm.q[1]; imu.orientation.y = sm.q[2]; imu.orientation.z = sm.q[3];
      imu.angular_velocity.x = sm.gyro[0]; imu.angular_velocity.y = sm.gyro[1]; imu.angular_velocity.z = sm.gyro[2];
      imu.linear_acceleration.x = sm.acc[0]; imu.linear_acceleration.y = sm.acc[1]; imu.linear_acceleration.z = sm.acc[2];
      imu_ns = sm.rx_mono_ns;
      imu_src_shm_ = true;
    } else {
      imu_src_shm_ = false;
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
        else if (i < 2 && !fb[i].zero_ok() && !homing_) {bad = std::string(names[i]) + " zero not resolved";}
      }
      // AK45-10 MIT replies can freeze (2026-10-08: blocked wheel + torque reversals -> replies keep coming
      // with the same position and torque, commands ignored for ~1 min, error 0). Fault when the reply
      // stays bit-identical for frozen_s while the commanded torque moved by more than frozen_cmd_nm.
      if (i >= 2 && mode_ == Mode::kBalance) {
        const int w = i - 2;
        const double cmd = wheel_tau_[w];
        if (fb[i].status.position_deg == frz_p_[w] && fb[i].status.current_a == frz_i_[w]) {
          frz_lo_[w] = std::min(frz_lo_[w], cmd); frz_hi_[w] = std::max(frz_hi_[w], cmd);
          if (bad.empty() && (t - frz_ns_[w]) * 1e-9 > frozen_s_ && frz_hi_[w] - frz_lo_[w] > frozen_cmd_nm_) {
            bad = std::string(names[i]) + " replies frozen (drive ignores commands)";
          }
        } else {
          frz_p_[w] = fb[i].status.position_deg; frz_i_[w] = fb[i].status.current_a; frz_ns_[w] = t;
          frz_lo_[w] = frz_hi_[w] = cmd;
        }
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
    f.th_kin += th_trim_;
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
          // sitting needs little torque: a leg pushing harder is blocked (2026-10-08: 9.9 A slipped leg_r's link)
          const double lim = sit_ ? sit_sat_a_ : sat_frac_ * c.current_limit_a;
          const bool sat = std::fabs(fb[k].status.current_a) > lim;
          sat_s_[k] = sat ? sat_s_[k] + 1.0 / rate_hz_ : 0.0;
          if (sat_s_[k] > sat_time_s_) {why = std::string(names[k]) + (sit_ ? " blocked while sitting" : " current saturated");}
        }
        if (why.empty() && imu_age_ms_ > imu_timeout_ * 1e3) {why = "IMU stale";}
        const bool hb_ok = (t - hb_ns_.load()) * 1e-9 <= hb_timeout_;
        if (why.empty() && !hb_ok) {
          if (mode_ == Mode::kBalance && link_stop_) {
            if (!link_lost_) {
              link_lost_ = true; t_lost_ = t;
              last_refusal_ = "operator link lost: stop, sit, disarm in " + std::to_string(static_cast<int>(link_disarm_s_)) + " s";
              RCLCPP_WARN(get_logger(), "%s", last_refusal_.c_str());
            }
            const double lost_s = (t - t_lost_) * 1e-9;
            if (!sit_ && lost_s > link_sit_after_s_) {
              sit_ = true; sit_t_ = 0.0; sit_hold_ = true;
              last_refusal_ = "operator link lost: sitting, balancing low";
              RCLCPP_WARN(get_logger(), "%s", last_refusal_.c_str());
            }
            if (lost_s > link_disarm_s_) {
              RCLCPP_WARN(get_logger(), "operator link lost for %.0f s: disarm", lost_s);
              enter(Mode::kDisarmed); want_ = Mode::kDisarmed;
            }
          } else {
            why = "operator heartbeat lost";
          }
        }
        if (hb_ok && link_lost_) {
          link_lost_ = false; last_refusal_.clear();
          RCLCPP_INFO(get_logger(), "operator link back%s", sit_hold_ ? " (staying low: sit or disarm to finish)" : "");
        }
        if (why.empty() && (std::fabs(pitch_) > max_tilt_ || std::fabs(roll_) > max_tilt_)) {why = "tilt limit (fell)";}
        if (!why.empty()) {enter(Mode::kFault, why); want_ = Mode::kFault;}
      }
    }
    if (mode_ != Mode::kStand && mode_ != Mode::kBalance) {
      // idle / fault: keep the legacy-MIT wheels visible (they answer only to commands) — zero-torque EXIT at 2 Hz
      if ((mode_ == Mode::kDisarmed || mode_ == Mode::kFault) && want_ == mode_ && !in_request_ && t - idle_ping_ns_ > 500000000LL) {
        idle_ping_ns_ = t;
        bus_->enable_tx(true);
        for (const auto & kv : idx_) {
          const auto & c = bus_->motors()[kv.second];
          if (c.protocol == "mit_legacy") {bus_->send(gen2_hardware::mit::exit(c.can_id), err_ping_);}
        }
        bus_->enable_tx(false);
      }
      return;
    }
    if (start_ && mode_ == Mode::kStand) {
      start_t_ += 1.0 / rate_hz_;
      if (homing_) {
        // both hips pushed gently toward the retract stop; stalled = |Md| < 0.05 rad/s for 0.3 s
        const bool still = std::fabs(Md[0]) < 0.05 && std::fabs(Md[1]) < 0.05;
        home_still_ = still && start_t_ > 0.3 ? home_still_ + 1.0 / rate_hz_ : 0.0;
        if (home_still_ > 0.3) {
          for (int k = 0; k < 2; ++k) {
            const std::size_t mi = idx_.at(names[k]);
            const auto & c = bus_->motors()[mi];
            if (fb[k].zero_ok() || !(c.wrap_deg > 0)) {continue;}
            // candidate whose joint angle is closest to the retract stop
            double best = 1e9, sh = 0.0;
            for (int w = -12; w <= 12; ++w) {
              const double j = c.raw_to_joint_pos(fb[k].status.position_deg + w * c.wrap_deg) * 180.0 / M_PI;
              if (std::fabs(j - c.stop_min_deg) < std::fabs(best - c.stop_min_deg)) {best = j; sh = w * c.wrap_deg;}
            }
            bus_->adopt_unwrap(mi, sh);
            RCLCPP_INFO(get_logger(), "%s: homed on the retract stop (%.1f deg, unwrap %+.0f)", names[k], best, sh);
          }
          homing_ = false; home_done_ = true; start_t_ = 0.0;
        } else if (start_t_ > home_timeout_s_) {
          std::lock_guard<std::mutex> lk(mode_m_);
          enter(Mode::kFault, "home: hips did not settle on the retract stop"); want_ = Mode::kFault;
          return;
        }
      } else if (home_done_) {
        // zero adopted by the bus thread on the next reply: restart the stand ramp from the true angle
        if (fb[0].zero_ok() && fb[1].zero_ok()) {
          home_done_ = false; t_mode_ = 0.0;
          for (int k = 0; k < 2; ++k) {M_start_[k] = M[k];}
        }
      } else if (t_mode_ > stand_ramp_s_) {
        if (std::fabs(pitch_) < auto_tilt_ && std::fabs(roll_) < auto_tilt_) {
          std::lock_guard<std::mutex> lk(mode_m_);
          if (want_ == Mode::kStand) {want_ = Mode::kBalance; RCLCPP_INFO(get_logger(), "start: upright -> balance");}
        } else if (start_t_ > stand_ramp_s_ + auto_wait_s_) {
          start_ = false;
          std::lock_guard<std::mutex> lk(mode_m_);
          last_refusal_ = "start: body not upright within " + std::to_string(static_cast<int>(auto_wait_s_)) + " s (still standing)";
        }
      }
    }
    if (link_lost_) {vx = wz = 0.0; vx_cmd_ = 0.0; wz_cmd_ = 0.0;}
    t_mode_ += 1.0 / rate_hz_;
    double sit_u = -1.0;     // < 0: not sitting; 0..1 ramp to the lowest height
    if (sit_) {
      if (sit_t_ == 0.0) {for (int k = 0; k < 2; ++k) {sit_from_[k] = M_[k];} sit_h0_ = h_set_ > 0 ? h_set_ : params_.idle_h;}
      sit_t_ += 1.0 / rate_hz_;
      sit_u = std::min(1.0, sit_t_ / sit_s_);
      if (sit_t_ > sit_s_ + 0.5 && !sit_hold_) {
        std::lock_guard<std::mutex> lk(mode_m_);
        enter(Mode::kDisarmed);
        want_ = Mode::kDisarmed;
        return;
      }
    }
    const double M_low = interp(kHMinSit, leg_h_, leg_th_) - theta0_;

    // ---- control
    double h_tgt[2], M_tgt[2], ffF = 0.0, kp = params_.vmc_kp, kd = params_.vmc_kd, wheel_tau[2] = {0, 0};
    const double M_idle = interp(params_.idle_h, leg_h_, leg_th_) - theta0_;
    if (mode_ == Mode::kStand) {
      // ramp in joint angle from where the leg really is (it may sit beyond the leg table, e.g. at the
      // retracted stop -12 deg < -7 deg: a height ramp would start with a step there)
      const double u = std::min(1.0, t_mode_ / stand_ramp_s_);
      for (int k = 0; k < 2; ++k) {
        M_tgt[k] = sit_u >= 0 ? sit_from_[k] + sit_u * (M_low - sit_from_[k]) : M_start_[k] + u * (M_idle - M_start_[k]);
        h_tgt[k] = interp(theta0_ + M_tgt[k], leg_th_, leg_h_);
      }
      ffF = stand_ff_ ? 0.5 * (model_ok_ ? m_pend_ : params_.m_pend) * 9.81 : 0.0;
    } else {
      // sitting: the leg mid height ramps from IDLE to the lowest height while the LQR keeps balancing
      const double hc = h_cmd_.load() > 0 ? h_cmd_.load() : params_.idle_h;
      const double dh = h_rate_ / rate_hz_;
      h_set_ = h_set_ > 0 ? h_set_ + std::clamp(hc - h_set_, -dh, dh) : params_.idle_h;
      const double h_mid = sit_u >= 0 ? sit_h0_ + sit_u * (kHMinSit - sit_h0_) : h_set_;
      const Output o = core_->step(f, sit_u >= 0 ? 0.0 : vx, sit_u >= 0 ? 0.0 : wz, params_.idle_h, h_mid);
      last_ = o;
      for (int k = 0; k < 2; ++k) {
        h_tgt[k] = params_.idle_h + 0.12 * o.act[k];
        M_tgt[k] = interp(std::clamp(h_tgt[k], leg_h_.front(), leg_h_.back()), leg_h_, leg_th_) - theta0_;
        wheel_tau[k] = o.act[2 + k] * params_.wheel_tau_max;
        if (notch_on_) {wheel_tau[k] = notch_[k].step(wheel_tau[k]);}
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
      } else if (hip_mode_ == "mit" || hip_mode_ == "mit_pos") {
        // drive: t = t_ff + kd_drive * (0 - w_drive); it converts with its own Kt (0.6) -> scale by kt_drive / kt
        const double kt = std::isfinite(c.kt_nm_per_a) ? c.kt_nm_per_a : mit_kt_drive_;
        const double sc = mit_kt_drive_ / kt;
        // mit_pos: the joint stiffness runs inside the drive (kHz loop) — p_des in the drive's own output
        // frame = servo-upload position [rad] (2026-10-09 probe: same frame, same sign). Host sends p, kp, ff.
        const bool in_drive = hip_mode_ == "mit_pos" && !(homing_ || home_done_);
        const double ff = ffF * interp(th[k], leg_th_, leg_dh_);
        double tau = homing_ || home_done_ ? -home_tau_ : in_drive ? ff : kp * (M_tgt[k] - M[k]) + ff;
        const double tmax = c.current_limit_a * kt;
        tau = std::clamp(tau, -tmax, tmax);
        gen2_hardware::mit::Command mc;
        mc.t = tau * c.direction * sc;
        if (in_drive) {
          const double p_raw_deg = fb[k].status.position_deg +
            (M_tgt[k] - M[k]) * 180.0 / M_PI * c.raw_deg_per_output_rev / 360.0 / c.direction;
          mc.p = p_raw_deg * M_PI / 180.0 * 360.0 / c.raw_deg_per_output_rev;
          mc.kp = kp * sc * mit_kp_scale_;
          tau = std::clamp(kp * (M_tgt[k] - M[k]) + ff, -tmax, tmax);   // logging: what the drive should produce
        }
        mc.kd = (homing_ || home_done_ ? home_kd_ : kd) * sc;
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
      if (cw.protocol == "mit_legacy") {
        // torque command, drive converts with its own Kt; limit = current_limit x Kt
        gen2_hardware::mit::Command mc;
        mc.t = wc * cw.kt_nm_per_a;      // = clamped joint torque in the drive frame (wc already has the sign)
        bus_->send(gen2_hardware::mit::encode(gen2_hardware::mit::Proto::kLegacy, cw.can_id, cw.mit_ranges, mc), err);
      } else {
        bus_->send(gen2_hardware::cubemars::encode_current(cw.can_id, wc), err);
      }
    }
    if (step_log_) {
      StepRec r{};
      r.step = step_n_++; r.t_ns = t; r.session = log_session_; r.mode = static_cast<int>(mode_);
      r.dt_ms = (t - prev_step_ns_) * 1e-6; r.imu_age_ms = imu_age_ms_; r.pitch = pitch_; r.roll = roll_;
      r.gx = f.w_b[0]; r.gy = f.w_b[1]; r.gz = f.w_b[2];
      const bool bal = mode_ == Mode::kBalance;
      r.th = bal ? last_.th : 0; r.thd = bal ? last_.thd : 0; r.th_kin = th_kin_; r.th_bias = bal ? last_.th_bias : 0;
      r.x_err = bal ? last_.x_err : 0; r.v = bal ? last_.v : 0; r.v_ref = bal ? last_.v_ref : 0; r.vx = vx; r.wz = wz;
      r.tau_lqr = bal ? last_.tau_lqr : 0; r.tau_yaw = bal ? last_.tau_yaw : 0;
      for (int k = 0; k < 2; ++k) {
        const auto & cw = bus_->motors()[idx_.at(names[2 + k])];
        r.h[k] = h_[k]; r.M[k] = M[k]; r.Md[k] = Md[k]; r.h_tgt[k] = h_tgt[k];
        r.hip_tau[k] = hip_tau_[k]; r.hip_cur_fb[k] = fb[k].status.current_a; r.hip_age_ms[k] = (t - fb[k].mono_ns) * 1e-6;
        r.w_wheel[k] = f.w_wheel_joint[k]; r.wheel_pre_lpf[k] = bal ? last_.wheel_pre_lpf[k] : 0; r.wheel_tau[k] = wheel_tau[k];
        r.wheel_tau_fb[k] = cw.current_to_joint_torque(fb[2 + k].status.current_a);
        r.wheel_age_ms[k] = (t - fb[2 + k].mono_ns) * 1e-6;
      }
      const uint64_t w = ring_w_.load(std::memory_order_relaxed);
      if (w - ring_r_.load(std::memory_order_acquire) < kRing) {ring_[w % kRing] = r; ring_w_.store(w + 1, std::memory_order_release);}
      else {++log_dropped_;}
    }
    prev_step_ns_ = t;
  }

  // non-RT: drain the ring into logs/steps/<date>_<session>.csv (new file per arm)
  void log_writer()
  {
    std::FILE * fp = nullptr;
    int cur = -1;
    int64_t idle_since = 0;
    std::error_code ec;
    std::filesystem::create_directories(step_log_dir_, ec);
    while (log_run_) {
      std::this_thread::sleep_for(20ms);
      uint64_t rd = ring_r_.load(std::memory_order_relaxed);
      const uint64_t wr = ring_w_.load(std::memory_order_acquire);
      if (rd == wr) {
        if (fp && idle_since && now_ns() - idle_since > 1000000000LL) {std::fclose(fp); fp = nullptr;}
        if (!idle_since) {idle_since = now_ns();}
        continue;
      }
      idle_since = 0;
      for (; rd != wr; ++rd) {
        const StepRec & r = ring_[rd % kRing];
        if (!fp || r.session != cur) {
          if (fp) {std::fclose(fp);}
          cur = r.session;
          char name[64];
          const std::time_t now = std::time(nullptr);
          std::strftime(name, sizeof(name), "%Y-%m-%d_%H%M%S", std::localtime(&now));
          const std::string path = step_log_dir_ + "/" + name + "_step.csv";
          fp = std::fopen(path.c_str(), "w");
          if (fp) {
            RCLCPP_INFO(get_logger(), "step log -> %s", path.c_str());
            std::fprintf(fp, "step,t_ns,mode,dt_ms,imu_age_ms,pitch,roll,gx,gy,gz,th,thd,th_kin,th_bias,x_err,v,v_ref,vx,wz,"
              "hL,hR,ML,MR,MdL,MdR,h_tgtL,h_tgtR,hip_tauL,hip_tauR,hip_curL,hip_curR,hip_ageL,hip_ageR,"
              "wwL,wwR,tau_lqr,tau_yaw,pre_lpfL,pre_lpfR,wheel_tauL,wheel_tauR,wheel_fbL,wheel_fbR,wheel_ageL,wheel_ageR\n");
          }
        }
        if (fp) {
          std::fprintf(fp, "%llu,%lld,%d,%.3f,%.2f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.4f,%.5f,%.5f,%.4f,%.4f,%.3f,%.3f,%.3f,"
            "%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,"
            "%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n",
            static_cast<unsigned long long>(r.step), static_cast<long long>(r.t_ns), r.mode, r.dt_ms, r.imu_age_ms,  // NOLINT
            r.pitch, r.roll, r.gx, r.gy, r.gz, r.th, r.thd, r.th_kin, r.th_bias, r.x_err, r.v, r.v_ref, r.vx, r.wz,
            r.h[0], r.h[1], r.M[0], r.M[1], r.Md[0], r.Md[1], r.h_tgt[0], r.h_tgt[1], r.hip_tau[0], r.hip_tau[1],
            r.hip_cur_fb[0], r.hip_cur_fb[1], r.hip_age_ms[0], r.hip_age_ms[1],
            r.w_wheel[0], r.w_wheel[1], r.tau_lqr, r.tau_yaw, r.wheel_pre_lpf[0], r.wheel_pre_lpf[1],
            r.wheel_tau[0], r.wheel_tau[1], r.wheel_tau_fb[0], r.wheel_tau_fb[1], r.wheel_age_ms[0], r.wheel_age_ms[1]);
        }
      }
      ring_r_.store(rd, std::memory_order_release);
      if (fp) {std::fflush(fp);}
    }
    if (fp) {std::fclose(fp);}
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
    s.imu_shm = imu_src_shm_;
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
  double mit_kp_scale_ = 1.0;
  double hip_speed_, hip_accel_, mit_kt_drive_, osc_window_s_, osc_vel_, sat_frac_, sat_time_s_, sit_sat_a_;
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
  // 200 Hz step log (SPSC ring: loop thread writes, log_writer reads)
  bool step_log_ = true;
  std::string step_log_dir_;
  std::array<StepRec, kRing> ring_{};
  std::atomic<uint64_t> ring_w_{0}, ring_r_{0}, log_dropped_{0};
  std::atomic<bool> log_run_{true};
  std::atomic<int> log_session_{0};
  std::thread log_th_;
  uint64_t step_n_ = 0;
  int64_t prev_step_ns_ = 0;
  gen2_sensors::ImuShmReader shm_;
  int64_t shm_try_ns_ = 0;
  bool use_shm_ = true;
  double th_trim_ = 0.0;
  std::atomic<bool> imu_src_shm_{false};
  std::atomic<double> dt_max_ms_{0.0};
  std::atomic<uint32_t> overruns_{0};
  std::atomic<bool> run_{true};
  std::thread th_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hb_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr h_sub_;
  std::atomic<double> h_cmd_{-1.0};   // < 0: idle_h
  double h_set_ = -1.0, h_rate_ = 0.05, sit_h0_ = 0.0;
  rclcpp::Subscription<gen2_msgs::msg::MotorStateArray>::SharedPtr ms_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stand_, srv_bal_, srv_dis_, srv_sit_;
  double frz_p_[2] = {1e9, 1e9}, frz_i_[2] = {1e9, 1e9}, frz_lo_[2] = {0, 0}, frz_hi_[2] = {0, 0};
  int64_t frz_ns_[2] = {0, 0};
  double frozen_s_ = 0.15, frozen_cmd_nm_ = 0.15;
  bool notch_on_ = false;
  Notch notch_[2];
  bool start_ = false, homing_ = false, home_done_ = false;
  int64_t idle_ping_ns_ = 0;
  std::atomic<bool> in_request_{false};
  std::string err_ping_;
  double start_t_ = 0.0, home_still_ = 0.0, home_tau_ = 0.4, home_kd_ = 0.3, home_timeout_s_ = 3.0, auto_tilt_ = 0.087, auto_wait_s_ = 10.0;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  bool sit_ = false, sit_hold_ = false, link_lost_ = false, link_stop_ = true;
  int64_t t_lost_ = 0;
  double link_sit_after_s_ = 2.0, link_disarm_s_ = 30.0;
  double sit_t_ = 0.0, sit_s_ = 3.0, sit_from_[2] = {0, 0};
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
