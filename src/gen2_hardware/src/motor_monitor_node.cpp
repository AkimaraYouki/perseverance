// STEP 3: read-only motor state monitor. Never transmits on the bus.

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "gen2_hardware/motor_bus.hpp"
#include "gen2_hardware/motor_params.hpp"
#include "gen2_msgs/msg/motor_state_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gen2_hardware
{
using diagnostic_msgs::msg::DiagnosticStatus;

class MotorMonitorNode : public rclcpp::Node
{
public:
  MotorMonitorNode()
  : Node("motor_monitor"), updater_(this)
  {
    const std::string ifname = declare_parameter("can_interface", std::string("can0"));
    const double rate = declare_parameter("publish_rate_hz", 100.0);
    auto motors = declare_motor_params(*this);
    if (motors.empty()) {
      RCLCPP_WARN(get_logger(), "No motors configured (motors.names empty)");
    }
    for (const auto & m : motors) {
      RCLCPP_INFO(get_logger(), "motor %s: id %u model %s pp %.0f gear %.2f dir %d kt %s",
        m.name.c_str(), m.can_id, m.model.c_str(), m.pole_pairs, m.gear_ratio, m.direction,
        std::isfinite(m.kt_nm_per_a) ? std::to_string(m.kt_nm_per_a).c_str() : "UNKNOWN");
    }
    bus_ = std::make_unique<MotorBus>(ifname, motors);
    bus_->enable_tx(false);
    bus_->start();
    last_counts_.assign(motors.size(), 0);
    rates_.assign(motors.size(), 0.0);

    pub_ = create_publisher<gen2_msgs::msg::MotorStateArray>("motors/state", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {publish();});

    updater_.setHardwareID(ifname);
    updater_.add("can: bus", this, &MotorMonitorNode::diag_bus);
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
      updater_.add("motor: " + bus_->motors()[i].name, [this, i](auto & s) {diag_motor(s, i);});
    }
    updater_.setPeriod(1.0);
  }

  ~MotorMonitorNode() override {bus_->stop();}

private:
  void publish()
  {
    const int64_t now = mono_now_ns();
    gen2_msgs::msg::MotorStateArray arr;
    arr.header.stamp = now_ros();
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
      const auto & c = bus_->motors()[i];
      const MotorFeedback fb = bus_->feedback(i);
      gen2_msgs::msg::MotorState m;
      m.name = c.name;
      m.can_id = c.can_id;
      m.rx_count = fb.rx_count;
      if (fb.valid) {
        m.header.stamp = fb.realtime_ns > 0 ?
          static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(fb.realtime_ns)) : arr.header.stamp;
        m.age_s = (now - fb.mono_ns) * 1e-9;
        m.stale = m.age_s > c.feedback_stale_timeout_s;
        m.raw_position_deg = fb.status.position_deg;
        m.raw_speed_erpm = fb.status.speed_erpm;
        m.position_rad = c.raw_to_joint_pos(fb.status.position_deg);
        m.velocity_rad_s = c.erpm_to_joint_vel(fb.status.speed_erpm);
        m.current_a = fb.status.current_a;
        m.torque_nm = c.current_to_joint_torque(fb.status.current_a);
        m.temperature_c = fb.status.temperature_c;
        m.error_code = fb.status.error;
        m.error_text = cubemars::error_text(fb.status.error);
      } else {
        m.header.stamp = arr.header.stamp;
        m.stale = true;
        m.age_s = std::numeric_limits<double>::infinity();
        m.position_rad = m.velocity_rad_s = m.current_a = m.torque_nm =
          std::numeric_limits<double>::quiet_NaN();
        m.error_text = "no feedback yet";
      }
      arr.motors.push_back(m);
    }
    pub_->publish(arr);
  }

  rclcpp::Time now_ros() {return now();}

  void diag_bus(diagnostic_updater::DiagnosticStatusWrapper & s)
  {
    const auto & st = bus_->stats();
    const int64_t now = mono_now_ns();
    const uint64_t rx = st.rx_frames.load();
    const double dt = last_bus_ns_ ? (now - last_bus_ns_) * 1e-9 : 0.0;
    const double rx_rate = dt > 0 ? (rx - last_bus_rx_) / dt : 0.0;
    const uint64_t err = st.rx_error_frames.load();
    const uint64_t new_err = err - last_err_;
    last_bus_ns_ = now;
    last_bus_rx_ = rx;
    last_err_ = err;
    const std::string oper = can_operstate(bus_->ifname());
    if (!st.socket_ok || oper != "up") {
      s.summary(DiagnosticStatus::ERROR, "CAN interface " + bus_->ifname() + " " + oper);
    } else if (new_err > 0) {
      s.summary(DiagnosticStatus::WARN, "CAN error frames");
    } else {
      s.summary(DiagnosticStatus::OK, "ok (read-only)");
    }
    s.add("operstate", oper);
    s.add("rx_rate_hz", rx_rate);
    s.add("rx_frames", rx);
    s.add("rx_unknown", st.rx_unknown.load());
    s.add("rx_error_frames", err);
    s.add("last_error_class_hex", std::to_string(st.last_error_class.load()));
    s.add("boot_frames_0x2C", st.boot_frames_any.load());
    s.add("tx_frames", st.tx_frames.load());
    s.add("tx_enabled", bus_->tx_enabled());
    s.add("socket_reopens", st.reopen_count.load());
  }

  void diag_motor(diagnostic_updater::DiagnosticStatusWrapper & s, std::size_t i)
  {
    const auto & c = bus_->motors()[i];
    const MotorFeedback fb = bus_->feedback(i);
    const int64_t now = mono_now_ns();
    const double rate = (fb.rx_count - last_counts_[i]) / std::max(1e-3, (now - last_motor_ns_) * 1e-9);
    if (i + 1 == bus_->motors().size()) {last_motor_ns_ = now;}
    last_counts_[i] = fb.rx_count;
    const double age = fb.valid ? (now - fb.mono_ns) * 1e-9 : std::numeric_limits<double>::infinity();
    if (!fb.valid) {
      s.summary(DiagnosticStatus::ERROR, "no feedback (powered? id? servo upload enabled?)");
    } else if (age > c.feedback_stale_timeout_s) {
      s.summary(DiagnosticStatus::ERROR, "feedback stale");
    } else if (fb.status.error != 0) {
      s.summary(DiagnosticStatus::ERROR, std::string("drive fault: ") +
        cubemars::error_text(fb.status.error));
    } else if (fb.status.temperature_c > c.max_temperature_c) {
      s.summary(DiagnosticStatus::WARN, "driver temperature high");
    } else {
      s.summary(DiagnosticStatus::OK, "ok");
    }
    s.add("can_id", static_cast<int>(c.can_id));
    s.add("model", c.model);
    s.add("feedback_rate_hz", rate);
    s.add("age_s", age);
    s.add("rx_count", fb.rx_count);
    s.add("drive_boot_frames", fb.boot_frames);
    s.add("position_deg", fb.valid ? c.raw_to_joint_pos(fb.status.position_deg) * 180.0 / M_PI : NAN);
    s.add("raw_position_deg", fb.status.position_deg);
    s.add("raw_speed_erpm", fb.status.speed_erpm);
    s.add("current_a", fb.status.current_a);
    s.add("temperature_c", static_cast<int>(fb.status.temperature_c));
    s.add("error_code", static_cast<int>(fb.status.error));
    s.add("kt_known", std::isfinite(c.kt_nm_per_a));
    s.add("verified", std::string(c.verified_direction ? "dir " : "") +
      (c.verified_position_scale ? "pos " : "") + (c.verified_velocity_scale ? "vel " : "") +
      (c.verified_kt ? "kt" : ""));
  }

  std::unique_ptr<MotorBus> bus_;
  rclcpp::Publisher<gen2_msgs::msg::MotorStateArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
  std::vector<uint64_t> last_counts_;
  std::vector<double> rates_;
  int64_t last_motor_ns_ = mono_now_ns();
  int64_t last_bus_ns_ = 0;
  uint64_t last_bus_rx_ = 0, last_err_ = 0;
};

}  // namespace gen2_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<gen2_hardware::MotorMonitorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_monitor"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
