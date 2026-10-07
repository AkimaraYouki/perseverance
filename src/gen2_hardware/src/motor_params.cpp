#include "gen2_hardware/motor_params.hpp"

#include <chrono>
#include <limits>
#include <set>

#include "gen2_hardware/can_socket.hpp"
#include <stdexcept>

namespace gen2_hardware
{

std::vector<MotorConfig> declare_motor_params(rclcpp::Node & node)
{
  const auto names = node.declare_parameter("motors.names", std::vector<std::string>{});
  std::vector<MotorConfig> out;
  for (const auto & n : names) {
    const std::string p = "motors." + n + ".";
    MotorConfig c;
    c.name = n;
    c.model = node.declare_parameter(p + "model", std::string(""));
    c.can_id = static_cast<uint8_t>(node.declare_parameter(p + "can_id", 0));
    c.pole_pairs = node.declare_parameter(p + "pole_pairs", 0.0);
    c.gear_ratio = node.declare_parameter(p + "gear_ratio", 1.0);
    c.raw_deg_per_output_rev = node.declare_parameter(p + "raw_deg_per_output_rev", 360.0);
    c.direction = static_cast<int>(node.declare_parameter(p + "direction", 1));
    c.position_offset_rad = node.declare_parameter(p + "position_offset_rad", 0.0);
    const double kt = node.declare_parameter(p + "kt_nm_per_a", -1.0);
    c.kt_nm_per_a = kt > 0.0 ? kt : std::numeric_limits<double>::quiet_NaN();
    c.current_limit_a = node.declare_parameter(p + "current_limit_a", 0.0);
    c.velocity_limit_rad_s = node.declare_parameter(p + "velocity_limit_rad_s", 0.0);
    c.joint_min_rad = node.declare_parameter(p + "joint_min_rad", -1e9);
    c.joint_max_rad = node.declare_parameter(p + "joint_max_rad", 1e9);
    c.max_temperature_c = node.declare_parameter(p + "max_temperature_c", 80.0);
    c.feedback_stale_timeout_s = node.declare_parameter(p + "feedback_stale_timeout_s", 0.1);
    c.command_timeout_s = node.declare_parameter(p + "command_timeout_s", 0.05);
    c.supports_disable_cmd = node.declare_parameter(p + "supports_disable_cmd", false);
    c.wrap_deg = node.declare_parameter(p + "wrap_deg", 0.0);
    c.stop_min_deg = node.declare_parameter(p + "stop_min_deg", -1e9);
    c.stop_max_deg = node.declare_parameter(p + "stop_max_deg", 1e9);
    c.stop_margin_deg = node.declare_parameter(p + "stop_margin_deg", 2.0);
    if (c.wrap_deg > 0.0 && !(c.stop_max_deg - c.stop_min_deg < 2.0 * c.wrap_deg)) {
      throw std::runtime_error("motor '" + n + "': wrap_deg needs measured stop_min_deg/stop_max_deg");
    }
    c.protocol = node.declare_parameter(p + "protocol", std::string("servo"));
    if (c.protocol != "servo" && c.protocol != "mit_v3" && c.protocol != "mit_legacy") {
      throw std::runtime_error("motor '" + n + "': protocol must be servo|mit_v3|mit_legacy");
    }
    c.mit_ranges.p_max = node.declare_parameter(p + "mit.p_max", 12.5);
    c.mit_ranges.v_max = node.declare_parameter(p + "mit.v_max", 50.0);
    c.mit_ranges.t_max = node.declare_parameter(p + "mit.t_max", 18.0);
    c.mit_ranges.kp_max = node.declare_parameter(p + "mit.kp_max", 500.0);
    c.mit_ranges.kd_max = node.declare_parameter(p + "mit.kd_max", 5.0);
    c.mit_test_kp = node.declare_parameter(p + "mit.test_kp", 20.0);
    c.mit_test_kd = node.declare_parameter(p + "mit.test_kd", 0.5);
    c.verified_direction = node.declare_parameter(p + "verified.direction", false);
    c.verified_position_scale = node.declare_parameter(p + "verified.position_scale", false);
    c.verified_velocity_scale = node.declare_parameter(p + "verified.velocity_scale", false);
    c.verified_kt = node.declare_parameter(p + "verified.kt", false);
    const bool is_auto = c.can_id == 0;
    if (is_auto) {c.can_id = 255;}  // placeholder for validation; assigned by the scan below
    const std::string err = validate(c);
    if (is_auto) {c.can_id = 0;}
    if (!err.empty()) {
      throw std::runtime_error("motor '" + n + "': " + err);
    }
    out.push_back(c);
  }
  // can_id 0 = auto: listen to the bus for CubeMars status uploads (0x29xx) and give the IDs not
  // used by fixed entries to the auto motors, ascending ID -> names order. Listens up to
  // can_scan_s (stops early once enough are found). Auto motors left without a drive are dropped
  // (logged); drives found later still appear read-only as id_<N>? in the monitor.
  std::size_t n_auto = 0;
  std::set<uint8_t> fixed;
  for (const auto & c : out) {if (c.can_id == 0) {++n_auto;} else {fixed.insert(c.can_id);}}
  if (n_auto == 0) {return out;}
  const std::string ifname = node.has_parameter("can_interface") ?
    node.get_parameter("can_interface").as_string() : std::string("can0");
  const double scan_s = node.declare_parameter("can_scan_s", 3.0);
  std::set<uint8_t> found;
  CanSocket sock;
  std::string err;
  if (sock.open(ifname, err)) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(scan_s);
    RxFrame f;
    // listen at least 0.5 s (all drives upload at >= 50 Hz) so the ascending-ID order does not
    // depend on which drive happened to talk first; then stop early once enough are found
    const auto min_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < end &&
      (found.size() < n_auto || std::chrono::steady_clock::now() < min_end))
    {
      if (sock.read(f, 50) == 1 && f.extended && !f.error_frame && (f.frame.id >> 8) == 0x29) {
        const uint8_t id = f.frame.id & 0xFF;
        if (id != 0 && !fixed.count(id)) {found.insert(id);}
      }
    }
  } else {
    RCLCPP_ERROR(node.get_logger(), "CAN ID scan: %s", err.c_str());
  }
  std::vector<MotorConfig> res;
  auto it = found.begin();
  for (auto & c : out) {
    if (c.can_id != 0) {res.push_back(c); continue;}
    if (it == found.end()) {
      RCLCPP_WARN(node.get_logger(), "CAN ID scan: no drive for auto motor '%s' (%.1f s on %s)",
        c.name.c_str(), scan_s, ifname.c_str());
      continue;
    }
    c.can_id = *it++;
    RCLCPP_INFO(node.get_logger(), "CAN ID scan: '%s' -> id %u", c.name.c_str(), c.can_id);
    res.push_back(c);
  }
  return res;
}

}  // namespace gen2_hardware
