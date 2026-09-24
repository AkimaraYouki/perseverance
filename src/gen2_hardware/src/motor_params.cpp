#include "gen2_hardware/motor_params.hpp"

#include <limits>
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
    c.verified_direction = node.declare_parameter(p + "verified.direction", false);
    c.verified_position_scale = node.declare_parameter(p + "verified.position_scale", false);
    c.verified_velocity_scale = node.declare_parameter(p + "verified.velocity_scale", false);
    c.verified_kt = node.declare_parameter(p + "verified.kt", false);
    const std::string err = validate(c);
    if (!err.empty()) {
      throw std::runtime_error("motor '" + n + "': " + err);
    }
    out.push_back(c);
  }
  return out;
}

}  // namespace gen2_hardware
