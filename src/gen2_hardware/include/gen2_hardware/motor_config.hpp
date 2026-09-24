// Per-actuator configuration. Everything hardware-specific lives in YAML, not in code.
#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gen2_hardware/cubemars_servo.hpp"

namespace gen2_hardware
{

struct MotorConfig
{
  std::string name;
  std::string model;
  uint8_t can_id = 0;

  // Drive -> joint conversion
  double pole_pairs = 0.0;              // ERPM -> rotor RPM
  double gear_ratio = 1.0;              // rotor turns per output turn
  double raw_deg_per_output_rev = 360.0;  // what the drive reports for one output revolution
  int direction = 1;                    // +1 / -1: joint sign convention
  double position_offset_rad = 0.0;     // joint = direction*raw_rad - offset
  double kt_nm_per_a = std::numeric_limits<double>::quiet_NaN();  // output-side; NaN = unknown

  // Limits (enforced by the command path, checked by the safety monitor)
  double current_limit_a = 0.0;
  double velocity_limit_rad_s = 0.0;
  double joint_min_rad = -std::numeric_limits<double>::infinity();
  double joint_max_rad = std::numeric_limits<double>::infinity();
  double max_temperature_c = 80.0;

  // Timing
  double feedback_stale_timeout_s = 0.1;
  double command_timeout_s = 0.05;

  bool supports_disable_cmd = false;    // AK 3.0 mode 15

  // Verification gates: commanding (ARM) is refused until the relevant items are true.
  bool verified_direction = false;
  bool verified_position_scale = false;
  bool verified_velocity_scale = false;
  bool verified_kt = false;

  double raw_to_joint_pos(double raw_deg) const
  {
    const double out_rad = raw_deg / raw_deg_per_output_rev * 2.0 * M_PI;
    return direction * out_rad - position_offset_rad;
  }
  double erpm_to_joint_vel(double erpm) const
  {
    if (!(pole_pairs > 0.0) || !(gear_ratio > 0.0)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return direction * erpm / pole_pairs / gear_ratio * 2.0 * M_PI / 60.0;
  }
  double current_to_joint_torque(double amps) const
  {
    return direction * amps * kt_nm_per_a;  // NaN when kt unknown
  }
  // joint-side torque request -> drive current, with limit. NaN kt -> 0.
  double joint_torque_to_current(double torque_nm) const
  {
    if (!std::isfinite(kt_nm_per_a) || kt_nm_per_a <= 0.0 || !std::isfinite(torque_nm)) {
      return 0.0;
    }
    const double a = direction * torque_nm / kt_nm_per_a;
    return std::max(-current_limit_a, std::min(current_limit_a, a));
  }
};

std::string validate(const MotorConfig & c);  // empty if structurally valid

}  // namespace gen2_hardware
