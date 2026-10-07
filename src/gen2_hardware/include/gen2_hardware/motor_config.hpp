// Per-actuator configuration. Everything hardware-specific lives in YAML, not in code.
#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gen2_hardware/cubemars_mit.hpp"
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

  // Power-up position ambiguity (geared drive, encoder on the rotor only): after a drive power
  // cycle the reported output position is only known modulo wrap_deg (AK60-6: 360/6 = 60 deg).
  // It is resolved from the measured mechanical range [stop_min_deg, stop_max_deg] (joint deg):
  // the unique candidate raw + k*wrap_deg inside the range (+-stop_margin_deg) is taken.
  double wrap_deg = 0.0;                // 0 = absolute (no ambiguity)
  double stop_min_deg = -1e9;           // joint deg, measured end stops
  double stop_max_deg = 1e9;
  double stop_margin_deg = 2.0;

  // Command protocol: "servo" (29-bit modes), "mit_v3" (AK 3.0 MIT, feedback = 0x29 upload),
  // "mit_legacy" (std frames, reply only to commands). MIT ranges must match the drive firmware.
  std::string protocol = "servo";
  mit::Ranges mit_ranges;
  double mit_test_kp = 20.0;    // position test stiffness (N·m/rad)
  double mit_test_kd = 0.5;     // position / velocity test damping (N·m·s/rad)
  bool is_mit() const {return protocol != "servo";}
  mit::Proto mit_proto() const {return protocol == "mit_v3" ? mit::Proto::kV3 : mit::Proto::kLegacy;}

  // Verification gates: commanding (ARM) is refused until the relevant items are true.
  bool verified_direction = false;
  bool verified_position_scale = false;
  bool verified_velocity_scale = false;
  bool verified_kt = false;

  // Candidates raw + k*wrap_deg whose joint angle lies inside the stop range. Returns how many
  // (0, 1 = resolved, 2 = ambiguous) and the shift [raw deg] of the one with the largest joint angle.
  int wrap_candidates(double raw_deg, double & shift_deg) const
  {
    int n = 0;
    double best = -1e18;
    shift_deg = 0.0;
    for (int k = -12; k <= 12; ++k) {
      const double sh = k * wrap_deg;
      const double j = raw_to_joint_pos(raw_deg + sh) * 180.0 / M_PI;
      if (j >= stop_min_deg - stop_margin_deg && j <= stop_max_deg + stop_margin_deg) {
        ++n;
        if (j > best) {best = j; shift_deg = sh;}
      }
    }
    return n;
  }

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
