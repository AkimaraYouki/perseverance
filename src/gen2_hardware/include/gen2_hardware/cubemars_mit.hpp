// CubeMars MIT (force control) codec.
//  kV3     : AK 3.0 drives (AK60-6 V3.0), manual V3.2.0 §4.2. Extended id (8<<8)|id,
//            data = KP12 KD12 P16 V12 T12. Feedback stays the 0x29 servo upload (periodic).
//  kLegacy : older MIT firmware (manual V1.0.15 §5.3). Standard id = drive id,
//            data = P16 V12 KP12 KD12 T12; special frames FF..FC enter / FD exit / FE zero.
//            The drive replies only to a command: std frame, data[0] = drive id, P16 V12 T12,
//            data[6] temperature (+40 offset per example), data[7] error.
// Ranges are per model (YAML). The vendor examples clamp p/v to 0 and float_to_uint overflows at
// the maximum — not copied: values are clamped and rounded to [0, 2^bits-1].
#pragma once

#include <cstdint>
#include <optional>

#include "gen2_hardware/cubemars_servo.hpp"

namespace gen2_hardware
{
namespace mit
{

enum class Proto {kV3, kLegacy};

struct Ranges
{
  double p_max = 12.5;   // rad (symmetric)
  double v_max = 50.0;   // rad/s
  double t_max = 18.0;   // N·m
  double kp_max = 500.0;
  double kd_max = 5.0;
};

struct Command {double p = 0, v = 0, kp = 0, kd = 0, t = 0;};

struct Reply
{
  uint8_t id = 0;
  double p = 0, v = 0, t = 0;  // output rad, rad/s, N·m
  int temperature_c = 0;
  uint8_t error = 0;
};

uint32_t to_uint(double x, double lo, double hi, int bits);
double to_double(uint32_t x, double lo, double hi, int bits);

cubemars::Frame encode(Proto proto, uint8_t id, const Ranges & r, const Command & c);
cubemars::Frame enter(uint8_t id);   // legacy only
cubemars::Frame exit(uint8_t id);    // legacy only
cubemars::Frame set_zero(uint8_t id);  // legacy only
std::optional<Reply> decode_legacy_reply(const cubemars::Frame & f, const Ranges & r);

}  // namespace mit
}  // namespace gen2_hardware
