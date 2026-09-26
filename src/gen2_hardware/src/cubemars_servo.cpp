#include "gen2_hardware/cubemars_servo.hpp"

#include <algorithm>
#include <cmath>

namespace gen2_hardware
{
namespace cubemars
{
namespace
{
int16_t be16(const uint8_t * p)
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
int32_t be32(const uint8_t * p)
{
  return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24) |
         (static_cast<uint32_t>(p[1]) << 16) | (static_cast<uint32_t>(p[2]) << 8) | p[3]);
}
void put32(uint8_t * p, int32_t v)
{
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u >> 24);
  p[1] = static_cast<uint8_t>(u >> 16);
  p[2] = static_cast<uint8_t>(u >> 8);
  p[3] = static_cast<uint8_t>(u);
}
int32_t clamp_round(double v, double lo, double hi)
{
  if (!std::isfinite(v)) {
    return 0;
  }
  return static_cast<int32_t>(std::lround(std::clamp(v, lo, hi)));
}
Frame int32_frame(Mode m, uint8_t id, int32_t v)
{
  Frame f;
  f.id = make_id(m, id);
  f.len = 4;
  put32(f.data, v);
  return f;
}
}  // namespace

std::optional<Status> decode_status(const Frame & f)
{
  if (id_function(f.id) != kUploadStatus || f.len < 8) {
    return std::nullopt;
  }
  Status s;
  s.position_deg = be16(f.data + 0) * 0.1;
  s.speed_erpm = be16(f.data + 2) * 10.0;
  s.current_a = be16(f.data + 4) * 0.01;
  s.temperature_c = static_cast<int8_t>(f.data[6]);
  s.error = f.data[7];
  return s;
}

std::optional<double> decode_position32_deg(const Frame & f)
{
  if (id_function(f.id) != kUploadPosition32 || f.len < 4) {
    return std::nullopt;
  }
  return be32(f.data) * 0.01;
}

bool is_boot_frame(const Frame & f)
{
  return id_function(f.id) == kUploadBoot && f.len >= 4 && f.data[0] == 0xFA &&
         f.data[1] == 0xFB && f.data[2] == 0xFC && f.data[3] == 0xFD;
}

const char * error_text(uint8_t code)
{
  switch (code) {
    case 0: return "ok";
    case 1: return "motor over-temperature";
    case 2: return "over-current";
    case 3: return "over-voltage";
    case 4: return "under-voltage";
    case 5: return "encoder fault";
    case 6: return "MOSFET over-temperature";
    case 7: return "motor stall / lock-up";
    case kDisableAck: return "disabled (ack)";
    default: return "unknown";
  }
}

Frame encode_current(uint8_t id, double amps)
{
  return int32_frame(Mode::kCurrent, id, clamp_round(amps * 1000.0, -60000.0, 60000.0));
}

Frame encode_current_brake(uint8_t id, double amps)
{
  return int32_frame(Mode::kCurrentBrake, id, clamp_round(amps * 1000.0, 0.0, 60000.0));
}

Frame encode_rpm(uint8_t id, double erpm)
{
  return int32_frame(Mode::kRpm, id, clamp_round(erpm, -100000.0, 100000.0));
}

Frame encode_position(uint8_t id, double deg)
{
  return int32_frame(Mode::kPosition, id, clamp_round(deg * 10000.0, -360000000.0, 360000000.0));
}

Frame encode_set_origin(uint8_t id, uint8_t mode)
{
  Frame f;
  f.id = make_id(Mode::kSetOrigin, id);
  f.len = 1;
  f.data[0] = mode;
  return f;
}

Frame encode_pos_spd(uint8_t id, double deg, double erpm, double erpm_per_s2)
{
  Frame f;
  f.id = make_id(Mode::kPosSpd, id);
  f.len = 8;
  put32(f.data, clamp_round(deg * 10000.0, -360000000.0, 360000000.0));
  const int32_t spd = clamp_round(std::fabs(erpm) / 10.0, 0.0, 32767.0);
  const int32_t acc = clamp_round(std::fabs(erpm_per_s2) / 10.0, 0.0, 32767.0);
  f.data[4] = static_cast<uint8_t>(spd >> 8);
  f.data[5] = static_cast<uint8_t>(spd);
  f.data[6] = static_cast<uint8_t>(acc >> 8);
  f.data[7] = static_cast<uint8_t>(acc);
  return f;
}

Frame encode_disable(uint8_t id)
{
  Frame f;
  f.id = make_id(Mode::kDisable, id);
  f.len = 0;
  return f;
}

}  // namespace cubemars
}  // namespace gen2_hardware
