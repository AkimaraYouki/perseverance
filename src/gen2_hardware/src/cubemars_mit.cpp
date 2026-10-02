#include "gen2_hardware/cubemars_mit.hpp"

#include <algorithm>
#include <cmath>

namespace gen2_hardware
{
namespace mit
{

uint32_t to_uint(double x, double lo, double hi, int bits)
{
  const double maxi = static_cast<double>((1u << bits) - 1u);
  if (!std::isfinite(x)) {x = 0.0;}
  x = std::clamp(x, lo, hi);
  return static_cast<uint32_t>(std::lround((x - lo) / (hi - lo) * maxi));
}

double to_double(uint32_t x, double lo, double hi, int bits)
{
  return static_cast<double>(x) * (hi - lo) / static_cast<double>((1u << bits) - 1u) + lo;
}

cubemars::Frame encode(Proto proto, uint8_t id, const Ranges & r, const Command & c)
{
  const uint32_t p = to_uint(c.p, -r.p_max, r.p_max, 16);
  const uint32_t v = to_uint(c.v, -r.v_max, r.v_max, 12);
  const uint32_t kp = to_uint(c.kp, 0.0, r.kp_max, 12);
  const uint32_t kd = to_uint(c.kd, 0.0, r.kd_max, 12);
  const uint32_t t = to_uint(c.t, -r.t_max, r.t_max, 12);
  cubemars::Frame f;
  f.len = 8;
  if (proto == Proto::kV3) {
    f.id = (8u << 8) | id;
    f.data[0] = kp >> 4;
    f.data[1] = ((kp & 0xF) << 4) | (kd >> 8);
    f.data[2] = kd & 0xFF;
    f.data[3] = p >> 8;
    f.data[4] = p & 0xFF;
    f.data[5] = v >> 4;
    f.data[6] = ((v & 0xF) << 4) | (t >> 8);
    f.data[7] = t & 0xFF;
  } else {
    f.standard = true;
    f.id = id;
    f.data[0] = p >> 8;
    f.data[1] = p & 0xFF;
    f.data[2] = v >> 4;
    f.data[3] = ((v & 0xF) << 4) | (kp >> 8);
    f.data[4] = kp & 0xFF;
    f.data[5] = kd >> 4;
    f.data[6] = ((kd & 0xF) << 4) | (t >> 8);
    f.data[7] = t & 0xFF;
  }
  return f;
}

static cubemars::Frame special(uint8_t id, uint8_t last)
{
  cubemars::Frame f;
  f.standard = true;
  f.id = id;
  f.len = 8;
  for (int i = 0; i < 7; ++i) {f.data[i] = 0xFF;}
  f.data[7] = last;
  return f;
}
cubemars::Frame enter(uint8_t id) {return special(id, 0xFC);}
cubemars::Frame exit(uint8_t id) {return special(id, 0xFD);}
cubemars::Frame set_zero(uint8_t id) {return special(id, 0xFE);}

std::optional<Reply> decode_legacy_reply(const cubemars::Frame & f, const Ranges & r)
{
  if (!f.standard || f.len < 6) {return std::nullopt;}
  // a command echo / special frame from another node is not a reply
  if (f.len == 8 && f.data[0] == 0xFF && f.data[1] == 0xFF && f.data[6] == 0xFF) {return std::nullopt;}
  Reply y;
  y.id = f.data[0];
  y.p = to_double((f.data[1] << 8) | f.data[2], -r.p_max, r.p_max, 16);
  y.v = to_double((f.data[3] << 4) | (f.data[4] >> 4), -r.v_max, r.v_max, 12);
  y.t = to_double(((f.data[4] & 0xF) << 8) | f.data[5], -r.t_max, r.t_max, 12);
  if (f.len >= 7) {y.temperature_c = static_cast<int>(f.data[6]) - 40;}
  if (f.len >= 8) {y.error = f.data[7];}
  return y;
}

}  // namespace mit
}  // namespace gen2_hardware
