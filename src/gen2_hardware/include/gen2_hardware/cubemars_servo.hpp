// CubeMars AK-series SERVO-mode CAN codec.
//
// Source of truth (copies in gen2_ws/docs/vendor/):
//   - AK Series Module Driver User Manual V1.0.15.X  (AK45-10, servo firmware 20250313)
//   - AK Series Product Manual V3.2.0 for AK 3.0     (AK60-6 V3.0)
// Extended 29-bit ID = (mode << 8) | driver_id. All multi-byte fields are big-endian.
// Verified on hardware 2026-09-24: AK45-10 id 69 uploads 0x2945 @ 50 Hz, 1 Mbit/s,
// matching "AK45-10 KV80.AppParams" (controller_id 69, send_can_status_rate_hz 50, can_baud_rate 3).
#pragma once

#include <cstdint>
#include <optional>

namespace gen2_hardware
{
namespace cubemars
{

// Host -> drive control modes (manual 5.1 / 4.1)
enum class Mode : uint8_t
{
  kDuty = 0,
  kCurrent = 1,        // Iq current loop, int32 mA, +-60 A
  kCurrentBrake = 2,   // int32 mA, 0..60 A
  kRpm = 3,            // int32 ERPM
  kPosition = 4,       // int32 deg*10000
  kSetOrigin = 5,      // uint8: 0 temporary, 1 permanent (dual-encoder models only)
  kPosSpd = 6,         // int32 deg*10000, int16 ERPM/10, int16 ERPM/s^2 /10
  kDisable = 15,       // AK 3.0 only: no payload, drive replies 0x29 with DATA[7]=0x77
  kFeedbackConfig = 16 // AK 3.0 only: writes FLASH, never send periodically
};

// Drive -> host function IDs (upload)
constexpr uint8_t kUploadStatus = 0x29;      // periodic status
constexpr uint8_t kUploadPosition32 = 0x2A;  // optional int32 position (AK 3.0)
constexpr uint8_t kUploadBoot = 0x2C;        // "entered servo mode", data FA FB FC FD
constexpr uint8_t kDisableAck = 0x77;        // DATA[7] of 0x29 after a disable command

struct Frame
{
  uint32_t id = 0;   // 29-bit extended id
  uint8_t len = 0;
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

inline uint32_t make_id(Mode m, uint8_t driver_id)
{
  return (static_cast<uint32_t>(m) << 8) | driver_id;
}
inline uint8_t id_driver(uint32_t id) {return static_cast<uint8_t>(id & 0xFFu);}
inline uint32_t id_function(uint32_t id) {return (id >> 8) & 0x1FFFFFu;}

// Status upload (0x29), manual 5.2.1 / 4.3.1
struct Status
{
  double position_deg = 0.0;   // int16 * 0.1
  double speed_erpm = 0.0;     // int16 * 10
  double current_a = 0.0;      // int16 * 0.01
  int8_t temperature_c = 0;    // driver board temperature
  uint8_t error = 0;           // 0 ok, 1..7 faults, 0x77 disable ack (AK 3.0)
};

std::optional<Status> decode_status(const Frame & f);
std::optional<double> decode_position32_deg(const Frame & f);  // 0x2A: int32 * 0.01 deg
bool is_boot_frame(const Frame & f);
const char * error_text(uint8_t code);

// Encoders (manual example functions comm_can_set_*). Values are clamped to manual ranges.
Frame encode_current(uint8_t driver_id, double amps);
Frame encode_current_brake(uint8_t driver_id, double amps);
Frame encode_rpm(uint8_t driver_id, double erpm);
Frame encode_position(uint8_t driver_id, double deg);
Frame encode_set_origin(uint8_t driver_id, uint8_t mode);
// Position-speed loop (mode 6, manual 5.1.7): position deg (int32 x10000), speed limit ERPM
// (int16 = ERPM/10), acceleration ERPM/s^2 (int16 = value/10, >= 0). The drive moves to the
// position with that speed and acceleration and holds it.
Frame encode_pos_spd(uint8_t driver_id, double deg, double erpm, double erpm_per_s2);
Frame encode_disable(uint8_t driver_id);

}  // namespace cubemars
}  // namespace gen2_hardware
