// HostFrame v1 — must match biped_sensor_hub.ino (ESP32-C3) byte for byte.
#pragma once

#include <cstddef>
#include <cstdint>

namespace gen2_sensor_hub
{

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "HostFrame is little-endian on the wire");

constexpr uint32_t kFrameMagic = 0x31524853UL;  // "SHR1"
constexpr uint16_t kFrameVersion = 1;

enum FrameFlags : uint32_t
{
  FLAG_PM1_VALID = 1u << 0,
  FLAG_PM2_VALID = 1u << 1,
  FLAG_GPS_PACKET_SEEN = 1u << 2,
  FLAG_GPS_FIX_OK = 1u << 3,
  FLAG_GPS_3D = 1u << 4,
  FLAG_GPS_HACC_OK = 1u << 5,
  FLAG_GPS_USABLE = 1u << 6,
  FLAG_GPS_TIME_VALID = 1u << 7,
  FLAG_MAG_VALID = 1u << 8,
};

struct __attribute__((packed)) HostFrame
{
  uint32_t magic;
  uint16_t version;
  uint16_t frame_size;

  uint32_t sequence;
  uint64_t esp_time_us;
  uint32_t flags;

  uint32_t usb_drop_count;
  uint32_t gps_checksum_error_count;

  uint16_t compute_mV;
  int32_t compute_mA;
  uint16_t motor_mV;
  int32_t motor_mA;

  uint32_t gps_iTOW_ms;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t gps_valid_flags;
  uint32_t gps_tAcc_ns;
  int32_t gps_nano_ns;

  uint8_t gps_fix_type;
  uint8_t gps_fix_flags;
  uint8_t gps_num_sv;
  uint8_t reserved0;

  int32_t lon_e7;
  int32_t lat_e7;
  int32_t height_mm;
  int32_t hMSL_mm;

  uint32_t hAcc_mm;
  uint32_t vAcc_mm;

  int32_t velN_mms;
  int32_t velE_mms;
  int32_t velD_mms;
  int32_t gSpeed_mms;
  int32_t headMot_e5;

  uint32_t sAcc_mms;
  uint32_t headAcc_e5;
  uint16_t pDOP_centi;
  uint16_t gps_rx_bytes_lo16;  // raw GPS UART byte counter (wraps); always 0 on original firmware

  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  uint16_t reserved2;

  uint32_t crc32;
};

static_assert(sizeof(HostFrame) == 136, "HostFrame layout changed; update firmware and parser together");
static_assert(offsetof(HostFrame, crc32) == 132, "CRC must be the last 4 bytes");

// CRC-32/IEEE (reflected 0xEDB88320, init 0xFFFFFFFF, final xor) — same as firmware crc32_ieee().
uint32_t crc32_ieee(const uint8_t * data, std::size_t len);

}  // namespace gen2_sensor_hub
