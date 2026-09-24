/*
 * biped_sensor_hub.ino
 *
 * ESP32-C3 Super Mini sensor hub for a ROS 2 wheeled-biped robot.
 *
 * HARDWARE
 * ---------------------------------------------------------------------------
 * PM02 V3 #1 (compute battery)
 *   VOLTAGE -> GPIO0
 *   CURRENT -> GPIO1
 *
 * PM02 V3 #2 (motor battery)
 *   VOLTAGE -> GPIO3
 *   CURRENT -> GPIO4
 *
 * Holybro M8N
 *   SDA -> GPIO6
 *   SCL -> GPIO7
 *   TX  -> GPIO20 (ESP32 RX)
 *   RX  -> GPIO21 (ESP32 TX)
 *   VCC -> 5V
 *   GND -> GND
 *
 * Holybro safety LED
 *   SAFETY_SWITCH_LED -> GPIO5
 *   VDD_3V3           -> ESP32 3V3
 *
 * Holybro RGB LED controller
 *   I2C address 0x38 on same SDA/SCL bus
 *
 * ESP32-C3 onboard status LED
 *   LED_BUILTIN -> GPIO8
 *
 * Jetson Orin Nano
 *   ESP32 USB-C -> Jetson USB
 *
 * IMPORTANT ARDUINO IDE SETTING
 * ---------------------------------------------------------------------------
 * Board: ESP32C3 Dev Module
 * USB CDC On Boot: Enabled
 *
 * With USB CDC enabled:
 *   Serial  = USB CDC link to Jetson
 *   Serial1 = GPS UART on GPIO20/21
 *
 * DESIGN
 * ---------------------------------------------------------------------------
 * - GNSS is parsed as binary UBX-NAV-PVT, NOT NMEA.
 * - NEO-M8N is configured for 5 Hz navigation by default.
 * - Source-side sensor packet is emitted at 100 Hz.
 * - Every packet includes:
 *      ESP32 monotonic timestamp (microseconds)
 *      GNSS iTOW
 *      fix-valid flags
 *      hAcc/vAcc/sAcc/headAcc
 *      position, velocity and heading-of-motion
 *      PM02 power measurements
 *      IST8310 raw magnetometer
 * - Fixed-size binary frames + CRC32.
 * - No String, no heap allocation, no printf in the real-time path.
 * - USB frames are dropped rather than blocking if the TX buffer is full.
 * - RGB/IST8310 are hot-recovered if they were not ready during cold boot.
 *
 * NOTE ON "REAL TIME"
 * ---------------------------------------------------------------------------
 * The ESP32 acquisition schedule is deterministic-ish, but USB + Linux is NOT
 * hard real-time. The Jetson must use timestamps and sequence numbers.
 * Balance / torque control must remain on direct IMU + CAN feedback, not this
 * sensor-hub USB stream.
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>

// ============================================================================
// Pin assignment
// ============================================================================
static constexpr uint8_t PIN_PM1_V = 0;
static constexpr uint8_t PIN_PM1_I = 1;
static constexpr uint8_t PIN_PM2_V = 3;
static constexpr uint8_t PIN_PM2_I = 4;

static constexpr uint8_t PIN_GPS_LED = 5;
static constexpr uint8_t PIN_STATUS_LED = 8; // ESP32-C3 Super Mini onboard LED
static constexpr uint8_t PIN_I2C_SDA = 6;
static constexpr uint8_t PIN_I2C_SCL = 7;

static constexpr uint8_t PIN_GPS_RX = 20;  // Holybro TX -> ESP RX
static constexpr uint8_t PIN_GPS_TX = 21;  // Holybro RX <- ESP TX

// Most ESP32-C3 Super Mini boards use an active-LOW onboard LED on GPIO8.
// If yours behaves reversed, change this to false.
static constexpr bool STATUS_LED_ACTIVE_LOW = true;

// ============================================================================
// Rates
// ============================================================================
static constexpr uint32_t HOST_RATE_HZ = 100;
static constexpr uint32_t HOST_PERIOD_US = 1000000UL / HOST_RATE_HZ;

static constexpr uint32_t POWER_RATE_HZ = 100;
static constexpr uint32_t POWER_PERIOD_US = 1000000UL / POWER_RATE_HZ;

static constexpr uint32_t MAG_RATE_HZ = 50;
static constexpr uint32_t MAG_PERIOD_US = 1000000UL / MAG_RATE_HZ;

// Holybro M8N commonly ships at 38400.
// We deliberately leave the UART baud unchanged and only configure UBX output.
static constexpr uint32_t GPS_UART_BAUD = 38400;

// NEO-M8N default concurrent GPS+GLONASS mode is conservatively run at 5 Hz.
static constexpr uint16_t GPS_MEAS_PERIOD_MS = 200;

// ============================================================================
// PM02 V3 scaling
// ============================================================================
static constexpr float PM02_VOLTAGE_DIVIDER = 18.182f;
static constexpr float PM02_AMPS_PER_VOLT   = 36.364f;

// Optional external input-divider ratio.
//
// Example: if a PM02 output is divided to 75% before it reaches the ESP ADC,
// set INPUT_RATIO to 0.75.
//
// For your present 3S / 6S system, voltage sense is well below the ADC upper
// region. Current sense may need a divider if you want to capture very high
// motor currents without clipping.
static constexpr float PM1_V_INPUT_RATIO = 1.0f;
static constexpr float PM1_I_INPUT_RATIO = 1.0f;
static constexpr float PM2_V_INPUT_RATIO = 1.0f;
static constexpr float PM2_I_INPUT_RATIO = 1.0f;

// Measured PM02 current-output voltage when real current == 0.
// Calibrate these later with the motor / Jetson disconnected from load.
static constexpr float PM1_CURRENT_ZERO_V = 0.0f;
static constexpr float PM2_CURRENT_ZERO_V = 0.0f;

// GPS usability gate. Do NOT pretend a fix is precise if hAcc is poor.
// 3000 mm = 3 m.
static constexpr uint32_t GPS_MAX_HACC_MM = 3000;

// ============================================================================
// I2C / IST8310
// ============================================================================
static constexpr uint8_t IST8310_ADDR = 0x0E;
static constexpr uint8_t IST8310_WHOAMI = 0x10;

static constexpr uint8_t RGB_LED_ADDR = 0x38;
static constexpr bool RGB_LED_BGR = false;
static constexpr uint32_t RGB_UPDATE_US = 20000;
static constexpr uint32_t RGB_RETRY_US  = 500000;   // retry every 0.5 s
static constexpr uint32_t MAG_RETRY_US  = 1000000;  // retry every 1.0 s

static uint64_t next_rgb_us = 0;
static uint64_t next_rgb_retry_us = 0;
static uint64_t next_mag_retry_us = 0;

static bool rgb_led_ok = false;
static bool mag_device_ok = false;

// ============================================================================
// Flags sent to Jetson
// ============================================================================
enum FrameFlags : uint32_t {
  FLAG_PM1_VALID         = 1u << 0,
  FLAG_PM2_VALID         = 1u << 1,
  FLAG_GPS_PACKET_SEEN   = 1u << 2,
  FLAG_GPS_FIX_OK        = 1u << 3,
  FLAG_GPS_3D            = 1u << 4,
  FLAG_GPS_HACC_OK       = 1u << 5,
  FLAG_GPS_USABLE        = 1u << 6,
  FLAG_GPS_TIME_VALID    = 1u << 7,
  FLAG_MAG_VALID         = 1u << 8,
};

// ============================================================================
// Fixed binary frame sent over USB
// ============================================================================
static constexpr uint32_t FRAME_MAGIC = 0x31524853UL; // "SHR1" little-endian
static constexpr uint16_t FRAME_VERSION = 1;

struct __attribute__((packed)) HostFrame {
  uint32_t magic;
  uint16_t version;
  uint16_t frame_size;

  uint32_t sequence;
  uint64_t esp_time_us;
  uint32_t flags;

  uint32_t usb_drop_count;
  uint32_t gps_checksum_error_count;

  // Power
  uint16_t compute_mV;
  int32_t  compute_mA;
  uint16_t motor_mV;
  int32_t  motor_mA;

  // GNSS epoch / UTC
  uint32_t gps_iTOW_ms;
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
  uint8_t  gps_valid_flags;
  uint32_t gps_tAcc_ns;
  int32_t  gps_nano_ns;

  // GNSS solution
  uint8_t  gps_fix_type;
  uint8_t  gps_fix_flags;
  uint8_t  gps_num_sv;
  uint8_t  reserved0;

  int32_t  lon_e7;
  int32_t  lat_e7;
  int32_t  height_mm;
  int32_t  hMSL_mm;

  uint32_t hAcc_mm;
  uint32_t vAcc_mm;

  int32_t  velN_mms;
  int32_t  velE_mms;
  int32_t  velD_mms;
  int32_t  gSpeed_mms;
  int32_t  headMot_e5;

  uint32_t sAcc_mms;
  uint32_t headAcc_e5;
  uint16_t pDOP_centi;
  uint16_t reserved1;

  // Magnetometer, raw IST8310 counts
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  uint16_t reserved2;

  uint32_t crc32;
};

static_assert(sizeof(HostFrame) == 136, "HostFrame layout changed; update Jetson parser too");

// ============================================================================
// Internal state
// ============================================================================
struct PowerState {
  float compute_v = 0.0f;
  float compute_a = 0.0f;
  float motor_v = 0.0f;
  float motor_a = 0.0f;
};

struct GpsPvt {
  bool seen = false;

  uint32_t iTOW_ms = 0;

  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint8_t valid = 0;

  uint32_t tAcc_ns = 0;
  int32_t nano_ns = 0;

  uint8_t fix_type = 0;
  uint8_t flags = 0;
  uint8_t flags2 = 0;
  uint8_t num_sv = 0;

  int32_t lon_e7 = 0;
  int32_t lat_e7 = 0;
  int32_t height_mm = 0;
  int32_t hMSL_mm = 0;

  uint32_t hAcc_mm = 0;
  uint32_t vAcc_mm = 0;

  int32_t velN_mms = 0;
  int32_t velE_mms = 0;
  int32_t velD_mms = 0;
  int32_t gSpeed_mms = 0;
  int32_t headMot_e5 = 0;

  uint32_t sAcc_mms = 0;
  uint32_t headAcc_e5 = 0;
  uint16_t pDOP_centi = 0;

  uint64_t rx_esp_time_us = 0;
};

struct MagState {
  bool valid = false;
  bool conversion_pending = false;
  uint64_t trigger_us = 0;
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};

static PowerState power_state;
static GpsPvt gps_pvt;
static MagState mag_state;

static uint32_t host_sequence = 0;
static uint32_t usb_drop_count = 0;
static uint32_t gps_checksum_error_count = 0;
static uint64_t last_usb_tx_ok_us = 0;

// Non-blocking USB TX state.
// A 136-byte HostFrame can be larger than one instantaneous CDC TX buffer.
// We therefore send only what Serial.availableForWrite() can accept each pass.
static uint8_t usb_tx_buffer[sizeof(HostFrame)];
static size_t usb_tx_offset = 0;
static size_t usb_tx_length = 0;

// Scheduled deadlines
static uint64_t next_power_us = 0;
static uint64_t next_mag_us = 0;
static uint64_t next_host_us = 0;

// ============================================================================
// CRC32
// ============================================================================
uint32_t crc32_ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1UL);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }

  return ~crc;
}

// ============================================================================
// Little-endian helpers
// ============================================================================
uint16_t getU2(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

int16_t getI2(const uint8_t* p) {
  return static_cast<int16_t>(getU2(p));
}

uint32_t getU4(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

int32_t getI4(const uint8_t* p) {
  return static_cast<int32_t>(getU4(p));
}

// ============================================================================
// UBX TX / GPS configuration
// ============================================================================
void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;

  auto addCk = [&](uint8_t b) {
    ckA = static_cast<uint8_t>(ckA + b);
    ckB = static_cast<uint8_t>(ckB + ckA);
  };

  Serial1.write(0xB5);
  Serial1.write(0x62);

  Serial1.write(cls); addCk(cls);
  Serial1.write(id);  addCk(id);

  const uint8_t lenL = len & 0xFF;
  const uint8_t lenH = (len >> 8) & 0xFF;

  Serial1.write(lenL); addCk(lenL);
  Serial1.write(lenH); addCk(lenH);

  for (uint16_t i = 0; i < len; ++i) {
    Serial1.write(payload[i]);
    addCk(payload[i]);
  }

  Serial1.write(ckA);
  Serial1.write(ckB);
}

void setUbxMessageRate(uint8_t msgClass, uint8_t msgId, uint8_t uart1Rate) {
  // UBX-CFG-MSG 8-byte payload:
  // msgClass, msgID, rateDDC, rateUART1, rateUART2, rateUSB, rateSPI, reserved
  uint8_t payload[8] = {
    msgClass, msgId,
    0,
    uart1Rate,
    0,
    0,
    0,
    0
  };

  sendUbx(0x06, 0x01, payload, sizeof(payload));
}

void configureGps() {
  // Navigation rate = 200 ms = 5 Hz, navRate = 1, timeRef = GPS time.
  uint8_t ratePayload[6] = {
    static_cast<uint8_t>(GPS_MEAS_PERIOD_MS & 0xFF),
    static_cast<uint8_t>((GPS_MEAS_PERIOD_MS >> 8) & 0xFF),
    1, 0,
    1, 0
  };

  sendUbx(0x06, 0x08, ratePayload, sizeof(ratePayload));
  delay(50);

  // Enable UBX-NAV-PVT on UART1.
  setUbxMessageRate(0x01, 0x07, 1);
  delay(20);

  // Disable common NMEA talkers on UART1 to reduce UART load / parsing jitter.
  // NMEA class = 0xF0
  const uint8_t nmeaIds[] = {
    0x00, // GGA
    0x01, // GLL
    0x02, // GSA
    0x03, // GSV
    0x04, // RMC
    0x05, // VTG
    0x08  // ZDA
  };

  for (uint8_t id : nmeaIds) {
    setUbxMessageRate(0xF0, id, 0);
    delay(5);
  }
}

// ============================================================================
// UBX parser
// ============================================================================
class UbxParser {
public:
  void feed(uint8_t b) {
    switch (state_) {
      case SYNC1:
        if (b == 0xB5) state_ = SYNC2;
        break;

      case SYNC2:
        if (b == 0x62) {
          resetChecksum();
          state_ = CLASS;
        } else {
          state_ = SYNC1;
        }
        break;

      case CLASS:
        cls_ = b;
        addChecksum(b);
        state_ = ID;
        break;

      case ID:
        id_ = b;
        addChecksum(b);
        state_ = LEN1;
        break;

      case LEN1:
        len_ = b;
        addChecksum(b);
        state_ = LEN2;
        break;

      case LEN2:
        len_ |= static_cast<uint16_t>(b) << 8;
        addChecksum(b);

        payload_index_ = 0;

        if (len_ > MAX_PAYLOAD) {
          state_ = SYNC1;
        } else if (len_ == 0) {
          state_ = CKA;
        } else {
          state_ = PAYLOAD;
        }
        break;

      case PAYLOAD:
        payload_[payload_index_++] = b;
        addChecksum(b);

        if (payload_index_ >= len_) {
          state_ = CKA;
        }
        break;

      case CKA:
        rx_ck_a_ = b;
        state_ = CKB;
        break;

      case CKB:
        if (rx_ck_a_ == ck_a_ && b == ck_b_) {
          handlePacket();
        } else {
          ++gps_checksum_error_count;
        }
        state_ = SYNC1;
        break;
    }
  }

private:
  static constexpr uint16_t MAX_PAYLOAD = 128;

  enum State : uint8_t {
    SYNC1,
    SYNC2,
    CLASS,
    ID,
    LEN1,
    LEN2,
    PAYLOAD,
    CKA,
    CKB
  };

  State state_ = SYNC1;

  uint8_t cls_ = 0;
  uint8_t id_ = 0;
  uint16_t len_ = 0;
  uint16_t payload_index_ = 0;

  uint8_t payload_[MAX_PAYLOAD];

  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t rx_ck_a_ = 0;

  void resetChecksum() {
    ck_a_ = 0;
    ck_b_ = 0;
  }

  void addChecksum(uint8_t b) {
    ck_a_ = static_cast<uint8_t>(ck_a_ + b);
    ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
  }

  void handlePacket() {
    if (cls_ != 0x01 || id_ != 0x07 || len_ != 92) {
      return;
    }

    gps_pvt.iTOW_ms = getU4(payload_ + 0);

    gps_pvt.year   = getU2(payload_ + 4);
    gps_pvt.month  = payload_[6];
    gps_pvt.day    = payload_[7];
    gps_pvt.hour   = payload_[8];
    gps_pvt.minute = payload_[9];
    gps_pvt.second = payload_[10];
    gps_pvt.valid  = payload_[11];

    gps_pvt.tAcc_ns = getU4(payload_ + 12);
    gps_pvt.nano_ns = getI4(payload_ + 16);

    gps_pvt.fix_type = payload_[20];
    gps_pvt.flags    = payload_[21];
    gps_pvt.flags2   = payload_[22];
    gps_pvt.num_sv   = payload_[23];

    gps_pvt.lon_e7    = getI4(payload_ + 24);
    gps_pvt.lat_e7    = getI4(payload_ + 28);
    gps_pvt.height_mm = getI4(payload_ + 32);
    gps_pvt.hMSL_mm   = getI4(payload_ + 36);

    gps_pvt.hAcc_mm = getU4(payload_ + 40);
    gps_pvt.vAcc_mm = getU4(payload_ + 44);

    gps_pvt.velN_mms   = getI4(payload_ + 48);
    gps_pvt.velE_mms   = getI4(payload_ + 52);
    gps_pvt.velD_mms   = getI4(payload_ + 56);
    gps_pvt.gSpeed_mms = getI4(payload_ + 60);
    gps_pvt.headMot_e5 = getI4(payload_ + 64);

    gps_pvt.sAcc_mms   = getU4(payload_ + 68);
    gps_pvt.headAcc_e5 = getU4(payload_ + 72);
    gps_pvt.pDOP_centi = getU2(payload_ + 76);

    gps_pvt.rx_esp_time_us = esp_timer_get_time();
    gps_pvt.seen = true;
  }
};

static UbxParser ubx_parser;

// ============================================================================
// GPS polling
// ============================================================================
void serviceGps() {
  while (Serial1.available() > 0) {
    ubx_parser.feed(static_cast<uint8_t>(Serial1.read()));
  }
}

// ============================================================================
// I2C helpers / IST8310
// ============================================================================
bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t got = Wire.requestFrom(addr, len);

  if (got != len) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < len; ++i) {
    data[i] = Wire.read();
  }

  return true;
}

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool initIst8310() {
  uint8_t who = 0;

  if (!i2cReadRegs(IST8310_ADDR, 0x00, &who, 1)) {
    return false;
  }

  if (who != IST8310_WHOAMI) {
    return false;
  }

  // Software reset
  if (!i2cWriteReg(IST8310_ADDR, 0x0B, 0x01)) {
    return false;
  }
  delay(20);

  // Averaging settings used by many flight-controller drivers.
  if (!i2cWriteReg(IST8310_ADDR, 0x41, 0x24)) {
    return false;
  }
  if (!i2cWriteReg(IST8310_ADDR, 0x42, 0xC0)) {
    return false;
  }

  mag_state.conversion_pending = false;
  mag_state.valid = false;
  return true;
}

void triggerMagConversion(uint64_t nowUs) {
  if (!mag_device_ok) {
    return;
  }

  if (i2cWriteReg(IST8310_ADDR, 0x0A, 0x01)) {
    mag_state.conversion_pending = true;
    mag_state.trigger_us = nowUs;
  } else {
    mag_state.valid = false;
    mag_state.conversion_pending = false;
    mag_device_ok = false;
    next_mag_retry_us = nowUs + MAG_RETRY_US;
  }
}

void serviceMag(uint64_t nowUs) {
  if (!mag_device_ok) {
    mag_state.valid = false;
    mag_state.conversion_pending = false;
    return;
  }
  // Complete the previous non-blocking single measurement.
  if (mag_state.conversion_pending &&
      nowUs - mag_state.trigger_us >= 7000) {

    uint8_t status = 0;

    if (!i2cReadRegs(IST8310_ADDR, 0x02, &status, 1)) {
      mag_state.valid = false;
      mag_state.conversion_pending = false;
      mag_device_ok = false;
      next_mag_retry_us = nowUs + MAG_RETRY_US;
      return;
    }

    if (status & 0x01) {
      uint8_t raw[6];

      if (i2cReadRegs(IST8310_ADDR, 0x03, raw, sizeof(raw))) {
        mag_state.x = getI2(raw + 0);
        mag_state.y = getI2(raw + 2);
        mag_state.z = getI2(raw + 4);
        mag_state.valid = true;
      } else {
        mag_state.valid = false;
        mag_device_ok = false;
        next_mag_retry_us = nowUs + MAG_RETRY_US;
      }
    }

    mag_state.conversion_pending = false;
  }

  // Start next conversion at the requested rate.
  if (!mag_state.conversion_pending && nowUs >= next_mag_us) {
    do {
      next_mag_us += MAG_PERIOD_US;
    } while (next_mag_us <= nowUs);

    triggerMagConversion(nowUs);
  }
}

// ============================================================================
// Power acquisition
// ============================================================================
float readAdcVolts(uint8_t pin) {
  // A small fixed sample count. No heap, bounded time.
  static constexpr uint8_t N = 4;

  uint32_t sum_mV = 0;

  for (uint8_t i = 0; i < N; ++i) {
    sum_mV += analogReadMilliVolts(pin);
  }

  return (sum_mV / static_cast<float>(N)) * 0.001f;
}

float iir(float oldValue, float newValue) {
  static constexpr float ALPHA = 0.25f;

  if (oldValue == 0.0f) {
    return newValue;
  }

  return oldValue + ALPHA * (newValue - oldValue);
}

void updatePower() {
  const float pm1_v_pin = readAdcVolts(PIN_PM1_V);
  const float pm1_i_pin = readAdcVolts(PIN_PM1_I);
  const float pm2_v_pin = readAdcVolts(PIN_PM2_V);
  const float pm2_i_pin = readAdcVolts(PIN_PM2_I);

  const float pm1_v_sensor = pm1_v_pin / PM1_V_INPUT_RATIO;
  const float pm1_i_sensor = pm1_i_pin / PM1_I_INPUT_RATIO;
  const float pm2_v_sensor = pm2_v_pin / PM2_V_INPUT_RATIO;
  const float pm2_i_sensor = pm2_i_pin / PM2_I_INPUT_RATIO;

  float computeV = pm1_v_sensor * PM02_VOLTAGE_DIVIDER;
  float computeA = (pm1_i_sensor - PM1_CURRENT_ZERO_V) * PM02_AMPS_PER_VOLT;

  float motorV = pm2_v_sensor * PM02_VOLTAGE_DIVIDER;
  float motorA = (pm2_i_sensor - PM2_CURRENT_ZERO_V) * PM02_AMPS_PER_VOLT;

  if (computeA < 0.0f) computeA = 0.0f;
  if (motorA < 0.0f) motorA = 0.0f;

  power_state.compute_v = iir(power_state.compute_v, computeV);
  power_state.compute_a = iir(power_state.compute_a, computeA);
  power_state.motor_v   = iir(power_state.motor_v, motorV);
  power_state.motor_a   = iir(power_state.motor_a, motorA);
}

// ============================================================================
// Holybro RGB LED @ I2C 0x38
// ============================================================================

struct RGBColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

bool ncp5623Write(uint8_t command) {
  Wire.beginTransmission(RGB_LED_ADDR);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

bool initRgbLed() {
  if (!ncp5623Write(0x3F)) return false; // current level max
  if (!ncp5623Write(0x40)) return false; // R off
  if (!ncp5623Write(0x60)) return false; // G off
  if (!ncp5623Write(0x80)) return false; // B off
  return true;
}

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  if (!rgb_led_ok) return;

  uint8_t rr = r >> 3;
  uint8_t gg = g >> 3;
  uint8_t bb = b >> 3;

  if (RGB_LED_BGR) {
    uint8_t t = rr;
    rr = bb;
    bb = t;
  }

  const bool ok =
      ncp5623Write(0x40 | rr) &&
      ncp5623Write(0x60 | gg) &&
      ncp5623Write(0x80 | bb);

  if (!ok) {
    rgb_led_ok = false;
    next_rgb_retry_us = esp_timer_get_time() + RGB_RETRY_US;
  }
}

uint8_t triangleWave8(uint64_t nowUs, uint32_t periodMs) {
  if (periodMs < 2) return 255;

  uint32_t t = static_cast<uint32_t>((nowUs / 1000ULL) % periodMs);
  uint32_t half = periodMs / 2;

  if (t < half) {
    return static_cast<uint8_t>((t * 255UL) / half);
  }

  return static_cast<uint8_t>(((periodMs - t) * 255UL) / half);
}

void serviceI2cPeripheralRecovery(uint64_t nowUs) {
  // Cold-boot case:
  // Jetson/USB may power the ESP32 before the Holybro board has finished
  // powering up. Never make peripheral detection a one-shot setup event.

  if (!rgb_led_ok && nowUs >= next_rgb_retry_us) {
    next_rgb_retry_us = nowUs + RGB_RETRY_US;

    if (i2cDevicePresent(RGB_LED_ADDR)) {
      rgb_led_ok = initRgbLed();

      if (rgb_led_ok) {
        // Short blue acknowledgement when the RGB controller appears.
        setRgb(0, 40, 255);
      }
    }
  }

  if (!mag_device_ok && nowUs >= next_mag_retry_us) {
    next_mag_retry_us = nowUs + MAG_RETRY_US;

    if (i2cDevicePresent(IST8310_ADDR)) {
      mag_device_ok = initIst8310();

      if (mag_device_ok) {
        next_mag_us = nowUs + MAG_PERIOD_US;
      }
    }
  }
}

void updateFancyRgb(uint32_t flags, uint64_t nowUs) {
  if (!rgb_led_ok) return;
  if (nowUs < next_rgb_us) return;

  do {
    next_rgb_us += RGB_UPDATE_US;
  } while (next_rgb_us <= nowUs);

  const bool usbOk =
      (last_usb_tx_ok_us != 0) &&
      (nowUs - last_usb_tx_ok_us < 500000ULL);

  // USB/Jetson fault: double red flash
  if (!usbOk) {
    uint32_t phase = static_cast<uint32_t>((nowUs / 1000ULL) % 1000);

    if (phase < 100) {
      setRgb(255, 0, 0);
    } else if (phase < 180) {
      setRgb(0, 0, 0);
    } else if (phase < 280) {
      setRgb(255, 0, 0);
    } else {
      setRgb(0, 0, 0);
    }
    return;
  }

  // No GPS packet yet: purple breathing
  if (!(flags & FLAG_GPS_PACKET_SEEN)) {
    uint8_t x = triangleWave8(nowUs, 1600);
    setRgb(x / 2, 0, x);
    return;
  }

  // GPS data present but no valid fix: orange breathing
  if (!(flags & FLAG_GPS_FIX_OK)) {
    uint8_t x = triangleWave8(nowUs, 1200);
    setRgb(x, x / 4, 0);
    return;
  }

  // Fix but not 3D: fast blue pulse
  if (!(flags & FLAG_GPS_3D)) {
    uint8_t x = triangleWave8(nowUs, 600);
    setRgb(0, x / 3, x);
    return;
  }

  // 3D but hAcc too large: cyan breathing
  if (!(flags & FLAG_GPS_HACC_OK)) {
    uint8_t x = triangleWave8(nowUs, 1000);
    setRgb(0, x, x);
    return;
  }

  // GPS usable + USB healthy: green/cyan breathing + white sparkle
  if (flags & FLAG_GPS_USABLE) {
    uint32_t phase = static_cast<uint32_t>((nowUs / 1000ULL) % 3000);

    if (phase < 50) {
      setRgb(255, 255, 255);
      return;
    }

    if (phase < 100) {
      setRgb(60, 255, 150);
      return;
    }

    uint8_t x = triangleWave8(nowUs, 2200);

    uint8_t green = static_cast<uint8_t>(
        100 + (static_cast<uint16_t>(x) * 155U) / 255U
    );

    uint8_t blue = static_cast<uint8_t>(
        20 + (static_cast<uint16_t>(x) * 100U) / 255U
    );

    setRgb(0, green, blue);
    return;
  }

  setRgb(20, 20, 20);
}

// ============================================================================
// Status LEDs
// ============================================================================

// Holybro GPS safety LED on GPIO5:
//   solid ON = GPS solution passes our usability gate
//   1 Hz blink = waiting for a usable GPS fix
void setGpsLed(bool on) {
  // Active-low open-drain.
  digitalWrite(PIN_GPS_LED, on ? LOW : HIGH);
}

void updateGpsLed(uint32_t flags, uint64_t nowUs) {
  if (flags & FLAG_GPS_USABLE) {
    setGpsLed(true);
  } else {
    // Toggle every 500 ms -> 1 Hz full blink cycle.
    setGpsLed(((nowUs / 500000ULL) & 1ULL) == 0);
  }
}

// ESP32-C3 onboard LED on GPIO8:
//   solid ON       = USB telemetry has been transmitted recently
//   fast blink     = USB link/TX is stale or unavailable
//
// This LED is intentionally NOT used for balance-control state.
// It only indicates sensor-hub <-> Jetson telemetry health.
void setStatusLed(bool on) {
  if (STATUS_LED_ACTIVE_LOW) {
    digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);
  } else {
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
  }
}

void updateStatusLed(uint64_t nowUs) {
  const bool usbRecentlyOk =
      (last_usb_tx_ok_us != 0) &&
      (nowUs - last_usb_tx_ok_us < 500000ULL);

  if (usbRecentlyOk) {
    setStatusLed(true);
  } else {
    // Fast blink: toggle every 125 ms -> 4 Hz full blink cycle.
    setStatusLed(((nowUs / 125000ULL) & 1ULL) == 0);
  }
}

// ============================================================================
// Non-blocking USB TX service
// ============================================================================
bool usbTxBusy() {
  return usb_tx_offset < usb_tx_length;
}

void serviceUsbTx(uint64_t nowUs) {
  if (!usbTxBusy()) {
    return;
  }

  const int room = Serial.availableForWrite();
  if (room <= 0) {
    return;
  }

  const size_t remaining = usb_tx_length - usb_tx_offset;
  const size_t chunk = min(remaining, static_cast<size_t>(room));

  const size_t written = Serial.write(
    usb_tx_buffer + usb_tx_offset,
    chunk
  );

  usb_tx_offset += written;

  if (usb_tx_offset >= usb_tx_length) {
    usb_tx_offset = 0;
    usb_tx_length = 0;
    last_usb_tx_ok_us = nowUs;
  }
}

bool queueUsbFrame(const HostFrame& frame) {
  // Never overwrite a frame that is still being transmitted.
  if (usbTxBusy()) {
    ++usb_drop_count;
    return false;
  }

  memcpy(usb_tx_buffer, &frame, sizeof(frame));
  usb_tx_offset = 0;
  usb_tx_length = sizeof(frame);
  return true;
}

// ============================================================================
// Frame building / USB TX
// ============================================================================
uint32_t buildFlags() {
  uint32_t flags = 0;

  if (power_state.compute_v > 5.0f && power_state.compute_v < 20.0f) {
    flags |= FLAG_PM1_VALID;
  }

  if (power_state.motor_v > 10.0f && power_state.motor_v < 35.0f) {
    flags |= FLAG_PM2_VALID;
  }

  if (gps_pvt.seen) {
    flags |= FLAG_GPS_PACKET_SEEN;

    const bool fixOk = (gps_pvt.flags & 0x01) != 0;
    const bool fix3d = gps_pvt.fix_type == 3;
    const bool hAccOk = gps_pvt.hAcc_mm <= GPS_MAX_HACC_MM;

    // validDate = bit0, validTime = bit1, fullyResolved = bit2
    const bool timeValid = (gps_pvt.valid & 0x03) == 0x03;

    if (fixOk) flags |= FLAG_GPS_FIX_OK;
    if (fix3d) flags |= FLAG_GPS_3D;
    if (hAccOk) flags |= FLAG_GPS_HACC_OK;
    if (timeValid) flags |= FLAG_GPS_TIME_VALID;

    if (fixOk && fix3d && hAccOk) {
      flags |= FLAG_GPS_USABLE;
    }
  }

  if (mag_state.valid) {
    flags |= FLAG_MAG_VALID;
  }

  return flags;
}

void sendHostFrame(uint64_t nowUs) {
  HostFrame frame = {};

  frame.magic = FRAME_MAGIC;
  frame.version = FRAME_VERSION;
  frame.frame_size = sizeof(HostFrame);

  frame.sequence = host_sequence++;
  frame.esp_time_us = nowUs;
  frame.flags = buildFlags();

  frame.usb_drop_count = usb_drop_count;
  frame.gps_checksum_error_count = gps_checksum_error_count;

  frame.compute_mV = static_cast<uint16_t>(
    constrain(power_state.compute_v * 1000.0f, 0.0f, 65535.0f)
  );

  frame.compute_mA = static_cast<int32_t>(
    power_state.compute_a * 1000.0f
  );

  frame.motor_mV = static_cast<uint16_t>(
    constrain(power_state.motor_v * 1000.0f, 0.0f, 65535.0f)
  );

  frame.motor_mA = static_cast<int32_t>(
    power_state.motor_a * 1000.0f
  );

  frame.gps_iTOW_ms = gps_pvt.iTOW_ms;

  frame.year = gps_pvt.year;
  frame.month = gps_pvt.month;
  frame.day = gps_pvt.day;
  frame.hour = gps_pvt.hour;
  frame.minute = gps_pvt.minute;
  frame.second = gps_pvt.second;
  frame.gps_valid_flags = gps_pvt.valid;
  frame.gps_tAcc_ns = gps_pvt.tAcc_ns;
  frame.gps_nano_ns = gps_pvt.nano_ns;

  frame.gps_fix_type = gps_pvt.fix_type;
  frame.gps_fix_flags = gps_pvt.flags;
  frame.gps_num_sv = gps_pvt.num_sv;

  frame.lon_e7 = gps_pvt.lon_e7;
  frame.lat_e7 = gps_pvt.lat_e7;
  frame.height_mm = gps_pvt.height_mm;
  frame.hMSL_mm = gps_pvt.hMSL_mm;

  frame.hAcc_mm = gps_pvt.hAcc_mm;
  frame.vAcc_mm = gps_pvt.vAcc_mm;

  frame.velN_mms = gps_pvt.velN_mms;
  frame.velE_mms = gps_pvt.velE_mms;
  frame.velD_mms = gps_pvt.velD_mms;
  frame.gSpeed_mms = gps_pvt.gSpeed_mms;
  frame.headMot_e5 = gps_pvt.headMot_e5;

  frame.sAcc_mms = gps_pvt.sAcc_mms;
  frame.headAcc_e5 = gps_pvt.headAcc_e5;
  frame.pDOP_centi = gps_pvt.pDOP_centi;

  frame.mag_x = mag_state.x;
  frame.mag_y = mag_state.y;
  frame.mag_z = mag_state.z;

  frame.crc32 = 0;
  frame.crc32 = crc32_ieee(
    reinterpret_cast<const uint8_t*>(&frame),
    sizeof(HostFrame) - sizeof(frame.crc32)
  );

  // Queue the complete frame without blocking.
  // If the previous frame is still pending, this new frame is dropped.
  queueUsbFrame(frame);

  updateGpsLed(frame.flags, nowUs);
  updateStatusLed(nowUs);
  updateFancyRgb(frame.flags, nowUs);
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  // Start native USB CDC immediately. Do NOT wait for Serial / host enumeration.
  // This allows the ESP32 to boot independently while Jetson is still starting.
  Serial.begin(921600);

  pinMode(PIN_GPS_LED, OUTPUT_OPEN_DRAIN);
  setGpsLed(false);

  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  // Brief boot indication on GPIO8.
  for (int i = 0; i < 3; ++i) {
    setStatusLed(true);
    delay(80);
    setStatusLed(false);
    delay(80);
  }

  // ADC
  analogReadResolution(12);

  analogSetPinAttenuation(PIN_PM1_V, ADC_11db);
  analogSetPinAttenuation(PIN_PM1_I, ADC_11db);
  analogSetPinAttenuation(PIN_PM2_V, ADC_11db);
  analogSetPinAttenuation(PIN_PM2_I, ADC_11db);

  // I2C master for Holybro compass / LED devices.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.setTimeOut(10);

  // Do not trust I2C peripherals to be ready at ESP32 boot.
  // Probe/re-initialize them continuously from loop().
  mag_device_ok = false;
  rgb_led_ok = false;

  // M8N UART
  Serial1.begin(
    GPS_UART_BAUD,
    SERIAL_8N1,
    PIN_GPS_RX,
    PIN_GPS_TX
  );

  delay(500);
  configureGps();

  const uint64_t nowUs = esp_timer_get_time();

  next_power_us = nowUs + POWER_PERIOD_US;
  next_mag_us   = nowUs + MAG_PERIOD_US;
  next_host_us  = nowUs + HOST_PERIOD_US;
  next_rgb_us   = nowUs + RGB_UPDATE_US;

  // First peripheral probes happen immediately in loop().
  next_rgb_retry_us = nowUs;
  next_mag_retry_us = nowUs;
}

// ============================================================================
// Main loop
// ============================================================================
void loop() {
  const uint64_t nowUs = esp_timer_get_time();

  // Drain any pending USB bytes without blocking.
  serviceUsbTx(nowUs);

  // GPS is serviced every pass so the UART RX buffer is drained immediately.
  serviceGps();

  // Automatically recover peripherals that were not powered/ready during boot.
  serviceI2cPeripheralRecovery(nowUs);

  serviceMag(nowUs);

  if (nowUs >= next_power_us) {
    do {
      next_power_us += POWER_PERIOD_US;
    } while (next_power_us <= nowUs);

    updatePower();
  }

  if (nowUs >= next_host_us) {
    do {
      next_host_us += HOST_PERIOD_US;
    } while (next_host_us <= nowUs);

    sendHostFrame(nowUs);
  }

  // Give USB another chance after the sensor work.
  serviceUsbTx(esp_timer_get_time());

  // Keep indicators responsive even between telemetry frames.
  updateStatusLed(nowUs);
  updateFancyRgb(buildFlags(), nowUs);

  // Yield without deliberately sleeping a whole millisecond.
  taskYIELD();
}
