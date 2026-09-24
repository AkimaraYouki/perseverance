#include <gtest/gtest.h>

#include "gen2_hardware/cubemars_servo.hpp"
#include "gen2_hardware/motor_config.hpp"
#include "gen2_hardware/seqlock.hpp"

using namespace gen2_hardware;
using namespace gen2_hardware::cubemars;

// Real frame captured 2026-09-24 from AK45-10 id 69: 0x2945 FF 17 00 00 00 00 36 00
TEST(CubemarsServo, DecodeRealStatusFrame)
{
  Frame f;
  f.id = 0x2945;
  f.len = 8;
  const uint8_t d[8] = {0xFF, 0x17, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00};
  std::copy(d, d + 8, f.data);
  ASSERT_EQ(id_driver(f.id), 69);
  auto s = decode_status(f);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->position_deg, -23.3, 1e-9);
  EXPECT_EQ(s->speed_erpm, 0.0);
  EXPECT_EQ(s->current_a, 0.0);
  EXPECT_EQ(s->temperature_c, 54);
  EXPECT_EQ(s->error, 0);
}

TEST(CubemarsServo, DecodeSignedFields)
{
  Frame f;
  f.id = make_id(static_cast<Mode>(0), 5) | (0x29u << 8);
  f.len = 8;
  // pos -32000 (0x8300), speed +32000 (0x7D00), current -6000 (0xE890), temp -20, err 7
  const uint8_t d[8] = {0x83, 0x00, 0x7D, 0x00, 0xE8, 0x90, 0xEC, 0x07};
  std::copy(d, d + 8, f.data);
  auto s = decode_status(f);
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->position_deg, -3200.0, 1e-9);
  EXPECT_NEAR(s->speed_erpm, 320000.0, 1e-9);
  EXPECT_NEAR(s->current_a, -60.0, 1e-9);
  EXPECT_EQ(s->temperature_c, -20);
  EXPECT_STREQ(error_text(s->error), "motor stall / lock-up");
}

TEST(CubemarsServo, BootFrame)
{
  Frame f;
  f.id = 0x2C00;
  f.len = 4;
  const uint8_t d[4] = {0xFA, 0xFB, 0xFC, 0xFD};
  std::copy(d, d + 4, f.data);
  EXPECT_TRUE(is_boot_frame(f));
  EXPECT_FALSE(decode_status(f));
}

TEST(CubemarsServo, EncodeCurrentMatchesManual)
{
  // comm_can_set_current: int32(current*1000) big-endian, id = driver | (1 << 8)
  Frame f = encode_current(69, -1.5);
  EXPECT_EQ(f.id, 0x0145u);
  ASSERT_EQ(f.len, 4);
  EXPECT_EQ(f.data[0], 0xFF);
  EXPECT_EQ(f.data[1], 0xFF);
  EXPECT_EQ(f.data[2], 0xFA);
  EXPECT_EQ(f.data[3], 0x24);  // -1500
  EXPECT_EQ(encode_current(69, 1000.0).data[1], 0x00);  // clamped to 60000 = 0x0000EA60
  EXPECT_EQ(encode_current(69, 1000.0).data[2], 0xEA);
}

TEST(CubemarsServo, EncodeOtherModes)
{
  EXPECT_EQ(encode_rpm(1, 0).id, 0x0301u);
  EXPECT_EQ(encode_position(1, 0).id, 0x0401u);
  Frame o = encode_set_origin(1, 0);
  EXPECT_EQ(o.id, 0x0501u);
  EXPECT_EQ(o.len, 1);
  EXPECT_EQ(encode_disable(1).id, 0x0F01u);
  EXPECT_EQ(encode_current_brake(1, -5).data[3], 0);  // negative brake clamped to 0
}

TEST(MotorConfig, Conversions)
{
  MotorConfig c;
  c.pole_pairs = 14;
  c.gear_ratio = 10;
  c.direction = -1;
  c.current_limit_a = 2.0;
  // 14 pole pairs, 10:1 -> 8400 ERPM = 600 rotor rpm = 60 output rpm = 2*pi rad/s
  EXPECT_NEAR(c.erpm_to_joint_vel(8400), -2 * M_PI, 1e-9);
  EXPECT_NEAR(c.raw_to_joint_pos(180.0), -M_PI, 1e-9);
  EXPECT_TRUE(std::isnan(c.current_to_joint_torque(1.0)));   // kt unknown
  EXPECT_EQ(c.joint_torque_to_current(5.0), 0.0);            // refuses without kt
  c.kt_nm_per_a = 0.5;
  EXPECT_NEAR(c.joint_torque_to_current(5.0), -2.0, 1e-12);  // limited
}

TEST(SeqLock, StoreLoad)
{
  struct S {int a; double b;};
  SeqLock<S> l;
  l.store({3, 4.5});
  auto v = l.load();
  EXPECT_EQ(v.a, 3);
  EXPECT_EQ(v.b, 4.5);
}
