#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "gen2_sensor_hub/frame_parser.hpp"

using gen2_sensor_hub::FrameParser;
using gen2_sensor_hub::HostFrame;

namespace
{
std::vector<uint8_t> make_frame(uint32_t seq)
{
  HostFrame f{};
  f.magic = gen2_sensor_hub::kFrameMagic;
  f.version = gen2_sensor_hub::kFrameVersion;
  f.frame_size = sizeof(HostFrame);
  f.sequence = seq;
  f.esp_time_us = 1000ull * seq;
  f.compute_mV = 12345;
  f.crc32 = gen2_sensor_hub::crc32_ieee(reinterpret_cast<uint8_t *>(&f), sizeof(f) - 4);
  std::vector<uint8_t> v(sizeof(f));
  std::memcpy(v.data(), &f, sizeof(f));
  return v;
}
}  // namespace

TEST(Crc32, KnownVector)
{
  const char * s = "123456789";
  EXPECT_EQ(gen2_sensor_hub::crc32_ieee(reinterpret_cast<const uint8_t *>(s), 9), 0xCBF43926u);
}

// Frame captured from the real ESP32 on 2026-09-24 (CRC verified against zlib).
TEST(FrameParser, RealCapturedFrame)
{
  const uint8_t raw[136] = {
    0x53, 0x48, 0x52, 0x31, 0x01, 0x00, 0x88, 0x00, 0xc1, 0x13, 0x00, 0x00, 0xa4, 0x93, 0x14, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0xbf, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xe0, 0x32, 0x65, 0x53, 0x00, 0x00, 0x1e, 0x2b, 0x23, 0x4f, 0x00, 0x00, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0xd4, 0x00, 0xbb, 0xff, 0x68, 0x00, 0x00, 0x00, 0xf9, 0x83, 0x35, 0x32};
  int n = 0;
  HostFrame got{};
  FrameParser p([&](const HostFrame & f) {++n; got = f;});
  p.feed(raw, sizeof(raw));
  ASSERT_EQ(n, 1);
  EXPECT_EQ(got.sequence, 5057u);
  EXPECT_EQ(got.flags, 0x103u);
  EXPECT_EQ(got.compute_mV, 13024u);
  EXPECT_EQ(got.motor_mV, 11038u);
  EXPECT_EQ(got.mag_x, 212);
  EXPECT_EQ(got.mag_y, -69);
  EXPECT_EQ(got.mag_z, 104);
}

TEST(FrameParser, SplitFeedAndGarbage)
{
  std::vector<uint8_t> stream = {0x00, 'S', 'H', 0x13, 0x37};
  for (uint32_t s = 1; s <= 5; ++s) {
    auto f = make_frame(s);
    stream.insert(stream.end(), f.begin(), f.end());
  }
  std::vector<uint32_t> seqs;
  FrameParser p([&](const HostFrame & f) {seqs.push_back(f.sequence);});
  for (std::size_t i = 0; i < stream.size(); i += 7) {
    p.feed(stream.data() + i, std::min<std::size_t>(7, stream.size() - i));
  }
  ASSERT_EQ(seqs.size(), 5u);
  EXPECT_EQ(seqs.front(), 1u);
  EXPECT_EQ(seqs.back(), 5u);
  EXPECT_EQ(p.stats().crc_errors, 0u);
}

TEST(FrameParser, CorruptedFrameRejectedAndResync)
{
  auto a = make_frame(1);
  auto b = make_frame(2);
  auto c = make_frame(3);
  b[50] ^= 0xFF;
  std::vector<uint8_t> stream;
  for (auto * v : {&a, &b, &c}) {stream.insert(stream.end(), v->begin(), v->end());}
  std::vector<uint32_t> seqs;
  FrameParser p([&](const HostFrame & f) {seqs.push_back(f.sequence);});
  p.feed(stream.data(), stream.size());
  ASSERT_EQ(seqs.size(), 2u);
  EXPECT_EQ(seqs[0], 1u);
  EXPECT_EQ(seqs[1], 3u);
  EXPECT_EQ(p.stats().crc_errors, 1u);
}
