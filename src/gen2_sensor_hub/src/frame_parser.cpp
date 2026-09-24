#include "gen2_sensor_hub/frame_parser.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace gen2_sensor_hub
{

uint32_t crc32_ieee(const uint8_t * data, std::size_t len)
{
  static const auto table = [] {
      std::array<uint32_t, 256> t{};
      for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
          c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        t[i] = c;
      }
      return t;
    }();
  uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < len; ++i) {
    crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return ~crc;
}

namespace
{
constexpr std::size_t kFrameSize = sizeof(HostFrame);
constexpr uint8_t kMagicBytes[4] = {'S', 'H', 'R', '1'};
}  // namespace

FrameParser::FrameParser(Callback cb)
: cb_(std::move(cb))
{
  buf_.reserve(4 * kFrameSize);
}

void FrameParser::reset()
{
  buf_.clear();
}

void FrameParser::feed(const uint8_t * data, std::size_t len)
{
  buf_.insert(buf_.end(), data, data + len);
  process();
  // Bound memory if the stream is garbage: keep at most one partial frame.
  if (buf_.size() > 8 * kFrameSize) {
    const std::size_t drop = buf_.size() - kFrameSize;
    stats_.bytes_discarded += drop;
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(drop));
  }
}

void FrameParser::process()
{
  std::size_t pos = 0;
  while (true) {
    // Find magic.
    auto it = std::search(buf_.begin() + static_cast<std::ptrdiff_t>(pos), buf_.end(),
        std::begin(kMagicBytes), std::end(kMagicBytes));
    const std::size_t start = static_cast<std::size_t>(it - buf_.begin());
    stats_.bytes_discarded += start - pos;
    if (it == buf_.end()) {
      // Keep up to 3 trailing bytes that might be the beginning of a magic.
      const std::size_t keep = std::min<std::size_t>(3, buf_.size() - pos);
      stats_.bytes_discarded -= keep;
      pos = buf_.size() - keep;
      break;
    }
    if (buf_.size() - start < kFrameSize) {
      pos = start;
      break;  // wait for more bytes
    }
    HostFrame f;
    std::memcpy(&f, buf_.data() + start, kFrameSize);
    if (f.version != kFrameVersion || f.frame_size != kFrameSize) {
      ++stats_.header_errors;
      pos = start + 1;
      continue;
    }
    const uint32_t crc = crc32_ieee(buf_.data() + start, kFrameSize - sizeof(uint32_t));
    if (crc != f.crc32) {
      ++stats_.crc_errors;
      pos = start + 1;  // resync inside the bad frame
      continue;
    }
    ++stats_.frames_ok;
    cb_(f);
    pos = start + kFrameSize;
  }
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
}

}  // namespace gen2_sensor_hub
