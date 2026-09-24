// Byte-stream synchronizer for HostFrame v1: magic search, size/version check, CRC32.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "gen2_sensor_hub/host_frame.hpp"

namespace gen2_sensor_hub
{

struct ParserStats
{
  uint64_t frames_ok = 0;
  uint64_t crc_errors = 0;
  uint64_t header_errors = 0;   // magic matched but version/size wrong
  uint64_t bytes_discarded = 0; // bytes skipped while searching for sync
};

class FrameParser
{
public:
  using Callback = std::function<void(const HostFrame &)>;

  explicit FrameParser(Callback cb);

  void feed(const uint8_t * data, std::size_t len);
  void reset();
  const ParserStats & stats() const {return stats_;}

private:
  void process();

  Callback cb_;
  std::vector<uint8_t> buf_;
  ParserStats stats_;
};

}  // namespace gen2_sensor_hub
