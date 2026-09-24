// SocketCAN raw socket (classic CAN) with kernel RX timestamps and error-frame reporting.
#pragma once

#include <cstdint>
#include <string>

#include "gen2_hardware/cubemars_servo.hpp"

namespace gen2_hardware
{

struct RxFrame
{
  cubemars::Frame frame;
  bool extended = false;
  bool error_frame = false;
  uint32_t error_class = 0;     // CAN_ERR_* mask when error_frame
  int64_t mono_ns = 0;          // CLOCK_MONOTONIC at receive (userspace)
  int64_t kernel_realtime_ns = 0;  // SO_TIMESTAMPNS (CLOCK_REALTIME) or 0
};

class CanSocket
{
public:
  CanSocket() = default;
  ~CanSocket();
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  // receive_own: loop back our own TX frames to this socket (off by default).
  bool open(const std::string & ifname, std::string & err, bool receive_own = false);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Returns 1 frame read, 0 timeout, -1 error (interface down / gone).
  int read(RxFrame & out, int timeout_ms);
  // Non-blocking write of an extended frame. Returns false on error (e.g. ENOBUFS).
  bool write_ext(const cubemars::Frame & f, std::string & err);

private:
  int fd_ = -1;
};

int64_t mono_now_ns();

// Interface state from sysfs: "up"/"down"/"unknown"/"missing".
std::string can_operstate(const std::string & ifname);

}  // namespace gen2_hardware
