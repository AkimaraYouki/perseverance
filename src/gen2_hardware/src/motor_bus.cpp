#include "gen2_hardware/motor_bus.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <sstream>

namespace gen2_hardware
{

std::string validate(const MotorConfig & c)
{
  std::ostringstream e;
  if (c.name.empty()) {e << "empty name; ";}
  if (c.can_id == 0) {e << "can_id 0 is reserved; ";}
  if (c.direction != 1 && c.direction != -1) {e << "direction must be +1/-1; ";}
  if (!(c.pole_pairs > 0)) {e << "pole_pairs must be > 0; ";}
  if (!(c.gear_ratio > 0)) {e << "gear_ratio must be > 0; ";}
  if (!(c.raw_deg_per_output_rev > 0)) {e << "raw_deg_per_output_rev must be > 0; ";}
  if (!(c.current_limit_a >= 0 && c.current_limit_a <= 60)) {e << "current_limit_a out of 0..60; ";}
  if (!(c.joint_min_rad < c.joint_max_rad)) {e << "joint_min_rad >= joint_max_rad; ";}
  if (!(c.feedback_stale_timeout_s > 0)) {e << "feedback_stale_timeout_s must be > 0; ";}
  return e.str();
}

MotorBus::MotorBus(std::string ifname, std::vector<MotorConfig> motors)
: ifname_(std::move(ifname)), motors_(std::move(motors)), unknown_slots_(256), unknown_writer_(256)
{
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    by_id_[motors_[i].can_id] = i;
    slots_.push_back(std::make_unique<SeqLock<MotorFeedback>>());
  }
  writer_state_.resize(motors_.size());
}

MotorBus::~MotorBus()
{
  stop();
  if (lock_fd_ >= 0) {::close(lock_fd_);}
}

bool MotorBus::claim_commander(std::string & err)
{
  if (lock_fd_ >= 0) {return true;}
  const std::string path = "/run/lock/gen2_can_" + ifname_ + ".lock";
  const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
  if (fd < 0) {
    err = "cannot open " + path + ": " + std::strerror(errno);
    return false;
  }
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    char buf[32] = {0};
    const ssize_t n = ::pread(fd, buf, sizeof(buf) - 1, 0);
    ::close(fd);
    err = "another process already commands " + ifname_ + " (pid " +
      (n > 0 ? std::string(buf, static_cast<std::size_t>(n)) : std::string("?")) +
      ", lock " + path + ")";
    return false;
  }
  const std::string pid = std::to_string(::getpid());
  if (::ftruncate(fd, 0) != 0 || ::pwrite(fd, pid.data(), pid.size(), 0) < 0) {}  // PID is informational
  lock_fd_ = fd;
  return true;
}

std::vector<uint8_t> MotorBus::unconfigured_ids() const
{
  std::vector<uint8_t> ids;
  for (int i = 0; i < 256; ++i) {
    if (unknown_seen_[i].load(std::memory_order_relaxed)) {ids.push_back(static_cast<uint8_t>(i));}
  }
  return ids;
}

void MotorBus::start()
{
  if (running_) {
    return;
  }
  running_ = true;
  rx_thread_ = std::thread([this] {rx_loop();});
}

void MotorBus::stop()
{
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  rx_sock_.close();
  tx_sock_.close();
}

bool MotorBus::send(const cubemars::Frame & f, std::string & err)
{
  if (!tx_enabled_ || lock_fd_ < 0) {
    err = lock_fd_ < 0 ? "not the commander of this bus" : "TX disabled (read-only bus)";
    return false;
  }
  if (!tx_sock_.is_open() && !tx_sock_.open(ifname_, err)) {
    ++stats_.tx_errors;
    return false;
  }
  if (!tx_sock_.write_ext(f, err)) {
    ++stats_.tx_errors;
    tx_sock_.close();  // reopen next time (interface may have been restarted)
    return false;
  }
  ++stats_.tx_frames;
  return true;
}

void MotorBus::rx_loop()
{
  RxFrame rx;
  while (running_) {
    if (!rx_sock_.is_open()) {
      std::string err;
      if (!rx_sock_.open(ifname_, err)) {
        stats_.socket_ok = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      ++stats_.reopen_count;
      stats_.socket_ok = true;
    }
    const int r = rx_sock_.read(rx, 20);
    if (r < 0) {
      stats_.socket_ok = false;
      rx_sock_.close();
      continue;
    }
    if (r == 0) {
      continue;
    }
    if (rx.error_frame) {
      ++stats_.rx_error_frames;
      stats_.last_error_class = rx.error_class;
      continue;
    }
    ++stats_.rx_frames;
    stats_.last_rx_mono_ns = rx.mono_ns;
    if (!rx.extended) {
      ++stats_.rx_unknown;
      continue;
    }
    if (cubemars::is_boot_frame(rx.frame)) {
      ++stats_.boot_frames_any;  // observed with driver id 0x00 on AK45-10: not attributable
    }
    const uint8_t drv = cubemars::id_driver(rx.frame.id);
    const auto it = by_id_.find(drv);
    if (it == by_id_.end()) {
      // Not configured: remember CubeMars status frames so the drive shows up (discovery only).
      // Driver id 0 is skipped: an AK45-10 sends two 0x2900 status frames while booting, before
      // its configured id is loaded (observed 2026-09-26) — it is not a real drive.
      if (drv == 0) {
        ++stats_.rx_unknown;
        continue;
      }
      if (auto st = cubemars::decode_status(rx.frame)) {
        MotorFeedback & fb = unknown_writer_[drv];
        fb.valid = true;
        fb.status = *st;
        fb.mono_ns = rx.mono_ns;
        fb.realtime_ns = rx.kernel_realtime_ns;
        ++fb.rx_count;
        unknown_slots_[drv].store(fb);
        unknown_seen_[drv].store(true, std::memory_order_relaxed);
        ++stats_.rx_unconfigured;
      } else {
        ++stats_.rx_unknown;
      }
      continue;
    }
    MotorFeedback & fb = writer_state_[it->second];
    if (auto st = cubemars::decode_status(rx.frame)) {
      fb.valid = true;
      fb.status = *st;
      fb.mono_ns = rx.mono_ns;
      fb.realtime_ns = rx.kernel_realtime_ns;
      ++fb.rx_count;
    } else if (auto p = cubemars::decode_position32_deg(rx.frame)) {
      fb.has_position32 = true;
      fb.position32_deg = *p;
    } else if (cubemars::is_boot_frame(rx.frame)) {
      ++fb.boot_frames;
    } else {
      ++stats_.rx_unknown;
      continue;
    }
    slots_[it->second]->store(fb);
  }
}

}  // namespace gen2_hardware
