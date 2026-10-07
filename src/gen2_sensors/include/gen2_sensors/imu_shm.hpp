// Latest IMU sample in POSIX shared memory (/dev/shm/gen2_imu), written by iahrs_node, read by the
// balance controller without DDS (seqlock, lock-free across processes). Body frame, as /imu/data.
#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>

namespace gen2_sensors
{

struct ImuSample
{
  int64_t rx_mono_ns = 0;   // CLOCK_MONOTONIC (steady_clock) when the serial line arrived
  int64_t count = 0;        // sensor 1 ms counter
  double q[4] = {1, 0, 0, 0};   // w x y z, world <- body
  double gyro[3] = {0, 0, 0};   // body frame [rad/s]
  double acc[3] = {0, 0, 0};    // body frame [m/s^2]
};

struct ImuShm
{
  std::atomic<uint32_t> seq{0};
  ImuSample s;
};

constexpr const char * kImuShmName = "/gen2_imu";

class ImuShmWriter
{
public:
  bool open()
  {
    const int fd = ::shm_open(kImuShmName, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {return false;}
    if (::ftruncate(fd, sizeof(ImuShm)) != 0) {::close(fd); return false;}
    void * p = ::mmap(nullptr, sizeof(ImuShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) {return false;}
    shm_ = static_cast<ImuShm *>(p);
    return true;
  }
  void write(const ImuSample & s)
  {
    if (!shm_) {return;}
    const uint32_t q = shm_->seq.load(std::memory_order_relaxed);
    shm_->seq.store(q + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(static_cast<void *>(&shm_->s), &s, sizeof(s));
    std::atomic_thread_fence(std::memory_order_release);
    shm_->seq.store(q + 2, std::memory_order_release);
  }

private:
  ImuShm * shm_ = nullptr;
};

class ImuShmReader
{
public:
  bool open()
  {
    const int fd = ::shm_open(kImuShmName, O_RDONLY, 0);
    if (fd < 0) {return false;}
    void * p = ::mmap(nullptr, sizeof(ImuShm), PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) {return false;}
    shm_ = static_cast<const ImuShm *>(p);
    return true;
  }
  bool is_open() const {return shm_ != nullptr;}
  bool read(ImuSample & out) const
  {
    if (!shm_) {return false;}
    for (int i = 0; i < 100; ++i) {
      const uint32_t a = shm_->seq.load(std::memory_order_acquire);
      if (a & 1u) {continue;}
      std::memcpy(&out, static_cast<const void *>(&shm_->s), sizeof(out));
      std::atomic_thread_fence(std::memory_order_acquire);
      if (shm_->seq.load(std::memory_order_relaxed) == a) {return a != 0;}
    }
    return false;
  }

private:
  const ImuShm * shm_ = nullptr;
};

}  // namespace gen2_sensors
