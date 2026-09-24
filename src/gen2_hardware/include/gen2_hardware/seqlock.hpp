// Single-writer / multi-reader sequence lock for small trivially-copyable structs.
// The writer never blocks; readers retry while a write is in progress.
#pragma once

#include <atomic>
#include <cstring>
#include <type_traits>

namespace gen2_hardware
{

template<typename T>
class SeqLock
{
  static_assert(std::is_trivially_copyable_v<T>);

public:
  void store(const T & v)
  {
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(static_cast<void *>(&data_), &v, sizeof(T));
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);
  }

  T load() const
  {
    T out;
    uint32_t s0, s1;
    do {
      s0 = seq_.load(std::memory_order_acquire);
      std::atomic_thread_fence(std::memory_order_acquire);
      std::memcpy(&out, static_cast<const void *>(&data_), sizeof(T));
      std::atomic_thread_fence(std::memory_order_acquire);
      s1 = seq_.load(std::memory_order_relaxed);
    } while ((s0 & 1u) || s0 != s1);
    return out;
  }

private:
  std::atomic<uint32_t> seq_{0};
  T data_{};
};

}  // namespace gen2_hardware
