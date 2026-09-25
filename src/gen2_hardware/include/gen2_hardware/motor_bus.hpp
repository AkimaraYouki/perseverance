// CAN bus owner: RX thread decodes CubeMars servo uploads into per-motor seqlock slots.
// Readers (control loop, ROS publishers) never block the RX thread.
#pragma once

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gen2_hardware/can_socket.hpp"
#include "gen2_hardware/cubemars_servo.hpp"
#include "gen2_hardware/motor_config.hpp"
#include "gen2_hardware/seqlock.hpp"

namespace gen2_hardware
{

struct MotorFeedback
{
  bool valid = false;
  cubemars::Status status;
  int64_t mono_ns = 0;
  int64_t realtime_ns = 0;
  uint64_t rx_count = 0;
  uint64_t boot_frames = 0;       // 0x2C "entered servo mode" count (drive reboot indicator)
  bool has_position32 = false;
  double position32_deg = 0.0;
};

struct BusStats
{
  std::atomic<uint64_t> rx_frames{0};
  std::atomic<uint64_t> rx_unknown{0};    // frames not matching any configured motor
  std::atomic<uint64_t> rx_error_frames{0};
  std::atomic<uint32_t> last_error_class{0};
  std::atomic<uint64_t> tx_frames{0};
  std::atomic<uint64_t> tx_errors{0};
  std::atomic<uint64_t> reopen_count{0};
  std::atomic<uint64_t> boot_frames_any{0};
  std::atomic<uint64_t> rx_unconfigured{0};   // CubeMars status frames from IDs not in motors.yaml
  std::atomic<bool> socket_ok{false};
  std::atomic<int64_t> last_rx_mono_ns{0};
};

class MotorBus
{
public:
  MotorBus(std::string ifname, std::vector<MotorConfig> motors);
  ~MotorBus();

  void start();
  void stop();

  const std::vector<MotorConfig> & motors() const {return motors_;}
  MotorFeedback feedback(std::size_t i) const {return slots_[i]->load();}
  const BusStats & stats() const {return stats_;}
  const std::string & ifname() const {return ifname_;}

  // Drives that send CubeMars status (0x29) frames but are not configured. Read-only discovery:
  // they are never commanded. IDs are returned in ascending order.
  std::vector<uint8_t> unconfigured_ids() const;
  MotorFeedback unconfigured_feedback(uint8_t id) const {return unknown_slots_[id].load();}

  // TX is refused unless enable_tx(true) was called (read-only by default).
  void enable_tx(bool on) {tx_enabled_ = on;}
  bool tx_enabled() const {return tx_enabled_;}
  bool send(const cubemars::Frame & f, std::string & err);

private:
  void rx_loop();

  std::string ifname_;
  std::vector<MotorConfig> motors_;
  std::map<uint8_t, std::size_t> by_id_;
  std::vector<std::unique_ptr<SeqLock<MotorFeedback>>> slots_;
  std::vector<MotorFeedback> writer_state_;  // RX thread private copy
  std::vector<SeqLock<MotorFeedback>> unknown_slots_;   // 256, indexed by driver id
  std::vector<MotorFeedback> unknown_writer_;           // RX thread private, 256
  std::array<std::atomic<bool>, 256> unknown_seen_{};
  CanSocket rx_sock_;
  CanSocket tx_sock_;
  std::thread rx_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> tx_enabled_{false};
  BusStats stats_;
};

}  // namespace gen2_hardware
