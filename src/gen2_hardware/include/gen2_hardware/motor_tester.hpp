// Guarded single-motor bench test engine (current / velocity), shared by motor_cli and
// motor_test_node. Runs its own 100 Hz thread; always ends with 0 A.
//
// Aborts on: stop(), stale feedback, drive fault, over-temperature, over-speed guard,
// TX failure, and (if enabled) missing operator heartbeat.
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "gen2_hardware/motor_bus.hpp"

namespace gen2_hardware
{

struct TestLimits
{
  double max_current_a = 1.0;
  double max_velocity_rad_s = 2.0;
  double max_duration_s = 3.0;
  double current_ramp_s = 0.3;
  double rate_hz = 100.0;
};

enum class TestMode {kCurrent, kVelocity};

struct TestStatus
{
  bool running = false;
  std::size_t motor = 0;
  TestMode mode = TestMode::kCurrent;
  double value = 0.0;
  double duration_s = 0.0;
  double elapsed_s = 0.0;
  std::string result = "idle";
  uint64_t test_id = 0;
  double peak_current_a = 0.0;
  double peak_velocity_rad_s = 0.0;
  double moved_rad = 0.0;
  double mean_velocity_rad_s = 0.0;
};

class MotorTester
{
public:
  MotorTester(MotorBus & bus, TestLimits limits);
  ~MotorTester();

  // Returns empty string if accepted, otherwise the rejection reason.
  std::string start(std::size_t motor, TestMode mode, double value, double duration_s);
  void stop(const std::string & why);
  void wait();
  TestStatus status() const;

  // Dead-man: when timeout > 0 a running test aborts if heartbeat() is not called in time.
  void require_heartbeat(double timeout_s) {hb_timeout_s_ = timeout_s;}
  void heartbeat() {last_hb_ns_ = mono_now_ns();}
  bool heartbeat_ok() const;

  double current_limit(std::size_t m) const;
  double velocity_limit(std::size_t m) const;
  const TestLimits & limits() const {return limits_;}

  std::string zero_all();
  std::string set_temporary_origin(std::size_t m);

private:
  void run(uint64_t id, std::size_t m, TestMode mode, double value, double duration);
  void send_zero(std::size_t m);

  MotorBus & bus_;
  TestLimits limits_;
  mutable std::mutex mtx_;
  TestStatus st_;
  std::thread th_;
  std::atomic<bool> stop_{false};
  std::string stop_why_;
  std::atomic<double> hb_timeout_s_{0.0};
  std::atomic<int64_t> last_hb_ns_{0};
};

}  // namespace gen2_hardware
