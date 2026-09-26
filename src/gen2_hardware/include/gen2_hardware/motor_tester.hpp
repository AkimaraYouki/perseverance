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
  double max_position_move_deg = 720.0;     // joint degrees per position test
  double default_accel_rad_s2 = 20.0;       // position test acceleration if not given
  double rate_hz = 100.0;
};

enum class TestMode {kCurrent, kVelocity, kPosition};

// Position tests are RELATIVE moves (joint degrees from the position at start) with the drive's
// position-speed loop (mode 6). The 0x29 feedback position is int16 x0.1 deg and wraps at
// +-3200 deg while the command is absolute multi-turn, so a test is refused when |raw| is near the
// wrap (set a temporary origin first), and one move is limited to max_position_move_deg.
constexpr double kPositionWrapGuardDeg = 3000.0;

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
  double speed_rad_s = 0.0;           // position mode: speed limit
  double target_rad = 0.0;            // position mode: joint target (absolute)
  double position_error_rad = 0.0;    // position mode: target - final
};

class MotorTester
{
public:
  MotorTester(MotorBus & bus, TestLimits limits);
  ~MotorTester();

  // Returns empty string if accepted, otherwise the rejection reason.
  // value: current [A] | velocity [joint rad/s] | position move [joint deg, relative].
  // speed_rad_s / accel_rad_s2: position mode only (speed required, accel <= 0 -> default).
  std::string start(std::size_t motor, TestMode mode, double value, double duration_s,
    double speed_rad_s = 0.0, double accel_rad_s2 = 0.0);
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
  void run(uint64_t id, std::size_t m, TestMode mode, double value, double duration,
    double speed, double accel);
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
