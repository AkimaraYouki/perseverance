#include "gen2_hardware/motor_tester.hpp"

#include <chrono>
#include <cmath>

namespace gen2_hardware
{

MotorTester::MotorTester(MotorBus & bus, TestLimits limits)
: bus_(bus), limits_(limits) {}

MotorTester::~MotorTester()
{
  stop("shutdown");
  wait();
  zero_all();
}

double MotorTester::current_limit(std::size_t m) const
{
  return std::min(bus_.motors()[m].current_limit_a, limits_.max_current_a);
}

double MotorTester::velocity_limit(std::size_t m) const
{
  const double v = bus_.motors()[m].velocity_limit_rad_s;
  return std::min(v > 0 ? v : limits_.max_velocity_rad_s, limits_.max_velocity_rad_s);
}

bool MotorTester::heartbeat_ok() const
{
  const double t = hb_timeout_s_.load();
  return t <= 0 || (mono_now_ns() - last_hb_ns_.load()) * 1e-9 < t;
}

std::string MotorTester::start(std::size_t m, TestMode mode, double value, double duration)
{
  if (m >= bus_.motors().size()) {return "no such motor";}
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (st_.running) {return "a test is already running";}
  }
  const auto & c = bus_.motors()[m];
  if (!std::isfinite(value) || value == 0.0) {return "value must be non-zero";}
  if (!(duration > 0) || duration > limits_.max_duration_s) {
    return "duration must be in (0, " + std::to_string(limits_.max_duration_s) + "] s";
  }
  if (mode == TestMode::kCurrent && std::fabs(value) > current_limit(m)) {
    return "|current| exceeds limit " + std::to_string(current_limit(m)) + " A";
  }
  if (mode == TestMode::kVelocity && std::fabs(value) > velocity_limit(m)) {
    return "|velocity| exceeds limit " + std::to_string(velocity_limit(m)) + " rad/s";
  }
  if (mode == TestMode::kVelocity && !(c.pole_pairs > 0 && c.gear_ratio > 0)) {
    return "pole_pairs/gear_ratio not configured";
  }
  const MotorFeedback fb = bus_.feedback(m);
  if (!fb.valid || (mono_now_ns() - fb.mono_ns) * 1e-9 > c.feedback_stale_timeout_s) {
    return "feedback not fresh";
  }
  if (fb.status.error != 0) {return std::string("drive fault: ") + cubemars::error_text(fb.status.error);}
  if (!heartbeat_ok()) {return "operator heartbeat missing";}
  if (th_.joinable()) {th_.join();}
  uint64_t id;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    id = st_.test_id + 1;
    st_ = TestStatus{};
    st_.running = true;
    st_.motor = m;
    st_.mode = mode;
    st_.value = value;
    st_.duration_s = duration;
    st_.result = "running";
    st_.test_id = id;
  }
  stop_ = false;
  th_ = std::thread([=] {run(id, m, mode, value, duration);});
  return "";
}

void MotorTester::stop(const std::string & why)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!st_.running) {return;}
    stop_why_ = why;
  }
  stop_ = true;
}

void MotorTester::wait()
{
  if (th_.joinable()) {th_.join();}
}

TestStatus MotorTester::status() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return st_;
}

void MotorTester::run(uint64_t id, std::size_t m, TestMode mode, double value, double duration)
{
  (void)id;
  const auto & c = bus_.motors()[m];
  bus_.enable_tx(true);
  const MotorFeedback fb0 = bus_.feedback(m);
  const double start_pos = c.raw_to_joint_pos(fb0.status.position_deg);
  // joint rad/s -> drive ERPM (inverse of MotorConfig::erpm_to_joint_vel)
  const double erpm = mode == TestMode::kVelocity ?
    value / c.direction * c.pole_pairs * c.gear_ratio * 60.0 / (2.0 * M_PI) : 0.0;
  const double vel_guard = mode == TestMode::kVelocity ? std::fabs(value) * 1.5 + 0.5 :
    velocity_limit(m) * 1.5;
  double peak_i = 0, peak_w = 0, w_sum = 0;
  int w_n = 0;
  std::string why = "done";
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / limits_.rate_hz));
  const auto t0 = std::chrono::steady_clock::now();
  auto next = t0;
  while (true) {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (t >= duration) {break;}
    if (stop_) {
      std::lock_guard<std::mutex> lk(mtx_);
      why = "stopped: " + stop_why_;
      break;
    }
    if (!heartbeat_ok()) {why = "operator heartbeat lost"; break;}
    const MotorFeedback fb = bus_.feedback(m);
    const double w = c.erpm_to_joint_vel(fb.status.speed_erpm);
    if ((mono_now_ns() - fb.mono_ns) * 1e-9 > c.feedback_stale_timeout_s) {why = "feedback stale"; break;}
    if (fb.status.error != 0) {why = std::string("drive fault: ") + cubemars::error_text(fb.status.error); break;}
    if (fb.status.temperature_c > c.max_temperature_c) {why = "over-temperature"; break;}
    if (std::fabs(w) > vel_guard) {why = "over-speed guard"; break;}
    peak_i = std::max(peak_i, std::fabs(fb.status.current_a));
    peak_w = std::max(peak_w, std::fabs(w));
    if (t > duration * 0.5) {w_sum += w; ++w_n;}
    const cubemars::Frame f = mode == TestMode::kCurrent ?
      cubemars::encode_current(c.can_id, value * std::min(1.0, t / limits_.current_ramp_s)) :
      cubemars::encode_rpm(c.can_id, erpm);
    std::string err;
    if (!bus_.send(f, err)) {why = "TX failed: " + err; break;}
    {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.elapsed_s = t;
      st_.peak_current_a = peak_i;
      st_.peak_velocity_rad_s = peak_w;
      st_.moved_rad = c.raw_to_joint_pos(fb.status.position_deg) - start_pos;
    }
    next += period;
    std::this_thread::sleep_until(next);
  }
  send_zero(m);
  const MotorFeedback fb1 = bus_.feedback(m);
  std::lock_guard<std::mutex> lk(mtx_);
  st_.running = false;
  st_.result = why;
  st_.moved_rad = c.raw_to_joint_pos(fb1.status.position_deg) - start_pos;
  st_.mean_velocity_rad_s = w_n ? w_sum / w_n : 0.0;
}

void MotorTester::send_zero(std::size_t m)
{
  if (!bus_.tx_enabled()) {return;}
  std::string err;
  for (int k = 0; k < 20; ++k) {
    bus_.send(cubemars::encode_current(bus_.motors()[m].can_id, 0.0), err);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

std::string MotorTester::zero_all()
{
  if (status().running) {stop("zero all"); wait();}
  bus_.enable_tx(true);
  for (std::size_t i = 0; i < bus_.motors().size(); ++i) {send_zero(i);}
  return "sent 0 A to all motors";
}

std::string MotorTester::set_temporary_origin(std::size_t m)
{
  if (m >= bus_.motors().size()) {return "no such motor";}
  if (status().running) {return "test running";}
  bus_.enable_tx(true);
  std::string err;
  return bus_.send(cubemars::encode_set_origin(bus_.motors()[m].can_id, 0), err) ?
         "temporary origin set" : "failed: " + err;
}

}  // namespace gen2_hardware
