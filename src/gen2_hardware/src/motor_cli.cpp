// motor_cli — interactive bench tool for CubeMars servo-mode actuators (STEP 3/4/5).
//
// Safety model:
//   * Every motion command needs the typed confirmation "yes".
//   * Current  test: |I| <= min(motor current_limit_a, cli.max_test_current_a), <= cli.max_test_duration_s
//   * Velocity test: |w| <= min(motor velocity_limit_rad_s, cli.max_test_velocity_rad_s)
//   * 100 Hz command stream; aborts on Enter, Ctrl+C, stale feedback, drive fault,
//     over-temperature or over-speed; always ends with 0 A for 0.2 s.
//   * If this process is killed hard, the drive's own timeout (AppParams timeout_msec, 1000 ms on
//     AK45-10) stops the motor. A hardware E-stop is still required.

#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gen2_hardware/motor_bus.hpp"
#include "gen2_hardware/motor_params.hpp"
#include "gen2_hardware/motor_tester.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace gen2_hardware;
using namespace std::chrono_literals;

namespace
{
std::atomic<bool> g_sigint{false};
void on_sigint(int) {g_sigint = true;}

bool enter_pressed()
{
  pollfd p{STDIN_FILENO, POLLIN, 0};
  if (::poll(&p, 1, 0) > 0) {
    std::string line;
    std::getline(std::cin, line);
    return true;
  }
  return false;
}

double rad2deg(double r) {return r * 180.0 / M_PI;}
}  // namespace

class MotorCli
{
public:
  explicit MotorCli(rclcpp::Node & node)
  {
    const std::string ifname = node.declare_parameter("can_interface", std::string("can0"));
    max_i_ = node.declare_parameter("cli.max_test_current_a", 1.0);
    max_w_ = node.declare_parameter("cli.max_test_velocity_rad_s", 2.0);
    max_t_ = node.declare_parameter("cli.max_test_duration_s", 3.0);
    bus_ = std::make_unique<MotorBus>(ifname, declare_motor_params(node));
    bus_->start();
    TestLimits lim;
    lim.max_current_a = max_i_;
    lim.max_velocity_rad_s = max_w_;
    lim.max_duration_s = max_t_;
    tester_ = std::make_unique<MotorTester>(*bus_, lim);
  }

  ~MotorCli()
  {
    tester_.reset();  // stops any test and sends 0 A
    bus_->stop();
  }

  void run()
  {
    help();
    std::this_thread::sleep_for(300ms);
    status();
    std::string line;
    while (true) {
      std::cout << "\nmotor> " << std::flush;
      g_sigint = false;
      if (!std::getline(std::cin, line)) {break;}
      std::istringstream in(line);
      std::string cmd;
      in >> cmd;
      if (cmd.empty()) {continue;}
      if (cmd == "q" || cmd == "quit") {break;}
      if (cmd == "h" || cmd == "help") {help();}
      else if (cmd == "s") {status();}
      else if (cmd == "w") {watch();}
      else if (cmd == "r") {scale_check(arg_motor(in));}
      else if (cmd == "c") {
        std::size_t m = arg_motor(in);
        double a = 0, t = 0;
        in >> a >> t;
        current_test(m, a, t);
      } else if (cmd == "v") {
        std::size_t m = arg_motor(in);
        double w = 0, t = 0;
        in >> w >> t;
        velocity_test(m, w, t);
      } else if (cmd == "o") {set_origin(arg_motor(in));}
      else if (cmd == "x") {std::cout << tester_->zero_all() << "\n";}
      else {std::cout << "unknown command, 'h' for help\n";}
    }
  }

private:
  void help()
  {
    std::printf(
      "\n=== Gen2 motor test CLI  (CAN %s, %zu motor(s)) ===\n"
      "  s                     status snapshot\n"
      "  w                     live watch (Enter to stop)            [read-only]\n"
      "  r <motor>             position-scale check: turn output by hand [read-only]\n"
      "  c <motor> <A> <sec>   current test   |A| <= %.2f, sec <= %.1f\n"
      "  v <motor> <rad/s> <s> velocity test  |w| <= %.2f rad/s\n"
      "  o <motor>             set TEMPORARY origin here (cleared at power off)\n"
      "  x                     send 0 A to all motors\n"
      "  q                     quit\n"
      "  <motor> = index or name. Motion commands ask for 'yes'. Enter/Ctrl+C aborts.\n",
      bus_->ifname().c_str(), bus_->motors().size(), max_i_, max_t_, max_w_);
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
      const auto & c = bus_->motors()[i];
      std::printf("  [%zu] %-10s id %3u %-10s limit %.1f A  kt %s  verified:%s%s%s%s\n", i,
        c.name.c_str(), c.can_id, c.model.c_str(), c.current_limit_a,
        std::isfinite(c.kt_nm_per_a) ? std::to_string(c.kt_nm_per_a).c_str() : "?",
        c.verified_direction ? " dir" : "", c.verified_position_scale ? " pos" : "",
        c.verified_velocity_scale ? " vel" : "", c.verified_kt ? " kt" : "");
    }
  }

  std::size_t arg_motor(std::istream & in)
  {
    std::string s;
    in >> s;
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
      if (s == bus_->motors()[i].name || s == std::to_string(i)) {return i;}
    }
    return SIZE_MAX;
  }

  bool valid_motor(std::size_t m)
  {
    if (m >= bus_->motors().size()) {
      std::cout << "no such motor\n";
      return false;
    }
    return true;
  }

  std::string line_for(std::size_t i)
  {
    const auto & c = bus_->motors()[i];
    const MotorFeedback fb = bus_->feedback(i);
    char buf[256];
    if (!fb.valid) {
      std::snprintf(buf, sizeof(buf), "[%zu] %-10s NO FEEDBACK", i, c.name.c_str());
      return buf;
    }
    const double age = (mono_now_ns() - fb.mono_ns) * 1e-9;
    std::snprintf(buf, sizeof(buf),
      "[%zu] %-10s raw %8.1f deg  joint %8.2f deg  %7.2f rad/s  %6.2f A  %3d C  err %u(%s)%s",
      i, c.name.c_str(), fb.status.position_deg, rad2deg(c.raw_to_joint_pos(fb.status.position_deg)),
      c.erpm_to_joint_vel(fb.status.speed_erpm), fb.status.current_a, fb.status.temperature_c,
      fb.status.error, cubemars::error_text(fb.status.error),
      age > c.feedback_stale_timeout_s ? "  STALE" : "");
    return buf;
  }

  void status()
  {
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {std::cout << line_for(i) << "\n";}
    const auto & st = bus_->stats();
    std::printf("bus: rx %lu  unknown %lu  error-frames %lu  tx %lu  tx-err %lu\n",
      st.rx_frames.load(), st.rx_unknown.load(), st.rx_error_frames.load(), st.tx_frames.load(),
      st.tx_errors.load());
  }

  void watch()
  {
    std::cout << "(Enter to stop)\n";
    while (!enter_pressed() && !g_sigint) {
      for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
        std::cout << "\r" << line_for(i) << "   " << (bus_->motors().size() > 1 ? "\n" : "");
      }
      std::cout << std::flush;
      std::this_thread::sleep_for(100ms);
      if (bus_->motors().size() > 1) {std::printf("\033[%zuA", bus_->motors().size());}
    }
    std::cout << "\n";
  }

  void scale_check(std::size_t m)
  {
    if (!valid_motor(m)) {return;}
    const auto & c = bus_->motors()[m];
    MotorFeedback fb = bus_->feedback(m);
    if (!fb.valid) {std::cout << "no feedback\n"; return;}
    // Track unwrapped raw position (int16 report wraps at +-3200 deg).
    double prev = fb.status.position_deg, total = 0.0;
    std::cout << "Mark the output shaft, rotate it by hand exactly N full turns (positive = the\n"
                 "direction you want as +), then press Enter.\n";
    while (!enter_pressed() && !g_sigint) {
      fb = bus_->feedback(m);
      double d = fb.status.position_deg - prev;
      if (d > 3200.0) {d -= 6400.0;}
      if (d < -3200.0) {d += 6400.0;}
      total += d;
      prev = fb.status.position_deg;
      std::printf("\r  raw delta %9.1f deg   (joint delta with current config %8.1f deg)   ",
        total, total / c.raw_deg_per_output_rev * 360.0 * c.direction);
      std::fflush(stdout);
      std::this_thread::sleep_for(20ms);
    }
    std::cout << "\nHow many output turns did you make (e.g. 1, -1)? " << std::flush;
    std::string s;
    std::getline(std::cin, s);
    double n = 0;
    try {n = std::stod(s);} catch (...) {}
    if (std::fabs(n) < 0.5) {std::cout << "skipped\n"; return;}
    const double per_rev = std::fabs(total / n);
    std::printf("=> raw_deg_per_output_rev: %.1f   (config %.1f)\n", per_rev, c.raw_deg_per_output_rev);
    std::printf("=> direction: %d   (raw increased when you turned the + way: %s)\n",
      total * n > 0 ? 1 : -1, total * n > 0 ? "yes" : "no");
    std::cout << "Put these in config/motors.yaml and set verified.position_scale / direction: true.\n";
  }

  bool confirm(const std::string & what)
  {
    std::cout << what << "\nRobot lifted / wheel free / hand on E-stop? type 'yes': " << std::flush;
    std::string st;
    std::getline(std::cin, st);
    return st == "yes";
  }

  void run_test(std::size_t m, TestMode mode, double value, double sec)
  {
    if (!valid_motor(m)) {return;}
    const auto & c = bus_->motors()[m];
    char q[200];
    if (mode == TestMode::kCurrent) {
      std::snprintf(q, sizeof(q), "CURRENT TEST %s: %.2f A for %.1f s (limit %.2f A)", c.name.c_str(),
        value, sec, tester_->current_limit(m));
    } else {
      std::snprintf(q, sizeof(q), "VELOCITY TEST %s: %.2f rad/s for %.1f s (limit %.2f rad/s)",
        c.name.c_str(), value, sec, tester_->velocity_limit(m));
    }
    if (!confirm(q)) {std::cout << "cancelled\n"; return;}
    const std::string err = tester_->start(m, mode, value, sec);
    if (!err.empty()) {std::cout << "rejected: " << err << "\n"; return;}
    while (tester_->status().running) {
      if (g_sigint) {tester_->stop("Ctrl+C");}
      if (enter_pressed()) {tester_->stop("Enter");}
      std::printf("\r  t %4.2fs  %s   ", tester_->status().elapsed_s, line_for(m).c_str());
      std::fflush(stdout);
      std::this_thread::sleep_for(50ms);
    }
    tester_->wait();
    const TestStatus st = tester_->status();
    std::printf("\n  end: %s\n  joint moved %.1f deg, peak |I| %.2f A, peak |w| %.2f rad/s, "
      "mean w (2nd half) %.3f rad/s\n", st.result.c_str(), rad2deg(st.moved_rad), st.peak_current_a,
      st.peak_velocity_rad_s, st.mean_velocity_rad_s);
    if (mode == TestMode::kCurrent) {
      std::cout << "  direction check: + current should move the joint in the direction you define as +.\n";
    } else {
      std::printf("  velocity-scale check: commanded %.3f rad/s, position-derived mean %.3f rad/s\n"
        "  (includes accel/decel; ratio far from 1 => pole_pairs/gear/raw_deg_per_output_rev wrong)\n",
        value, st.moved_rad / sec);
    }
  }

  void current_test(std::size_t m, double amps, double sec) {run_test(m, TestMode::kCurrent, amps, sec);}
  void velocity_test(std::size_t m, double w, double sec) {run_test(m, TestMode::kVelocity, w, sec);}

  void set_origin(std::size_t m)
  {
    if (!valid_motor(m)) {return;}
    std::cout << "Set TEMPORARY origin on " << bus_->motors()[m].name
              << " (mode 0, cleared at power off). type 'yes': " << std::flush;
    std::string st;
    std::getline(std::cin, st);
    if (st != "yes") {std::cout << "cancelled\n"; return;}
    std::cout << tester_->set_temporary_origin(m) << "\n";
  }

  std::unique_ptr<MotorBus> bus_;
  std::unique_ptr<MotorTester> tester_;
  double max_i_, max_w_, max_t_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_sigint);
  int rc = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("motor_cli");
    MotorCli cli(*node);
    cli.run();
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n";
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
