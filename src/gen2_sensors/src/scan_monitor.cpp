// Diagnostics "lidar: scan" from /scan (rate, valid points) for the LCD and bench check.
// Hot-swap: rplidar_node does not recover after the USB is unplugged and plugged back (it keeps
// running without data). With no scan for restart_after_s, SIGTERM the driver (same user) and the
// launch respawns it; repeated every restart_after_s while there is no data.
#include <dirent.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdio>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class ScanMonitor : public rclcpp::Node
{
public:
  ScanMonitor()
  : Node("scan_monitor")
  {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
          ++n_;
          last_scan_ = std::chrono::steady_clock::now();
          valid_ = 0;
          for (float r : m->ranges) {if (std::isfinite(r) && r >= m->range_min) {++valid_;}}
          total_ = m->ranges.size();
        });
    restart_after_s_ = declare_parameter("restart_after_s", 5.0);
    driver_ = declare_parameter("driver_process", std::string("rplidar_node"));
    last_scan_ = last_kill_ = std::chrono::steady_clock::now();
    watchdog_ = create_wall_timer(1s, [this] {
          const auto now = std::chrono::steady_clock::now();
          const double since = std::chrono::duration<double>(now - last_scan_).count();
          const double since_kill = std::chrono::duration<double>(now - last_kill_).count();
          if (since > restart_after_s_ && since_kill > restart_after_s_) {
            last_kill_ = now;
            restart_driver();
          }
        });
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(2s, [this] {
          diagnostic_msgs::msg::DiagnosticStatus st;
          st.name = "lidar: scan";
          st.hardware_id = "rplidar_c1";
          const double hz = n_ / 2.0;
          char b[40];
          std::snprintf(b, sizeof(b), "%.0fHz %zu pts", hz, valid_);
          st.message = hz > 0 ? b : "no scans";
          st.level = hz >= 5.0 && valid_ > 50 ? diagnostic_msgs::msg::DiagnosticStatus::OK :
          hz > 0 ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          diagnostic_msgs::msg::KeyValue kv;
          kv.key = "points_total"; kv.value = std::to_string(total_); st.values.push_back(kv);
          n_ = 0;
          diagnostic_msgs::msg::DiagnosticArray a;
          a.header.stamp = now();
          a.status.push_back(st);
          diag_->publish(a);
        });
  }

private:
  void restart_driver()
  {
    DIR * d = opendir("/proc");
    if (!d) {return;}
    while (dirent * e = readdir(d)) {
      const int pid = std::atoi(e->d_name);
      if (pid <= 0) {continue;}
      std::ifstream f(std::string("/proc/") + e->d_name + "/comm");
      std::string comm;
      std::getline(f, comm);
      if (comm == driver_.substr(0, 15) && ::kill(pid, SIGTERM) == 0) {
        RCLCPP_WARN(get_logger(), "no scan for %.0f s: restarting %s (pid %d)", restart_after_s_,
          driver_.c_str(), pid);
      }
    }
    closedir(d);
  }

  double restart_after_s_;
  std::string driver_;
  std::chrono::steady_clock::time_point last_scan_, last_kill_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  int n_ = 0;
  std::size_t valid_ = 0, total_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMonitor>());
  rclcpp::shutdown();
  return 0;
}
