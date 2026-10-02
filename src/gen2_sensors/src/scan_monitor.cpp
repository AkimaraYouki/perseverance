// Diagnostics "lidar: scan" from /scan (rate, valid points) for the LCD and bench check.
// Hot-swap: rplidar_node does not recover after the USB is unplugged and plugged back (it keeps
// running without data). The driver is restarted (SIGTERM, launch respawns it) only when
//  - there is no scan AND the device node was re-created (USB re-plug: new inode), or
//  - there is no scan for stall_restart_s (default 30 s) — a real driver hang.
// Short gaps are NOT restarts: on the campus Wi-Fi the address drops for ~0.1–5 s at every AP roam
// (every 2–5 min), DDS delivery stalls, and a driver restarted inside that window cannot even create
// its DDS participant ("wlP1p1s0: does not match an available interface") -> respawn loop.
#include <dirent.h>
#include <sys/stat.h>
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
    restart_after_s_ = declare_parameter("restart_after_s", 5.0);   // with a re-plugged device
    stall_restart_s_ = declare_parameter("stall_restart_s", 30.0);
    device_ = declare_parameter("device", std::string("/dev/gen2_lidar"));
    dev_id_ = device_id();
    startup_grace_s_ = declare_parameter("startup_grace_s", 15.0);  // driver spin-up time after a restart
    driver_ = declare_parameter("driver_process", std::string("rplidar_node"));
    last_scan_ = last_kill_ = std::chrono::steady_clock::now();
    watchdog_ = create_wall_timer(1s, [this] {
          const auto now = std::chrono::steady_clock::now();
          const double since = std::chrono::duration<double>(now - last_scan_).count();
          const double since_kill = std::chrono::duration<double>(now - last_kill_).count();
          const auto id = device_id();
          const bool replugged = id != 0 && id != dev_id_;
          if (since_kill > startup_grace_s_ &&
            ((replugged && since > restart_after_s_) || since > stall_restart_s_))
          {
            last_kill_ = now;
            dev_id_ = id;
            RCLCPP_WARN(get_logger(), "%s", replugged ? "LiDAR USB re-plugged" : "LiDAR stalled");
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

  // inode + change time of the tty behind the udev symlink; 0 when absent (unplugged)
  uint64_t device_id() const
  {
    struct stat st{};
    if (::stat(device_.c_str(), &st) != 0) {return 0;}
    return (static_cast<uint64_t>(st.st_ino) << 32) ^ static_cast<uint64_t>(st.st_ctim.tv_sec);
  }

  double restart_after_s_, startup_grace_s_, stall_restart_s_;
  std::string device_;
  uint64_t dev_id_ = 0;
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
