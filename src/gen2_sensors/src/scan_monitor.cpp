// Diagnostics "lidar: scan" from /scan (rate, valid points) for the LCD and bench check.
#include <chrono>
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
          valid_ = 0;
          for (float r : m->ranges) {if (std::isfinite(r) && r >= m->range_min) {++valid_;}}
          total_ = m->ranges.size();
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
