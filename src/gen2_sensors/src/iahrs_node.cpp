// iAHRS RB-SDA-v1 driver. The sensor is configured once (saved in its flash, see config/iahrs.yaml
// comment): b2=921600, so=1, sp=2 (500 Hz), sd=0x8D -> each line is
//   count_ms, ax, ay, az [g], gx, gy, gz [deg/s], qw, qx, qy, qz
// A dedicated thread reads the port (no executor latency) and publishes sensor_msgs/Imu in the
// two topics:
//   imu/data_raw  SENSOR frame (frame_id imu_link)
//   imu/data      BODY frame (frame_id base_link, REP-103 x fwd / y left / z up): accel and gyro
//                 rotated by the mounting rotation mount_rpy_deg (imu_link in base_link, from
//                 `ros2 run gen2_tools imu_cal`), orientation q_world_body = q_world_sensor * q_body_sensor^-1.
// Stamp = Jetson
// receive time (steady line arrival). Diagnostics "imu: iahrs": rate, count gaps, max RX gap.
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class IahrsNode : public rclcpp::Node
{
public:
  IahrsNode()
  : Node("iahrs")
  {
    port_ = declare_parameter("port", std::string("/dev/gen2_imu"));
    frame_ = declare_parameter("frame_id", std::string("imu_link"));
    expected_hz_ = declare_parameter("expected_rate_hz", 500.0);
    body_frame_ = declare_parameter("body_frame_id", std::string("base_link"));
    const auto rpy = declare_parameter("mount_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
    if (rpy.size() != 3) {throw std::runtime_error("mount_rpy_deg needs 3 values");}
    set_mount(rpy[0] * M_PI / 180.0, rpy[1] * M_PI / 180.0, rpy[2] * M_PI / 180.0);
    RCLCPP_INFO(get_logger(), "mount rpy %.2f %.2f %.2f deg (imu_link in %s)", rpy[0], rpy[1], rpy[2],
      body_frame_.c_str());
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(1s, [this] {report();});
    th_ = std::thread([this] {loop();});
  }
  ~IahrsNode() override
  {
    run_ = false;
    if (th_.joinable()) {th_.join();}
    if (fd_ >= 0) {::close(fd_);}
  }

private:
  // R = Rz(yaw) Ry(pitch) Rx(roll): v_body = R v_sensor; q_bs the same rotation as a quaternion
  void set_mount(double r, double p, double y)
  {
    const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    R_[0][0] = cy * cp; R_[0][1] = cy * sp * sr - sy * cr; R_[0][2] = cy * sp * cr + sy * sr;
    R_[1][0] = sy * cp; R_[1][1] = sy * sp * sr + cy * cr; R_[1][2] = sy * sp * cr - cy * sr;
    R_[2][0] = -sp;     R_[2][1] = cp * sr;                R_[2][2] = cp * cr;
    const double hr = r / 2, hp = p / 2, hy = y / 2;
    qbs_[0] = std::cos(hr) * std::cos(hp) * std::cos(hy) + std::sin(hr) * std::sin(hp) * std::sin(hy);
    qbs_[1] = std::sin(hr) * std::cos(hp) * std::cos(hy) - std::cos(hr) * std::sin(hp) * std::sin(hy);
    qbs_[2] = std::cos(hr) * std::sin(hp) * std::cos(hy) + std::sin(hr) * std::cos(hp) * std::sin(hy);
    qbs_[3] = std::cos(hr) * std::cos(hp) * std::sin(hy) - std::sin(hr) * std::sin(hp) * std::cos(hy);
  }
  void rot(const double in[3], double out[3]) const
  {
    for (int i = 0; i < 3; ++i) {out[i] = R_[i][0] * in[0] + R_[i][1] * in[1] + R_[i][2] * in[2];}
  }

  bool open_port()
  {
    fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {return false;}
    termios t{};
    tcgetattr(fd_, &t);
    cfmakeraw(&t);
    cfsetispeed(&t, B921600);
    cfsetospeed(&t, B921600);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 2;  // 0.2 s read timeout
    tcsetattr(fd_, TCSANOW, &t);
    tcflush(fd_, TCIFLUSH);
    return true;
  }

  void loop()
  {
    std::string line;
    char buf[512];
    while (run_ && rclcpp::ok()) {
      if (fd_ < 0 && !open_port()) {
        set_err("cannot open " + port_);
        std::this_thread::sleep_for(1s);
        continue;
      }
      const ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n < 0) {
        set_err("read error, reopening");
        ::close(fd_);
        fd_ = -1;
        std::this_thread::sleep_for(500ms);
        continue;
      }
      const auto now = std::chrono::steady_clock::now();
      if (n == 0) {   // USB unplugged often gives silent reads, not an error: reopen after 2 s
        if (now - last_data_ > 2s) {
          set_err("no data, reopening " + port_);
          ::close(fd_);
          fd_ = -1;
          last_data_ = now;
        }
        continue;
      }
      last_data_ = now;
      for (ssize_t i = 0; i < n; ++i) {
        if (buf[i] == '\n') {handle(line, now); line.clear();}
        else if (buf[i] != '\r' && line.size() < 256) {line.push_back(buf[i]);}
      }
    }
  }

  void handle(const std::string & l, std::chrono::steady_clock::time_point rx)
  {
    long cnt;
    double a[3], g[3], q[4];
    if (std::sscanf(l.c_str(), "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &cnt, &a[0], &a[1],
      &a[2], &g[0], &g[1], &g[2], &q[0], &q[1], &q[2], &q[3]) != 11)
    {
      std::lock_guard<std::mutex> lk(m_);
      ++parse_err_;
      return;
    }
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_;
    msg.orientation.w = q[0];
    msg.orientation.x = q[1];
    msg.orientation.y = q[2];
    msg.orientation.z = q[3];
    constexpr double d2r = M_PI / 180.0, g0 = 9.80665;
    double w_s[3] = {g[0] * d2r, g[1] * d2r, g[2] * d2r}, a_s[3] = {a[0] * g0, a[1] * g0, a[2] * g0};
    msg.angular_velocity.x = w_s[0];
    msg.angular_velocity.y = w_s[1];
    msg.angular_velocity.z = w_s[2];
    msg.linear_acceleration.x = a_s[0];
    msg.linear_acceleration.y = a_s[1];
    msg.linear_acceleration.z = a_s[2];
    raw_pub_->publish(msg);
    // body frame
    double w_b[3], a_b[3];
    rot(w_s, w_b);
    rot(a_s, a_b);
    msg.header.frame_id = body_frame_;
    msg.angular_velocity.x = w_b[0];
    msg.angular_velocity.y = w_b[1];
    msg.angular_velocity.z = w_b[2];
    msg.linear_acceleration.x = a_b[0];
    msg.linear_acceleration.y = a_b[1];
    msg.linear_acceleration.z = a_b[2];
    // q_wb = q_ws * conj(q_bs)
    const double aw = q[0], ax = q[1], ay = q[2], az = q[3];
    const double bw = qbs_[0], bx = -qbs_[1], by = -qbs_[2], bz = -qbs_[3];
    msg.orientation.w = aw * bw - ax * bx - ay * by - az * bz;
    msg.orientation.x = aw * bx + ax * bw + ay * bz - az * by;
    msg.orientation.y = aw * by - ax * bz + ay * bw + az * bx;
    msg.orientation.z = aw * bz + ax * by - ay * bx + az * bw;
    pub_->publish(msg);
    std::lock_guard<std::mutex> lk(m_);
    if (have_last_) {
      const long dc = cnt - last_cnt_;
      if (dc > expected_dc_ * 1.5) {gaps_ += 1;}
      max_gap_ms_ = std::max(max_gap_ms_,
          std::chrono::duration<double, std::milli>(rx - last_rx_).count());
    }
    have_last_ = true;
    last_cnt_ = cnt;
    last_rx_ = rx;
    ++lines_;
    err_.clear();
  }

  void set_err(const std::string & e) {std::lock_guard<std::mutex> lk(m_); err_ = e;}

  void report()
  {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "imu: iahrs";
    st.hardware_id = port_;
    double hz;
    {
      std::lock_guard<std::mutex> lk(m_);
      hz = static_cast<double>(lines_);
      auto kv = [&st](const std::string & k, const std::string & v) {
          diagnostic_msgs::msg::KeyValue x; x.key = k; x.value = v; st.values.push_back(x);
        };
      kv("rate_hz", std::to_string(lines_));
      kv("count_gaps", std::to_string(gaps_total_ += gaps_));
      kv("max_rx_gap_ms", std::to_string(max_gap_ms_));
      kv("parse_errors", std::to_string(parse_err_));
      if (hz < 0.5 * expected_hz_) {
        st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        st.message = err_.empty() ? "no data" : err_;
      } else if (gaps_ > 0 || hz < 0.9 * expected_hz_ || max_gap_ms_ > 20.0) {
        st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        char b[48]; std::snprintf(b, sizeof(b), "%.0fHz gaps %ld", hz, gaps_); st.message = b;
      } else {
        st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        char b[32]; std::snprintf(b, sizeof(b), "%.0fHz", hz); st.message = b;
      }
      lines_ = 0; gaps_ = 0; max_gap_ms_ = 0.0;
    }
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    diag_->publish(arr);
  }

  std::string port_, frame_, body_frame_;
  double R_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double qbs_[4] = {1, 0, 0, 0};
  double expected_hz_;
  long expected_dc_ = 2;
  int fd_ = -1;
  std::atomic<bool> run_{true};
  std::thread th_;
  std::mutex m_;
  long lines_ = 0, gaps_ = 0, gaps_total_ = 0, parse_err_ = 0, last_cnt_ = 0;
  bool have_last_ = false;
  double max_gap_ms_ = 0.0;
  std::chrono::steady_clock::time_point last_rx_, last_data_{std::chrono::steady_clock::now()};
  std::string err_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_, raw_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IahrsNode>());
  rclcpp::shutdown();
  return 0;
}
