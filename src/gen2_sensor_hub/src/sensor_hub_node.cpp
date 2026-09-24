// ESP32-C3 sensor hub driver: HostFrame v1 over USB CDC -> ROS 2 topics + diagnostics.
//
// Not part of any control loop. GNSS / compass / power / diagnostics only.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "gen2_msgs/msg/gnss_pvt.hpp"
#include "gen2_sensor_hub/frame_parser.hpp"
#include "gen2_sensor_hub/serial_port.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace gen2_sensor_hub
{

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Lower-envelope estimate of (host_rx - esp_time): transport delay is always >= 0, so the
// minimum over a sliding window tracks the clock offset plus the fastest-path latency.
class OffsetEstimator
{
public:
  explicit OffsetEstimator(int64_t window_ns)
  : window_ns_(window_ns) {}
  void reset() {has_ = false;}
  int64_t update(int64_t sample_ns, int64_t now_ns)
  {
    if (!has_ || now_ns - bucket_start_ns_ > window_ns_) {
      prev_min_ = has_ ? cur_min_ : sample_ns;
      cur_min_ = sample_ns;
      bucket_start_ns_ = now_ns;
      has_ = true;
    } else {
      cur_min_ = std::min(cur_min_, sample_ns);
    }
    return std::min(prev_min_, cur_min_);
  }

private:
  int64_t window_ns_;
  bool has_ = false;
  int64_t bucket_start_ns_ = 0;
  int64_t cur_min_ = 0;
  int64_t prev_min_ = 0;
};

struct RateMeter
{
  std::deque<int64_t> t;
  void tick(int64_t now) {t.push_back(now); trim(now);}
  void trim(int64_t now) {while (!t.empty() && now - t.front() > 2'000'000'000LL) {t.pop_front();}}
  double hz(int64_t now) {trim(now); return t.size() / 2.0;}
};

double lipo_percentage(double cell_v)
{
  // Rough open-circuit LiPo curve (V/cell -> fraction). Under load this under-reads.
  static const double v[] = {3.30, 3.50, 3.60, 3.70, 3.75, 3.80, 3.85, 3.95, 4.05, 4.20};
  static const double s[] = {0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.70, 0.85, 1.00};
  if (!(cell_v > v[0])) {return 0.0;}
  for (int i = 1; i < 10; ++i) {
    if (cell_v <= v[i]) {
      return s[i - 1] + (s[i] - s[i - 1]) * (cell_v - v[i - 1]) / (v[i] - v[i - 1]);
    }
  }
  return 1.0;
}
}  // namespace

class SensorHubNode : public rclcpp::Node
{
public:
  struct BatteryCfg
  {
    std::string name;
    int cells;
    double capacity_ah;
    double voltage_scale;
    double current_offset_a;
    double current_scale;
    double min_present_cell_v;
    double max_present_cell_v;
    double warn_cell_v;
    double error_cell_v;
    double nominal_cell_v;
    double reserve_fraction;
    double power_tau_s;
    double soc_settle_s;
    bool jetson_current_fallback;  // compute only: derive current from Jetson INA3221 VDD_IN
    double jetson_power_factor;    // VDD_IN power -> battery power (1.0 = as Jetson reports)
    double pm02_min_current_a;     // below this the PM02/ESP32 ADC reading is not trusted
  };

  // Coulomb-counting state of charge + runtime estimate for one pack.
  struct Fuel
  {
    bool present = false;
    int64_t present_since_ns = 0;
    int64_t absent_since_ns = 0;
    bool power_estimated = false;
    bool initialized = false;
    double soc0 = 0.0;          // SoC at initialization (from resting voltage)
    double v_settle_sum = 0.0;
    int v_settle_n = 0;
    double consumed_ah = 0.0;
    double p_avg_w = 0.0;
    uint64_t resets = 0;
  };

  SensorHubNode()
  : Node("sensor_hub"), updater_(this), offset_(declare_parameter("clock_offset_window_s", 5.0) * 1e9)
  {
    port_ = declare_parameter("port", std::string("auto"));
    by_id_patterns_ = declare_parameter(
      "by_id_patterns", std::vector<std::string>{"/dev/serial/by-id/usb-Espressif*"});
    allow_acm_fallback_ = declare_parameter("allow_acm_fallback", false);
    reconnect_period_s_ = declare_parameter("reconnect_period_s", 1.0);
    stale_timeout_s_ = declare_parameter("stale_timeout_s", 0.5);
    probe_timeout_s_ = declare_parameter("probe_timeout_s", 2.0);
    gps_frame_ = declare_parameter("gps_frame_id", std::string("gps_link"));
    mag_frame_ = declare_parameter("mag_frame_id", std::string("gps_link"));
    mag_scale_ = declare_parameter("mag_tesla_per_lsb", 3.0e-7);  // IST8310: 0.3 uT/LSB
    mag_var_ = declare_parameter("mag_variance_t2", 0.0);
    min_itow_step_ms_ = declare_parameter("gps_min_itow_step_ms", 1);

    compute_ = load_battery("compute", 3, 3.3);
    motor_ = load_battery("motor", 6, 3.3);

    pub_compute_ = create_publisher<sensor_msgs::msg::BatteryState>("power/compute", 10);
    pub_motor_ = create_publisher<sensor_msgs::msg::BatteryState>("power/motor", 10);
    pub_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    pub_vel_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gps/velocity", 10);
    pub_pvt_ = create_publisher<gen2_msgs::msg::GnssPvt>("gps/nav_pvt", 10);
    pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>("gps/mag", 10);

    updater_.setHardwareID("esp32c3_sensor_hub");
    updater_.add("sensor_hub: link", this, &SensorHubNode::diag_link);
    updater_.add("sensor_hub: power compute", [this](auto & s) {diag_power(s, compute_, 0);});
    updater_.add("sensor_hub: power motor", [this](auto & s) {diag_power(s, motor_, 1);});
    updater_.add("sensor_hub: gnss", this, &SensorHubNode::diag_gnss);
    updater_.add("sensor_hub: magnetometer", this, &SensorHubNode::diag_mag);
    updater_.setPeriod(1.0);

    find_jetson_ina();
    ina_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {read_jetson_ina();});
    parser_ = std::make_unique<FrameParser>([this](const HostFrame & f) {on_frame(f);});
    reader_ = std::thread([this] {reader_loop();});
  }

  ~SensorHubNode() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
  }

private:
  BatteryCfg load_battery(const std::string & n, int cells, double cap)
  {
    BatteryCfg b;
    b.name = n;
    b.cells = declare_parameter(n + ".cells", cells);
    b.capacity_ah = declare_parameter(n + ".capacity_ah", cap);
    b.voltage_scale = declare_parameter(n + ".voltage_scale", 1.0);
    b.current_offset_a = declare_parameter(n + ".current_offset_a", 0.0);
    b.current_scale = declare_parameter(n + ".current_scale", 1.0);
    b.min_present_cell_v = declare_parameter(n + ".min_present_cell_v", 2.8);
    b.max_present_cell_v = declare_parameter(n + ".max_present_cell_v", 4.35);
    b.warn_cell_v = declare_parameter(n + ".warn_cell_v", 3.50);
    b.error_cell_v = declare_parameter(n + ".error_cell_v", 3.30);
    b.nominal_cell_v = declare_parameter(n + ".nominal_cell_v", 3.70);
    b.reserve_fraction = declare_parameter(n + ".reserve_fraction", 0.20);
    b.power_tau_s = declare_parameter(n + ".power_avg_tau_s", 30.0);
    b.soc_settle_s = declare_parameter(n + ".soc_init_settle_s", 2.0);
    b.jetson_current_fallback = declare_parameter(n + ".jetson_current_fallback", false);
    b.jetson_power_factor = declare_parameter(n + ".jetson_power_factor", 1.0);
    b.pm02_min_current_a = declare_parameter(n + ".pm02_min_current_a", 1.5);
    return b;
  }

  // Jetson module input power (INA3221 channel labelled VDD_IN), read at 5 Hz from sysfs.
  void find_jetson_ina()
  {
    for (int h = 0; h < 16; ++h) {
      const std::string d = "/sys/class/hwmon/hwmon" + std::to_string(h) + "/";
      if (read_line(d + "name") != "ina3221") {continue;}
      for (int i = 1; i <= 3; ++i) {
        if (read_line(d + "in" + std::to_string(i) + "_label") == "VDD_IN") {
          ina_v_path_ = d + "in" + std::to_string(i) + "_input";
          ina_i_path_ = d + "curr" + std::to_string(i) + "_input";
        }
      }
    }
    if (ina_v_path_.empty()) {
      RCLCPP_WARN(get_logger(), "Jetson INA3221 VDD_IN not found; compute power floor disabled");
    }
  }
  static std::string read_line(const std::string & p)
  {
    std::ifstream f(p);
    std::string s;
    std::getline(f, s);
    return s;
  }
  void read_jetson_ina()
  {
    if (ina_v_path_.empty()) {return;}
    try {
      const double w = std::stod(read_line(ina_v_path_)) * 1e-3 * std::stod(read_line(ina_i_path_)) * 1e-3;
      jetson_w_ = w;
    } catch (...) {
      jetson_w_ = std::numeric_limits<double>::quiet_NaN();
    }
  }
  // Effective pack current. The ESP32-C3 ADC cannot resolve PM02 current near 0 A
  // (27.5 mV/A), so for the compute pack a low PM02 reading is replaced by
  // Jetson VDD_IN power (same value as jetson power GUI / INA3221) x factor / pack voltage.
  double pack_current(const BatteryCfg & c, double v, double a_pm02, bool & estimated) const
  {
    estimated = false;
    const double j = jetson_w_.load();
    if (c.jetson_current_fallback && std::isfinite(j) && v > 1.0 && a_pm02 < c.pm02_min_current_a) {
      estimated = true;
      return j * c.jetson_power_factor / v;
    }
    return a_pm02;
  }

  static double cal_voltage(const BatteryCfg & c, uint16_t mv) {return mv * 1e-3 * c.voltage_scale;}
  static double cal_current(const BatteryCfg & c, int32_t ma)
  {
    return (ma * 1e-3 - c.current_offset_a) * c.current_scale;
  }
  static bool present(const BatteryCfg & c, double v)
  {
    const double cell = v / c.cells;
    return cell >= c.min_present_cell_v && cell <= c.max_present_cell_v;
  }

  // ---------------------------------------------------------------- reader thread
  void reader_loop()
  {
    std::vector<uint8_t> buf(4096);
    std::size_t candidate = 0;
    while (running_ && rclcpp::ok()) {
      if (!port_io_.is_open()) {
        if (!try_open(candidate)) {
          std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_period_s_));
          continue;
        }
      }
      const long n = port_io_.read_some(buf.data(), buf.size(), 50);
      const int64_t now = steady_ns();
      if (n < 0) {
        disconnect("device read error / unplugged");
        continue;
      }
      if (n > 0) {
        std::lock_guard<std::mutex> lk(mtx_);
        bytes_rx_ += static_cast<uint64_t>(n);
      }
      if (n > 0) {
        parser_->feed(buf.data(), static_cast<std::size_t>(n));
      }
      const int64_t last = last_frame_ns_.load();
      const double since_open = (now - opened_ns_) * 1e-9;
      if (last < opened_ns_) {
        if (since_open > probe_timeout_s_) {
          ++candidate;  // try the next candidate port
          disconnect("no valid HostFrame within probe timeout");
        }
      } else if ((now - last) * 1e-9 > std::max(stale_timeout_s_, 0.05) * 4.0) {
        // Port still open but stream stopped (ESP32 reset/hang): reopen.
        disconnect("stream stalled");
      }
    }
    port_io_.close();
  }

  bool try_open(std::size_t & candidate)
  {
    std::vector<std::string> ports = discover_ports(port_, by_id_patterns_);
    if (port_ == "auto" && !allow_acm_fallback_) {
      ports.erase(std::remove_if(ports.begin(), ports.end(), [](const std::string & p) {
          return p.rfind("/dev/serial/by-id/", 0) != 0;
        }), ports.end());
    }
    if (ports.empty()) {
      set_link_status("no candidate port found");
      return false;
    }
    const std::string path = ports[candidate % ports.size()];
    std::string err;
    if (!port_io_.open(path, err)) {
      ++candidate;
      set_link_status("open failed: " + err);
      return false;
    }
    parser_->reset();
    opened_ns_ = steady_ns();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ++connects_;
      link_path_ = path;
    }
    set_link_status("opened");
    RCLCPP_INFO(get_logger(), "Opened %s", path.c_str());
    return true;
  }

  void disconnect(const std::string & why)
  {
    if (port_io_.is_open()) {
      RCLCPP_WARN(get_logger(), "Closing %s: %s", port_io_.path().c_str(), why.c_str());
    }
    port_io_.close();
    set_link_status(why);
    std::lock_guard<std::mutex> lk(mtx_);
    have_seq_ = false;  // next frame re-establishes sequence tracking
    ++disconnects_;
  }

  void set_link_status(const std::string & s)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    link_status_ = s;
  }

  // ---------------------------------------------------------------- frame handling
  void on_frame(const HostFrame & f)
  {
    const int64_t now_steady = steady_ns();
    const rclcpp::Time now_ros = now();
    last_frame_ns_ = now_steady;

    const int64_t esp_ns = static_cast<int64_t>(f.esp_time_us) * 1000;
    bool esp_reset = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (have_seq_) {
        if (f.sequence == last_seq_ + 1) {
        } else if (f.sequence > last_seq_ + 1) {
          seq_drops_ += f.sequence - last_seq_ - 1;
        } else if (f.esp_time_us < last_esp_us_) {
          drops_at_connect_ = f.usb_drop_count;
          esp_reset = true;
          ++esp_resets_;
        } else {
          ++seq_out_of_order_;
        }
      }
      if (!have_seq_) {drops_at_connect_ = f.usb_drop_count;}
      have_seq_ = true;
      last_seq_ = f.sequence;
      last_esp_us_ = f.esp_time_us;
      // GPS UART byte-rate (16-bit wrapping counter), updated about once per second
      if (f.esp_time_us - gps_rate_t_us_ >= 1000000ULL) {
        const uint16_t d = static_cast<uint16_t>(f.gps_rx_bytes_lo16 - gps_rate_last_);
        gps_uart_rate_ = gps_rate_t_us_ ? d / ((f.esp_time_us - gps_rate_t_us_) * 1e-6) : 0.0;
        gps_rate_last_ = f.gps_rx_bytes_lo16;
        gps_rate_t_us_ = f.esp_time_us;
      }
      last_ = f;
      frame_rate_.tick(now_steady);
    }
    if (esp_reset) {
      offset_.reset();
      have_itow_ = false;
      RCLCPP_WARN(get_logger(), "ESP32 reset detected (esp_time went backwards)");
    }

    // Map ESP32 monotonic time onto host clocks.
    const int64_t offset = offset_.update(now_steady - esp_ns, now_steady);
    const int64_t sample_steady = esp_ns + offset;
    const int64_t latency_ns = std::max<int64_t>(0, now_steady - sample_steady);
    const rclcpp::Time stamp = now_ros - rclcpp::Duration::from_nanoseconds(latency_ns);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latency_s_.push_back(latency_ns * 1e-9);
      if (latency_s_.size() > 500) {latency_s_.pop_front();}
    }

    publish_battery(stamp, compute_, f.compute_mV, f.compute_mA, pub_compute_);
    publish_battery(stamp, motor_, f.motor_mV, f.motor_mA, pub_motor_);
    integrate_energy(f, now_steady);
    publish_mag(f, stamp);
    publish_gps(f, stamp);
  }

  void publish_battery(
    const rclcpp::Time & stamp, const BatteryCfg & cfg, uint16_t mv, int32_t ma,
    const rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr & pub)
  {
    sensor_msgs::msg::BatteryState m;
    m.header.stamp = stamp;
    m.header.frame_id = "base_link";
    const double volts = cal_voltage(cfg, mv);
    const bool valid = present(cfg, volts);
    m.voltage = static_cast<float>(volts);
    bool est = false;
    const double amps = pack_current(cfg, volts, cal_current(cfg, ma), est);
    m.current = static_cast<float>(-amps);  // ROS convention: negative while discharging
    m.temperature = std::numeric_limits<float>::quiet_NaN();
    FuelOut fo;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      fo = fuel_out(&cfg == &compute_ ? fuel_[0] : fuel_[1], cfg);
    }
    m.charge = static_cast<float>(fo.remaining_ah);            // Ah remaining (coulomb count)
    m.capacity = static_cast<float>(cfg.capacity_ah);
    m.design_capacity = static_cast<float>(cfg.capacity_ah);
    m.percentage = static_cast<float>(fo.soc);                 // 0..1, NaN until initialized
    m.power_supply_status = valid ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING :
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    m.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    if (valid && m.voltage / cfg.cells < cfg.error_cell_v) {
      m.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else if (valid) {
      m.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    m.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    m.present = valid;
    m.location = cfg.name;
    pub->publish(m);
  }

  void publish_mag(const HostFrame & f, const rclcpp::Time & stamp)
  {
    if (!(f.flags & FLAG_MAG_VALID)) {
      return;
    }
    // Firmware sends the latest 50 Hz sample in every 100 Hz frame without its own counter;
    // publish only when the raw sample changes.
    if (have_mag_ && f.mag_x == last_mag_[0] && f.mag_y == last_mag_[1] && f.mag_z == last_mag_[2]) {
      return;
    }
    have_mag_ = true;
    last_mag_[0] = f.mag_x;
    last_mag_[1] = f.mag_y;
    last_mag_[2] = f.mag_z;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      mag_rate_.tick(steady_ns());
    }
    sensor_msgs::msg::MagneticField m;
    m.header.stamp = stamp;
    m.header.frame_id = mag_frame_;
    m.magnetic_field.x = f.mag_x * mag_scale_;
    m.magnetic_field.y = f.mag_y * mag_scale_;
    m.magnetic_field.z = f.mag_z * mag_scale_;
    m.magnetic_field_covariance[0] = m.magnetic_field_covariance[4] =
      m.magnetic_field_covariance[8] = mag_var_;
    pub_mag_->publish(m);
  }

  void publish_gps(const HostFrame & f, const rclcpp::Time & stamp)
  {
    if (!(f.flags & FLAG_GPS_PACKET_SEEN)) {
      return;
    }
    // 100 Hz frames repeat the same 5 Hz epoch: only a new iTOW is a new measurement.
    if (have_itow_ && f.gps_iTOW_ms == last_itow_) {
      return;
    }
    if (have_itow_) {
      const int64_t step = static_cast<int64_t>(f.gps_iTOW_ms) - static_cast<int64_t>(last_itow_);
      std::lock_guard<std::mutex> lk(mtx_);
      if (step < 0 && step > -600000000) {++gps_itow_backwards_;}  // ignore week rollover
      if (step >= 0 && step < min_itow_step_ms_) {return;}
    }
    have_itow_ = true;
    last_itow_ = f.gps_iTOW_ms;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      gps_rate_.tick(steady_ns());
      ++gps_epochs_;
    }

    const bool fix_ok = (f.gps_fix_flags & 0x01) != 0;
    const double lat = f.lat_e7 * 1e-7;
    const double lon = f.lon_e7 * 1e-7;
    const double h_acc = f.hAcc_mm * 1e-3;
    const double v_acc = f.vAcc_mm * 1e-3;
    const double s_acc = f.sAcc_mms * 1e-3;

    gen2_msgs::msg::GnssPvt p;
    p.header.stamp = stamp;
    p.header.frame_id = gps_frame_;
    p.itow_ms = f.gps_iTOW_ms;
    p.year = f.year;
    p.month = f.month;
    p.day = f.day;
    p.hour = f.hour;
    p.minute = f.minute;
    p.second = f.second;
    p.valid_flags = f.gps_valid_flags;
    p.t_acc_ns = f.gps_tAcc_ns;
    p.nano_ns = f.gps_nano_ns;
    p.fix_type = f.gps_fix_type;
    p.fix_flags = f.gps_fix_flags;
    p.gnss_fix_ok = fix_ok;
    p.num_sv = f.gps_num_sv;
    p.latitude_deg = lat;
    p.longitude_deg = lon;
    p.height_ellipsoid_m = f.height_mm * 1e-3;
    p.height_msl_m = f.hMSL_mm * 1e-3;
    p.h_acc_m = h_acc;
    p.v_acc_m = v_acc;
    p.vel_north_mps = f.velN_mms * 1e-3;
    p.vel_east_mps = f.velE_mms * 1e-3;
    p.vel_down_mps = f.velD_mms * 1e-3;
    p.ground_speed_mps = f.gSpeed_mms * 1e-3;
    p.heading_of_motion_deg = f.headMot_e5 * 1e-5;
    p.speed_acc_mps = s_acc;
    p.heading_acc_deg = f.headAcc_e5 * 1e-5;
    p.pdop = f.pDOP_centi * 0.01;
    p.usable = (f.flags & FLAG_GPS_USABLE) != 0;
    pub_pvt_->publish(p);

    sensor_msgs::msg::NavSatFix fix;
    fix.header = p.header;
    const bool has_position = fix_ok && (f.gps_fix_type == 2 || f.gps_fix_type == 3 ||
      f.gps_fix_type == 4);
    fix.status.status = has_position ? sensor_msgs::msg::NavSatStatus::STATUS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude = f.height_mm * 1e-3;  // NavSatFix altitude is above the WGS84 ellipsoid
    fix.position_covariance[0] = h_acc * h_acc;
    fix.position_covariance[4] = h_acc * h_acc;
    fix.position_covariance[8] = v_acc * v_acc;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    pub_fix_->publish(fix);

    geometry_msgs::msg::TwistWithCovarianceStamped v;
    v.header = p.header;
    // ENU convention in gps frame: x = east, y = north, z = up
    v.twist.twist.linear.x = p.vel_east_mps;
    v.twist.twist.linear.y = p.vel_north_mps;
    v.twist.twist.linear.z = -p.vel_down_mps;
    const double big = 1e6;
    v.twist.covariance[0] = v.twist.covariance[7] = v.twist.covariance[14] =
      has_position ? s_acc * s_acc : big;
    v.twist.covariance[21] = v.twist.covariance[28] = v.twist.covariance[35] = big;
    pub_vel_->publish(v);
  }

  void integrate_energy(const HostFrame & f, int64_t now_ns)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const double dt = last_energy_ns_ ? std::min(0.1, (now_ns - last_energy_ns_) * 1e-9) : 0.0;
    last_energy_ns_ = now_ns;
    update_fuel(fuel_[0], compute_, cal_voltage(compute_, f.compute_mV),
      cal_current(compute_, f.compute_mA), dt, now_ns, energy_wh_[0]);
    update_fuel(fuel_[1], motor_, cal_voltage(motor_, f.motor_mV),
      cal_current(motor_, f.motor_mA), dt, now_ns, energy_wh_[1]);
  }

  void update_fuel(
    Fuel & fu, const BatteryCfg & c, double v, double a, double dt, int64_t now_ns,
    double & energy_wh)
  {
    const bool pres = present(c, v);
    const double cell = v / c.cells;
    // (Re)initialize whenever a pack is (re)connected. A swap always passes through "absent";
    // voltage jumps are NOT used (load release on the motor pack recovers >0.2 V/cell).
    if (pres && !fu.present) {
      const uint64_t resets = fu.resets + (fu.present_since_ns ? 1 : 0);
      fu = Fuel{};
      fu.resets = resets;
      fu.present = true;
      fu.present_since_ns = now_ns;
    }
    if (!pres) {
      // Debounce: a pack counts as removed only after 1 s of implausible readings.
      if (fu.present) {
        if (!fu.absent_since_ns) {fu.absent_since_ns = now_ns;}
        if ((now_ns - fu.absent_since_ns) * 1e-9 > 1.0) {fu.present = false;}
      }
      return;
    }
    fu.absent_since_ns = 0;
    bool est = false;
    const double a_eff = std::max(0.0, pack_current(c, v, a, est));
    fu.power_estimated = est;
    const double p = v * a_eff;
    energy_wh += p * dt / 3600.0;
    if (!fu.initialized) {
      // Average voltage for a short settle window, then map to SoC. Best if the pack is at rest.
      fu.v_settle_sum += cell;
      ++fu.v_settle_n;
      if ((now_ns - fu.present_since_ns) * 1e-9 >= c.soc_settle_s && fu.v_settle_n > 0) {
        fu.soc0 = lipo_percentage(fu.v_settle_sum / fu.v_settle_n);
        fu.initialized = true;
        fu.p_avg_w = p;
      }
      return;
    }
    fu.consumed_ah += a_eff * dt / 3600.0;
    const double alpha = c.power_tau_s > 0 ? std::min(1.0, dt / c.power_tau_s) : 1.0;
    fu.p_avg_w += alpha * (p - fu.p_avg_w);
  }

  struct FuelOut
  {
    bool valid = false;
    double soc = std::numeric_limits<double>::quiet_NaN();
    double remaining_ah = std::numeric_limits<double>::quiet_NaN();
    double runtime_s = std::numeric_limits<double>::quiet_NaN();
    double p_avg_w = std::numeric_limits<double>::quiet_NaN();
  };

  static FuelOut fuel_out(const Fuel & fu, const BatteryCfg & c)
  {
    FuelOut o;
    if (!fu.present || !fu.initialized) {
      return o;
    }
    o.valid = true;
    o.remaining_ah = std::max(0.0, c.capacity_ah * fu.soc0 - fu.consumed_ah);
    o.soc = c.capacity_ah > 0 ? o.remaining_ah / c.capacity_ah : 0.0;
    o.p_avg_w = fu.p_avg_w;
    const double usable_wh = std::max(0.0, o.remaining_ah - c.capacity_ah * c.reserve_fraction) *
      c.cells * c.nominal_cell_v;
    o.runtime_s = fu.p_avg_w > 0.5 ? usable_wh / fu.p_avg_w * 3600.0 :
      std::numeric_limits<double>::infinity();
    return o;
  }

  // ---------------------------------------------------------------- diagnostics
  void diag_link(diagnostic_updater::DiagnosticStatusWrapper & s)
  {
    const int64_t now = steady_ns();
    std::lock_guard<std::mutex> lk(mtx_);
    const auto & st = parser_->stats();
    const double rate = frame_rate_.hz(now);
    const int64_t last = last_frame_ns_.load();
    const double age = last > 0 ? (now - last) * 1e-9 : std::numeric_limits<double>::infinity();
    double lat_mean = 0.0, lat_max = 0.0;
    for (double l : latency_s_) {lat_mean += l; lat_max = std::max(lat_max, l);}
    if (!latency_s_.empty()) {lat_mean /= latency_s_.size();}

    if (age > stale_timeout_s_) {
      s.summary(DiagnosticStatus::ERROR, "no frames: " + link_status_);
    } else if (rate < 80.0) {
      s.summary(DiagnosticStatus::WARN, "low frame rate");
    } else {
      s.summary(DiagnosticStatus::OK, "streaming");
    }
    s.add("port", link_path_);
    s.add("link_status", link_status_);
    s.add("frame_rate_hz", rate);
    s.add("last_frame_age_s", age);
    s.add("frames_ok", st.frames_ok);
    s.add("crc_errors", st.crc_errors);
    s.add("header_errors", st.header_errors);
    s.add("bytes_discarded", st.bytes_discarded);
    s.add("bytes_rx", bytes_rx_);
    s.add("sequence", last_seq_);
    s.add("sequence_drops", seq_drops_);
    s.add("sequence_out_of_order", seq_out_of_order_);
    s.add("esp_resets", esp_resets_);
    s.add("connects", connects_);
    s.add("disconnects", disconnects_);
    s.add("esp_usb_drop_count_total", last_.usb_drop_count);
    s.add("esp_usb_drops_since_connect", last_.usb_drop_count - drops_at_connect_);
    s.add("rel_latency_mean_ms", lat_mean * 1e3);
    s.add("rel_latency_max_ms", lat_max * 1e3);
  }

  void diag_power(diagnostic_updater::DiagnosticStatusWrapper & s, const BatteryCfg & cfg, int idx)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const double v = cal_voltage(cfg, idx == 0 ? last_.compute_mV : last_.motor_mV);
    const double a = cal_current(cfg, idx == 0 ? last_.compute_mA : last_.motor_mA);
    const double cell = v / cfg.cells;
    if (!present(cfg, v)) {
      s.summary(DiagnosticStatus::WARN, "cell voltage implausible: battery absent or PM02 uncalibrated");
    } else if (cell < cfg.error_cell_v) {
      s.summary(DiagnosticStatus::ERROR, "battery critically low");
    } else if (cell < cfg.warn_cell_v) {
      s.summary(DiagnosticStatus::WARN, "battery low");
    } else {
      s.summary(DiagnosticStatus::OK, "ok");
    }
    s.add("voltage_v", v);
    bool est = false;
    const double ae = pack_current(cfg, v, a, est);
    s.add("current_a", ae);
    s.add("power_w", v * ae);
    s.add("current_pm02_a", a);
    s.add("current_source", est ? "jetson_vdd_in" : "pm02");
    s.add("jetson_vdd_in_w", jetson_w_.load());
    s.add("cell_voltage_v", cell);
    s.add("present", present(cfg, v));
    s.add("energy_wh", energy_wh_[idx]);
    const FuelOut fo = fuel_out(fuel_[idx], cfg);
    s.add("soc_percent", fo.soc * 100.0);
    s.add("remaining_ah", fo.remaining_ah);
    s.add("avg_power_w", fo.p_avg_w);
    s.add("runtime_min", fo.runtime_s / 60.0);
    s.add("reserve_percent", cfg.reserve_fraction * 100.0);
    s.add("fuel_resets", fuel_[idx].resets);
    s.add("cells", cfg.cells);
    s.add("raw_mV", idx == 0 ? last_.compute_mV : last_.motor_mV);
    s.add("raw_mA", idx == 0 ? last_.compute_mA : last_.motor_mA);
    s.add("voltage_scale", cfg.voltage_scale);
    s.add("current_offset_a", cfg.current_offset_a);
    s.add("current_scale", cfg.current_scale);
  }

  void diag_gnss(diagnostic_updater::DiagnosticStatusWrapper & s)
  {
    const int64_t now = steady_ns();
    std::lock_guard<std::mutex> lk(mtx_);
    const uint32_t fl = last_.flags;
    if (!(fl & FLAG_GPS_PACKET_SEEN)) {
      s.summary(DiagnosticStatus::WARN, "no UBX-NAV-PVT received yet");
    } else if (!(fl & FLAG_GPS_USABLE)) {
      s.summary(DiagnosticStatus::WARN, "no usable fix");
    } else {
      s.summary(DiagnosticStatus::OK, "usable 3D fix");
    }
    s.add("epoch_rate_hz", gps_rate_.hz(now));
    s.add("epochs", gps_epochs_);
    s.add("itow_ms", last_.gps_iTOW_ms);
    s.add("fix_type", static_cast<int>(last_.gps_fix_type));
    s.add("gnss_fix_ok", (last_.gps_fix_flags & 1) != 0);
    s.add("num_sv", static_cast<int>(last_.gps_num_sv));
    s.add("h_acc_m", last_.hAcc_mm * 1e-3);
    s.add("v_acc_m", last_.vAcc_mm * 1e-3);
    s.add("speed_acc_mps", last_.sAcc_mms * 1e-3);
    s.add("pdop", last_.pDOP_centi * 0.01);
    s.add("ubx_checksum_errors", last_.gps_checksum_error_count);
    s.add("uart_rx_bytes_per_s", gps_uart_rate_);  // 0 => no bytes, or firmware without counter
    s.add("itow_backwards", gps_itow_backwards_);
  }

  void diag_mag(diagnostic_updater::DiagnosticStatusWrapper & s)
  {
    const int64_t now = steady_ns();
    std::lock_guard<std::mutex> lk(mtx_);
    if (last_.flags & FLAG_MAG_VALID) {
      s.summary(DiagnosticStatus::OK, "IST8310 ok (auxiliary only, uncalibrated)");
    } else {
      s.summary(DiagnosticStatus::WARN, "IST8310 not valid");
    }
    s.add("sample_rate_hz", mag_rate_.hz(now));
    s.add("raw_x", last_.mag_x);
    s.add("raw_y", last_.mag_y);
    s.add("raw_z", last_.mag_z);
  }

  // params
  std::string port_;
  std::vector<std::string> by_id_patterns_;
  bool allow_acm_fallback_;
  double reconnect_period_s_, stale_timeout_s_, probe_timeout_s_;
  std::string gps_frame_, mag_frame_;
  double mag_scale_, mag_var_;
  int64_t min_itow_step_ms_;
  BatteryCfg compute_, motor_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_compute_, pub_motor_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<gen2_msgs::msg::GnssPvt>::SharedPtr pub_pvt_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  diagnostic_updater::Updater updater_;

  SerialPort port_io_;
  std::unique_ptr<FrameParser> parser_;
  std::thread reader_;
  std::atomic<bool> running_{true};
  std::atomic<int64_t> last_frame_ns_{0};
  int64_t opened_ns_ = 0;
  OffsetEstimator offset_;

  // reader-thread only
  bool have_itow_ = false;
  uint32_t last_itow_ = 0;
  bool have_mag_ = false;
  int16_t last_mag_[3] = {0, 0, 0};

  // shared (mtx_)
  std::mutex mtx_;
  HostFrame last_{};
  bool have_seq_ = false;
  uint32_t last_seq_ = 0;
  uint64_t last_esp_us_ = 0;
  uint32_t drops_at_connect_ = 0;
  uint64_t gps_rate_t_us_ = 0;
  uint16_t gps_rate_last_ = 0;
  double gps_uart_rate_ = 0.0;
  int64_t last_energy_ns_ = 0;
  double energy_wh_[2] = {0.0, 0.0};  // since node start
  std::string ina_v_path_, ina_i_path_;
  std::atomic<double> jetson_w_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::TimerBase::SharedPtr ina_timer_;
  Fuel fuel_[2];
  uint64_t seq_drops_ = 0, seq_out_of_order_ = 0, esp_resets_ = 0;
  uint64_t connects_ = 0, disconnects_ = 0, bytes_rx_ = 0;
  uint64_t gps_epochs_ = 0, gps_itow_backwards_ = 0;
  std::string link_status_ = "starting", link_path_;
  RateMeter frame_rate_, gps_rate_, mag_rate_;
  std::deque<double> latency_s_;
};

}  // namespace gen2_sensor_hub

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gen2_sensor_hub::SensorHubNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
