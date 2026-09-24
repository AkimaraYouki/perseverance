// motor_test_node — ROS service front-end for MotorTester (used by the motor test GUI).
//
//   srv  motor_test/start   gen2_msgs/srv/MotorTest
//   srv  motor_test/stop    std_srvs/srv/Trigger
//   srv  motor_test/zero_all std_srvs/srv/Trigger
//   sub  motor_test/heartbeat std_msgs/Empty   (operator dead-man, required while testing)
//   pub  motor_test/status  gen2_msgs/msg/MotorTestStatus (20 Hz)
//
// The 100 Hz command loop runs in C++ (MotorTester); the GUI never streams motor commands.

#include <memory>
#include <string>

#include "gen2_hardware/motor_bus.hpp"
#include "gen2_hardware/motor_params.hpp"
#include "gen2_hardware/motor_tester.hpp"
#include "gen2_msgs/msg/motor_test_status.hpp"
#include "gen2_msgs/srv/motor_test.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace gen2_hardware
{

class MotorTestNode : public rclcpp::Node
{
public:
  MotorTestNode()
  : Node("motor_test")
  {
    const std::string ifname = declare_parameter("can_interface", std::string("can0"));
    TestLimits lim;
    lim.max_current_a = declare_parameter("cli.max_test_current_a", 1.0);
    lim.max_velocity_rad_s = declare_parameter("cli.max_test_velocity_rad_s", 2.0);
    lim.max_duration_s = declare_parameter("cli.max_test_duration_s", 3.0);
    const double hb = declare_parameter("heartbeat_timeout_s", 0.5);
    bus_ = std::make_unique<MotorBus>(ifname, declare_motor_params(*this));
    bus_->start();
    tester_ = std::make_unique<MotorTester>(*bus_, lim);
    tester_->require_heartbeat(hb);

    hb_sub_ = create_subscription<std_msgs::msg::Empty>("motor_test/heartbeat", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {tester_->heartbeat();});
    pub_ = create_publisher<gen2_msgs::msg::MotorTestStatus>("motor_test/status", 10);
    start_srv_ = create_service<gen2_msgs::srv::MotorTest>("motor_test/start",
        [this](const std::shared_ptr<gen2_msgs::srv::MotorTest::Request> rq,
        std::shared_ptr<gen2_msgs::srv::MotorTest::Response> rs) {on_start(*rq, *rs);});
    stop_srv_ = create_service<std_srvs::srv::Trigger>("motor_test/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> rs) {
          tester_->stop("operator STOP");
          rs->success = true;
          rs->message = "stop requested";
        });
    zero_srv_ = create_service<std_srvs::srv::Trigger>("motor_test/zero_all",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> rs) {
          rs->message = tester_->zero_all();
          rs->success = true;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {publish();});
    RCLCPP_INFO(get_logger(), "motor_test ready: %zu motor(s), limits %.2f A / %.2f rad/s / %.1f s, "
      "heartbeat %.2f s", bus_->motors().size(), lim.max_current_a, lim.max_velocity_rad_s,
      lim.max_duration_s, hb);
  }

  ~MotorTestNode() override
  {
    tester_.reset();
    bus_->stop();
  }

private:
  void on_start(const gen2_msgs::srv::MotorTest::Request & rq, gen2_msgs::srv::MotorTest::Response & rs)
  {
    if (!rq.confirm_lifted) {
      rs.accepted = false;
      rs.message = "operator must confirm robot lifted / wheel free";
      return;
    }
    std::size_t m = SIZE_MAX;
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
      if (bus_->motors()[i].name == rq.motor) {m = i;}
    }
    TestMode mode;
    if (rq.mode == "current") {mode = TestMode::kCurrent;}
    else if (rq.mode == "velocity") {mode = TestMode::kVelocity;}
    else {rs.accepted = false; rs.message = "mode must be current|velocity"; return;}
    const std::string err = tester_->start(m, mode, rq.value, rq.duration_s);
    rs.accepted = err.empty();
    rs.message = err.empty() ? "started" : err;
    RCLCPP_INFO(get_logger(), "start %s %s %.3f for %.2f s -> %s", rq.motor.c_str(), rq.mode.c_str(),
      rq.value, rq.duration_s, rs.message.c_str());
  }

  void publish()
  {
    const TestStatus st = tester_->status();
    gen2_msgs::msg::MotorTestStatus m;
    m.header.stamp = now();
    m.running = st.running;
    m.motor = st.motor < bus_->motors().size() ? bus_->motors()[st.motor].name : "";
    m.mode = st.mode == TestMode::kCurrent ? "current" : "velocity";
    m.value = st.value;
    m.elapsed_s = st.elapsed_s;
    m.duration_s = st.duration_s;
    m.result = st.result;
    m.test_id = st.test_id;
    m.peak_current_a = st.peak_current_a;
    m.peak_velocity_rad_s = st.peak_velocity_rad_s;
    m.moved_rad = st.moved_rad;
    m.mean_velocity_rad_s = st.mean_velocity_rad_s;
    m.heartbeat_ok = tester_->heartbeat_ok();
    for (std::size_t i = 0; i < bus_->motors().size(); ++i) {
      m.motor_names.push_back(bus_->motors()[i].name);
      m.max_current_a.push_back(tester_->current_limit(i));
      m.max_velocity_rad_s.push_back(tester_->velocity_limit(i));
    }
    m.max_duration_s = tester_->limits().max_duration_s;
    pub_->publish(m);
  }

  std::unique_ptr<MotorBus> bus_;
  std::unique_ptr<MotorTester> tester_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hb_sub_;
  rclcpp::Publisher<gen2_msgs::msg::MotorTestStatus>::SharedPtr pub_;
  rclcpp::Service<gen2_msgs::srv::MotorTest>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_, zero_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gen2_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<gen2_hardware::MotorTestNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_test"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
