// Load MotorConfig list from ROS parameters:
//   motors.names: [a, b]
//   motors.a.can_id: 69 ...
#pragma once

#include <string>
#include <vector>

#include "gen2_hardware/motor_config.hpp"
#include "rclcpp/node.hpp"

namespace gen2_hardware
{

std::vector<MotorConfig> declare_motor_params(rclcpp::Node & node);

}  // namespace gen2_hardware
