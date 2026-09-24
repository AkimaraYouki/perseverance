cmake_minimum_required(VERSION 3.8)
project(gen2_sensor_hub)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Wno-packed-bitfield-compat)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(gen2_msgs REQUIRED)

add_library(host_frame STATIC src/frame_parser.cpp src/serial_port.cpp)
target_include_directories(host_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(host_frame PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(sensor_hub_node src/sensor_hub_node.cpp)
target_link_libraries(sensor_hub_node host_frame)
ament_target_dependencies(sensor_hub_node rclcpp sensor_msgs geometry_msgs diagnostic_updater gen2_msgs)

install(TARGETS sensor_hub_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_parser test/test_frame_parser.cpp)
  target_link_libraries(test_frame_parser host_frame)
endif()

ament_package()
