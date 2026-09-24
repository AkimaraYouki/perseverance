cmake_minimum_required(VERSION 3.8)
project(gen2_hardware)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(gen2_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS-free core (codec, socket, bus) — reused by the real-time controller process.
add_library(gen2_hardware_core SHARED
  src/cubemars_servo.cpp src/can_socket.cpp src/motor_bus.cpp src/motor_tester.cpp)
target_include_directories(gen2_hardware_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(gen2_hardware_core Threads::Threads)

add_library(gen2_hardware_params SHARED src/motor_params.cpp)
target_link_libraries(gen2_hardware_params gen2_hardware_core)
ament_target_dependencies(gen2_hardware_params rclcpp)

add_executable(motor_monitor_node src/motor_monitor_node.cpp)
target_link_libraries(motor_monitor_node gen2_hardware_core gen2_hardware_params)
ament_target_dependencies(motor_monitor_node rclcpp diagnostic_updater gen2_msgs)

add_executable(motor_cli src/motor_cli.cpp)
target_link_libraries(motor_cli gen2_hardware_core gen2_hardware_params)
ament_target_dependencies(motor_cli rclcpp)

add_executable(motor_test_node src/motor_test_node.cpp)
target_link_libraries(motor_test_node gen2_hardware_core gen2_hardware_params)
ament_target_dependencies(motor_test_node rclcpp std_msgs std_srvs gen2_msgs)

install(TARGETS gen2_hardware_core gen2_hardware_params EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS motor_monitor_node motor_cli motor_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cubemars test/test_cubemars.cpp)
  target_link_libraries(test_cubemars gen2_hardware_core)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_package()
