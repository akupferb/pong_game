cmake_minimum_required(VERSION 3.8)
project(pong_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pong_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pong_controller src/PongController.cpp)
ament_target_dependencies(pong_controller
  rclcpp
  pong_msgs
  geometry_msgs
)

add_executable(pong_ball src/PongBall.cpp)
ament_target_dependencies(pong_ball
  rclcpp
  pong_msgs
  geometry_msgs
  tf2_ros
)

install(
  TARGETS pong_controller pong_ball
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
