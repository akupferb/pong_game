cmake_minimum_required(VERSION 3.8)
project(pong_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pong_controller REQUIRED)
find_package(pong_ball REQUIRED)

# Install headers
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
