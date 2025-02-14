cmake_minimum_required(VERSION 3.8)
project(pong_ball)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pong_msgs REQUIRED)

add_executable(pong_ball
  src/main.cpp
)
add_library(pong_ball_node SHARED
  src/PongBall.cpp
)
ament_target_dependencies(pong_ball_node
  rclcpp
  pong_msgs
)

target_link_libraries(pong_ball pong_ball_node)

install(
  TARGETS pong_ball
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
