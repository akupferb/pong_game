cmake_minimum_required(VERSION 3.8)
project(pong_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pong_msgs REQUIRED)

add_executable(pong_controller
  src/main.cpp
)
add_library(pong_controller_node SHARED
  src/PongController.cpp
)
ament_target_dependencies(pong_controller_node
  rclcpp
  pong_msgs
)

target_link_libraries(pong_controller pong_controller_node)

install(
  TARGETS pong_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
