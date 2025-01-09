#include <rclcpp/rclcpp.hpp>
#include "PongController.cpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto nh = std::make_shared<pong::PongController>();
  executor.add_node(nh);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}