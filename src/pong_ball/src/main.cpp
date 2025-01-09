#include <rclcpp/rclcpp.hpp>
#include "PongBall.cpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto nh = std::make_shared<pong::PongBall>();
  executor.add_node(nh);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}