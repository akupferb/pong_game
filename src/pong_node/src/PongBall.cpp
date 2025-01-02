#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <random>

#include "pong_msgs/msg/ball_position.hpp"

using namespace std::chrono_literals;

namespace pong
{
	class PongBall : public rclcpp::Node
	{
	public:
		PongBall(const rclcpp::NodeOptions &) : Node("pong_ball")
		{
			RCLCPP_INFO(get_logger(), "Initializing the pong ball...");
			init();

			ballPositionPublisher_ = create_publisher<pong_msgs::msg::BallPosition>("/ball_position",10);
			ballMarkerPublisher_ = create_publisher<geometry_msgs::msg::PointStamped>("/ball_marker",10);
			publishTimer_ = create_wall_timer(100ms, std::bind(&PongBall::timerCallback, this));
		}

	private:
		void init()
		{
			windowSizeH_ = declare_parameter<int>("windowSizeH", 1024);
			windowSizeV_ = declare_parameter<int>("windowSizeV", 768);

			//Generate random start values:
			std::random_device rseed;
			std::mt19937 randomNumberGenerator(rseed());
			// Distribution for random ball start y-position
			std::uniform_int_distribution<int> distPosY(0,windowSizeV_);
			// "" for random ball velocities (ball will move faster across the screen than up/down)
			std::uniform_int_distribution<int> distSpeedH(-20,-10);
			std::uniform_int_distribution<int> distSpeedV(-10,10);			

			ballData_.set__ball_size_pixels(5);
			ballData_.set__velocity_x(distSpeedH(randomNumberGenerator));
			ballData_.set__velocity_y(distSpeedV(randomNumberGenerator));
			ballData_.set__x(windowSizeH_);
			ballData_.set__y(distPosY(randomNumberGenerator));
			previousTime_ = now();

			ballPosX_ = ballData_.x;
			ballPosY_ = ballData_.y;
		}

		void timerCallback()
		{
			// Get delta time
			rclcpp::Time currentTime = now();
			double elapsedTime = (currentTime - previousTime_).seconds();
			// Update ball position (velocity is constant)
			ballPosX_ += ballData_.velocity_x * elapsedTime;
			ballData_.x = static_cast<int>(round(ballPosX_));
			ballPosY_ += ballData_.velocity_y * elapsedTime;
			ballData_.y = static_cast<int>(round(ballPosY_));
			// Check if ball has hit a horizontal wall and if so, invert velocity direction
			if(ballData_.y <= 0 || ballData_.y >= windowSizeV_) ballData_.set__velocity_y(-ballData_.velocity_y);
			// Publish updated data
			ballPositionPublisher_->publish(ballData_);
			// ballMarkerPublisher_->publish(geometry_msgs::msg::PointStamped()
			// 	.set__point(geometry_msgs::msg::Point().set__x(ballData_.x).set__y(ballData_.y))
			// 	.set__header(std_msgs::msg::Header().set__frame_id("ball").set__stamp(currentTime)));
			// Update time
			previousTime_ = now();
		}

		int windowSizeH_;
		int windowSizeV_;

		pong_msgs::msg::BallPosition ballData_;
		double ballPosX_;
		double ballPosY_;

		rclcpp::Time previousTime_;
		rclcpp::TimerBase::SharedPtr publishTimer_;
		rclcpp::Publisher<pong_msgs::msg::BallPosition>::SharedPtr ballPositionPublisher_;
		rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ballMarkerPublisher_;
	};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<pong::PongBall>(node_options));
  rclcpp::shutdown();
  return 0;
}