#include <rclcpp/rclcpp.hpp>
#include <random>

#include "pong_msgs/msg/ball_position.hpp"
#include "pong_msgs/srv/stick_position.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace pong
{
	class PongBall : public rclcpp::Node
	{
	public:
		PongBall() : Node("pong_ball")
		{
			RCLCPP_INFO(get_logger(), "Initializing the pong ball...");
			init();

			ballPositionPublisher_ = create_publisher<pong_msgs::msg::BallPosition>("/ball_position",10);
			stickPosQueryService_ = create_client<pong_msgs::srv::StickPosition>("/stick_pos");

			publishTimer_ = create_wall_timer(100ms, std::bind(&PongBall::timerCallback, this));
		}

	private:
		void init()
		{
			windowSizeH_ = declare_parameter<int>("windowSizeH", 1024);
			windowSizeV_ = declare_parameter<int>("windowSizeV", 768);
			endZoneX_ = declare_parameter<int>("endZoneX", 20);

			//Generate random start values:
			std::random_device rseed;
			std::mt19937 randomNumberGenerator(rseed());
			// Distribution for random ball start y-position
			std::uniform_int_distribution<int> distPosY(0,windowSizeV_);
			// "" for random ball velocities (ball will move faster across the screen than up/down)
			std::uniform_int_distribution<int> distSpeedH(-60,-40);
			std::uniform_int_distribution<int> distSpeedV(-20,20);			

			ballData_.set__ball_size_pixels(5);
			ballData_.set__velocity_x(distSpeedH(randomNumberGenerator));
			ballData_.set__velocity_y(distSpeedV(randomNumberGenerator));
			ballData_.set__x(windowSizeH_);
			ballData_.set__y(distPosY(randomNumberGenerator));
			previousTime_ = now();

			// These data members are "double"s to allow for fractional tracking of ball position
			ballActualPosX_ = ballData_.x;
			ballActualPosY_ = ballData_.y;
		}

		void timerCallback()
		{
			// Get delta time
			rclcpp::Time currentTime = now();
			double elapsedTime = (currentTime - previousTime_).seconds();
			// Calculate updated position
			double newBallPoseX = ballActualPosX_ + ballData_.velocity_x * elapsedTime;
			double newBallPoseY = ballActualPosY_ + ballData_.velocity_y * elapsedTime;
			// Update time
			previousTime_ = now();

			// Check if new ball position will cross endzone
			if(newBallPoseX <= endZoneX_)
			{
				if(!stickPosQueryService_->wait_for_service(1s))
					throw std::runtime_error("Service not ready");
				RCLCPP_INFO_ONCE(this->get_logger(), "Service Client and Server are ready");

				serviceResponseTimeout_ = create_wall_timer(1s, [this](){ throw std::runtime_error("No response from Service Client"); });
				// Send service request for current stick position
				auto request = std::make_shared<pong_msgs::srv::StickPosition::Request>();
				stickPosQueryService_->async_send_request(request, [this, newBallPoseX, newBallPoseY](rclcpp::Client<pong_msgs::srv::StickPosition>::SharedFuture future)
				{
					this->serviceResponseTimeout_->cancel();
					auto stickPos = future.get();
					// Check if stick has blocked the ball
					if(stickPos->y_min_stick_pos <= newBallPoseY && newBallPoseY <= stickPos->y_max_stick_pos)
					{
						RCLCPP_INFO(this->get_logger(), "Ball blocked by stick!");
						// Ball will now bounce back
						this->ballData_.velocity_x *= -1;
						updateBallPosition(endZoneX_, newBallPoseY);
					}
				});
				return; // Ball position will be updated in service callback
			}
			// Update ball position (velocity is constant)
			updateBallPosition(newBallPoseX, newBallPoseY);
		}

		void updateBallPosition(double newBallPosX, double newBallPosY)
		{
			// Check if new ball position will hit a horizontal wall and rebound
			if(newBallPosY <= 0 || newBallPosY >= windowSizeV_)
			{
				RCLCPP_INFO(this->get_logger(), "Ball rebounded off wall");
				ballData_.velocity_y *= -1;
				newBallPosY = (newBallPosY <= 0 ? 0.0 : windowSizeV_);
			}
			// Check if new ball position will hit far end and rebound
			if(newBallPosX > windowSizeH_)
			{
				RCLCPP_INFO(this->get_logger(), "Ball rebounded off wall");
				// Ball will bounce back toward endzone
				ballData_.velocity_x *= -1;
				newBallPosX = windowSizeH_;
			}

			ballActualPosX_ = newBallPosX;
			ballActualPosY_ = newBallPosY;
			ballData_.x = static_cast<int>(round(ballActualPosX_));
			ballData_.y = static_cast<int>(round(ballActualPosY_));
			// Publish updated data
			ballPositionPublisher_->publish(ballData_);
		}

		int windowSizeH_;
		int windowSizeV_;
		int endZoneX_;

		pong_msgs::msg::BallPosition ballData_;
		double ballActualPosX_;
		double ballActualPosY_;

		rclcpp::Time previousTime_;
		rclcpp::TimerBase::SharedPtr publishTimer_;
		rclcpp::Publisher<pong_msgs::msg::BallPosition>::SharedPtr ballPositionPublisher_;

		rclcpp::Client<pong_msgs::srv::StickPosition>::SharedPtr stickPosQueryService_;
		rclcpp::TimerBase::SharedPtr serviceResponseTimeout_;
	};
}
