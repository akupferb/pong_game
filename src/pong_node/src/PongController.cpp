#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "pong_msgs/msg/ball_position.hpp"

using namespace std::placeholders;

namespace pong
{
	class PongController : public rclcpp::Node
	{
	public:
		PongController(const rclcpp::NodeOptions &) : Node("pong_controller")
		{
			RCLCPP_INFO(get_logger(), "Created the PongController Node.");
			init();

			stickAccelerationPublisher_ = create_publisher<std_msgs::msg::Float32>("/stick_acc",rclcpp::SystemDefaultsQoS());
			ballPositionSubscriber_ = create_subscription<pong_msgs::msg::BallPosition>(
				"/ball_position", 10, std::bind(&PongController::topicCallback, this, _1));
		}

	private:
		void init()
		{
			stickLaneX_ = declare_parameter<int>("paddleLaneX", 20);
			stickLength_ = declare_parameter<int>("paddleLength", 30);
			windowSizeV_ = declare_parameter<int>("windowSizeV", 768);
			stickCenterPos_ = windowSizeV_ / 2;
			stickAcc_ = 0.0;
			stickVel_ = 0.0;
			//next
			previousTime_ = now();
		}

		void topicCallback(const pong_msgs::msg::BallPosition & msg)
		{
			auto& clk = *this->get_clock();
			RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "ball pos: [%d,%d]", msg.x, msg.y);
			double ballEndYPos = calculateBallEndPointY(msg);
			RCLCPP_INFO_ONCE(this->get_logger(), "ball speeds = [%.1f,%.1f]", msg.velocity_x,msg.velocity_y);
			RCLCPP_INFO_ONCE(this->get_logger(), "ball endpoint: %1f", ballEndYPos);
			// Check if stick needs to move into position to catch ball:
			rclcpp::Time currentTime = now();
			double elapsedTime = (currentTime - previousTime_).seconds();
			int distanceMoved;
			if(ballEndYPos < (stickCenterPos_ - stickLength_ / 2) || ballEndYPos > (stickCenterPos_ + stickLength_ / 2))
			{
				int distanceToMove = ballEndYPos - stickCenterPos_ - stickLength_ / 3; // This allows the stick to catch the ball with tolerance
				// As the game controller is massless, and there is no drag or acceleration constraint,
				// the controller stick moves from rest to full velocity in virtually a single moment
				if(abs(distanceToMove) > 10)
					distanceMoved = (ballEndYPos > stickCenterPos_ ? 10 : -10);
				else
					distanceMoved = distanceToMove;
				stickCenterPos_ += distanceMoved;
				RCLCPP_INFO(this->get_logger(), "stick center y: [%d]", stickCenterPos_);
			}
			else // stick will catch ball in current position
				distanceMoved = 0;

			// Calculate and publish stick acceleration
			double newStickVel = round(distanceMoved / elapsedTime);
			stickAcc_ = (newStickVel - stickVel_) / elapsedTime;
			stickAccelerationPublisher_->publish(std_msgs::msg::Float32().set__data(stickAcc_));
			// reset elements
			stickVel_ = newStickVel;
			previousTime_ = currentTime;
		}

		double calculateBallEndPointY(const pong_msgs::msg::BallPosition & ball)
		{
			// Calculate Y position of ball when it will reach the paddle lane...
			double timeToPaddleLane = (stickLaneX_ - ball.x) / ball.velocity_x;
			double ballEndYPos = ball.y + ball.velocity_y * timeToPaddleLane;
			// Check if ball will rebound off upper/lower wall before reaching the paddle lane
			while(ballEndYPos < 0 || ballEndYPos > windowSizeV_) // ball will rebound first
			{
				RCLCPP_WARN_ONCE(this->get_logger(), "ball rebounds");
				int reboundPointY = (ball.velocity_y > 0 ? windowSizeV_ : 0);
				double timeToRebound = (reboundPointY - ball.y) / ball.velocity_y;
				// Now calculate actual ball Y position after rebound (Y velocity inverted)
				ballEndYPos = reboundPointY + (-1*ball.velocity_y) * (timeToPaddleLane - timeToRebound);
			}
			return ballEndYPos;
		}

		int windowSizeV_;
		int stickLength_;
		int stickCenterPos_;
		int stickLaneX_;

		double stickAcc_;
		double stickVel_;
		rclcpp::Time previousTime_;

		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stickAccelerationPublisher_;
		rclcpp::Subscription<pong_msgs::msg::BallPosition>::SharedPtr ballPositionSubscriber_;
	};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<pong::PongController>(node_options));
  rclcpp::shutdown();
  return 0;
}