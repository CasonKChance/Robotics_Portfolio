#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CommandVelocityPublisher : public rclcpp::Node {
    public:

        CommandVelocityPublisher() : 
            Node("command_velocity_publisher")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            this->declare_parameter<double>("linear_velocity", 0.0);
            this->declare_parameter<double>("angular_velocity", 0.0);

            timer_ = this->create_wall_timer(100ms, std::bind(&CommandVelocityPublisher::timerCallback, this));
        }

    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

        void timerCallback() {
            auto message = geometry_msgs::msg::Twist();

            message.linear.x = this->get_parameter("linear_velocity").as_double();
            message.angular.z = normalizeAngle(this->get_parameter("angular_velocity").as_double());

            RCLCPP_INFO(this->get_logger(), "\nPublishing: \n"
                                            "\tLinear: %.2f m/s\n"
                                            "\tAngular: %.2f rad/s\n", message.linear.x, message.angular.z);

            publisher_->publish(message);
        }

        double normalizeAngle(double angle) const {
            // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
            return std::atan2(std::sin(angle), std::cos(angle));
        }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandVelocityPublisher>());
  rclcpp::shutdown();

  return 0;
}