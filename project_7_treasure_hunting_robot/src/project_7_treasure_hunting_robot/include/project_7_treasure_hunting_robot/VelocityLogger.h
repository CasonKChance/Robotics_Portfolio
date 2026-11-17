#ifndef IMU_VELOCITY_LOGGER_H
#define IMU_VELOCITY_LOGGER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.cpp"

#include <memory>

struct Twist {
    double linear_velocity_x{0.0};
    double linear_velocity_y{0.0};
    double linear_velocity_z{0.0};
    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};
}

class VelocityLogger: public rclcpp::Node
{
public:

    explicit VelocityLogger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~VelocityLogger();

private:
    Twist current_twist_;
    bool has_received_imu_data_ = false;

    rclcpp::Subscription < sensor_msgs::msg::Imu > ::SharedPtr imu_subscription_;
    rclcpp::Time last_imu_time_;

    void IMUTopicCallback(sensor_msgs::msg::Imu::UniquePtr msg);

    void logCurrentTwist();
}

#endif