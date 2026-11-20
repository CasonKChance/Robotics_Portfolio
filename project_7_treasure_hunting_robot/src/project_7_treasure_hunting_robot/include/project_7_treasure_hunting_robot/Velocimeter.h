#ifndef VELOCIMETER_H
#define VELOCIMETER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>

/**
 * @brief Struct used to store the current twist of the robot.
 */
struct Twist {
    double linear_velocity_x{0.0};
    double linear_velocity_y{0.0};
    double linear_velocity_z{0.0};
    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};
};

/**
 * @brief Provides a velocimeter which presents the current linear and angular velocities of the robot. 
 */
class Velocimeter: public rclcpp::Node
{
public:

    /**
     * @brief Constructs the velocimeter by initializing imu and odometry subscriptions.
     * @param options Configuration options for Node initialization
     */
    explicit Velocimeter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    Twist current_twist_;                // Represents the current linear and angular velocity components of the robot.
    bool has_received_imu_data_ = false; // Flag that details whether or not IMU data has been received.

    rclcpp::Subscription < sensor_msgs::msg::Imu > ::SharedPtr imu_subscription_;
    rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_subscription_;
    rclcpp::Time last_imu_time_;

    /**
     * @brief Processes data published to the /imu topic.
     * 
     * Updates the internal twist struct with angular velocity data
     * and linear acceleration data (integrated with respect to time)
     * received from the IMU sensor.
     * 
     * @param msg Data packet received from the IMU sensor
     */
    void imuTopicCallback(sensor_msgs::msg::Imu::UniquePtr msg);

    /**
     * @brief Processes data published to the /odom topic.
     * 
     * Updates the internal twist struct to zeros when the robot is stationary.
     * Used to protect against IMU data drift.
     * 
     * @param msg Data packet recieved from odometry.
     */
    void odomTopicCallback(nav_msgs::msg::Odometry::UniquePtr msg);

    /**
     * @brief Logs the current twist and prints it to the console.
     */
    void printTwist();
};

#endif