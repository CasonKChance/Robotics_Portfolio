#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <RoboticArmKinematics/Kinematics.h>

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <numbers>
#include <memory>

class RobotArm: public rclcpp::Node
{
public:

    explicit RobotArm(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    double joint_pedestal_arm1_angle_ {0.0};
    double joint_arm1_arm2_angle_ {0.0};
    double joint_arm2_arm3_angle_ {0.0};
    double arm1_length_ {0.0};
    double arm2_length_ {0.0};
    double arm3_length_ {0.0};
    double y_offset_ {0.0};
    double z_offset_ {0.0};
    Eigen::Vector3d joint_pedestal_arm1_axis_ {Eigen::Vector3d::Zero(3)};
    Eigen::Vector3d joint_arm1_arm2_axis_ {Eigen::Vector3d::Zero(3)};
    Eigen::Vector3d joint_arm2_arm3_axis_ {Eigen::Vector3d::Zero(3)};
    Eigen::Vector3d end_effector_position_ {Eigen::Vector3d::Zero(3)};
    bool robot_description_ready_ {false};

    rclcpp::Subscription< std_msgs::msg::String >::SharedPtr robot_description_subscription_;
    rclcpp::Subscription< sensor_msgs::msg::JointState >::SharedPtr joint_state_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    void robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message);

    void jointStateTopicCallback(sensor_msgs::msg::JointState::UniquePtr message);

    void logEndEffectorPosition() const;
};

#endif