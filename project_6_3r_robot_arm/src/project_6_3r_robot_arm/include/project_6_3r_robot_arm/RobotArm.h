#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <RoboticArmKinematics/Kinematics.h>

#include "project_6_3r_robot_arm_interfaces/msg/joint_velocities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <numbers>
#include <memory>

struct JointVelocities {
    double joint_pedestal_arm1_velocity;
    double joint_arm1_arm2_velocity;
    double joint_arm2_arm3_velocity;
};

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
    JointVelocities joint_velocities_ {0.0, 0.0, 0.0};
    JointVelocities commanded_joint_velocities_ {0.0, 0.0, 0.0};
    bool robot_description_ready_ {false};
    bool ik_mode_ {false};

    rclcpp::Publisher< sensor_msgs::msg::JointState >::SharedPtr joint_state_publisher_;
    rclcpp::Subscription< project_6_3r_robot_arm_interfaces::msg::JointVelocities >::SharedPtr joint_velocities_subscription_;
    rclcpp::Subscription< std_msgs::msg::String >::SharedPtr robot_description_subscription_;
    rclcpp::Subscription< sensor_msgs::msg::JointState >::SharedPtr joint_state_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Time last_update_time_;

    void update(double dt);

    void updateJointVelocities(double dt);

    void jointVelocitiesTopicCallback(project_6_3r_robot_arm_interfaces::msg::JointVelocities::UniquePtr message);

    void robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message);

    void jointStateTopicCallback(sensor_msgs::msg::JointState::UniquePtr message);

    void publishJointStates() const;

    void logEndEffectorPosition() const;

    /**
     * @brief Normalizes an angle into the range [-π, π] radians.
     * @param angle Angle in radians.
     * @return Equivalent angle wrapped within [-π, π].
     */
    double normalizeAngle(double angle) const;
};

#endif