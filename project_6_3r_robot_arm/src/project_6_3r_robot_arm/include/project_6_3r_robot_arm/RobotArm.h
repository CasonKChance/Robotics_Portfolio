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
#include <array>

struct JointDynamics {
    double max_effort {0.0};      // N*m
    double max_velocity {0.0};    // rad/s
    double damping {0.0};         // N*m*s/rad
    double friction {0.0};        // N*m
    Eigen::Vector3d axis {Eigen::Vector3d::Zero(3)};
    Eigen::Vector3d origin_position {Eigen::Vector3d::Zero()}; // Joint origin relative to parent link frame
    Eigen::Matrix3d origin_rotation {Eigen::Matrix3d::Identity()};
};

struct LinkDynamics {
    double mass {0.0};
    double length {0.0};
    Eigen::Vector3d center_of_mass {Eigen::Vector3d::Zero()}; // Center of mass in the link frame
    Eigen::Matrix3d inertia {Eigen::Matrix3d::Zero()}; // Inertia tensor about center of mass, expressed in the link frame
};

class RobotArm: public rclcpp::Node
{
public:

    explicit RobotArm(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    Eigen::Vector3d end_effector_position_ {Eigen::Vector3d::Zero(3)};
    std::array<JointDynamics, 3> joint_dynamics_;
    std::array<LinkDynamics, 4> link_dynamics_;
    Eigen::Vector3d joint_angles_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3d joint_velocities_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3d commanded_joint_velocities_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3d applied_joint_torques_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_kp_ {Eigen::Vector3d::Constant(5.0)}; // Velocity-controller proportional gain. Units: N*m / (rad/s)
    Eigen::Vector3d gravity_ {0.0, 0.0, -9.81};
    double y_offset_ {0.0};
    double z_offset_ {0.0};
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