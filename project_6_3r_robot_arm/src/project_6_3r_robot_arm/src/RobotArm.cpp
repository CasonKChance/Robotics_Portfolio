#include <project_6_3r_robot_arm/RobotArm.h>

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using JointVelocities = project_6_3r_robot_arm_interfaces::msg::JointVelocities;

/* Public Member Functions */

RobotArm::RobotArm(const rclcpp::NodeOptions & options)
: Node("robot_arm", options)
{
  this->declare_parameter<std::string>("mode", "default");
  ik_mode_ = this->get_parameter("mode").as_string() == "IK";

  robot_description_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(
          &RobotArm::robotDescriptionTopicCallback,
          this,
          _1
      )
  );

  if (ik_mode_) {
    joint_velocities_subscription_ = this->create_subscription<JointVelocities>(
        "/joint_velocities",
        10,
        std::bind(
            &RobotArm::jointVelocitiesTopicCallback,
            this,
            _1
        )
    );

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
      10);

    robot_state_publisher_ = this->create_publisher<project_6_3r_robot_arm_interfaces::msg::RobotState>("/robot_state", 10);
  } else {
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(
            &RobotArm::jointStateTopicCallback,
            this,
            _1
        )
    );
  }

  last_update_time_ = this->get_clock()->now();

  update_timer_ = this->create_wall_timer(10ms, [this]() {
        if (ik_mode_) {
          const auto now = this->get_clock()->now();

          const double dt = (now - last_update_time_).seconds();

          last_update_time_ = now;

          this->update(dt);
        }

        this->logEndEffectorPosition();
  });
}

/* Private Member Functions */

void RobotArm::update(double dt)
{
  if (dt <= 0.0 || !robot_description_ready_) {
    return;
  }

  updateJointVelocities(dt);

  joint_angles_[0] = normalizeAngle(joint_angles_[0] + joint_velocities_[0] * dt);
  joint_angles_[1] = normalizeAngle(joint_angles_[1] + joint_velocities_[1] * dt);
  joint_angles_[2] = normalizeAngle(joint_angles_[2] + joint_velocities_[2] * dt);

  end_effector_position_ = Kinematics::forwardKinematics(
    {joint_angles_[0], joint_angles_[1], joint_angles_[2]},
    {link_dynamics_[0].length, link_dynamics_[1].length, link_dynamics_[2].length},
    {joint_dynamics_[0].axis, joint_dynamics_[1].axis, joint_dynamics_[2].axis}
  );

  Eigen::Vector3d offset(
    -y_offset_ * std::sin(joint_angles_[0]),
    y_offset_ * std::cos(joint_angles_[0]),
    z_offset_
  );

  end_effector_position_ += offset;

  publishJointStates();
  publishRobotState();
}

void RobotArm::updateJointVelocities(double dt)
{
  const Eigen::Vector3d effective_inertia = computeApproximateJointInertias();

  for (int i = 0; i < 3; ++i) {
    const auto & joint = joint_dynamics_[i];

    const double max_effort = joint.max_effort;
    const double max_velocity = joint.max_velocity;

    const double current_velocity = joint_velocities_[i];

    const double target_velocity = std::clamp(
            commanded_joint_velocities_[i],
            -max_velocity,
            max_velocity
    );

    // Calculate velocity tracking error.
    const double velocity_error = target_velocity - current_velocity;

    // Proportional velocity controller.
    const double requested_torque = velocity_kp_[i] * velocity_error;

    // Enforce motor effort limit.
    const double motor_torque = std::clamp(
            requested_torque,
            -max_effort,
            max_effort
    );

    // Viscous damping.
    const double damping_torque = joint.damping * current_velocity;

    // Coulomb friction magnitude.
    // For now, apply it only during motion.
    double friction_torque = 0.0;

    constexpr double velocity_epsilon = 1e-6;

    if (std::abs(current_velocity) > velocity_epsilon) {
      friction_torque = std::copysign(
                joint.friction,
                current_velocity
      );
    }

    // Calculate net torque.
    const double net_torque = motor_torque - damping_torque - friction_torque;

    // Rotational form of Newton's second law.
    const double acceleration = net_torque / effective_inertia[i];

    joint_accelerations_[i] = acceleration;

    // Integrate acceleration.
    double new_velocity = current_velocity + acceleration * dt;

    // Prevent numerical overshoot when approaching the commanded velocity.
    if ((target_velocity - current_velocity) * (target_velocity - new_velocity) < 0.0) {
      new_velocity = target_velocity;
    }

    // Enforce URDF velocity limit.
    new_velocity = std::clamp(
            new_velocity,
            -max_velocity,
            max_velocity
    );

    joint_velocities_[i] = new_velocity;
  }
}

void RobotArm::jointVelocitiesTopicCallback(JointVelocities::UniquePtr message)
{
  if (!robot_description_ready_) {
    return;
  }

  Eigen::Vector3d command(
    message->joint_pedestal_arm1_velocity,
    message->joint_arm1_arm2_velocity,
    message->joint_arm2_arm3_velocity
  );

  if (!command.allFinite()) {
    RCLCPP_WARN(
            this->get_logger(),
            "Rejecting non-finite joint velocity command."
    );

    return;
  }

  for (int i = 0; i < 3; ++i) {
    const double limit = joint_dynamics_[i].max_velocity;

    command[i] = std::clamp(command[i], -limit, limit);
  }

  if (command.isApprox(commanded_joint_velocities_)) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Updating commanded with: \n"
                                    "\tjoint_pedestal_arm1_velocity: %.2f rads/s\n"
                                    "\tjoint_arm1_arm2_velocity: %.2f rads/s\n"
                                    "\tjoint_arm2_arm3_velocity: %.2f rads/s\n",
                                    command[0],
                                    command[1],
                                    command[2]
  );

  commanded_joint_velocities_[0] = command[0];
  commanded_joint_velocities_[1] = command[1];
  commanded_joint_velocities_[2] = command[2];
}

void RobotArm::robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message)
{
  // Pull model
  urdf::Model model;

  if (!model.initString(message->data)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to parse robot description.");
    return;
  }

  const auto joint_pedestal_arm1 = model.getJoint("joint_pedestal_arm1");
  const auto joint_arm1_arm2 = model.getJoint("joint_arm1_arm2");
  const auto joint_arm2_arm3 = model.getJoint("joint_arm2_arm3");

  const auto arm_link_1 = model.getLink("arm_link_1");
  const auto arm_link_2 = model.getLink("arm_link_2");
  const auto arm_link_3 = model.getLink("arm_link_3");

  const auto end_effector_link = model.getLink("end_effector_link");

  const auto joint_base_pedestal = model.getJoint("joint_base_pedestal");

  if (!joint_pedestal_arm1 ||
    !joint_arm1_arm2 ||
    !joint_arm2_arm3 ||
    !arm_link_1 ||
    !arm_link_2 ||
    !arm_link_3 ||
    !end_effector_link ||
    !joint_base_pedestal)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Required robot arm links/joints not found in URDF."
    );

    return;
  }

  // Validate model

  if (!joint_pedestal_arm1->limits ||
    !joint_arm1_arm2->limits ||
    !joint_arm2_arm3->limits ||
    !arm_link_1->inertial ||
    !arm_link_2->inertial ||
    !arm_link_3->inertial ||
    !end_effector_link->inertial ||
    !arm_link_1->collision ||
    !arm_link_2->collision ||
    !arm_link_3->collision ||
    !end_effector_link->collision)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Arm limits, inertials, or collision geometry missing."
    );

    return;
  }

  // Extract arm geometry

  const auto * arm_link_1_cylinder =
    dynamic_cast<const urdf::Cylinder *>(arm_link_1->collision->geometry.get());
  const auto * arm_link_2_cylinder =
    dynamic_cast<const urdf::Cylinder *>(arm_link_2->collision->geometry.get());
  const auto * arm_link_3_cylinder =
    dynamic_cast<const urdf::Cylinder *>(arm_link_3->collision->geometry.get());

  if (!arm_link_1_cylinder || !arm_link_2_cylinder || !arm_link_3_cylinder) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Arm collision geometry must be cylindrical."
    );

    return;
  }

  // Extract arm dynamics

  auto extractArmDynamics = [](const urdf::LinkConstSharedPtr & link) {
      const auto & inertial = *link->inertial;

      LinkDynamics result;

      result.mass = inertial.mass;

      result.center_of_mass <<
        inertial.origin.position.x,
        inertial.origin.position.y,
        inertial.origin.position.z;

      Eigen::Matrix3d inertia_inertial_frame;

      inertia_inertial_frame <<
        inertial.ixx, inertial.ixy, inertial.ixz,
        inertial.ixy, inertial.iyy, inertial.iyz,
        inertial.ixz, inertial.iyz, inertial.izz;

      const auto & r = inertial.origin.rotation;

      const Eigen::Quaterniond rotation(r.w, r.x, r.y, r.z);

      const Eigen::Matrix3d R = rotation.normalized().toRotationMatrix();

      result.inertia = R * inertia_inertial_frame * R.transpose();

      double maxLength = 0.0;

      for (const auto & collision : link->collision_array) {
        if (!collision || !collision->geometry) {
          continue;
        }

        const auto * cylinder = dynamic_cast<const urdf::Cylinder *>(collision->geometry.get());

        if (cylinder) {
          maxLength = std::max(maxLength, cylinder->length);
        }
      }

      result.length = maxLength;

      return result;
    };

  link_dynamics_[0] = extractArmDynamics(arm_link_1);
  link_dynamics_[1] = extractArmDynamics(arm_link_2);
  link_dynamics_[2] = extractArmDynamics(arm_link_3);
  link_dynamics_[3] = extractArmDynamics(end_effector_link);

  for (const auto & link : link_dynamics_) {
    if (!std::isfinite(link.mass) ||
      link.mass <= 0 ||
      !std::isfinite(link.length) ||
      link.length <= 0 ||
      !link.center_of_mass.allFinite() ||
      !link.inertia.allFinite() ||
      !link.inertia.isApprox(link.inertia.transpose().eval(), 1e-9))
    {
      RCLCPP_ERROR(
                this->get_logger(),
                "Invalid link dynamics parameters."
      );

      return;
    }
  }

  // Extract joint dynamics

  auto extractJointDynamics = [](const urdf::JointConstSharedPtr & joint) {
      JointDynamics result;

      result.max_effort = joint->limits->effort;
      result.max_velocity = joint->limits->velocity;

      if (joint->dynamics) {
        result.damping = joint->dynamics->damping;
        result.friction = joint->dynamics->friction;
      }

      result.axis << joint->axis.x, joint->axis.y, joint->axis.z;

      return result;
    };

  joint_dynamics_[0] = extractJointDynamics(joint_pedestal_arm1);
  joint_dynamics_[1] = extractJointDynamics(joint_arm1_arm2);
  joint_dynamics_[2] = extractJointDynamics(joint_arm2_arm3);

  for (const auto & joint : joint_dynamics_) {
    if (!std::isfinite(joint.max_effort) ||
      joint.max_effort <= 0.0 ||
      !std::isfinite(joint.max_velocity) ||
      joint.max_velocity <= 0.0 ||
      !joint.axis.allFinite() ||
      joint.axis.norm() < 1e-9)
    {
      RCLCPP_ERROR(
                this->get_logger(),
                "Invalid joint dynamics parameters."
      );

      return;
    }
  }

  // Extract y-offsets created by joint seats

  y_offset_ = joint_arm1_arm2->parent_to_joint_origin_transform.position.y +
    joint_arm2_arm3->parent_to_joint_origin_transform.position.y;

  // Extract z-offset created by the height of the base and pedestal

  z_offset_ = joint_base_pedestal->parent_to_joint_origin_transform.position.z +
    joint_pedestal_arm1->parent_to_joint_origin_transform.position.z;

  robot_description_ready_ = true;

  RCLCPP_INFO(
        this->get_logger(),
        "Loaded robotic arm parameters:\n"
        "\tArm1 length: %.3f m\n"
        "\tArm2 length: %.3f m\n"
        "\tArm3 length: %.3f m\n"
        "\tJoint_pedestal_arm1 axis: [%.3f, %.3f, %.3f]\n"
        "\tJoint_arm1_arm2 axis: [%.3f, %.3f, %.3f]\n"
        "\tJoint_arm2_arm3 axis: [%.3f, %.3f, %.3f]",
        link_dynamics_[0].length,
        link_dynamics_[1].length,
        link_dynamics_[2].length,
        joint_dynamics_[0].axis(0),
        joint_dynamics_[0].axis(1),
        joint_dynamics_[0].axis(2),
        joint_dynamics_[1].axis(0),
        joint_dynamics_[1].axis(1),
        joint_dynamics_[1].axis(2),
        joint_dynamics_[2].axis(0),
        joint_dynamics_[2].axis(1),
        joint_dynamics_[2].axis(2)
  );
}

void RobotArm::jointStateTopicCallback(sensor_msgs::msg::JointState::UniquePtr message)
{
  if (!robot_description_ready_) {
    return;
  }

  if (message->name.size() != message->position.size() || message->position.size() < 3) {
    RCLCPP_WARN(this->get_logger(),
            "Incomplete joint state data... Joints: %d. Joint angles: %d",
      static_cast<int>(message->name.size()), static_cast<int>(message->position.size())
    );

    return;
  }

  const std::array<std::string, 3> joint_names = {
    "joint_pedestal_arm1",
    "joint_arm1_arm2",
    "joint_arm2_arm3"
  };

  Eigen::Vector3d new_joint_angles;

  for (int i = 0; i < 3; ++i) {
    const auto joint_name_index = std::find(message->name.begin(), message->name.end(),
      joint_names[i]);

    if (joint_name_index == message->name.end()) {
      RCLCPP_WARN(
                this->get_logger(),
                "Missing joint state for %s.",
                joint_names[i].c_str()
      );

      return;
    }

    const auto index = std::distance(message->name.begin(), joint_name_index);

    const double angle = message->position[index];

    if (!std::isfinite(angle)) {
      RCLCPP_WARN(
                this->get_logger(),
                "Non-finite joint angle for %s.",
                joint_names[i].c_str()
      );
      return;
    }

    new_joint_angles[i] = angle;
  }

  joint_angles_ = new_joint_angles;

  end_effector_position_ = Kinematics::forwardKinematics(
    {joint_angles_[0], joint_angles_[1], joint_angles_[2]},
    {link_dynamics_[0].length, link_dynamics_[1].length, link_dynamics_[2].length},
    {joint_dynamics_[0].axis, joint_dynamics_[1].axis, joint_dynamics_[2].axis}
  );

  Eigen::Vector3d offset(
    -y_offset_ * std::sin(joint_angles_[0]),
    y_offset_ * std::cos(joint_angles_[0]),
    z_offset_
  );

  end_effector_position_ += offset;
}

void RobotArm::publishJointStates() const
{
  sensor_msgs::msg::JointState message;

  message.header.stamp = this->get_clock()->now();

  message.name = {
    "joint_pedestal_arm1",
    "joint_arm1_arm2",
    "joint_arm2_arm3"
  };

  message.position = {
    joint_angles_[0],
    joint_angles_[1],
    joint_angles_[2]
  };

  message.velocity = {
    joint_velocities_[0],
    joint_velocities_[1],
    joint_velocities_[2],
  };

  joint_state_publisher_->publish(message);
}

void RobotArm::publishRobotState() const
{
  project_6_3r_robot_arm_interfaces::msg::RobotState message;

  const auto maximum_accelerations = computeMaximumJointAccelerations();

  message.end_effector_x = end_effector_position_.x();
  message.end_effector_y = end_effector_position_.y();
  message.end_effector_z = end_effector_position_.z();
  message.arm1_length = link_dynamics_[0].length;
  message.arm2_length = link_dynamics_[1].length;
  message.arm3_length = link_dynamics_[2].length;
  message.joint_pedestal_arm1_angle = joint_angles_[0];
  message.joint_arm1_arm2_angle = joint_angles_[1];
  message.joint_arm2_arm3_angle = joint_angles_[2];
  message.joint_pedestal_arm1_velocity = joint_velocities_[0];
  message.joint_arm1_arm2_velocity = joint_velocities_[1];
  message.joint_arm2_arm3_velocity = joint_velocities_[2];
  message.joint_pedestal_arm1_maximum_acceleration = maximum_accelerations[0];
  message.joint_arm1_arm2_maximum_acceleration = maximum_accelerations[1];
  message.joint_arm2_arm3_maximum_acceleration = maximum_accelerations[2];
  message.joint_pedestal_arm1_maximum_velocity = joint_dynamics_[0].max_velocity;
  message.joint_arm1_arm2_maximum_velocity = joint_dynamics_[1].max_velocity;
  message.joint_arm2_arm3_maximum_velocity = joint_dynamics_[2].max_velocity;
  message.x_offset = -y_offset_ * std::sin(joint_angles_[0]);
  message.y_offset = y_offset_ * std::cos(joint_angles_[0]);
  message.z_offset = z_offset_;

  robot_state_publisher_->publish(message);
}

void RobotArm::logEndEffectorPosition() const
{
  if (!robot_description_ready_ || end_effector_position_.isZero()) {
    return;
  }

  RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "End Effector located at:\n"
        "\tx: %.3f\n"
        "\ty: %.3f\n"
        "\tz: %.3f\n",
        end_effector_position_[0],
        end_effector_position_[1],
        end_effector_position_[2]
  );
}

double RobotArm::normalizeAngle(double angle) const
{
  // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
  return std::atan2(std::sin(angle), std::cos(angle));
}


Eigen::Vector3d RobotArm::computeApproximateJointInertias() const
{
  Eigen::Vector3d effective_inertia =
    Eigen::Vector3d::Zero();

  for (int i = 0; i < 3; ++i) {

    // Joint i accelerates link i and all downstream links.
    // Link 3 is the fixed end effector.
    for (int j = i; j < 4; ++j) {

      const auto & link = link_dynamics_[j];

      // Approximate distance from joint i to the downstream link origin.
      double distance = 0.0;

      for (int k = i; k < j && k < 3; ++k) {
        distance += link_dynamics_[k].length;
      }

      // Approximate distance to the link COM.
      distance += link.center_of_mass.norm();

      // Approximate rotational inertia about the joint axis using the mean principal moment.
      const double rotational_inertia = link.inertia.trace() / 3.0;

      // Parallel-axis approximation.
      effective_inertia[i] += rotational_inertia + link.mass * distance * distance;
    }
  }

  return effective_inertia;
}

Eigen::Vector3d RobotArm::computeMaximumJointAccelerations() const
{
  const Eigen::Vector3d inertias = computeApproximateJointInertias();

  Eigen::Vector3d max_accelerations;

  for (int i = 0; i < 3; ++i) {
    max_accelerations[i] = joint_dynamics_[i].max_effort / inertias[i];
  }

  return max_accelerations;
}
