#include <project_6_3r_robot_arm/RobotArm.h>

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

/* Public Member Functions */

RobotArm::RobotArm(const rclcpp::NodeOptions& options)
: Node("robot_arm", options)
{
    robot_description_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(
                &RobotArm::robotDescriptionTopicCallback,
                this,
                _1
            )
        );

    joint_state_subscription_ = 
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(
                &RobotArm::jointStateTopicCallback,
                this,
                _1
            )
    );

    update_timer_ = this->create_wall_timer(
        10ms, [this]() {
            this->logEndEffectorPosition();
        }
    );
}

/* Private Member Functions */

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
      "Required robot arm links/joints not found in URDF.");
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
      "Arm limits, inertials, or collision geometry missing.");
    return;
  }

  // Extract Arm geometry

  const auto * arm_link_1_cylinder = dynamic_cast<const urdf::Cylinder *>(arm_link_1->collision->geometry.get());
  const auto * arm_link_2_cylinder = dynamic_cast<const urdf::Cylinder *>(arm_link_2->collision->geometry.get());
  const auto * arm_link_3_cylinder = dynamic_cast<const urdf::Cylinder *>(arm_link_3->collision->geometry.get());

  if (!arm_link_1_cylinder || !arm_link_2_cylinder || !arm_link_3_cylinder) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Arm collision geometry must be cylindrical."
    );
    return;
  }

  // Extract arm lengths

    auto extractArmLength = [](const urdf::LinkConstSharedPtr& link) {
        double maxLength = 0.0;

        for (const auto& collision : link->collision_array) {
            if (!collision || !collision->geometry) {
                continue;
            }

            const auto* cylinder = dynamic_cast<const urdf::Cylinder*>(collision->geometry.get());

            if (cylinder) {
                maxLength = std::max(maxLength, cylinder->length);
            }
        }

        return maxLength;
    };

    arm1_length_ = extractArmLength(arm_link_1);
    arm2_length_ = extractArmLength(arm_link_2);
    arm3_length_ = extractArmLength(arm_link_3);

    if (arm1_length_ <= 0.0 ||
        arm2_length_ <= 0.0 ||
        arm3_length_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to extract arm lengths.");

        return;
    }

    // Extract joint axes

    joint_pedestal_arm1_axis_ << joint_pedestal_arm1->axis.x, joint_pedestal_arm1->axis.y, joint_pedestal_arm1->axis.z;
    joint_arm1_arm2_axis_ << joint_arm1_arm2->axis.x, joint_arm1_arm2->axis.y, joint_arm1_arm2->axis.z;
    joint_arm2_arm3_axis_ << joint_arm2_arm3->axis.x, joint_arm2_arm3->axis.y, joint_arm2_arm3->axis.z;

    // Extract y-offsets created by joint seats

    y_offset_ = joint_arm1_arm2->parent_to_joint_origin_transform.position.y + joint_arm2_arm3->parent_to_joint_origin_transform.position.y;

    // Extract z-offset created by the height of the base and pedestal

    z_offset_ = joint_base_pedestal->parent_to_joint_origin_transform.position.z + joint_pedestal_arm1->parent_to_joint_origin_transform.position.z;

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
        arm1_length_,
        arm2_length_,
        arm3_length_,
        joint_pedestal_arm1_axis_(0),
        joint_pedestal_arm1_axis_(1),
        joint_pedestal_arm1_axis_(2),
        joint_arm1_arm2_axis_(0),
        joint_arm1_arm2_axis_(1),
        joint_arm1_arm2_axis_(2),
        joint_arm2_arm3_axis_(0),
        joint_arm2_arm3_axis_(1),
        joint_arm2_arm3_axis_(2)
    );
}

void RobotArm::jointStateTopicCallback(sensor_msgs::msg::JointState::UniquePtr message)
{
    if (!robot_description_ready_) {
        return;
    }

    if (message->name.size() != message->position.size() || message->position.size() < 3) {
        RCLCPP_WARN(this->get_logger(), 
            "Incomplete joint state date. Joints: %d. Joint angles: %d", static_cast<int>(message->name.size()), static_cast<int>(message->position.size())
        );
    }

    joint_pedestal_arm1_angle_ = message->position[0];
    joint_arm1_arm2_angle_ = message->position[1];
    joint_arm2_arm3_angle_ = message->position[2];

    end_effector_position_ = Kinematics::forwardKinematics(
        {joint_pedestal_arm1_angle_, joint_arm1_arm2_angle_, joint_arm2_arm3_angle_},
        {arm1_length_, arm2_length_, arm3_length_},
        {joint_pedestal_arm1_axis_, joint_arm1_arm2_axis_, joint_arm2_arm3_axis_}
    );

    Eigen::Vector3d offset(
        -y_offset_ * std::sin(joint_pedestal_arm1_angle_),
        y_offset_ * std::cos(joint_pedestal_arm1_angle_),
        z_offset_
    );

    end_effector_position_ += offset;
}

void RobotArm::logEndEffectorPosition() const
{
    if (!robot_description_ready_ || end_effector_position_.isZero()) {
        return;
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2500,
        "End Effector located at:\n"
        "\tx: %.3f\n"
        "\ty: %.3f\n"
        "\tz: %.3f\n",
        end_effector_position_.x(),
        end_effector_position_.y(),
        end_effector_position_.z()
    );
}
