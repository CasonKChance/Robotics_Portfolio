#include <project_6_2r_robot_arm/RobotArmAnalyzer.h>

#include <PlanarRoboticArmKinematics/Kinematics.h>

#include "urdf/model.hpp"

#include <numbers>
#include <vector>
#include <chrono>

using namespace std::placeholders;

/* Public Member Functions */

RobotArmAnalyzer::RobotArmAnalyzer(const rclcpp::NodeOptions & options)
: Node("robot_arm_analyzer", options)
{
  robotDescriptionSubscription_ =
    this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(
                &RobotArmAnalyzer::robotDescriptionTopicCallback,
                this,
                _1
            )
    );

  taskSpacePublisher_ = this->create_publisher<geometry_msgs::msg::Point>("/task_space",
    rclcpp::QoS(80000).reliable());

  subscriberCheckTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
            &RobotArmAnalyzer::checkTaskSpaceSubscriber,
            this
        )
  );
}

/* Private Member Functions */

void RobotArmAnalyzer::robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message)
{
  urdf::Model model;

  if (!model.initString(message->data)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to parse robot description.");

    return;
  }

  const auto elbowJoint = model.getJoint("joint_arm1_arm2");
  const auto endEffectorJoint = model.getJoint("joint_arm2_end-effector");

  if (!elbowJoint || !endEffectorJoint) {
    RCLCPP_ERROR(
            this->get_logger(),
            "Required robot arm joints not found in URDF."
    );

    return;
  }

  // Both arm segments extend along their local +X axes.
  armLink1Length_ = elbowJoint->parent_to_joint_origin_transform.position.x;
  armLink2Length_ = endEffectorJoint->parent_to_joint_origin_transform.position.x;

  if (armLink1Length_ <= 0.0 || armLink2Length_ <= 0.0) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Arm link lengths must be greater than zero."
    );

    return;
  }

  robotDescriptionReady_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded robot arm parameters:\n"
    "\tArm link 1 length: %.3f m\n"
    "\tArm link 2 length: %.3f m",
    armLink1Length_,
    armLink2Length_
  );
}


void RobotArmAnalyzer::checkTaskSpaceSubscriber()
{
  if (!robotDescriptionReady_) {
    return;
  }

  if (taskSpacePublished_) {
    return;
  }

  if (taskSpacePublisher_->get_subscription_count() == 0) {
    RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for a subscriber on /task_space..."
    );

    return;
  }

  RCLCPP_INFO(
        this->get_logger(),
        "Subscriber connected to /task_space. Publishing task space."
  );

  taskSpacePublished_ = true;
  subscriberCheckTimer_->cancel();

  publishTaskSpace();
}


void RobotArmAnalyzer::publishTaskSpace() const
{
  if (!robotDescriptionReady_) {
    RCLCPP_WARN(
            this->get_logger(),
            "Robot description not yet loaded."
    );

    return;
  }

  // Loop over values from -pi to pi in steps of 0.05 for each joint
  int totalSteps = static_cast<int>((2 * std::numbers::pi) / 0.05);
  for (int i = 0; i <= totalSteps; ++i) {
    double joint1Angle = -std::numbers::pi + i * 0.05;
    for (int j = 0; j <= totalSteps; ++j) {
      double joint2Angle = -std::numbers::pi + j * 0.05;

      const auto pointCandidate = Kinematics::forwardKinematics(
                std::vector<double>{joint1Angle, joint2Angle},
                std::vector<double>{armLink1Length_, armLink2Length_}
      );

      // Verify point
      if (Kinematics::isReachable(pointCandidate,
        std::vector<double>{armLink1Length_, armLink2Length_}))
      {
        // The library calculates in the x,y plane but our robot operates in the x,z plane.
        // The mapping is 1:1.
        geometry_msgs::msg::Point point;
        point.x = pointCandidate.x();
        point.z = pointCandidate.y();

        taskSpacePublisher_->publish(point);
      }
    }
  }

  RCLCPP_INFO(
        this->get_logger(),
        "Task space published."
  );
}
