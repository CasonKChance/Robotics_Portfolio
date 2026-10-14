#include <project_6_2r_robot_arm/RobotArmAnalyzer.h>

#include <PlanarRoboticArmKinematics/Kinematics.h>

#include "urdf/model.hpp"

#include <numbers>
#include <vector>

using namespace std::placeholders;

/* Public Member Functions */

RobotArmAnalyzer::RobotArmAnalyzer(const rclcpp::NodeOptions& options)
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

    taskSpacePublisher_ = this->create_publisher<geometry_msgs::msg::Point>("/task_space", rclcpp::QoS(10).transient_local().reliable());

    publishTaskSpace();
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

  const auto armLink1 = model.getLink("arm_link_1");
  const auto armLink2 = model.getLink("arm_link_2");

  if (!armLink1 || !armLink2) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Required robot arm links not found in URDF."
    );

    return;
  }

  const auto * armLink1Cylinder = dynamic_cast<const urdf::Cylinder *>(armLink1->collision->geometry.get());
  const auto * armLink2Cylinder = dynamic_cast<const urdf::Cylinder *>(armLink2->collision->geometry.get());

  if (!armLink1Cylinder || !armLink2Cylinder) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Arm links must be represented as cylinders"
    );
  }

  armLink1Length_ = armLink1Cylinder->length;
  armLink2Length_ = armLink2Cylinder->length;

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
    "\tArm link 2 length: %.3f m\n",
    armLink1Length_,
    armLink2Length_
  );
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

    // Loop over values from -pi to pi in steps of 0.01 for each joint
    int totalSteps = static_cast<int>((2 * std::numbers::pi) / 0.01);
    for (int i = 0; i <= totalSteps; ++i) {
        double joint1Angle = -std::numbers::pi + i * 0.01;
        for (int j = 0; j <= totalSteps; ++j) {
            double joint2Angle = -std::numbers::pi + j * 0.01;

            const auto pointCandidate = Kinematics::forwardKinematics(
                std::vector<double>{joint1Angle, joint2Angle}, 
                std::vector<double>{armLink1Length_, armLink2Length_}
            );

            // Verify point
            if (Kinematics::isReachable(pointCandidate, std::vector<double>{armLink1Length_, armLink2Length_})) {
                // The library calculates in the x,y plane but our robot operates in the x,z plane.
                // The mapping is 1:1.
                geometry_msgs::msg::Point point;
                point.x = pointCandidate.x;
                point.z = pointCandidate.y;

                taskSpacePublisher_->publish(point);
            }
        }
    }
}