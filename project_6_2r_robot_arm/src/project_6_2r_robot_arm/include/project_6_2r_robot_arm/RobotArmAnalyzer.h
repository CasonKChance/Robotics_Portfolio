#ifndef ROBOT_ARM_ANALYZER_H
#define ROBOT_ARM_ANALYZER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>

/**
 * @brief Analyzes the physical properties of the robot arm
 * Currently, this includes the task space
 */
class RobotArmAnalyzer: public rclcpp::Node
{
public:

  /**
   * @brief Constructs the analyzer node
   * @param options options to pass to the node
   */
  RobotArmAnalyzer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double armLink1Length_ {0.0};  // Length of the first arm link in the chain
  double armLink2Length_ {0.0};  // Length of the second arm link in the chain
  bool robotDescriptionReady_ {false}; // Describes whether or not the robot description has been loaded
  bool taskSpacePublished_ {false}; // Describes whether or not the task space has already been published

  rclcpp::Subscription < std_msgs::msg::String > ::SharedPtr robotDescriptionSubscription_;
  rclcpp::Publisher < geometry_msgs::msg::Point > ::SharedPtr taskSpacePublisher_;
  rclcpp::TimerBase::SharedPtr subscriberCheckTimer_;

  /**
   * @brief Loads and saves the robot description into member variables
   * @param message the String message containing the robot description
   */
  void robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message);

  /**
   * @brief Checks if the task space publisher has an active subscriber
   */
  void checkTaskSpaceSubscriber();

  /**
   * @brief Publishes points to the /task_space topic
   */
  void publishTaskSpace() const;
};

#endif
