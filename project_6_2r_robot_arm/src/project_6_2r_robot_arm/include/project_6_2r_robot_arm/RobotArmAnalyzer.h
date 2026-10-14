#ifndef ROBOT_ARM_ANALYZER_H
#define ROBOT_ARM_ANALYZER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>

class RobotArmAnalyzer: public rclcpp::Node
{
public:

    RobotArmAnalyzer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    double armLink1Length_ {0.0};
    double armLink2Length_ {0.0};
    bool robotDescriptionReady_ {false};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr taskSpacePublisher_;

    void robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message);

    void publishTaskSpace() const;
};

#endif