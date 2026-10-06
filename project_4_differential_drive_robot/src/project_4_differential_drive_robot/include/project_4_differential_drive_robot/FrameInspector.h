#ifndef FRAME_INSPECTOR_H
#define FRAME_INSPECTOR_H

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

class FrameInspector: public rclcpp::Node
{
public:
  explicit FrameInspector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::string fromFrame_;
  std::string toFrame_;
  std::shared_ptr < tf2_ros::TransformListener > tfListener_ {nullptr};
  std::unique_ptr < tf2_ros::Buffer > tfBuffer_;
  rclcpp::TimerBase::SharedPtr timer_ {nullptr};

  void timerCallback() const;
};

#endif
