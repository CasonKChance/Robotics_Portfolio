#ifndef FRAME_INSPECTOR_H
#define FRAME_INSPECTOR_H

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

/**
 * @brief Allows inspection of the transformation between two arbitrary coordninate frames within the system.
 */
class FrameInspector: public rclcpp::Node
{
public:
  /**
   * @brief Constructs the frame inspector node and initializes the tf listener and buffer.
   */
  explicit FrameInspector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::string fromFrame_;
  std::string toFrame_;
  std::shared_ptr < tf2_ros::TransformListener > tfListener_ {nullptr};
  std::unique_ptr < tf2_ros::Buffer > tfBuffer_;
  rclcpp::TimerBase::SharedPtr timer_ {nullptr};

  /**
   * @brief Callback for the timer that inspects the transformation between the specified frames.
   */
  void timerCallback() const;
};

#endif
