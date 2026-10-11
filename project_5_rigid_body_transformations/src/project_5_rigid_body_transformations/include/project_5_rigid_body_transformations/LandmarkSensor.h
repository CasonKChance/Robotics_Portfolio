#ifndef LANDMARK_SENSOR_H
#define LANDMARK_SENSOR_H

#include <Transformation/Transformation.h>
#include <Transformation/Rotation.h>

#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <memory>

/**
 * @brief The LandmarkSensor class simulates a sensor reading and transmitting data regarding an object in the world.
 * It subscribes to a topic that provides synthetic sensor data describing the position of a landmark in the world frame,
 * it then transforms this data into the sensor's frame of reference and publishes it to another topic.
 */
class LandmarkSensor: public rclcpp::Node
{
public:

  /**
   * @brief Construct a new Landmark Sensor object
   */
  explicit LandmarkSensor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  Eigen::Matrix4d T_world_robot_;    // The transform describing the robot's pose in the World frame.
  Eigen::Matrix4d T_robot_sensor_;    // The transform describing the sensor's position in the robot's frame.

  rclcpp::Publisher < geometry_msgs::msg::Point > ::SharedPtr landmarkPublisher_;
  rclcpp::Subscription < geometry_msgs::msg::Point > ::SharedPtr landmarkSubscription_;

  std::shared_ptr < tf2_ros::TransformListener > tfListener_ {nullptr};
  std::unique_ptr < tf2_ros::Buffer > tfBuffer_;

  /**
   * @brief Retrieves the transforms describing World->robot and robot->sensor.
   */
  void getTransforms();

  /**
   * @brief Callback function for handling incoming landmark messages.
   * @param msg The incoming landmark message.
   */
  void landmarkTopicCallback(const geometry_msgs::msg::Point::SharedPtr msg);

  /**
   * @brief Publishes the position of a landmark in the sensor's frame of reference.
   * @param landmarkPosition The position of the landmark in the sensor's frame.
   */
  void publishLandmarkPosition(const Eigen::Vector3d & landmarkPosition);
};

#endif
