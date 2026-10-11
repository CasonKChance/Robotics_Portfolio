#ifndef LANDMARK_SENSOR_H
#define LANDMARK_SENSOR_H

#include <Transformation/Transformation.h>
#include <Transformation/Rotation.h>

#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <memory>

class LandmarkSensor: public rclcpp::Node
{
public:
    
    explicit LandmarkSensor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    Eigen::Matrix4d worldToRobotTransform_;
    Eigen::Matrix4d robotToSensorTransform_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr landmarkPublisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr landmarkSubscription_;

    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

    void getTransforms();

    void landmarkTopicCallback(const geometry_msgs::msg::Point::SharedPtr msg);

    void publishLandmarkPosition(const Eigen::Vector3d & landmarkPosition);
};

#endif
