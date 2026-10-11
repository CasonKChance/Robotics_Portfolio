#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("landmark");
  auto markerPublisher = node->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);
  auto positionPublisher = node->create_publisher<geometry_msgs::msg::Point>("/landmark_position",
    10);

  auto timer = node->create_wall_timer(1000ms, [&]() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.pose.position.x = 3.0;
        marker.pose.position.y = 3.0;
        marker.pose.position.z = marker.scale.z / 2.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point position;
        position.x = marker.pose.position.x;
        position.y = marker.pose.position.y;

        markerPublisher->publish(marker);
        positionPublisher->publish(position);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
