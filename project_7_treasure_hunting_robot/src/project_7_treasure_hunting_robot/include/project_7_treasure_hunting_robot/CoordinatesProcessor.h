#ifndef COORDINATES_PROCESSOR_H
#define COORDINATES_PROCESSOR_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <unordered_set>

struct Coordinates
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
}

class CoordinatesProcessor: public rclcpp::Node
{
public:

    explicit CoordinatesProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    Coordinates current_hint_coordinates_;
    std::unordered_set<Coordinates> found_hints_{};

    rclcpp::Subscription < geometry_msgs::msg::Point > ::SharedPtr hint_coordinates_subscription_;
    rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr visualization_marker_publisher_;

    void hintCoordinatesTopicCallback(geometry_msgs::msg::Point::UniquePtr msg);

    void publishVisualizationMarker();
};

#endif