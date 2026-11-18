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

    bool operator==(const Coordinates& other) const
    {
        return x == other.x &&
               y == other.y &&
               z == other.z;
    }
};

struct CoordinatesHash
{
    std::size_t operator()(const Coordinates& c) const
    {
        std::size_t h1 = std::hash<double>{}(c.x);
        std::size_t h2 = std::hash<double>{}(c.y);
        std::size_t h3 = std::hash<double>{}(c.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class CoordinatesProcessor: public rclcpp::Node
{
public:

    explicit CoordinatesProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    Coordinates current_hint_coordinates_;
    std::unordered_set<Coordinates, CoordinatesHash> found_hints_{};

    rclcpp::Subscription < geometry_msgs::msg::Point > ::SharedPtr hint_coordinates_subscription_;
    rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr visualization_marker_publisher_;

    void hintCoordinatesTopicCallback(geometry_msgs::msg::Point::UniquePtr msg);

    void publishVisualizationMarker();
};

#endif