#ifndef COORDINATES_PROCESSOR_H
#define COORDINATES_PROCESSOR_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "project_7_treasure_hunting_robot_interfaces/msg/hint_coordinates.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <memory>
#include <unordered_set>

using HintCoordinates = project_7_treasure_hunting_robot_interfaces::msg::HintCoordinates;

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

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription < HintCoordinates > ::SharedPtr hint_coordinates_subscription_;
    rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr visualization_marker_publisher_;
    geometry_msgs::msg::TransformStamped map_to_odom_tf_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    void hintCoordinatesTopicCallback(HintCoordinates::UniquePtr msg);

    void correctOdometryPositioning(const Coordinates& current_coordinates);

    void publishVisualizationMarker();
};

#endif