#ifndef COORDINATES_PROCESSOR_H
#define COORDINATES_PROCESSOR_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "project_7_treasure_hunting_robot_interfaces/msg/hint_coordinates.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <memory>
#include <unordered_set>

using HintCoordinates = project_7_treasure_hunting_robot_interfaces::msg::HintCoordinates;

/**
 * @brief Stores the 3D coordinates of a hint.
 */
struct Coordinates
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    /**
     * @brief Compares two coordinate sets for equality.
     * @param other Coordinates to compare against
     * @return True if all coordinate components are equal, false otherwise
     */
    bool operator==(const Coordinates& other) const
    {
        return x == other.x &&
               y == other.y &&
               z == other.z;
    }
};

/**
 * @brief Provides a hash function for storing coordinates in an unordered set.
 */
struct CoordinatesHash
{
    /**
     * @brief Computes a combined hash of the coordinate components.
     * @param c Coordinates to hash
     * @return Hash value representing the coordinates
     */
    std::size_t operator()(const Coordinates& c) const
    {
        std::size_t h1 = std::hash<double>{}(c.x);
        std::size_t h2 = std::hash<double>{}(c.y);
        std::size_t h3 = std::hash<double>{}(c.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * @brief Processes hint coordinates, corrects robot odometry drift, and publishes visualization markers.
 */
class CoordinatesProcessor: public rclcpp::Node
{
public:

    /**
     * @brief Constructs the coordinates processor and initializes ROS interfaces and TF2 transformations.
     * @param options Configuration options for Node initialization
     */
    explicit CoordinatesProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    Coordinates current_hint_coordinates_;
    std::unordered_set<Coordinates, CoordinatesHash> found_hints_{};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription < HintCoordinates > ::SharedPtr hint_coordinates_subscription_;
    rclcpp::Subscription < sensor_msgs::msg::Imu > ::SharedPtr imu_subscription_;
    rclcpp::Publisher < visualization_msgs::msg::Marker > ::SharedPtr visualization_marker_publisher_;
    geometry_msgs::msg::TransformStamped map_to_odom_tf_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    /**
     * @brief Processes received hint coordinates and corrects robot positioning data.
     * @param msg Message containing the robot's current position and next hint coordinates
     */
    void hintCoordinatesTopicCallback(HintCoordinates::UniquePtr msg);

    /**
     * @brief Processes IMU orientation data to correct odometry heading drift.
     * @param msg Message containing the robot's IMU measurements
     */
    void imuTopicCallback(sensor_msgs::msg::Imu::UniquePtr msg);

    /**
     * @brief Corrects odometry position drift using known location coordinates.
     * @param current_coordinates Known robot coordinates in the map frame
     */
    void correctOdometryPositioning(const Coordinates& current_coordinates);

    /**
     * @brief Publishes a circular visualization marker at the new hint coordinates.
     */
    void publishVisualizationMarker();

    /**
     * @brief Normalizes an angle to the range [-pi, pi].
     * @param angle Angle to normalize in radians
     * @return Normalized angle in radians
     */
    double normalizeAngle(double angle) const;
};

#endif