#include <project_7_treasure_hunting_robot/CoordinatesProcessor.h>

#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

static constexpr double kHintMarkerRadius = 0.6;
static constexpr double kHintMarkerLineWidth = 0.03;
static constexpr double kRobotSpawnYaw = 1.57;

/* Public Member Functions */

CoordinatesProcessor::CoordinatesProcessor(const rclcpp::NodeOptions & options)
: rclcpp::Node("coordinates_processor", options)
{
    current_hint_coordinates_ = Coordinates{0.0, 0.0, 0.0};
    found_hints_.insert(current_hint_coordinates_);

    hint_coordinates_subscription_ = this->create_subscription<HintCoordinates>(
        "hint_coordinates",
        10,
        std::bind(&CoordinatesProcessor::hintCoordinatesTopicCallback, this, _1)
    );

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        10,
        std::bind(&CoordinatesProcessor::imuTopicCallback, this, _1)
    );

    visualization_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker",
        rclcpp::QoS(1).transient_local()
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Set up map->odom transform
    map_to_odom_tf_.header.frame_id = "map";
    map_to_odom_tf_.child_frame_id = "odom";
    map_to_odom_tf_.transform.translation.x = 0.0;
    map_to_odom_tf_.transform.translation.y = 0.0;
    map_to_odom_tf_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, kRobotSpawnYaw);
    map_to_odom_tf_.transform.rotation.x = q.x();
    map_to_odom_tf_.transform.rotation.y = q.y();
    map_to_odom_tf_.transform.rotation.z = q.z();
    map_to_odom_tf_.transform.rotation.w = q.w();

    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() {
            map_to_odom_tf_.header.stamp = this->get_clock()->now();
            tf_broadcaster_->sendTransform(map_to_odom_tf_);
        }
    );
}

/* Private Member Functions */

void CoordinatesProcessor::hintCoordinatesTopicCallback(HintCoordinates::UniquePtr msg)
{
    Coordinates new_coordinates = Coordinates{msg->next_hint_x, msg->next_hint_y, msg->next_hint_z};
    Coordinates current_coordinates = Coordinates{msg->current_hint_x, msg->current_hint_y, msg->current_hint_z};

    correctOdometryPositioning(current_coordinates);

    if (found_hints_.contains(new_coordinates)) {
        return;
    }

    RCLCPP_INFO(
        this->get_logger(), 
        "\n\tNew hint coordinates given: \n"
        "\t\tx: %.2f\n"
        "\t\ty: %.2f\n"
        "\t\tz: %.2f",
        new_coordinates.x,
        new_coordinates.y,
        new_coordinates.z
    );

    current_hint_coordinates_ = new_coordinates;
    found_hints_.insert(current_hint_coordinates_);

    publishVisualizationMarker();
}

void CoordinatesProcessor::imuTopicCallback(sensor_msgs::msg::Imu::UniquePtr msg)
{
    // Use IMU data to correct odom heading drift
    geometry_msgs::msg::TransformStamped odom_to_base;

    try {
        odom_to_base = tf_buffer_->lookupTransform(
            "odom", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Could not transform: %s", ex.what());
      return;
    }
    
    double odom_yaw = tf2::getYaw(odom_to_base.transform.rotation);
    double imu_yaw = tf2::getYaw(msg->orientation) + kRobotSpawnYaw;

    // Set map -> odom rotation so: map_to_odom_yaw + odom_yaw == imu_yaw
    double map_to_odom_yaw = imu_yaw - odom_yaw;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_to_odom_yaw);
    map_to_odom_tf_.transform.rotation = tf2::toMsg(q);
}

void CoordinatesProcessor::correctOdometryPositioning(const Coordinates& current_coordinates)
{
    geometry_msgs::msg::TransformStamped map_to_base;

    try {
        map_to_base = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Could not transform: %s", ex.what());
      return;
    }

    double error_x = current_coordinates.x - map_to_base.transform.translation.x;
    double error_y = current_coordinates.y - map_to_base.transform.translation.y;
    double error_z = current_coordinates.z - map_to_base.transform.translation.z;

    map_to_odom_tf_.transform.translation.x += error_x;
    map_to_odom_tf_.transform.translation.y += error_y;
    map_to_odom_tf_.transform.translation.z += error_z;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "\n\tCorrecting robot odometry position to: \n"
        "\t\tx: %.2f\n"
        "\t\ty: %.2f\n"
        "\t\tz: %.2f",
        current_coordinates.x,
        current_coordinates.y,
        current_coordinates.z
    );

    map_to_odom_tf_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(map_to_odom_tf_);
}

void CoordinatesProcessor::publishVisualizationMarker()
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "hint_marker";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = current_hint_coordinates_.x;
    marker.pose.position.y = current_hint_coordinates_.y;
    marker.pose.position.z = current_hint_coordinates_.z;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = kHintMarkerLineWidth;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    constexpr int kSegments = 48;
    marker.points.reserve(kSegments + 1);

    for (int i = 0; i <= kSegments; ++i) {
        double theta = (2.0 * M_PI * i) / kSegments;
        geometry_msgs::msg::Point p;
        p.x = kHintMarkerRadius * std::cos(theta);
        p.y = kHintMarkerRadius * std::sin(theta);
        p.z = 0.0;
        marker.points.push_back(p);
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "\n\tPublishing marker at: \n"
        "\t\tx: %.2f\n"
        "\t\ty: %.2f\n"
        "\t\tz: %.2f",
        current_hint_coordinates_.x,
        current_hint_coordinates_.y,
        current_hint_coordinates_.z
    );

    visualization_marker_publisher_->publish(marker);
}