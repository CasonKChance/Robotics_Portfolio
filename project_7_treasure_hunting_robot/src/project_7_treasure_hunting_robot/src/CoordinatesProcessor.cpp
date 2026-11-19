#include <project_7_treasure_hunting_robot/CoordinatesProcessor.h>

#include "tf2/exceptions.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::placeholders;

static constexpr double kHintMarkerRadius = 0.05;
static constexpr double kHintMarkerHeight = 1.0;

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

    visualization_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker",
        rclcpp::QoS(1).transient_local()
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
        "\n\tCurrent hint coordinates given: \n"
        "\t\tx: %.2f\n"
        "\t\ty: %.2f\n"
        "\t\tz: %.2f",
        current_coordinates.x,
        current_coordinates.y,
        current_coordinates.z
    );

    current_hint_coordinates_ = new_coordinates;
    found_hints_.insert(current_hint_coordinates_);

    publishVisualizationMarker();
}

void CoordinatesProcessor::correctOdometryPositioning(const Coordinates& current_coordinates)
{
    geometry_msgs::msg::TransformStamped existing_tf;

    try {
      existing_tf = tf_buffer_->lookupTransform(
          "map", "odom", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Could not transform: %s", ex.what());
      return;
    }

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";

    tf.transform.translation.x = current_coordinates.x;
    tf.transform.translation.y = current_coordinates.y;
    tf.transform.translation.z = current_coordinates.z;

    tf.transform.rotation = existing_tf.transform.rotation;

    broadcaster_->sendTransform(tf);
}

void CoordinatesProcessor::publishVisualizationMarker()
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "hint_marker";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = current_hint_coordinates_.x;
    marker.pose.position.y = current_hint_coordinates_.y;
    marker.pose.position.z = current_hint_coordinates_.z + (kHintMarkerHeight / 2);

    marker.pose.orientation.w = 1.0;

    marker.scale.x = kHintMarkerRadius * 2;
    marker.scale.y = kHintMarkerRadius * 2;
    marker.scale.z = kHintMarkerHeight;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

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