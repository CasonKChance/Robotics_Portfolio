#include <project_4_differential_drive_robot/FrameInspector.h>

using namespace std::chrono_literals;

/* Public Member Functions */

FrameInspector::FrameInspector(const rclcpp::NodeOptions & options)
: rclcpp::Node("frame_inspector", options)
{
  fromFrame_ = this->declare_parameter<std::string>("from_frame", "base_link");
  toFrame_ = this->declare_parameter<std::string>("to_frame", "odom");

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this);

  timer_ = this->create_wall_timer(1s, std::bind(&FrameInspector::timerCallback, this));
}

/* Private Member Functions */

void FrameInspector::timerCallback() const
{
  geometry_msgs::msg::TransformStamped t;

  rclcpp::Time now = this->get_clock()->now();
  try {
    t = tfBuffer_->lookupTransform(
        toFrame_.c_str(),
        fromFrame_.c_str(),
        tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
        fromFrame_.c_str(), toFrame_.c_str(), ex.what());
    return;
  }

  RCLCPP_INFO(
        this->get_logger(),
        "Transform from [%s] to [%s]:\n"
        "  Translation: [x: %.3f, y: %.3f, z: %.3f]\n"
        "  Rotation:    [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
        fromFrame_.c_str(),
        toFrame_.c_str(),
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z,
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
  );
}
