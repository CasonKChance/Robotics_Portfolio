#include <project_5_rigid_body_transformations/LandmarkSensor.h>

#include <chrono>

using namespace std::chrono_literals;

/* Public Member Functions */

LandmarkSensor::LandmarkSensor(const rclcpp::NodeOptions & options)
: rclcpp::Node("landmark_sensor", options)
{
    landmarkSubscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/landmark_position", 10,
        std::bind(&LandmarkSensor::landmarkTopicCallback, this, std::placeholders::_1));
    landmarkPublisher_ = this->create_publisher<geometry_msgs::msg::Point>("/sensor_data", 10);

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

/* Private Member Functions */

void LandmarkSensor::getTransforms()
{
    rclcpp::Time now = this->get_clock()->now();

    auto Twb = tfBuffer_->lookupTransform("world", "base_link", now, 50ms);
    Eigen::Quaterniond quaternionTwb = Eigen::Quaterniond(Twb.transform.rotation.w, Twb.transform.rotation.x, Twb.transform.rotation.y, Twb.transform.rotation.z);
    Eigen::Matrix3d rotationTwb = quaternionTwb.toRotationMatrix();
    Eigen::Vector3d translationTwb(Twb.transform.translation.x, Twb.transform.translation.y, Twb.transform.translation.z);
    worldToRobotTransform_ = Transformation::makeTransform(rotationTwb, translationTwb);

    auto Tbs = tfBuffer_->lookupTransform("base_link", "landmark_sensor_link", tf2::TimePointZero);
    Eigen::Quaterniond quaternionTbs = Eigen::Quaterniond(Tbs.transform.rotation.w, Tbs.transform.rotation.x, Tbs.transform.rotation.y, Tbs.transform.rotation.z);
    Eigen::Matrix3d rotationTbs = quaternionTbs.toRotationMatrix();
    Eigen::Vector3d translationTbs(Tbs.transform.translation.x, Tbs.transform.translation.y, Tbs.transform.translation.z);
    robotToSensorTransform_ = Transformation::makeTransform(rotationTbs, translationTbs);
}

void LandmarkSensor::landmarkTopicCallback(const geometry_msgs::msg::Point::SharedPtr message)
{
    getTransforms();

    Eigen::Vector3d landmarkPosition(message->x, message->y, message->z);

    // Transform the position from the world frame to the sensor frame
    landmarkPosition = (Transformation::inverseTransform(worldToRobotTransform_ * robotToSensorTransform_) * landmarkPosition.homogeneous()).head<3>();

    publishLandmarkPosition(landmarkPosition);
}

void LandmarkSensor::publishLandmarkPosition(const Eigen::Vector3d & landmarkPosition)
{
    auto message = geometry_msgs::msg::Point();
    message.x = landmarkPosition.x();
    message.y = landmarkPosition.y();
    message.z = landmarkPosition.z();

    landmarkPublisher_->publish(message);
}