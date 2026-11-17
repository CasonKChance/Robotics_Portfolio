#include <project_7_treasure_hunting_robot/VelocityLogger.h>

#include <functional>

using namespace std::placeholders;

static constexpr double kGravityConstant = 9.8;
static constexpr double kAccelerationTolerance = 5e-3;

/* Public Member Functions */

VelocityLogger::VelocityLogger(const rclcpp::NodeOptions & options)
: rclcpp::Node("velocity_logger", options)
{
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        10,
        std::bind(&VelocityLogger::imuTopicCallback, this, _1)
    );

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        std::bind(&VelocityLogger::odomTopicCallback, this, _1)
    );
}

/* Private Member Functions */

void VelocityLogger::imuTopicCallback(sensor_msgs::msg::Imu::UniquePtr msg)
{
    rclcpp::Time current_time(msg->header.stamp);

    if (!has_received_imu_data_) {
        last_imu_time_ = current_time;
        has_received_imu_data_ = true;

        logCurrentTwist();

        return;
    }

    const double dt = (current_time - last_imu_time_).seconds();

    // Discard negative and zero time skips
    if (dt <= 0.0) {
        return;
    }

    last_imu_time_ = current_time;

    current_twist_.linear_velocity_x += msg->linear_acceleration.x * dt;
    current_twist_.linear_velocity_y += msg->linear_acceleration.y * dt;
    current_twist_.linear_velocity_z += (msg->linear_acceleration.z * dt) - (kGravityConstant * dt);
    current_twist_.angular_velocity_x = msg->angular_velocity.x;
    current_twist_.angular_velocity_y = msg->angular_velocity.y;
    current_twist_.angular_velocity_z = msg->angular_velocity.z;

    logCurrentTwist();
}

void VelocityLogger::OdomTopicCallback(nav_msgs::msg::Odometry::UniquePtr msg)
{
    // Use odom data to determine when the robot is stationary
    if (std::abs(msg->twist.twist.linear.x) <= kAccelerationTolerance &&
        std::abs(msg->twist.twist.linear.y) <= kAccelerationTolerance &&
        std::abs(msg->twist.twist.angular.z) <= kAccelerationTolerance)
    {
        current_twist_.linear_velocity_x = 0.0;
        current_twist_.linear_velocity_y = 0.0;
        current_twist_.linear_velocity_z = 0.0;
        current_twist_.angular_velocity_x = 0.0;
        current_twist_.angular_velocity_y = 0.0;
        current_twist_.angular_velocity_z = 0.0;
    }
}

void VelocityLogger::logCurrentTwist()
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "\tLinear Velocity (m/s):\n"
        "\t\tx: %.2f\n"
        "\t\ty: %.2f\n"
        "\t\tz: %.2f\n"
        "\tAngularVelocity (rad/s):\n"
        "\t\tx: %.2f\n"
        "\t\ty: %.2f\n"
        "\t\tz: %.2f\n",
        current_twist_.linear_velocity_x,
        current_twist_.linear_velocity_y,
        current_twist_.linear_velocity_z,
        current_twist_.angular_velocity_x,
        current_twist_.angular_velocity_y,
        current_twist_.angular_velocity_z
    );

    static bool initialized = false;

    if (!initialized) {
        // Clear the screen and move cursor to top-left (1,1)
        // Set scroll region from row 11 to the bottom of the terminal (\033[11;999r)
        // Move cursor to row 11 so subsequent ROS logs start below the speedometer
        std::cout << "\033[2J\033[1;1H\033[11;999r\033[11;1H";
        initialized = true;
    }

    char buffer[256];
    std::snprintf(
        buffer, sizeof(buffer),
        "\n\tLinear Velocity (m/s):\033[K\n"
        "\t\tx: %7.2f\033[K\n"
        "\t\ty: %7.2f\033[K\n"
        "\t\tz: %7.2f\033[K\n"
        "\tAngular Velocity (rad/s):\033[K\n"
        "\t\tx: %7.2f\033[K\n"
        "\t\ty: %7.2f\033[K\n"
        "\t\tz: %7.2f\033[K",
        current_twist_.linear_velocity_x,
        current_twist_.linear_velocity_y,
        current_twist_.linear_velocity_z,
        current_twist_.angular_velocity_x,
        current_twist_.angular_velocity_y,
        current_twist_.angular_velocity_z
    );

    // Save current cursor position (\033[s), jump to top-left (\033[1;1H),
    // print speedometer, then restore cursor position (\033[u)
    std::cout << "\033[s\033[1;1H" << buffer << "\033[u" << std::flush;
}