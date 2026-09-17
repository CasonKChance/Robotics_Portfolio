#include <project_3_ros_unicycle_robot_sim_2d_v2/CommandVelocityPublisherNode.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class CommandVelocityPublisherNodeTest : public ::testing::Test {
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

// Helper function to spin nodes until a condition is met or timeout occurs
bool spinUntil(
  rclcpp::Node::SharedPtr node1, rclcpp::Node::SharedPtr node2,
  const std::function<bool()> & condition, std::chrono::milliseconds timeout = 500ms)
{
  auto start = std::chrono::steady_clock::now();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node1);
  executor.add_node(node2);

  while (std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

// Test 1: Verify default parameters publish zero velocity
TEST_F(CommandVelocityPublisherNodeTest, TestDefaultPublishing) {
    auto node = std::make_shared<CommandVelocityPublisherNode>();
    auto sub_node = std::make_shared<rclcpp::Node>("test_subscriber");

    geometry_msgs::msg::Twist received_msg;
    bool message_received = false;

    auto sub = sub_node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
    [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_msg = *msg;
      message_received = true;
    });

    // Spin until timer triggers and publisher sends message
    bool success = spinUntil(node, sub_node, [&]() {return message_received;});

    ASSERT_TRUE(success);
    EXPECT_DOUBLE_EQ(received_msg.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received_msg.angular.z, 0.0);
}

// Test 2: Verify custom parameters and angle normalization implicitly through publication
TEST_F(CommandVelocityPublisherNodeTest, TestCustomPublishingAndNormalization) {
    rclcpp::NodeOptions options;
    // Set angular_velocity to 7.0 rad, which should normalize to ~0.7168 rad [-pi, pi]
    options.append_parameter_override("linear_velocity", 2.5);
    options.append_parameter_override("angular_velocity", 7.0);

    auto node = std::make_shared<CommandVelocityPublisherNode>(options);
    auto sub_node = std::make_shared<rclcpp::Node>("test_subscriber");

    geometry_msgs::msg::Twist received_msg;
    bool message_received = false;

    auto sub = sub_node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
    [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_msg = *msg;
      message_received = true;
    });

    bool success = spinUntil(node, sub_node, [&]() {return message_received;});

    ASSERT_TRUE(success);
    EXPECT_DOUBLE_EQ(received_msg.linear.x, 2.5);

    // Expect 7.0 mapped into [-pi, pi] -> (7.0 - 2*pi) ≈ 0.7168146
    double expected_normalized_angular = std::atan2(std::sin(7.0), std::cos(7.0));
    EXPECT_NEAR(received_msg.angular.z, expected_normalized_angular, 1e-4);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
