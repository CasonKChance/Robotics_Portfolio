#include <project_3_ros_unicycle_robot_sim_2d_v2/SimulatorNode.h>
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/srv/send_world_data.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class SimulatorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Spins the executor until a condition is satisfied or timeout occurs.
   */
  static bool spinUntil(
    rclcpp::executors::SingleThreadedExecutor & executor,
    const std::function<bool()> & condition,
    std::chrono::milliseconds timeout = 2000ms)
  {
    const auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < timeout) {
      executor.spin_some();

      if (condition()) {
        return true;
      }

      std::this_thread::sleep_for(10ms);
    }

    return false;
  }

  /**
   * @brief Creates NodeOptions containing a complete world configuration.
   */
  static rclcpp::NodeOptions makeWorldOptions(
    bool goalEnabled = false,
    double goalX = 0.0,
    double goalY = 0.0,
    double goalRadius = 0.0)
  {
    rclcpp::NodeOptions options;

    options.append_parameter_override("world.bounds.max_x", 10.0);
    options.append_parameter_override("world.bounds.max_y", 10.0);

    options.append_parameter_override("world.goal.enabled", goalEnabled);
    options.append_parameter_override("world.goal.x", goalX);
    options.append_parameter_override("world.goal.y", goalY);
    options.append_parameter_override("world.goal.radius", goalRadius);

    options.append_parameter_override(
      "world.obstacles.x",
      std::vector<double>{});

    options.append_parameter_override(
      "world.obstacles.y",
      std::vector<double>{});

    options.append_parameter_override(
      "world.obstacles.radius",
      std::vector<double>{});

    return options;
  }
};


/* Construction/Default State */

/**
 * The simulator should start in the Running state when there is no goal or
 * obstacle overlapping the robot's initial pose.
 */
TEST_F(SimulatorNodeTest, DefaultConstructionStartsRunning)
{
  auto node = std::make_shared<SimulatorNode>();

  EXPECT_EQ(
    node->getStatus(),
    SimulationStatus::Running);
}


/**
 * A goal that does not overlap the robot should leave the simulation running.
 */
TEST_F(SimulatorNodeTest, GoalOutsideRobotStartsRunning)
{
  auto options = makeWorldOptions(
    true,
    8.0,
    8.0,
    1.0);

  auto node = std::make_shared<SimulatorNode>(options);

  EXPECT_EQ(
    node->getStatus(),
    SimulationStatus::Running);
}


/**
 * A goal centered on the robot's initial pose should immediately produce
 * GoalReached.
 *
 * SimulatorNode initializes the robot at (1.0, 1.0, 0.0).
 */
TEST_F(SimulatorNodeTest, GoalAtInitialPoseReportsGoalReached)
{
  auto options = makeWorldOptions(
    true,
    1.0,
    1.0,
    0.5);

  auto node = std::make_shared<SimulatorNode>(options);

  EXPECT_EQ(
    node->getStatus(),
    SimulationStatus::GoalReached);
}


/**
 * An obstacle centered on the robot's initial pose should immediately
 * produce ObstacleCollision.
 */
TEST_F(SimulatorNodeTest, ObstacleAtInitialPoseReportsCollision)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override(
    "world.obstacles.x",
    std::vector<double>{1.0});

  options.append_parameter_override(
    "world.obstacles.y",
    std::vector<double>{1.0});

  options.append_parameter_override(
    "world.obstacles.radius",
    std::vector<double>{0.5});

  auto node = std::make_shared<SimulatorNode>(options);

  EXPECT_EQ(
    node->getStatus(),
    SimulationStatus::ObstacleCollision);
}


/**
 * When the goal is disabled, goal coordinates should have no effect.
 */
TEST_F(SimulatorNodeTest, DisabledGoalDoesNotTerminateSimulation)
{
  auto options = makeWorldOptions(
    false,
    1.0,
    1.0,
    5.0);

  auto node = std::make_shared<SimulatorNode>(options);

  EXPECT_EQ(
    node->getStatus(),
    SimulationStatus::Running);
}


/* Parameter Validation */

/**
 * All three obstacle arrays must contain the same number of elements.
 */
TEST_F(SimulatorNodeTest, MismatchedObstacleParametersThrow)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override(
    "world.obstacles.x",
    std::vector<double>{2.0, 4.0, 6.0});

  options.append_parameter_override(
    "world.obstacles.y",
    std::vector<double>{4.0, 2.0});

  options.append_parameter_override(
    "world.obstacles.radius",
    std::vector<double>{1.0, 1.0, 1.75});

  EXPECT_THROW(
    std::make_shared<SimulatorNode>(options),
    std::runtime_error);
}


/**
 * An obstacle outside the world should be rejected by World.
 */
TEST_F(SimulatorNodeTest, OutOfBoundsObstacleThrows)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override(
    "world.obstacles.x",
    std::vector<double>{11.0});

  options.append_parameter_override(
    "world.obstacles.y",
    std::vector<double>{5.0});

  options.append_parameter_override(
    "world.obstacles.radius",
    std::vector<double>{1.0});

  EXPECT_THROW(
    std::make_shared<SimulatorNode>(options),
    std::invalid_argument);
}


/**
 * An enabled goal outside the world should be rejected by World.
 */
TEST_F(SimulatorNodeTest, OutOfBoundsGoalThrows)
{
  auto options = makeWorldOptions(
    true,
    11.0,
    5.0,
    1.0);

  EXPECT_THROW(
    std::make_shared<SimulatorNode>(options),
    std::invalid_argument);
}


/* Configured World */

/**
 * Verify that multiple obstacles can be loaded successfully.
 *
 * We cannot directly inspect SimulatorNode::world_ because it is private,
 * so the test verifies the configured world through its observable behavior.
 */
TEST_F(SimulatorNodeTest, MultipleObstaclesLoadSuccessfully)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override(
    "world.obstacles.x",
    std::vector<double>{2.0, 4.0, 6.0});

  options.append_parameter_override(
    "world.obstacles.y",
    std::vector<double>{4.0, 2.0, 6.0});

  options.append_parameter_override(
    "world.obstacles.radius",
    std::vector<double>{1.0, 1.0, 1.75});

  auto node = std::make_shared<SimulatorNode>(options);

  EXPECT_EQ(
    node->getStatus(),
    SimulationStatus::Running);
}


/* ROS cmd_vel Integration */

/**
 * Verify that a positive linear velocity command is received by SimulatorNode
 * and causes the robot to eventually collide with an obstacle positioned
 * directly ahead of its initial pose.
 *
 * Robot initial pose:
 *   x = 1.0
 *   y = 1.0
 *   theta = 0.0
 *
 * Therefore positive linear velocity moves the robot in +X.
 */
TEST_F(SimulatorNodeTest, LinearVelocityCommandMovesRobotIntoObstacle)
{
  rclcpp::NodeOptions options;

  // Place an obstacle directly in front of the robot.
  options.append_parameter_override(
    "world.obstacles.x",
    std::vector<double>{1.30});

  options.append_parameter_override(
    "world.obstacles.y",
    std::vector<double>{1.0});

  options.append_parameter_override(
    "world.obstacles.radius",
    std::vector<double>{0.05});

  auto simulator = std::make_shared<SimulatorNode>(options);

  ASSERT_EQ(
    simulator->getStatus(),
    SimulationStatus::Running);

  auto commandNode =
    std::make_shared<rclcpp::Node>("simulator_test_command_publisher");

  auto publisher =
    commandNode->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(simulator);
  executor.add_node(commandNode);

  // Give DDS discovery time to establish the publisher/subscriber connection.
  std::this_thread::sleep_for(100ms);

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.angular.z = 0.0;

  publisher->publish(command);

  const bool collisionDetected = spinUntil(
    executor,
    [&]() {
      return simulator->getStatus() ==
             SimulationStatus::ObstacleCollision;
    },
    2s);

  EXPECT_TRUE(collisionDetected);
  EXPECT_EQ(
    simulator->getStatus(),
    SimulationStatus::ObstacleCollision);
}


/**
 * Verify that an angular velocity command is accepted by the simulator.
 *
 * The robot initially faces +X. A positive angular velocity turns it toward
 * +Y. An obstacle is placed above the robot's initial position so that the
 * resulting curved trajectory eventually reaches it.
 */
TEST_F(SimulatorNodeTest, AngularVelocityCommandCausesCurvedPathCollision)
{
  rclcpp::NodeOptions options;

  // Obstacle positioned in the +Y direction.
    options.parameter_overrides({
    {"world.goal.enabled", false},
    {"world.obstacles.x", std::vector<double>{1.7}},
    {"world.obstacles.y", std::vector<double>{1.3}},
    {"world.obstacles.radius", std::vector<double>{0.15}}
    });

  auto simulator = std::make_shared<SimulatorNode>(options);

  ASSERT_EQ(
    simulator->getStatus(),
    SimulationStatus::Running);

  auto commandNode =
    std::make_shared<rclcpp::Node>("simulator_test_angular_command_publisher");

  auto publisher =
    commandNode->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(simulator);
  executor.add_node(commandNode);

  std::this_thread::sleep_for(100ms);

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.angular.z = 1.0;

  publisher->publish(command);

  const bool collisionDetected = spinUntil(
    executor,
    [&]() {
      return simulator->getStatus() ==
             SimulationStatus::ObstacleCollision;
    },
    3s);

  EXPECT_TRUE(collisionDetected);
  EXPECT_EQ(
    simulator->getStatus(),
    SimulationStatus::ObstacleCollision);
}


/* Goal Integration */

/**
 * A commanded forward motion should eventually cause the robot to reach a
 * goal positioned directly ahead of it.
 */
TEST_F(SimulatorNodeTest, LinearVelocityCommandReachesGoal)
{
  auto options = makeWorldOptions(
    true,
    1.30,
    1.0,
    0.05);

  auto simulator = std::make_shared<SimulatorNode>(options);

  ASSERT_EQ(
    simulator->getStatus(),
    SimulationStatus::Running);

  auto commandNode =
    std::make_shared<rclcpp::Node>("simulator_test_goal_command_publisher");

  auto publisher =
    commandNode->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(simulator);
  executor.add_node(commandNode);

  std::this_thread::sleep_for(100ms);

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.angular.z = 0.0;

  publisher->publish(command);

  const bool goalReached = spinUntil(
    executor,
    [&]() {
      return simulator->getStatus() ==
             SimulationStatus::GoalReached;
    },
    2s);

  EXPECT_TRUE(goalReached);
  EXPECT_EQ(
    simulator->getStatus(),
    SimulationStatus::GoalReached);
}

/**
 * Verify that SimulatorNode correctly sends its configured world data through
 * the send_world_data service.
 *
 * The service should return:
 *   - the configured world bounds
 *   - the configured goal
 *   - all configured obstacles
 */
TEST_F(SimulatorNodeTest, SendWorldDataReturnsConfiguredWorld)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override(
    "world.bounds.max_x",
    10.0);

  options.append_parameter_override(
    "world.bounds.max_y",
    8.0);

  options.append_parameter_override(
    "world.goal.enabled",
    true);

  options.append_parameter_override(
    "world.goal.x",
    7.0);

  options.append_parameter_override(
    "world.goal.y",
    6.0);

  options.append_parameter_override(
    "world.goal.radius",
    0.5);

  options.append_parameter_override(
    "world.obstacles.x",
    std::vector<double>{2.0, 4.0, 6.0});

  options.append_parameter_override(
    "world.obstacles.y",
    std::vector<double>{1.0, 3.0, 5.0});

  options.append_parameter_override(
    "world.obstacles.radius",
    std::vector<double>{0.25, 0.5, 0.75});

  auto simulator = std::make_shared<SimulatorNode>(options);

  auto worldDataServer =
    simulator->create_service<
    project_3_ros_unicycle_robot_sim_2d_v2_interfaces::srv::SendWorldData>(
    "send_world_data",
    std::bind(
      &SimulatorNode::handleWorldDataService,
      simulator.get(),
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  auto clientNode =
    std::make_shared<rclcpp::Node>(
    "simulator_test_world_data_client");

  auto client =
    clientNode->create_client<
    project_3_ros_unicycle_robot_sim_2d_v2_interfaces::srv::SendWorldData>(
    "send_world_data");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(simulator);
  executor.add_node(clientNode);

  ASSERT_TRUE(
    client->wait_for_service(2s));

  auto request =
    std::make_shared<
    project_3_ros_unicycle_robot_sim_2d_v2_interfaces::srv::SendWorldData::Request>();

  auto future = client->async_send_request(request);

  const bool responseReceived = spinUntil(
    executor,
    [&]() {
      return future.wait_for(0ms) ==
             std::future_status::ready;
    },
    2s);

  ASSERT_TRUE(responseReceived);

  auto response = future.get();

  ASSERT_NE(response, nullptr);

  // Verify world bounds.
  EXPECT_DOUBLE_EQ(
    response->max_x,
    10.0);

  EXPECT_DOUBLE_EQ(
    response->max_y,
    8.0);

  // Verify goal.
  ASSERT_EQ(
    response->goal.size(),
    1u);

  EXPECT_DOUBLE_EQ(
    response->goal[0].center.x,
    7.0);

  EXPECT_DOUBLE_EQ(
    response->goal[0].center.y,
    6.0);

  EXPECT_DOUBLE_EQ(
    response->goal[0].radius,
    0.5);

  // Verify obstacles.
  ASSERT_EQ(
    response->obstacles.size(),
    3u);

  EXPECT_DOUBLE_EQ(
    response->obstacles[0].center.x,
    2.0);

  EXPECT_DOUBLE_EQ(
    response->obstacles[0].center.y,
    1.0);

  EXPECT_DOUBLE_EQ(
    response->obstacles[0].radius,
    0.25);

  EXPECT_DOUBLE_EQ(
    response->obstacles[1].center.x,
    4.0);

  EXPECT_DOUBLE_EQ(
    response->obstacles[1].center.y,
    3.0);

  EXPECT_DOUBLE_EQ(
    response->obstacles[1].radius,
    0.5);

  EXPECT_DOUBLE_EQ(
    response->obstacles[2].center.x,
    6.0);

  EXPECT_DOUBLE_EQ(
    response->obstacles[2].center.y,
    5.0);

  EXPECT_DOUBLE_EQ(
    response->obstacles[2].radius,
    0.75);

  EXPECT_DOUBLE_EQ(
    response->robot_pose.x,
    1.0);

  EXPECT_DOUBLE_EQ(
    response->robot_pose.y,
    1.0);

  EXPECT_DOUBLE_EQ(
    response->robot_pose.theta,
    0.0);
}

/* Main */

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  const int result = RUN_ALL_TESTS();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return result;
}
