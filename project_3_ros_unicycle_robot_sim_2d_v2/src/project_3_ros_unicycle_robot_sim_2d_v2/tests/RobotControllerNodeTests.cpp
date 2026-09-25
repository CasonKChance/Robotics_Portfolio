#include <project_3_ros_unicycle_robot_sim_2d_v2/RobotControllerNode.h>

#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/action/go_to_pose.hpp"
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/msg/robot_state.hpp"
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/msg/simulator_status.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

using GoToPose =
  project_3_ros_unicycle_robot_sim_2d_v2_interfaces::action::GoToPose;

using RobotStateMessage =
  project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::RobotState;

using SimulatorStatus =
  project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::SimulatorStatus;

using GoToPoseClient =
  rclcpp_action::Client<GoToPose>;

using GoalHandleGoToPose =
  rclcpp_action::ClientGoalHandle<GoToPose>;


/**
 * @brief Integration tests for RobotControllerNode.
 *
 * The controller is tested through its public ROS interfaces:
 *
 *   - go_to_pose action
 *   - cmd_vel topic
 *   - robot_state topic
 *   - simulator_status topic
 *
 * The robot_state publisher acts as a deterministic mock robot. This lets
 * tests exercise the controller state machine without depending on the
 * simulator's physical integration loop.
 */
class RobotControllerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ =
      std::make_shared<RobotControllerNode>();

    testNode_ =
      std::make_shared<rclcpp::Node>(
        "robot_controller_test_node");

    robotStatePublisher_ =
      testNode_->create_publisher<RobotStateMessage>(
        "robot_state",
        10);

    simulatorStatusPublisher_ =
      testNode_->create_publisher<SimulatorStatus>(
        "simulator_status",
        10);

    commandVelocitySubscription_ =
      testNode_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
      [this](geometry_msgs::msg::Twist::UniquePtr message)
      {
        std::lock_guard<std::mutex> lock(commandMutex_);
        receivedCommands_.push_back(*message);
        });

    actionClient_ =
      rclcpp_action::create_client<GoToPose>(
        testNode_,
        "go_to_pose");

    executor_ =
      std::make_unique<
      rclcpp::executors::SingleThreadedExecutor>();

    executor_->add_node(controller_);
    executor_->add_node(testNode_);

    ASSERT_TRUE(
      actionClient_->wait_for_action_server(
        std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
      executor_->remove_node(controller_);
      executor_->remove_node(testNode_);
    }

    actionClient_.reset();

    commandVelocitySubscription_.reset();
    simulatorStatusPublisher_.reset();
    robotStatePublisher_.reset();

    testNode_.reset();
    controller_.reset();

    executor_.reset();
  }

  bool spinUntil(
    const std::function<bool()> & condition,
    std::chrono::milliseconds timeout = 2000ms)
  {
    const auto start = std::chrono::steady_clock::now();

    while (
      std::chrono::steady_clock::now() - start < timeout)
    {
      executor_->spin_some();

      if (condition()) {
        return true;
      }

      std::this_thread::sleep_for(5ms);
    }

    return false;
  }

  void spinFor(std::chrono::milliseconds duration = 2000ms)
  {
    const auto start = std::chrono::steady_clock::now();

    while (
      std::chrono::steady_clock::now() - start < duration)
    {
      executor_->spin_some();

      std::this_thread::sleep_for(5ms);
    }
  }

  void publishRobotState(
    double x,
    double y,
    double theta,
    double linearVelocity = 0.0,
    double angularVelocity = 0.0)
  {
    RobotStateMessage message;

    message.x = x;
    message.y = y;
    message.theta = theta;
    message.linear_velocity = linearVelocity;
    message.angular_velocity = angularVelocity;

    robotStatePublisher_->publish(message);

    spinFor(25ms);
  }

  geometry_msgs::msg::Twist latestCommand()
  {
    std::lock_guard<std::mutex> lock(commandMutex_);

    if (receivedCommands_.empty()) {
      return geometry_msgs::msg::Twist();
    }

    return receivedCommands_.back();
  }

  std::size_t commandCount()
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    return receivedCommands_.size();
  }

  void clearCommands()
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    receivedCommands_.clear();
  }

  std::shared_ptr<GoalHandleGoToPose> sendGoal(
    double x,
    double y,
    double theta)
  {
    GoToPose::Goal goal;

    goal.x = x;
    goal.y = y;
    goal.theta = theta;

    auto future =
      actionClient_->async_send_goal(goal);

    const bool receivedGoalHandle =
      spinUntil(
      [&]() {
        return future.wait_for(0ms) ==
               std::future_status::ready;
      });

    EXPECT_TRUE(receivedGoalHandle);

    if (!receivedGoalHandle) {
      return nullptr;
    }

    return future.get();
  }

  typename GoalHandleGoToPose::WrappedResult waitForResult(
    const std::shared_ptr<GoalHandleGoToPose> & goalHandle,
    std::chrono::milliseconds timeout = 3000ms)
  {
    auto resultFuture =
      actionClient_->async_get_result(goalHandle);

    const bool resultReceived =
      spinUntil(
      [&]() {
        return resultFuture.wait_for(0ms) ==
               std::future_status::ready;
      },
      timeout);

    EXPECT_TRUE(resultReceived);

    if (!resultReceived) {
      typename GoalHandleGoToPose::WrappedResult emptyResult;
      return emptyResult;
    }

    return resultFuture.get();
  }

  rclcpp::Node::SharedPtr testNode_;

  std::shared_ptr<RobotControllerNode> controller_;

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>
  executor_;

  GoToPoseClient::SharedPtr actionClient_;

  rclcpp::Publisher<RobotStateMessage>::SharedPtr
    robotStatePublisher_;

  rclcpp::Publisher<SimulatorStatus>::SharedPtr
    simulatorStatusPublisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    commandVelocitySubscription_;

  std::vector<geometry_msgs::msg::Twist>
  receivedCommands_;

  std::mutex commandMutex_;
};


/* -------------------------------------------------------------------------- */
/* Construction / Parameters                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Controller should construct with its documented default parameters.
 */
TEST_F(
  RobotControllerNodeTest,
  DefaultConstructionUsesExpectedParameters)
{
  EXPECT_DOUBLE_EQ(
    controller_->get_parameter("robot.max_linear_velocity").as_double(),
    5.0);

  EXPECT_DOUBLE_EQ(
    controller_->get_parameter("robot.max_angular_velocity").as_double(),
    std::numbers::pi);

  EXPECT_DOUBLE_EQ(
    controller_->get_parameter("robot.linear_acceleration").as_double(),
    2.5);

  EXPECT_DOUBLE_EQ(
    controller_->get_parameter("robot.angular_acceleration").as_double(),
    std::numbers::pi / 2.0);

  EXPECT_DOUBLE_EQ(
    controller_->get_parameter(
      "robot.goal_pose_positional_tolerance").as_double(),
    0.01);

  EXPECT_DOUBLE_EQ(
    controller_->get_parameter(
      "robot.goal_pose_heading_tolerance").as_double(),
    0.0174533);
}


/**
 * @brief Controller should honor configured physical limits and tolerances.
 */
TEST_F(
  RobotControllerNodeTest,
  ConfiguredParametersAreLoaded)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override(
    "robot.max_linear_velocity",
    2.0);

  options.append_parameter_override(
    "robot.max_angular_velocity",
    1.5);

  options.append_parameter_override(
    "robot.linear_acceleration",
    1.0);

  options.append_parameter_override(
    "robot.angular_acceleration",
    0.5);

  options.append_parameter_override(
    "robot.goal_pose_positional_tolerance",
    0.05);

  options.append_parameter_override(
    "robot.goal_pose_heading_tolerance",
    0.1);

  auto node =
    std::make_shared<RobotControllerNode>(options);

  EXPECT_DOUBLE_EQ(
    node->get_parameter(
      "robot.max_linear_velocity").as_double(),
    2.0);

  EXPECT_DOUBLE_EQ(
    node->get_parameter(
      "robot.max_angular_velocity").as_double(),
    1.5);

  EXPECT_DOUBLE_EQ(
    node->get_parameter(
      "robot.linear_acceleration").as_double(),
    1.0);

  EXPECT_DOUBLE_EQ(
    node->get_parameter(
      "robot.angular_acceleration").as_double(),
    0.5);

  EXPECT_DOUBLE_EQ(
    node->get_parameter(
      "robot.goal_pose_positional_tolerance").as_double(),
    0.05);

  EXPECT_DOUBLE_EQ(
    node->get_parameter(
      "robot.goal_pose_heading_tolerance").as_double(),
    0.1);

  node.reset();
}


/* -------------------------------------------------------------------------- */
/* Goal Acceptance / Rejection                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief A valid goal should be accepted when the controller is idle.
 */
TEST_F(
  RobotControllerNodeTest,
  GoalIsAcceptedWhenControllerIsIdle)
{
  publishRobotState(0.0, 0.0, 0.0);

  auto goalHandle =
    sendGoal(5.0, 0.0, 0.0);

  ASSERT_NE(goalHandle, nullptr);

  EXPECT_TRUE(goalHandle->get_goal_id() != rclcpp_action::GoalUUID{});
}


/**
 * @brief A second goal should be rejected while another goal is active.
 *
 * RobotControllerNode explicitly rejects goals whenever activeGoalHandle_
 * is non-null.
 */
TEST_F(
  RobotControllerNodeTest,
  SecondGoalIsRejectedWhileFirstGoalIsActive)
{
  publishRobotState(0.0, 0.0, 0.0);

  auto firstGoal =
    sendGoal(100.0, 0.0, 0.0);

  ASSERT_NE(firstGoal, nullptr);

  auto secondGoal =
    sendGoal(200.0, 0.0, 0.0);

  EXPECT_EQ(secondGoal, nullptr);
}


/* -------------------------------------------------------------------------- */
/* Rotation To Goal Position                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief The controller should rotate positively when the goal is to +Y.
 */
TEST_F(
  RobotControllerNodeTest,
  RotatesCounterClockwiseTowardGoalPosition)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  clearCommands();

  auto goalHandle =
    sendGoal(
    0.0,
    5.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  const bool commandReceived =
    spinUntil(
    [&]() {
      return commandCount() > 0;
    });

  ASSERT_TRUE(commandReceived);

  const auto command = latestCommand();

  EXPECT_DOUBLE_EQ(
    command.linear.x,
    0.0);

  EXPECT_DOUBLE_EQ(
    command.angular.z,
    std::numbers::pi);
}


/**
 * @brief The controller should rotate negatively when the goal is to -Y.
 */
TEST_F(
  RobotControllerNodeTest,
  RotatesClockwiseTowardGoalPosition)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  clearCommands();

  auto goalHandle =
    sendGoal(
    0.0,
    -5.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  const bool commandReceived =
    spinUntil(
    [&]() {
      return commandCount() > 0;
    });

  ASSERT_TRUE(commandReceived);

  const auto command = latestCommand();

  EXPECT_DOUBLE_EQ(
    command.linear.x,
    0.0);

  EXPECT_DOUBLE_EQ(
    command.angular.z,
    -std::numbers::pi);
}


/**
 * @brief A robot already facing the target position should not rotate.
 */
TEST_F(
  RobotControllerNodeTest,
  DoesNotRotateWhenAlreadyFacingGoalPosition)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  clearCommands();

  auto goalHandle =
    sendGoal(
    5.0,
    0.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  const bool commandReceived =
    spinUntil(
    [&]() {
      return commandCount() > 0;
    });

  ASSERT_TRUE(commandReceived);

  const auto command = latestCommand();

  EXPECT_DOUBLE_EQ(
    command.linear.x,
    0.0);

  EXPECT_DOUBLE_EQ(
    command.angular.z,
    0.0);
}


/* -------------------------------------------------------------------------- */
/* Driving To Goal                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Once oriented toward the goal, the controller should drive forward
 * at the configured maximum linear velocity.
 */
TEST_F(
  RobotControllerNodeTest,
  DrivesForwardTowardGoalPosition)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    5.0,
    0.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  /*
   * First control-loop iteration transitions:
   *
   * RotatingToGoalPosition
   *   -> WaitingForRotationToGoalPositionStop
   *
   * because no rotation is required.
   *
   * The next iteration transitions:
   *
   * WaitingForRotationToGoalPositionStop
   *   -> DrivingToGoalPose
   *
   * The following iteration publishes the forward command.
   */
  clearCommands();

  const bool driveCommandReceived =
    spinUntil(
    [&]() {
      const auto command = latestCommand();

      return commandCount() > 0 &&
             std::abs(command.linear.x) > 0.0;
    });

  ASSERT_TRUE(driveCommandReceived);

  const auto command = latestCommand();

  EXPECT_DOUBLE_EQ(
    command.linear.x,
    5.0);

  EXPECT_DOUBLE_EQ(
    command.angular.z,
    0.0);
}


/**
 * @brief The controller should command zero velocity when the remaining
 * distance is within stopping distance.
 */
TEST_F(
  RobotControllerNodeTest,
  StopsWhenWithinLinearStoppingDistance)
{
  /*
   * With v = 1 m/s and a = 2.5 m/s^2:
   *
   * stopping distance = v^2 / (2a)
   *                   = 1 / 5
   *                   = 0.2 m
   *
   * Place the goal only 0.1 m away.
   */
  publishRobotState(
    0.0,
    0.0,
    0.0,
    1.0,
    0.0);

  auto goalHandle =
    sendGoal(
    0.1,
    0.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  clearCommands();

  const bool stopCommandReceived =
    spinUntil(
    [&]() {
      if (commandCount() == 0) {
        return false;
      }

      const auto command = latestCommand();

      return
        std::abs(command.linear.x) < 1e-9 &&
        std::abs(command.angular.z) < 1e-9;
    });

  EXPECT_TRUE(stopCommandReceived);
}


/* -------------------------------------------------------------------------- */
/* Final Rotation                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief The controller should rotate positively to a positive final heading.
 */
TEST_F(
  RobotControllerNodeTest,
  RotatesPositiveTowardFinalHeading)
{
  /*
   * Start already at the desired position.
   *
   * The controller will:
   *
   *   rotate-to-position
   *   drive
   *   rotate-to-final-heading
   */
  publishRobotState(
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    0.0,
    0.0,
    std::numbers::pi / 2.0);

  ASSERT_NE(goalHandle, nullptr);

  /*
   * Keep the robot stationary at the goal position so the controller
   * progresses through its state machine.
   */
  clearCommands();

  const bool positiveRotationReceived =
    spinUntil(
    [&]() {
      if (commandCount() == 0) {
        return false;
      }

      const auto command = latestCommand();

      return command.angular.z > 0.0;
    });

  ASSERT_TRUE(positiveRotationReceived);

  EXPECT_DOUBLE_EQ(
    latestCommand().angular.z,
    std::numbers::pi);
}


/**
 * @brief The controller should rotate negatively to a negative final heading.
 */
TEST_F(
  RobotControllerNodeTest,
  RotatesNegativeTowardFinalHeading)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    0.0,
    0.0,
    -std::numbers::pi / 2.0);

  ASSERT_NE(goalHandle, nullptr);

  clearCommands();

  const bool negativeRotationReceived =
    spinUntil(
    [&]() {
      if (commandCount() == 0) {
        return false;
      }

      return latestCommand().angular.z < 0.0;
    });

  ASSERT_TRUE(negativeRotationReceived);

  EXPECT_DOUBLE_EQ(
    latestCommand().angular.z,
    -std::numbers::pi);
}


/* -------------------------------------------------------------------------- */
/* Feedback                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief The action should publish feedback while actively moving.
 */
TEST_F(
  RobotControllerNodeTest,
  PublishesFeedbackWhileDriving)
{
  std::mutex feedbackMutex;

  std::vector<GoToPose::Feedback> feedbackMessages;

  rclcpp_action::Client<GoToPose>::SendGoalOptions options;

  options.feedback_callback =
    [&](GoalHandleGoToPose::SharedPtr,
    const std::shared_ptr<const GoToPose::Feedback> feedback)
    {
      std::lock_guard<std::mutex> lock(feedbackMutex);
      feedbackMessages.push_back(*feedback);
    };

  publishRobotState(
    0.0,
    0.0,
    0.0);

  GoToPose::Goal goal;
  goal.x = 5.0;
  goal.y = 0.0;
  goal.theta = 0.0;

  auto goalFuture =
    actionClient_->async_send_goal(
    goal,
    options);

  ASSERT_TRUE(
    spinUntil(
      [&]() {
        return goalFuture.wait_for(0ms) ==
               std::future_status::ready;
      }));

  auto goalHandle = goalFuture.get();

  ASSERT_NE(goalHandle, nullptr);

  const bool feedbackReceived =
    spinUntil(
    [&]() {
      std::lock_guard<std::mutex> lock(feedbackMutex);
      return !feedbackMessages.empty();
    });

  EXPECT_TRUE(feedbackReceived);

  if (feedbackReceived) {
    std::lock_guard<std::mutex> lock(feedbackMutex);

    EXPECT_GT(
      feedbackMessages.back().distance_remaining,
      0.0);
  }
}


/* -------------------------------------------------------------------------- */
/* Complete Goal Execution                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief A goal should eventually succeed when robot state updates show that
 * the robot has completed each stage of the maneuver.
 */
TEST_F(
  RobotControllerNodeTest,
  GoalSucceedsAfterCompleteThreeStageManeuver)
{
  /*
   * Goal:
   *
   *   position = (1, 0)
   *   heading  = 0
   *
   * Robot starts at:
   *
   *   position = (0, 0)
   *   heading  = 0
   *
   * No initial rotation is required.
   */
  publishRobotState(
    0.0,
    0.0,
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    1.0,
    0.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  /*
   * Let the controller begin driving.
   */
  const bool driveCommandReceived =
    spinUntil(
    [&]() {
      if (commandCount() == 0) {
        return false;
      }

      return latestCommand().linear.x > 0.0;
    });

  ASSERT_TRUE(driveCommandReceived);

  /*
   * Simulate reaching the goal and stopping.
   */
  publishRobotState(
    1.0,
    0.0,
    0.0,
    0.0,
    0.0);

  auto wrappedResult =
    waitForResult(goalHandle);

  EXPECT_EQ(
    wrappedResult.code,
    rclcpp_action::ResultCode::SUCCEEDED);

  EXPECT_NEAR(
    wrappedResult.result->x,
    1.0,
    1e-9);

  EXPECT_NEAR(
    wrappedResult.result->y,
    0.0,
    1e-9);

  EXPECT_NEAR(
    wrappedResult.result->theta,
    0.0,
    1e-9);
}


/**
 * @brief The controller should correct position if it reaches its final
 * state but remains outside the configured positional tolerance.
 */
TEST_F(
  RobotControllerNodeTest,
  CorrectsPositionOutsideGoalTolerance)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    1.0,
    0.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  /*
   * Allow the controller to start driving.
   */
  ASSERT_TRUE(
    spinUntil(
      [&]() {
        return commandCount() > 0 &&
               latestCommand().linear.x > 0.0;
      }));

  /*
   * Put the robot near, but not within, the final positional tolerance.
   */
  publishRobotState(
    0.5,
    0.0,
    0.0,
    0.0,
    0.0);

  clearCommands();

  /*
   * The controller should eventually issue another forward command rather
   * than immediately succeeding.
   */
  const bool correctionStarted =
    spinUntil(
    [&]() {
      if (commandCount() == 0) {
        return false;
      }

      return latestCommand().linear.x > 0.0;
    });

  EXPECT_TRUE(correctionStarted);
}


/**
 * @brief The controller should correct heading if position is within tolerance
 * but heading is outside the configured heading tolerance.
 */
TEST_F(
  RobotControllerNodeTest,
  CorrectsHeadingOutsideGoalTolerance)
{
  publishRobotState(
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    1.0,
    0.0,
    std::numbers::pi / 2.0);

  ASSERT_NE(goalHandle, nullptr);

  /*
   * Let the controller start its maneuver.
   */
  ASSERT_TRUE(
    spinUntil(
      [&]() {
        return commandCount() > 0;
      }));

  /*
   * Simulate arriving at the goal position but with an incorrect heading.
   */
  publishRobotState(
    1.0,
    0.0,
    0.0,
    0.0,
    0.0);

  clearCommands();

  /*
   * The controller should rotate rather than succeed.
   */
  const bool correctionStarted =
    spinUntil(
    [&]() {
      if (commandCount() == 0) {
        return false;
      }

      return std::abs(latestCommand().angular.z) > 0.0;
    });

  EXPECT_TRUE(correctionStarted);
}

/* -------------------------------------------------------------------------- */
/* Robot State Subscription                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Robot state updates should affect the controller's commanded motion.
 */
TEST_F(
  RobotControllerNodeTest,
  RobotStateUpdatesAreUsedByController)
{
  /*
   * Goal is directly ahead.
   */
  publishRobotState(
    0.0,
    0.0,
    0.0);

  auto goalHandle =
    sendGoal(
    5.0,
    0.0,
    0.0);

  ASSERT_NE(goalHandle, nullptr);

  /*
   * The robot should initially drive forward.
   */
  ASSERT_TRUE(
    spinUntil(
      [&]() {
        return commandCount() > 0 &&
               latestCommand().linear.x > 0.0;
      }));

  /*
   * Move the mock robot to the goal.
   */
  publishRobotState(
    5.0,
    0.0,
    0.0,
    0.0,
    0.0);

  auto wrappedResult =
    waitForResult(goalHandle);

  EXPECT_EQ(
    wrappedResult.code,
    rclcpp_action::ResultCode::SUCCEEDED);
}


/* -------------------------------------------------------------------------- */
/* Simulator Status                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief A simulator status of false should shut down ROS.
 */
TEST_F(
  RobotControllerNodeTest,
  SimulatorStoppedStatusShutsDownRos)
{
  SimulatorStatus message;
  message.is_simulator_running = false;

  simulatorStatusPublisher_->publish(message);

  const bool shutdownObserved =
    spinUntil(
    []() {
      return !rclcpp::ok();
    });

  EXPECT_TRUE(shutdownObserved);
}


/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
