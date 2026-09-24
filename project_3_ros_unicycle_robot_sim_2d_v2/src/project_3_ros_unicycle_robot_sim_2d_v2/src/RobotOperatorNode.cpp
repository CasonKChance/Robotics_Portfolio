#include "project_3_ros_unicycle_robot_sim_2d_v2/RobotOperatorNode.h"

using GoToPose = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

using namespace std::chrono_literals;

/* Public Member Functions */

RobotOperatorNode::RobotOperatorNode(const rclcpp::NodeOptions & options)
: Node("robot_operator_node", options)
{
  setupGoToPoseClient();

  userInputThread_ = std::thread([this]() {
        this->receiveGoalPose();
  });
}

RobotOperatorNode::~RobotOperatorNode()
{
  if (userInputThread_.joinable()) {
    userInputThread_.join();
  }
}

/* Private Member Functions */

void RobotOperatorNode::setupGoToPoseClient()
{
  this->goToPoseActionClient_ = rclcpp_action::create_client<GoToPose>(
        this,
        "go_to_pose");
}

void RobotOperatorNode::receiveGoalPose()
{
  Pose goalPose{0.0, 0.0, 0.0};

  while (rclcpp::ok()) {
    if (!this->goToPoseActionClient_->wait_for_action_server(1000ms)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for Robot Controller connection...");
      continue;
    }

    std::cout << "\n---Input pose to send to robot---\n";

    goalPose.x = getValidInput("Input x position: ");
    goalPose.y = getValidInput("Input y position: ");

    double theta = getValidInput("Input heading (in radians): ");
    goalPose.theta = normalizeAngle(theta);

    std::cout << "---------------------------------\n";

    RCLCPP_INFO(
            this->get_logger(),
            "Received valid Pose:\n"
            "\tx: %.2f\n"
            "\ty: %.2f\n"
            "\ttheta: %.2f",
            goalPose.x, goalPose.y, goalPose.theta);

    auto resultFuture = sendGoalPose(goalPose);

    if (!resultFuture.valid()) {
      RCLCPP_ERROR(this->get_logger(), "Robot Controller disconnected. Resetting operator...");
      setupGoToPoseClient();
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for robot to complete movement...");

    bool completed = false;
    while (rclcpp::ok() && !completed) {
      auto status = resultFuture.wait_for(500ms);

      if (status == std::future_status::ready) {
        completed = true;
      } else if (!this->goToPoseActionClient_->wait_for_action_server(1000ms)) {
        RCLCPP_ERROR(this->get_logger(), "Robot Controller disconnected. Resetting operator...");
        setupGoToPoseClient();
        break;
      }
    }

    if (!completed) {
      continue;
    }

    GoalHandleGoToPose::WrappedResult result = resultFuture.get();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Result received: \n"
                                  "\tx: %.2f\n"
                                  "\ty: %.2f\n"
                                  "\ttheta: %.2f", result.result->x, result.result->y,
        result.result->theta);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Goal was aborted.");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        return;
    }
  }
}

std::shared_future<GoalHandleGoToPose::WrappedResult> RobotOperatorNode::sendGoalPose(
  const Pose & goalPose)
{
  if (!this->goToPoseActionClient_->wait_for_action_server(1000ms)) {
    RCLCPP_WARN(this->get_logger(), "Robot Controller not available after waiting.");

    return {};
  }

  auto goalMessage = GoToPose::Goal();
  goalMessage.x = goalPose.x;
  goalMessage.y = goalPose.y;
  goalMessage.theta = goalPose.theta;

  RCLCPP_INFO(this->get_logger(), "Sending goal.");

  auto sendGoalOptions = rclcpp_action::Client<GoToPose>::SendGoalOptions();
  sendGoalOptions.goal_response_callback = [this](const GoalHandleGoToPose::SharedPtr & goalHandle)
    {
      if (!goalHandle) {
        RCLCPP_WARN(this->get_logger(), "Goal was rejected by server.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result.");
      }
    };

  sendGoalOptions.feedback_callback = [this](
    GoalHandleGoToPose::SharedPtr,
    const std::shared_ptr<const GoToPose::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "\nDistance remaining: %.2f\n"
      "Rotation remaining: %.2f",
      feedback->distance_remaining, feedback->rotation_remaining);
    };

  auto goalHandleFuture = this->goToPoseActionClient_->async_send_goal(goalMessage,
    sendGoalOptions);

  auto goalHandle = goalHandleFuture.get();

  if (!goalHandle) {
    return {};
  }

  return this->goToPoseActionClient_->async_get_result(goalHandle);
}

double RobotOperatorNode::getValidInput(const std::string & prompt) const
{
  double value;

  while (true) {
    std::cout << prompt << std::flush;

    if (std::cin >> value) {
      return value;
    }

    std::cout
      << "Invalid input. Please enter a valid numerical value.\n"
      << std::flush;

    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}

double RobotOperatorNode::normalizeAngle(double angle) const
{
  // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
  return std::atan2(std::sin(angle), std::cos(angle));
}
