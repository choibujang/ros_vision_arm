#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "ros_interfaces/action/dispatch_manipulation_task.hpp"
#include "robot_arm_controllers/cam/cam_controller.hpp"
#include "robot_arm_controllers/arm/arm_controller.h"
#include "robot_arm_controllers/net/net_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class RobotArmNode : public rclcpp::Node
{
public:
  using DispatchManipulationTask = ros_interfaces::action::DispatchManipulationTask;
  using GoalHandleDMT = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;

  explicit RobotArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_arm_node", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<DispatchManipulationTask>(
      this,
      "dispatch_manipulation_task",
      std::bind(&RobotArmNode::handle_goal, this, _1, _2),
      std::bind(&RobotArmNode::handle_cancel, this, _1),
      std::bind(&RobotArmNode::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
  CamController cam_controller();
  NetController net_controller("192.168.0.92", 8080);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DispatchManipulationTask::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with items:");
    for (const std::string& item : goal->item_names) {
      RCLCPP_INFO(this->get_logger(), " %s", item.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "with quantities:");
    for (const int& quantity : goal->item_quantities) {
      RCLCPP_INFO(this->get_logger(), " %d", quantity);
    }

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDMT> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDMT> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotArmNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDMT> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
    auto result = std::make_shared<DispatchManipulationTask::Result>();

    for (int i = 1; (i < size(goal->item_names)) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_code = 100;
        result->error_msg = "client cancel";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->progress = i;
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      result->error_code = 0;
      result->error_msg = "";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArmNode>());
  rclcpp::shutdown();
  return 0;
}