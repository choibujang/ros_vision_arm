#include <functional>
#include <memory>
#include <thread>
#include <cstring>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "deli_arm_interfaces/action/dispatch_manipulation_task.hpp"
#include "deli_arm_controller.h"

namespace deli_arm
{
class DeliArmRos : public rclcpp::Node
{
public:
  using DispatchManipulationTask = deli_arm_interfaces::action::DispatchManipulationTask;
  using ProductDetection = deli_arm_interfaces::srv::ProductDetection
  using GoalHandleDispatchManipulationTask = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;

  explicit DeliArmRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("deli_arm_ros", options),
    deli_arm_controller_(std::make_shared<DeliArmController>()),
    deli_cam_controller_(std::make_shared<DeliArmController>())
  {
    using namespace std::placeholders;

    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DispatchManipulationTask::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with target station %s", goal->target_station.c_str());
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

    this->action_server_ = rclcpp_action::create_server<DispatchManipulationTask>(
      this,
      "DispatchManipulationTask",
      handle_goal,
      handle_cancel,
      handle_accepted);
    
    this->service_client_ = this->create_client<ProductDetection>("ProductDetectService");
    
  }

private:
  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
  rclcpp::Client<ProductDetection>::SharedPtr service_client_;
  std::shared_ptr<DeliArmController> deli_arm_controller_;
  std::shared_ptr<DeliCamController> deli_cam_controller_;

  void execute(const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();

    std::string target = goal->target_station;
    std::vector<std::string> items = goal->item_names;
    std::vector<int32_t> quantities = goal->item_quantities;

    auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
    auto result = std::make_shared<DispatchManipulationTask::Result>();
    auto request = std::make_shared<ProductDetection>();

    request->item_names = items;
    request->item_quantities = quantities;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    std::vector<float> target_x = result.get()->x;
    std::vector<float> target_y = result.get()->y;
    std::vector<float> target_z = result.get()->z;


    for (int i = 0; (i < 5) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_code = 1000;
        result->error_msg = "client cancel";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
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
      result->error_msg = "success";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

};  // class FibonacciActionServer

}  // namespace custom_action_cpp


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<deli_arm::DeliArmActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}