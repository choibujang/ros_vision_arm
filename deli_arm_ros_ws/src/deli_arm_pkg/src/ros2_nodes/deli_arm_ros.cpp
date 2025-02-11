#include <functional>
#include <memory>
#include <thread>
#include <cstring>
#include <cstddef>
#include <vector>
#include <string>
#include <unordered_map>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "deli_interfaces/action/dispatch_manipulation_task.hpp"
#include "deli_interfaces/srv/product_detection.hpp"
#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"


namespace deli_arm
{
class DeliArmRos : public rclcpp::Node
{
public:
  using DispatchManipulationTask = deli_interfaces::action::DispatchManipulationTask;
  using GoalHandleDispatchManipulationTask = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;

  explicit DeliArmRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("deli_arm_ros", options),
    deli_arm_controller_(std::make_shared<DeliArmController>()),
    deli_cam_controller_(std::make_shared<DeliCamController>())
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
    
    this->service_client_ = this->create_client<deli_interfaces::srv::ProductDetection>("ProductDetectService");
    
  }

private:
  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
  rclcpp::Client<deli_interfaces::srv::ProductDetection>::SharedPtr service_client_;
  std::shared_ptr<DeliArmController> deli_arm_controller_;
  std::shared_ptr<DeliCamController> deli_cam_controller_;

  void execute(const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);

    deli_cam_controller_->startCam();

    const auto goal = goal_handle->get_goal();

    auto request = std::make_shared<deli_interfaces::srv::ProductDetection::Request>();

    auto future = service_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "서비스 응답 수신!");


      std::vector<std::string> items = goal->item_names;
      std::vector<int32_t> quantities = goal->item_quantities;

      std::unordered_map<std::string, int> product_counts;
      for (size_t i = 0; i < items.size(); i++) {
        product_counts[items[i]] = quantities[i];
      }

      std::unordered_map<std::string, std::vector<std::tuple<int,int,int,int>>> product_locations;
      for (size_t i = 0; i < response->product_names.size(); i++) {
        product_locations[response->product_names[i]].push_back({response->width[i], response->height[i], response->cx[i], response->cy[i]}); 
      }

      for (const auto& [key, value] : product_locations) {
        std::cout << "Product: " << key << "\n";

        for (const auto& t : value) {
          int width, height, cx, cy;
          std::tie(width, height, cx, cy) = t;
          std::cout << "Coordinates: (" << width << ", " << height << ", " << cx << ", " << cy << ")\n";
        }
      }
      std::cout << std::endl;

    }

    auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
    auto result = std::make_shared<DispatchManipulationTask::Result>();


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
  auto node = std::make_shared<deli_arm::DeliArmRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}