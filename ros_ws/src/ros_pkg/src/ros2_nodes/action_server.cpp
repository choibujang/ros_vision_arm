#include <functional>
#include <memory>
#include <thread>
#include <cstring>
#include <cstddef>
#include <vector>
#include <string>
#include <unordered_map>
#include <tuple>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros_interfaces/action/dispatch_manipulation_task.hpp"
#include "ros_interfaces/srv/product_detection.hpp"

class RosRobotHandler : public rclcpp::Node
{
public:
  using DispatchManipulationTask = ros_interfaces::action::DispatchManipulationTask;
  using GoalHandleDispatchManipulationTask = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;

  explicit RosRobotHandler(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ros_robot_handler", options),
  {
    using namespace std::placeholders;

    /**************************************
      called when client call SendGoal().
      accept goal and execute later.
     **************************************/
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DispatchManipulationTask::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with target station %s", goal->target_station.c_str());
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    };

    /***************************************
      called when client call CancelGoal().
      always accept cancel request.
     ***************************************/
    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    /********************************************************************
      called when there's accepted goal from handle_goal().
      this function push goal to queue, and execute them sequentially.
     ********************************************************************/
    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle)
    {
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        goal_queue_.push(goal_handle);
      }
    
      if (!is_executing_) {
        is_executing_ = true;
        std::thread([this]() {
          while (true) {
            std::shared_ptr<GoalHandleDispatchManipulationTask> next_goal;
            {
              std::lock_guard<std::mutex> lock(queue_mutex_);
              if (goal_queue_.empty()) {
                is_executing_ = false;
                break;
              }
              next_goal = goal_queue_.front();
              goal_queue_.pop();
            }
    
            this->execute(next_goal);
          }
        }).detach();
      }
    };
    
    /* create action server */
    this->dispatch_manipulation_task_ = rclcpp_action::create_server<DispatchManipulationTask>(
      this,
      "dispatch_manipulation_task",
      handle_goal,
      handle_cancel,
      handle_accepted);
    
    /* create service client */
    this->open_port_client_ = this->create_client<ros_interfaces::srv::OpenPort>("open_port")
    this->product_detection_client_ = this->create_client<ros_interfaces::srv::ProductDetection>("product_detection");
    
  }

private:
  std::queue<std::shared_ptr<GoalHandleDispatchManipulationTask>> goal_queue_;
  std::mutex queue_mutex_;
  std::atomic_bool is_executing_ = false; // prevent race condition

  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr dispatch_manipulation_task_;
  rclcpp::Client<deli_interfaces::srv::ProductDetection>::SharedPtr service_client_;

  void execute(const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);  // loop inside execute function circle once in 1 seconds

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
    auto result = std::make_shared<DispatchManipulationTask::Result>();

    /**************************************
      send port open request to AI server.
      try request 3 times
    ***************************************/ 
    auto request = std::make_shared<ros_interfaces::srv::OpenPort::Request>();
    request->client_ip = "192.168.0.10"

    const int max_attempts = 3;
    const auto timeout = std::chrono::seconds(1);

    bool open_port_success = false;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
      RCLCPP_INFO(this->get_logger(), "Attempt %d to call OpenPort service", attempt);

      auto future = open_port_client_->async_send_request(request);

      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout)
        == rclcpp::FutureReturnCode::SUCCESS) {
          auto response = future.get();

          if (response->result == 1) {
            RCLCPP_INFO(this->get_logger(), "AI Server port opened");
            open_port_success = true;
            break;
          } else {
            RCLCPP_WARN(this->get_logger(), "Received response %d (expected 1)", response->result);
          }
      } else {
          RCLCPP_WARN(this->get_logger(), "Timeout on attempt %d", attempt);
      }

      if (attempt < max_attempts) {
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    if (open_port_success) {
      feedback->progress = 20.0;
      goal_handle->publish_feedback(feedback);
    } else {
      result->success = false;
      result->error_code = 101;
      result->error_msg = "Unable to open AI Server's port";
      goal_handle->abort(result);
      return;
    }

    // start camera, send rgb data to AI server
    

    // get response from AI server



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



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosRObotHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}