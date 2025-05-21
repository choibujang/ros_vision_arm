#include "robot_arm_node.hpp"

rclcpp_action::GoalResponse RobotArmNode::handle_goal(
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

  if (goal->item_names.size() != goal->item_quantities.size()) {
    RCLCPP_WARN(this->get_logger(), "Rejected goal: mismatched name/quantity sizes");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotArmNode::handle_cancel(
  const std::shared_ptr<GoalHandleDMT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotArmNode::handle_accepted(const std::shared_ptr<GoalHandleDMT> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&RobotArmNode::execute, this, _1), goal_handle}.detach();
}

void RobotArmNode::execute(const std::shared_ptr<GoalHandleDMT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
  auto result = std::make_shared<DispatchManipulationTask::Result>();

  // 카메라 시작, 이미지 프레임 가져오기
  cam_controller_.startCameraPipeline();
  std::vector<DetectedObject> confirmed_objects;

  while (rclcpp::ok()) {
    if (!cam_controller_.getFrameSet()) {
      cam_controller_.stopCameraPipeline();
      result->success = false;
      result->error_code = 2;
      result->error_msg = "Failed to get frame set from camera";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Goal aborted");
      return;
    }

    net_controller_.sendMjpegData(cam_controller_.getColorData());

    bool goal_met = false;

    std::lock_guard<std::mutex> lock(detected_mutex_);

    // goal 항목들이 latest_detected_objects_에 다 있는지 확인
    goal_met = true;
    for (size_t i = 0; i < goal->item_names.size(); ++i) {
      const std::string& name = goal->item_names[i];
      int required_count = goal->item_quantities[i];
      int detected_count = 0;

      for (const auto& obj : latest_detected_objects_) {
        if (obj.name == name) {
          detected_count += obj.count;
        }
      }

      if (detected_count < required_count) {
        goal_met = false;
        break;
      }
    }
  
    if (goal_met) {
      confirmed_objects = latest_detected_objects_;
      RCLCPP_INFO(this->get_logger(), "All target items detected.");
      break;
    }
  
    rclcpp::Rate(5).sleep();
  }

  cv::Mat depth_data = cam_controller_.getDepthData()
  cv::Mat depth_map = cam_controller_.createDepthMap(depth_data);

  vector<vector<float>> comfirmed_objects_3d = convertTo3DCoords(confirmed_objects, depth_map);




  



  for (int i = 1; (i < size(goal->item_names)) && rclcpp::ok(); i++) {
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
 


std::vector<std::vector<float>> RobotArmNode::convertTo3DCoords(const std::vector<DetectedObject>& objects, const cv::Mat& depth_map) {
  std::vector<std::vector<float>> result;

  for (const auto& obj : objects) {
    for (const auto& pixel : obj.pixels) {
      result.push_back(cam_controller_.pixelToCameraCoords(pixel.x, pixel.y, depth_map));
    }
  }

  return result;
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArmNode>());
  rclcpp::shutdown();
  return 0;
}