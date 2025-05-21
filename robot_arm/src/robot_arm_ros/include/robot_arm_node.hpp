#ifndef ROBOT_ARM_NODE_HPP
#define ROBOT_ARM_NODE_HPP

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "ros_interfaces/action/dispatch_manipulation_task.hpp"
#include "ros_interfaces/msg/DetectedObject.hpp"
#include "ros_interfaces/msg/DetectedObjectArray.hpp"
#include "robot_arm_controllers/cam/cam_controller.hpp"
#include "robot_arm_controllers/arm/arm_controller.h"
#include "robot_arm_controllers/net/net_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * @class RobotArmNode
 * @brief Service Server로부터 Request를 받아 실행하는 dispatch_manipulation_task 액션 서버와
 *        /detected_object 토픽을 listen하는 subscriber로 이루어져 있다.
 */
class RobotArmNode : public rclcpp::Node
{
public:
    using DispatchManipulationTask = ros_interfaces::action::DispatchManipulationTask;
    using GoalHandleDMT = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;
    using DetectedObject = ros_interfaces::msg::DetectedObject;
    using DetectedObjectArray = ros_interfaces::msg::DetectedObjectArray;

    explicit RobotArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("robot_arm_node", options),
        arm_controller(),
        cam_controller(),
        net_controller("192.168.0.92", 8080)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<DispatchManipulationTask>(
        this,
        "dispatch_manipulation_task",
        std::bind(&RobotArmNode::handle_goal, this, _1, _2),
        std::bind(&RobotArmNode::handle_cancel, this, _1),
        std::bind(&RobotArmNode::handle_accepted, this, _1));

        this->detected_sub_ = this->create_subscription<DetectedObjectArray>(
        "/detected_object", 
        10,
        std::bind(&RobotArmNode::detectedCallback, this, std::placeholders::_1)
        );
        
    }

private:
    /**
    * @brief goal의 item_name 배열 길이와 item_quantities 배열 길이가 다르면 Reject한다.
    */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DispatchManipulationTask::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDMT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleDMT> goal_handle);

  /**
    * @brief 
    */
  void execute(const std::shared_ptr<GoalHandleDMT> goal_handle);
  std::vector<std::vector<float>> convertTo3DCoords(const std::vector<DetectedObject>& objects, const cv::Mat& depth_map);

  void RobotArmNode::detectedCallback(const DetectedObjectArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(detected_mutex_);
    latest_detected_objects_ = msg->objects;
  }

  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
  rclcpp::Subscription<DetectedObjectArray>::SharedPtr detected_sub_;
  std::vector<DetectedObject> latest_detected_objects_;
  std::mutex detected_mutex_;

  ArmController arm_controller_;
  CamController cam_controller_;
  NetController net_controller_;
};

#endif