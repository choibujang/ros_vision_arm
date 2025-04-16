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

#include "deli_interfaces/srv/product_detection.hpp"
#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"


class TestServiceClient : public rclcpp::Node
{
public:
    explicit TestServiceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("test_service_client", options),
        deli_arm_controller_(std::make_shared<DeliArmController>()),
        deli_cam_controller_(std::make_shared<DeliCamController>())
    {    
        RCLCPP_INFO(this->get_logger(), "starting test_service_client node");

        startCamThread();

        deli_arm_controller_->move321JointsToCameraPos();

        std::this_thread::sleep_for(std::chrono::seconds(10));

        this->service_client_ = this->create_client<deli_interfaces::srv::ProductDetection>("detect_products");

        while (!service_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Waiting for service...");
        }

        RCLCPP_INFO(this->get_logger(), "Service is available!");

        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(5),  // 5초마다 실행
            std::bind(&TestServiceClient::send_request, this));    
    }

private:
    rclcpp::Client<deli_interfaces::srv::ProductDetection>::SharedPtr service_client_;
    std::shared_ptr<DeliArmController> deli_arm_controller_;
    std::shared_ptr<DeliCamController> deli_cam_controller_;
    std::thread cam_thread_; 
    rclcpp::TimerBase::SharedPtr timer_;

    void startCamThread() {
        cam_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Starting camera in a separate thread...");
            deli_cam_controller_->startCam();
        });
    }

    void stopCamThread() {
        deli_cam_controller_->start_cam = false;
        if (cam_thread_.joinable()) {
            cam_thread_.join(); 
        }
        RCLCPP_INFO(this->get_logger(), "Camera thread stopped.");
    }

    void send_request() {
        auto request = std::make_shared<deli_interfaces::srv::ProductDetection::Request>();
        request->item_names = {"peach", "apple"};
        request->item_quantities = {1, 1};

        auto future = service_client_->async_send_request(request, 
            std::bind(&TestServiceClient::handle_service_response, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Sent service request!");
    }

    void handle_service_response(rclcpp::Client<deli_interfaces::srv::ProductDetection>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service response received!");

        std::unordered_map<std::string, std::vector<std::tuple<int, int, int, int>>> product_locations;
        for (size_t i = 0; i < response->product_names.size(); i++) {
            product_locations[response->product_names[i]].push_back(
                {response->width[i], response->height[i], response->cx[i], response->cy[i]});
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
}; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestServiceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
