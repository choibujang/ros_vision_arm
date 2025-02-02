#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher()
        : Node("camera_publisher") {
        // 퍼블리셔 생성
        image_pub_ = image_transport::create_publisher(this, "camera/image");

        // OpenCV로 카메라 열기 (0번 카메라: 기본 웹캠)
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "카메라를 열 수 없습니다!");
            rclcpp::shutdown();
        }

        // 30ms 간격으로 이미지 퍼블리시
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&CameraPublisher::publish_image, this)
        );
    }

private:
    void publish_image() {
        cv::Mat frame;
        cap_ >> frame; // 카메라에서 이미지 가져오기
        if (frame.empty()) return;

        // OpenCV 이미지를 ROS 2 메시지로 변환
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // 메시지 퍼블리시
        image_pub_.publish(*msg);
        RCLCPP_INFO(this->get_logger(), "이미지 퍼블리시 완료!");
    }

    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
