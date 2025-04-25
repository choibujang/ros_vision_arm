#include "cam_controller.hpp"

int main() {
    CamController cam_controller;
    cam_controller.getCameraParam();

    cam_controller.startCameraPipeline();

    if (cam_controller.getFrameSet()) {
        std::vector<uint8_t> mjpeg_data = cam_controller.getMjpegColorData();
        cv::Mat decoded_mjpeg_data = cv::imdecode(mjpeg_data, cv::IMREAD_COLOR);
        cv::imwrite("color.png", decoded_mjpeg_data);

        cv::Mat depth_image = cam_controller.getMatDepthData();
        cv::imwrite("depth.png", depth_image);
    }

    cam_controller.stopCameraPipeline();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    return 0;
}
