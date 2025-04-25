#include "cam_controller.hpp"

int main() {
    CamController cam_controller;
    cam_controller.getCameraParam();
    cam_controller.startCameraStream();
    
    std::vector<cv::Mat> images = cam_controller.getColorDepthImage();

    cv::imwrite("color.png", images[0]);
    std::cout << "write color image" << std::endl;

    cv::imwrite("depth.png", images[1]);
    std::cout << "write depth image" << std::endl;

    cam_controller.stopCameraStream();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    return 0;
}
