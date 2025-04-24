#include "cam_controller.hpp"

int main() {
    CamController deli_cam;
    std::vector<float> camera_pose(3);

    // [{'product_name': 'peach', 'width': 63, 'height': 53, 'cx': 300, 'cy': 255}]

    int center_x = 231;
    int center_y = 386;

    // auto product_pose = deli_cam.convertCoorPixToCam(center_x, center_y);

    // std::cout << "X: " << product_pose.at<float>(0,0) << ", " << "Y: " << product_pose.at<float>(1,0) << ", " << "Z: " << product_pose.at<float>(2,0) << std::endl;
    // X: -57.0843, Y: 68.4777, Z: 244
    // from base frame, about X: 22, Y: 6.5, Z: 1.5 cm
    
    return 0;
}
