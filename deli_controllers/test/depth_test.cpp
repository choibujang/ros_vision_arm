#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"

int main() {
    DeliArmController deli_arm;
    DeliCamController deli_cam;
    std::vector<float> camera_pose(3);

    deli_arm.move321JointsToCameraPos();

    int center_x = 345;
    int center_y = 323;

    camera_pose = deli_cam.convertCoor(center_x, center_y);

    std::cout << "X: " << camera_pose[0] << ", " << "Y: " << camera_pose[1] << ", " << "Z: " << camera_pose[2] << std::endl;
    // X: -57.0843, Y: 68.4777, Z: 244
    // from base frame, about X: 22, Y: 6.5, Z: 1.5 cm
    
    return 0;
}
