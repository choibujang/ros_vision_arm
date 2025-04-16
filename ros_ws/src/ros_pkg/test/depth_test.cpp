#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"

int main() {
    DeliArmController deli_arm;
    DeliCamController deli_cam;

    deli_arm.move321JointsToCameraPos();

    int center_x = 204;
    int center_y = 330;

    //float depth = deli_cam.getDepthValue(204, 330);

    float depth = 244.0;
    std::cout << "depth: " << depth << std::endl;
    
    float camera_x = ((center_x - deli_cam.cx) * depth) / deli_cam.fx;
    float camera_y = ((center_y - deli_cam.cy) * depth) / deli_cam.fy;
    float camera_z = depth;

    std::cout << "X: " << camera_x << ", " << "Y: " << camera_y << ", " << "Z: " << camera_z << std::endl;
    // X: -57.0843, Y: 68.4777, Z: 244
    // from base frame, about X: 22, Y: 6.5, Z: 1.5 cm
    
    return 0;
}
