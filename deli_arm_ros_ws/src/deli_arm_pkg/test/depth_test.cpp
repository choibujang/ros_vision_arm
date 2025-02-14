#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"

int main() {
    DeliArmController deli_arm;
    DeliCamController deli_cam;

    deli_arm.move321JointsToCameraPos();

    float depth = deli_cam.getDepthValue(235, 382);

    std::cout << "depth: " << depth << std::endl;
    

    return 0;
}
