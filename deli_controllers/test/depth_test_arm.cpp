#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"

int main() {
    DeliArmController deli_arm;

    deli_arm.move321JointsToCameraPos();
    
    return 0;
}
