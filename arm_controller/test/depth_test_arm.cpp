#include "deli_arm_controller/deli_arm_controller.h"

int main() {
    DeliArmController deli_arm;

    deli_arm.move321JointsToCameraPos();
    
    return 0;
}
