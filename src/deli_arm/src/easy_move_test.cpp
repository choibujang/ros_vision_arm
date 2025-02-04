#include "../include/DeliArm.h"

int main() {
    DeliArm deli_arm;

    deli_arm.writeRasp({90.0,90.0,110.0,170.0,120.0,100.0});

    // deli_arm.calcIK(200,200,150);

    // for (double d : deli_arm.getGoalJoint()) {
    //     std::cout << d << " ";
    // }
    // std::cout << std::endl;

}