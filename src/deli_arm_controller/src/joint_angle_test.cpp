#include "../include/DeliArm.h"
#include <iostream>
#include <unistd.h>

int main() {
    DeliArm deli_arm;
    int target_joint;
    double target_angle;

    while (true) {
        std::cout << "target joint: " << std::endl;
        std::cin >> target_joint;

        std::cout << "target angle: " << std::endl;
        std::cin >> target_angle;

        deli_arm.writeRasp(target_joint, target_angle);
    }


    return 0;
}