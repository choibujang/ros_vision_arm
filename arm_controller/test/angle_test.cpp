#include "deli_arm_controller/deli_arm_controller.h"

int main() {
    DeliArmController deli_arm;
    // 270 -70 0
    // 200 -200 -220
    std::vector<double> pick_target_pos(3);
    std::vector<double> place_target_pos(3);
    // 4 joint goals

    for (int i=0; i < 3; i++) {
        std::cout << "pick_target_pos" << std::endl;
        if (i == 0) {
            std::cout << "x: ";
        } else if (i == 1) {
            std::cout << "y: ";
        } else {
            std::cout << "z: ";
        }
        std::cout << std::endl;
        std::cin >> pick_target_pos[i];
    }

    for (int i=0; i < 3; i++) {
        std::cout << "pick_target_pos" << std::endl;
        if (i == 0) {
            std::cout << "x: ";
        } else if (i == 1) {
            std::cout << "y: ";
        } else {
            std::cout << "z: ";
        }
        std::cout << std::endl;
        std::cin >> place_target_pos[i];
    }

    std::vector<double> ik_result = deli_arm.calcIK(pick_target_pos);
    
    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;

    ik_result = deli_arm.calcIK(place_target_pos);
    
    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;
}