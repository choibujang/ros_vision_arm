#include "../include/deli_arm_controller.h"

int main() {
    DeliArmController deli_arm;

    std::vector<double> pick_target_pos = {80, -300, 80};
    std::vector<double> place_target_pos = {200, 260, 90};

    // 4 joint goals
    std::vector<double> ik_result = deli_arm.calcIK(pick_target_pos);
    
    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;

    deli_arm.move321JointsToNeatPos();

    deli_arm.moveBaseLink(ik_result[0]);

    deli_arm.openGripper();

    deli_arm.move321Joints(ik_result);

    deli_arm.closeGripper();

    deli_arm.move321JointsToNeatPos();

    ik_result = deli_arm.calcIK(place_target_pos);

    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;

    deli_arm.moveBaseLink(ik_result[0]);

    deli_arm.move321Joints(ik_result);

    deli_arm.openGripper();

    deli_arm.closeGripper();

    deli_arm.move321JointsToNeatPos();

    deli_arm.moveBaseLink(95.0);
}