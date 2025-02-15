#include "deli_arm_controller/deli_arm_controller.h"

int main() {
    DeliArmController deli_arm;
    // 270 -70 0
    // 200 -200 -220
    std::vector<double> pick_target_pos1(3);
    std::vector<double> ik_result(4);
    int open_gripper_value;
    int close_gripper_value;
    // 4 joint goals

    for (int i=0; i < 3; i++) {
        std::cout << "pick_target_pos1 ";
        if (i == 0) {
            std::cout << "x: ";
        } else if (i == 1) {
            std::cout << "y: ";
        } else {
            std::cout << "z: ";
        }
        std::cout << std::endl;
        std::cin >> pick_target_pos1[i];
    }

    std::cout << "open gripper value: ";
    std::cin >> open_gripper_value;
    std::cout << "close gripper value: ";
    std::cin >> close_gripper_value;

    ik_result = deli_arm.calcIK(pick_target_pos1);
    
    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;
    
    deli_arm.move321JointsToNeatPos();
    deli_arm.moveBaseLink(ik_result[0]);
    deli_arm.openGripper(open_gripper_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    deli_arm.move321Joints(ik_result);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    deli_arm.closeGripper(close_gripper_value);
    deli_arm.move321JointsToNeatPos();
    deli_arm.moveBaseLink(95.0);
}