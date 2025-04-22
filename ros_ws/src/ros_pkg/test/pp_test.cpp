#include "deli_arm_pkg/deli_arm_controller.h"

int main() {
    DeliArmController deli_arm;
    // 270 -70 0
    // 200 -200 -220
    std::vector<double> pick_target_pos1(3);
    std::vector<double> pick_target_pos2(3);
    std::vector<double> place_target_pos(3);
    std::vector<double> ik_result(4);
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

    for (int i=0; i < 3; i++) {
        std::cout << "pick_target_pos2 ";
        if (i == 0) {
            std::cout << "x: ";
        } else if (i == 1) {
            std::cout << "y: ";
        } else {
            std::cout << "z: ";
        }
        std::cout << std::endl;
        std::cin >> pick_target_pos2[i];
    }

    for (int i=0; i < 3; i++) {
        std::cout << "place_target_pos " << std::endl;
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

    ik_result = deli_arm.calcIK(pick_target_pos1);
    
    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;
    
    deli_arm.move321JointsToNeatPos();
    deli_arm.moveBaseLink(ik_result[0]);
    deli_arm.openGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    deli_arm.move321Joints(ik_result);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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


    ik_result = deli_arm.calcIK(pick_target_pos2);
    
    for (double d : ik_result) {
        std::cout << d << " ";
        if (std::isnan(d))
            return -1;
    }
    std::cout << std::endl;
    
    deli_arm.move321JointsToNeatPos();
    deli_arm.moveBaseLink(ik_result[0]);
    deli_arm.openGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    deli_arm.move321Joints(ik_result);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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