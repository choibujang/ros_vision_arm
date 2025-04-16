#include "deli_arm_controller/deli_arm_controller.h"

int main() {
    DeliArmController deli_arm;

    deli_arm.move321JointsToCameraPos();

    // while(true) {
    //     std::this_thread::sleep_for(std::chrono::seconds(3));

    //     for (int i = 95; i >= 85; i--) {
    //         deli_arm.moveBaseLink(i);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }

    //     std::this_thread::sleep_for(std::chrono::seconds(3));

    //     for (int i = 85; i <= 95; i++) {
    //         deli_arm.moveBaseLink(i);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }

    //     std::this_thread::sleep_for(std::chrono::seconds(3));

    //     for (int i = 95; i <= 105; i++) {
    //         deli_arm.moveBaseLink(i);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }

    //     std::this_thread::sleep_for(std::chrono::seconds(3));

    //     for (int i = 105; i >= 95; i--) {
    //         deli_arm.moveBaseLink(i);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }


    return 0;
}