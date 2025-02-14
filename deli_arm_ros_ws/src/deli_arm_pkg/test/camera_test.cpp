#include "deli_arm_pkg/deli_arm_controller.h"
#include "deli_arm_pkg/deli_cam_controller.hpp"

int main() {
    DeliArmController deli_arm;
    DeliCamController deli_cam;

    deli_arm.move321JointsToCameraPos();

    std::thread cam_thread(&DeliCamController::startCam, &deli_cam);

    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(3));

        for (int i = 95; i >= 85; i--) {
            deli_arm.moveBaseLink(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::this_thread::sleep_for(std::chrono::seconds(3));

        for (int i = 85; i <= 95; i++) {
            deli_arm.moveBaseLink(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::this_thread::sleep_for(std::chrono::seconds(3));

        for (int i = 95; i <= 105; i++) {
            deli_arm.moveBaseLink(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::this_thread::sleep_for(std::chrono::seconds(3));

        for (int i = 105; i >= 95; i--) {
            deli_arm.moveBaseLink(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (cam_thread.joinable()) {
        cam_thread.join();
    }

    return 0;
}