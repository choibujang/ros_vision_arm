#include "../include/deli_cam_controller.hpp"

int main() {
    DeliCamController deli_cam_controller;

    if (!deli_cam_controller.isValid()) {
        std::cerr << "fail to create deli_cam_controller object" << std::endl;
        return 1;
    }

    deli_cam_controller.startCam("color");

}