#include "../include/deli_cam_controller.hpp"

void DeliCamController::startCam(char *type) {
    int pid;
    if (type == "color") {
        pid = 0x0511;
    }

    ob::Context ctx;
    auto devList = ctx.queryDeviceList();
    // if (devList->deviceCount() == 0) {
    //     std::cerr << "No Orbbec device found!" << std::endl;
    //     return -1;
    // }

    std::shared_ptr<ob::Device> selectedDevice = nullptr;
    for (int i = 0; i < devList->deviceCount(); i++) {
        auto dev = devList->getDevice(i);
        if (dev->pid() == pid) {  // USB 2.0 Camera의 PID
            selectedDevice = dev;
            break;
        }
    }

    this->pipe.setDevice(selectedDevice);
    this->start_cam = true;
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR, this->width, this->height, 30, OB_FORMAT_RGB888);

    this->pipe.start(config);


    while (this->start_cam) {
        auto frameSet = this->pipe.waitForFrames(500);
        if(frameSet == nullptr) {
            std::cerr << "Error: frameSet is nullptr!" << std::endl;
            continue;
        }

        auto colorFrame = frameSet->colorFrame();
        if (colorFrame == nullptr) {
            std::cerr << "Warning: colorFrame is nullptr!" << std::endl;
            continue;
        }

        uint8_t *rgb_data = (uint8_t *)colorFrame->data();
        if (rgb_data == nullptr) {
            std::cerr << "Error: rgb_data is nullptr!" << std::endl;
            continue;
        }

        int width = colorFrame->width();
        int height = colorFrame->height();
        int frame_size = width * height * 3;

        std::vector<uint8_t> buffer(rgb_data, rgb_data + frame_size);

        ssize_t sent_len = sendto(this->sock, buffer.data(), buffer.size(), 0,
                                (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
        
        if (sent_len < 0) {
            std::cerr << "Sending failed!" << std::endl;
        } else {
            std::cout << "Frame sent: " << sent_len << " bytes" << std::endl;
        }
    }
}

// std::vector<float> DeliCamController::calcCamCoordinatePose(int u, int v, int z) {

//     float camera_x = 
// }

// int DeliCamController::sendImage() {
//     int cnt = 0;
//     int result;
  

//     return result;
// }
    