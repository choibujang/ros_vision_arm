#include "../include/deli_cam_controller.hpp"

void DeliCamController::startCam() {
    this->start_cam = true;

    this->pipe.start();
    auto lastTime = std::chrono::high_resolution_clock::now();


    while (this->start_cam) {
        auto frameSet = this->pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            std::cerr << "Error: frameSet is nullptr!" << std::endl;
            continue;
        }

        auto colorFrame = frameSet->colorFrame();

        auto now = std::chrono::high_resolution_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() >= 30) {
            if(colorFrame) {
                uint16_t *data = (uint16_t *)colorFrame->data();

                int width = colorFrame->width();
                int height = colorFrame->height();
                int frame_size = width * height * 3;

                std::vector<uint8_t> buffer(data, data + frame_size);

                ssize_t sent_len = sendto(this->sock, buffer.data(), buffer.size(), 0,
                                (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
        
                if (sent_len < 0) {
                    std::cerr << "Sending failed! Error: " << strerror(errno) << std::endl;
                } else {
                    std::cout << "Frame sent: " << sent_len << " bytes" << std::endl;
                }
            }
                
        }
        lastTime = now;
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
    