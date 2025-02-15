#include "deli_cam_controller.hpp"

void DeliCamController::startCam() {
    std::cout << "DeliCamController::startColorStream started" << std::endl;
    this->start_cam = true;

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);

    this->pipe.start(config);

    std::cout << "pipe started" << std::endl;

    std::cout << "Waiting for camera to initialize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초 대기
    std::cout << "Camera initialized. Requesting frames..." << std::endl;

    uint32_t frame_id = 0; 

    while (this->start_cam) {
        auto frameSet = this->pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            std::cerr << "Error: frameSet is nullptr!" << std::endl;
            continue;
        }

        auto colorFrame = frameSet->colorFrame();

        std::vector<uint8_t> mjpeg_data((uint8_t*)colorFrame->data(), (uint8_t*)colorFrame->data() + colorFrame->dataSize());

        int total_size = mjpeg_data.size();
        int num_chunks = (total_size + max_chunk_size - 1) / max_chunk_size;

        for (int i = 0; i < num_chunks; i++) {
            int offset = i * max_chunk_size;
            int chunk_size = std::min(max_chunk_size, total_size - offset);

            std::vector<uint8_t> packet(header_size + chunk_size);

            memcpy(packet.data(), &frame_id, 4);
            uint16_t chunk_idx = i;
            uint16_t total_chunks = num_chunks;
            memcpy(packet.data() + 4, &chunk_idx, 2);
            memcpy(packet.data() + 6, &total_chunks, 2);

            memcpy(packet.data() + header_size, mjpeg_data.data() + offset, chunk_size);

            ssize_t sent_len = sendto(this->sock, packet.data(), packet.size(), 0,
                                      (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));

            if (sent_len < 0) {
                std::cerr << "Sending failed! Error: " << strerror(errno) << std::endl;
                break;
            }
        }
        sendto(this->sock, frame_end, strlen(frame_end), 0,
               (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
        frame_id++;
    }    
    this->pipe.stop(); 
}

float DeliCamController::getDepthValue(int center_x, int center_y) {
    std::cout << "DeliCamController::getDepthValue started" << std::endl;

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_DEPTH);

    this->pipe.start(config);

    std::cout << "pipe started" << std::endl;

    std::cout << "Waiting for camera to initialize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초 대기
    std::cout << "Camera initialized. Requesting frames..." << std::endl;

    while(true) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
	    std::cout << "frameSet nullptr" << std::endl;
            continue;
        }

        auto depthFrame = frameSet->depthFrame();

        // for Y16 format depth frame, print the distance of the center pixel every 30 frames
        if(depthFrame->index() % 30 == 0 && depthFrame->format() == OB_FORMAT_Y16) {
            uint32_t  width  = depthFrame->width();
            uint32_t  height = depthFrame->height();
            float     scale  = depthFrame->getValueScale();
            uint16_t *data   = (uint16_t *)depthFrame->data();

            // pixel value multiplied by scale is the actual distance value in millimeters
            float centerDistance = data[width * center_y + center_x] * scale;
            std::cout << "width, height: " << width << ", " << height << std::endl;
	        std::cout << "centerDistance: " << centerDistance << std::endl;

            // attention: if the distance is 0, it means that the depth camera cannot detect the object（may be out of detection range）
            if (centerDistance) {
                this->pipe.stop();
                return centerDistance;
            }
        }
    }


}

std::vector<float> DeliCamController::convertCoor(int px, int py) {
    std::vector<float> camera_coor(3);

    float depth = getDepthValue(px, py);

    float xc = px - cx;
    float yc = py - cy;

    camera_coor[0] = (xc * depth) / fx;
    camera_coor[1] = (yc * depth) / fy;
    camera_coor[2] = depth;

    return camera_coor;
}

// std::vector<float> DeliCamController::calcCamCoordinatePose(int u, int v, int z) {

//     float camera_x = 
// }

// int DeliCamController::sendImage() {
//     int cnt = 0;
//     int result;
  

//     return result;
// }
    
