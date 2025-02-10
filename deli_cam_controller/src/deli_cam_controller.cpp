#include "../include/deli_cam_controller.hpp"

void DeliCamController::startCam() {
    this->start_cam = true;

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);

    std::cout << "Color frame format: " << colorFrame->format() << std::endl;

    this->pipe.start(config);

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
            } else {
            std::cout << "Frame sent: " << sent_len << " bytes" << std::endl;
            }
        }
        sendto(this->sock, frame_end, strlen(frame_end), 0,
               (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
        frame_id++;
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
    