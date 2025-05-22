#include "robot_arm_controllers/net/net_controller.hpp"

NetController::NetController() {
    int try_cnt = 0;
    while (try_cnt < 5) {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ >= 0) {
            break;
        }

        try_cnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    if (sockfd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(server_port_);
    inet_pton(AF_INET, server_ip_.c_str(), &(server_addr_.sin_addr));

    std::cout << "Finished NetController initialization" << std::endl;

}


void NetController::sendMjpegData(std::vector<uint8_t> mjpeg_data) {
    std::cout << "Start sending data" << std::endl;
    uint32_t device_id = device_id_;
    int total_size = mjpeg_data.size();
    int num_chunks = (total_size + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;

    for (int i = 0; i < num_chunks; i++) {
        int offset = i * MAX_CHUNK_SIZE;
        int chunk_size = std::min(MAX_CHUNK_SIZE, total_size - offset);

        std::vector<uint8_t> packet(HEADER_SIZE + chunk_size);

        memcpy(packet.data(), &device_id, 4);
        memcpy(packet.data() + 4, &frame_id_, 4);
        uint16_t chunk_idx = i;
        uint16_t total_chunks = num_chunks;
        memcpy(packet.data() + 8, &chunk_idx, 2);
        memcpy(packet.data() + 10, &total_chunks, 2);

        memcpy(packet.data() + HEADER_SIZE, mjpeg_data.data() + offset, chunk_size);

        sendto(sockfd_, packet.data(), packet.size(), 0, (struct sockaddr *)&(server_addr_), sizeof(server_addr_));
    }
    frame_id_++;
}

