#include "robot_arm_controllers/net/net_controller.hpp"

NetController::NetController(const char* ip, const int port)
: server_ip(ip), server_port(port)
{
    int try_cnt = 0;
    while (try_cnt < 5) {
        this->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (this->sockfd >= 0) {
            break;
        }

        try_cnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    if (this->sockfd < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    memset(&this->server_addr, 0, sizeof(this->server_addr));
    this->server_addr.sin_family = AF_INET;
    this->server_addr.sin_port = htons(this->server_port);
    inet_pton(AF_INET, this->server_ip, &(this->server_addr.sin_addr));

}

/*************************************
@brief  하나의 MJPEG 이미지 데이터를 여러 개의 UDP 패킷으로 나누고
        각 패킷 앞에 헤더를 붙여 전송. 
        모든 패킷을 전송하면 마지막에 "END" 문자열을 전송하여
        하나의 이미지의 끝을 알린다.

@param  mjpeg_data: MJPEG로 인코딩된 이미지 데이터
@note   헤더 구조:
        [0-3] frame_id: 이미지 프레임 고유 식별자
        [4-5] chunk_idx: 패킷 인덱스
        [6-7] total_chunks: 전체 패킷 수
**************************************/
void NetController::sendMjpegData(std::vector<uint8_t> mjpeg_data) {
    int total_size = mjpeg_data.size();
    // 패킷 개수 계산
    int num_chunks = (total_size + this->max_chunk_size - 1) / this->max_chunk_size;

    for (int i = 0; i < num_chunks; i++) {
        int offset = i * this->max_chunk_size;
        int chunk_size = std::min(this->max_chunk_size, total_size - offset);

        std::vector<uint8_t> packet(this->header_size + chunk_size);

        memcpy(packet.data(), &this->frame_id, 4);
        uint16_t chunk_idx = i;
        uint16_t total_chunks = num_chunks;
        memcpy(packet.data() + 4, &chunk_idx, 2);
        memcpy(packet.data() + 6, &total_chunks, 2);

        memcpy(packet.data() + this->header_size, mjpeg_data.data() + offset, chunk_size);

        ssize_t sent_len = sendto(this->sockfd, packet.data(), packet.size(), 0,
                                    (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));

        if (sent_len < 0) {
            std::cerr << "Sending failed! Error: " << strerror(errno) << std::endl;
            break;
        }
    }
    sendto(this->sockfd, this->frame_end, strlen(this->frame_end), 0,
            (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
    this->frame_id++;

}

