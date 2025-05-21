#ifndef NET_CONTROLLER_HPP
#define NET_CONTROLLER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <thread>

class NetController {
public:
    NetController(std::string ip, int port);

    /**
     * @brief  하나의 MJPEG 이미지 데이터를 여러 개의 UDP 패킷으로 나누고
     *         각 패킷 앞에 헤더를 붙여 전송. 
     * @param mjpeg_data MJPEG로 인코딩된 이미지 데이터
     * @note 헤더 구조:
     *       [0-3] frame_id: 이미지 프레임 고유 식별자
     *       [4-5] chunk_idx: 패킷 인덱스
     *       [6-7] total_chunks: 전체 패킷 수
     */
    void sendMjpegData(std::vector<uint8_t> mjpeg_data);
private:
    int sockfd_;
    struct sockaddr_in server_addr_;
    std::string server_ip_;
    int server_port_;
    
    int frame_id_ = 0;
    const int HEADER_SIZE = 8;
    const int MAX_UDP_PAYLOAD = 65000;
    const int MAX_CHUNK_SIZE = (MAX_UDP_PAYLOAD - HEADER_SIZE);
};

#endif