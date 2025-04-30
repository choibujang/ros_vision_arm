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
    NetController(const char* ip, int port);
    void sendMjpegData(std::vector<uint8_t> mjpeg_data);
private:
    int sockfd;
    struct sockaddr_in server_addr;
    const char* server_ip;
    const int server_port;
    
    int frame_id = 0;
    const int header_size = 8;
    const int max_udp_payload = 65000;
    const int max_chunk_size = (max_udp_payload - header_size);
    const char* frame_end = "END";
};

#endif