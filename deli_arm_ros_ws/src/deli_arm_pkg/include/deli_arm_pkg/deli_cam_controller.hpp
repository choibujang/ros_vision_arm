#ifndef DELI_CAM_CONTROLLER_HPP
#define DELI_CAM_CONTROLLER_HPP

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <cerrno>
#include <thread>


#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/ObSensor.hpp"


class DeliCamController {
public:
    DeliCamController() : valid(false) {
        this->sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (this->sock < 0) {
            perror("fail to create socket");
            return;
        }

        std::cout << "socket created" << std::endl;

        memset(&(this->server_addr), 0, sizeof(this->server_addr));
        this->server_addr.sin_family = AF_INET;
        this->server_addr.sin_port = htons(this->server_port);
        if (inet_pton(AF_INET, this->server_ip, &(this->server_addr.sin_addr)) <= 0) {
            perror("fail to convert ip");
            close(this->sock);
            return;
        }

        std::cout << "converted ip" << std::endl;

        valid = true;
    }

    ~DeliCamController() {
        if (this->sock >= 0) {
            close(this->sock);
        }
    }

    bool isValid() const { return valid; }
    void startCam();
    float getDepthValue(int center_x, int center_y);

    void sendImage();
    float calcCam3D(int u, int v);

    bool start_cam;
    
private:
    ob::Pipeline pipe;
    int width = 640;
    int height = 400;
    float fx = 475.328;
    float fy = 475.328;
    float cx = 315.204;
    float cy = 196.601;

    const char* server_ip = "192.168.191.174";
    int server_port = 8080;
    bool valid;
    const int header_size = 8;
    const int max_udp_payload = 65000;
    const int max_chunk_size = (max_udp_payload - header_size);
    const char* frame_end = "END";

    int sock;
    struct sockaddr_in server_addr;


/*
width: 640
height: 400
*/
};

#endif
