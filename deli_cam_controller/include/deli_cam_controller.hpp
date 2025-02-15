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
#include <opencv2/opencv.hpp>


#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Sensor.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/h/ObTypes.h"


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
    cv::Mat convertCoorPixToCam(int center_x, int center_y);
    void getCamParam();
    
private:
    ob::Pipeline pipe;


    const char* server_ip = "192.168.0.76";
    int server_port = 8080;
    bool valid;
    const int header_size = 8;
    const int max_udp_payload = 65000;
    const int max_chunk_size = (max_udp_payload - header_size);
    const char* frame_end = "END";

    int sock;
    struct sockaddr_in server_addr;

    bool start_cam;
    float depth_fx = 475.328;
    float depth_fy = 475.328;
    float depth_cx = 315.204;
    float depth_cy = 196.601;
    float depth_width = 640;
    float depth_height = 400;

    float rgb_fx = 453.183;
    float rgb_fy = 453.183;
    float rgb_cx = 333.191;
    float rgb_cy = 241.26;
    float rgb_width = 640;
    float rgb_height = 480;

    // RGB to Depth
    cv::Mat rgb_to_depth_rot = (cv::Mat_<float>(3, 3) << 
        0.999983, 0.0050659, -0.0028331,
        -0.0050672, 0.999987,-0.00045272,
        0.00283077, 0.000467068, 0.999996
    );

    // Translation Vector (3x1)
    cv::Mat rgb_to_depth_trans = (cv::Mat_<float>(3, 1) << 9.98615, -0.0882425, 0.675267);

    cv::Mat depth_to_rgb_rot = (cv::Mat_<float>(3, 3) <<
        0.999983, -0.0050672, 0.00283077,
        0.0050659, 0.999987, 0.000467068,
        -0.0028331, -0.00045272, 0.999996
    );

    cv::Mat depth_to_rgb_trans = (cv::Mat_<float>(3, 1) << -9.98834, 0.0373371, -0.647012);



/*
width: 640
height: 400
*/
};

#endif
