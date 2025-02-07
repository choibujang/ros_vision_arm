#ifndef DELI_CAM_CONTROLLER_HPP
#define DELI_CAM_CONTROLLER_HPP

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8080
#define BUFFER_SIZE 1024

class DeliCamController {
public:
    void sendImage();
    float getDepth(int cx, int cy);

private:
    ob::Pipeline pipe;
};

#endif