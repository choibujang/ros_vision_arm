#include "../include/deli_cam_controller.hpp"

std::vector<float> DeliCamController::calcCamCoordinatePose(int u, int v, int z) {

    float camera_x = 
}

int DeliCamController::sendImage() {
    // create socket
    int sockfd;
    struct sockaddr_in server_addr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid server IP address!" << std::endl;
        close(sockfd);
        return -1;
    }

    // start camera stream
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);
    pipe.start(config);

    int cnt = 0;

    while (cnt < 5) {
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        auto colorFrame = frameSet->colorFrame();
        if(colorFrame == nullptr) {
            continue;
        }

        uint8_t *rgb_data = (uint8_t *)colorFrame->data();
        int width = colorFrame->width();
        int height = colorFrame->height();
        int frame_size = width * height * 3;

        std::vector<uint8_t> buffer(rgb_data, rgb_data + frame_size);

        ssize_t sent_len = sendto(sockfd, buffer.data(), buffer.size(), 0,
                                  (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (sent_len < 0) {
            std::cerr << "Sending failed!" << std::endl;
            close(sockfd);
            return -1;
        }

        cnt++;
    }

    close(sockfd);

}
    