// #include "utils.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/ObSensor.hpp"
#include <chrono>
#include <iostream>
#define ESC 27

int main(int argc, char **argv) try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR, 640, 400, 30, OB_FORMAT_BGR888);

    // Start the pipeline with config (default: depth and color streams)
    pipe.start(config);

    auto lastTime = std::chrono::high_resolution_clock::now();

    while(true) {
        // if(kbhit() && getch() == ESC) {
        //     break;
        // }
        // Wait for up to 100ms for a frameset in blocking mode.
        // 카메라 데이터를 가져오기 위한 프레임이 준비될 때까지 대기하는 역할
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();
        auto now = std::chrono::high_resolution_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() >= 30) {
            if(colorFrame) {
                uint16_t *data = (uint16_t *)colorFrame->data();
                std::cout << *data << std::endl;
            }
            lastTime = now;
        }

        // // for Y16 format depth frame, print the distance of the center pixel every 30 frames
        // if(depthFrame->index() % 30 == 0 && depthFrame->format() == OB_FORMAT_Y16) {
        //     uint32_t  width  = depthFrame->width();
        //     uint32_t  height = depthFrame->height();
        //     float     scale  = depthFrame->getValueScale();
        //     // depth 프레임 데이터를 담고 있는 버퍼. 픽셀 단위의 depth 값을 포함하는 배열
        //     uint16_t *data   = (uint16_t *)depthFrame->data();

        //     // pixel value multiplied by scale is the actual distance value in millimeters
        //     float centerDistance = data[width * height / 2 + width / 2] * scale;

        //     // attention: if the distance is 0, it means that the depth camera cannot detect the object（may be out of detection range）
        //     std::cout << "Facing an object " << centerDistance << " mm away. " << std::endl;
        // }
    }

    // Stop the pipeline
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}