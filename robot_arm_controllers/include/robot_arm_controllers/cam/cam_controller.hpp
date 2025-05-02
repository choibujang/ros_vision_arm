/*
라즈베리 파이에 연결되어 있는 카메라를 담당하는 클래스.

카메라를 끄고 키고,
카메라 프레임을 ai server에 보내고,
카메라 기준 3d pose를 계산하는 역할을 담당한다.
*/

#ifndef CAM_CONTROLLER_HPP
#define CAM_CONTROLLER_HPP

#include <iostream>
// #include <sys/socket.h>
// #include <netinet/in.h>

// #include <cstdlib>
// #include <chrono>
// #include <cerrno>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>

#include "libobsensor/ObSensor.hpp"



class CamController {
public:
    cv::Mat alighDepthToRGB(const cv::Mat& depth);
    void getCameraParam();
    void startCameraPipeline();
    bool getFrameSet(int timeout_ms=100, int max_cnt=10);
    std::vector<uint8_t> getMjpegColorData();
    cv::Mat getMatDepthData();
    void stopCameraPipeline();

    
private:
    ob::Pipeline pipe;
    std::shared_ptr<ob::FrameSet> current_frameset;

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


};

#endif
