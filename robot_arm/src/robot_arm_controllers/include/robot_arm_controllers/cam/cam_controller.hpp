#ifndef CAM_CONTROLLER_HPP
#define CAM_CONTROLLER_HPP

#include <iostream>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>

#include "libobsensor/ObSensor.hpp"

/**
 * @class CamController
 * @brief 카메라 on/off, 프레임 가져오기, depth map 생성, 픽셀 좌표->3d좌표 변환 기능 제공
 */
class CamController {
public:
    /**
     * @brief 카메라의 color, depth 스트리밍을 활성화한다.  
     */
    void startCameraPipeline();

    /**
     * @brief 활성화된 스트림에서 frameset을 가져와 클래스 멤버 current_frameset에 저장한다.
     * @param timeout_ms 프레임 수신 대기 최대 시간
     * @param max_cnt 최대 재시도 횟수
     * @return frameset 수신 성공 여부
     */
    bool getFrameSet(int timeout_ms=100, int max_cnt=10);

    /**
     * @brief 카메라 스트리밍을 중지한다.
     */
    void stopCameraPipeline();

    /**
     * @brief current_frameset에서 color frame을 가져온다.
     * @return 이미지 데이터를 담은 vector
     */
    std::vector<uint8_t> getColorData();

    /**
     * @brief current_frameset에서 depth frame을 가져온다.
     * @return depth 데이터를 담은 640*400 행렬
     */ 
    cv::Mat getDepthData();

    /**
     * @brief Depth 픽셀을 3D 좌표로 변환한 후, RGB 카메라 좌표계로 변환하고
     *        이를 다시 RGB 이미지의 픽셀 위치로 변환하여, RGB 프레임 기준의 Depth map 생성.
     *        RGB 이미지와 Depth 이미지의 해상도 차이로 인해
     *        Depth = 0인 빈 영역은 3x3 주변 평균으로 보간하여 채움.
     * @param depth 원본 Depth 데이터 (640 * 400)
     * @return RGB 픽셀에 대응하는 depth map (640 * 480)
     */
    cv::Mat createDepthMap(const cv::Mat& depth);

    /**
     * @brief RGB 이미지의 특정 픽셀 좌표를 카메라 좌표계 기준 3D 좌표로 변환한다.
     * @param depth 원본 Depth 데이터 (640 * 400)
     * @return RGB 픽셀에 대응하는 depth map (640 * 480)
     */
    std::vector<float> pixelToCameraCoords(int u, int v, const cv::Mat& depth_map);

    /**
     * @brief 카메라의 intrinsic parameter들을 출력한다.
     */ 
    void getCameraParam();
    
private:
    ob::Pipeline pipe_;
    std::shared_ptr<ob::FrameSet> current_frameset_;

    float depth_fx_ = 475.328;
    float depth_fy_ = 475.328;
    float depth_cx_ = 315.204;
    float depth_cy_ = 196.601;
    float depth_width_ = 640;
    float depth_height_ = 400;

    float rgb_fx_ = 453.183;
    float rgb_fy_ = 453.183;
    float rgb_cx_ = 333.191;
    float rgb_cy_ = 241.26;
    float rgb_width_ = 640;
    float rgb_height_ = 480;

    // RGB to Depth
    cv::Mat rgb_to_depth_rot_ = (cv::Mat_<float>(3, 3) << 
        0.999983, 0.0050659, -0.0028331,
        -0.0050672, 0.999987,-0.00045272,
        0.00283077, 0.000467068, 0.999996
    );

    // Translation Vector
    cv::Mat rgb_to_depth_trans_ = (cv::Mat_<float>(3, 1) << 9.98615, -0.0882425, 0.675267);

    cv::Mat depth_to_rgb_rot_ = (cv::Mat_<float>(3, 3) <<
        0.999983, -0.0050672, 0.00283077,
        0.0050659, 0.999987, 0.000467068,
        -0.0028331, -0.00045272, 0.999996
    );

    cv::Mat depth_to_rgb_trans_ = (cv::Mat_<float>(3, 1) << -9.98834, 0.0373371, -0.647012);


};

#endif
