#include "robot_arm_controllers/cam/cam_controller.hpp"

void CamController::updateDepthMap() {
    cv::Mat current_depth_frame = getDepthFrame();
    cv::Mat transformed = cv::Mat::zeros(rgb_height_, rgb_width_, CV_16UC1);

    for (int vd = 0; vd < current_depth_frame.rows; ++vd) {
        for (int ud = 0; ud < current_depth_frame.cols;) {
            uint16_t z = current_depth_frame.at<uint16_t>(vd, ud);
            if (z == 0) continue;

            // Depth 픽셀 좌표를 Depth 카메라 좌표계의 3d 좌표로 변환
            float Xd = (ud - depth_cx_) * z / depth_fx_;
            float Yd = (vd - depth_cy_) * z / depth_fy_;
            float Zd = z;

            // Depth → RGB 카메라 좌표계로 변환
            cv::Mat depth_3d = (cv::Mat_<float>(3,1) << Xd, Yd, Zd);
            cv::Mat rgb_3d = depth_to_rgb_rot_ * depth_3d + depth_to_rgb_trans_;

            float Xr = rgb_3d.at<float>(0);
            float Yr = rgb_3d.at<float>(1);
            float Zr = rgb_3d.at<float>(2);

            if (Zr <= 0) continue; // 뒤에 있는 점 무시

            // RGB 카메라에서의 픽셀 위치
            int u_rgb = static_cast<int>(rgb_fx_ * Xr / Zr + rgb_cx_);
            int v_rgb = static_cast<int>(rgb_fy_ * Yr / Zr + rgb_cy_);

            // 범위 체크 후 저장
            if (u_rgb >= 0 && u_rgb < rgb_width_ && v_rgb >= 0 && v_rgb < rgb_height_) {
                uint16_t& current = transformed.at<uint16_t>(v_rgb, u_rgb);
                // 이미 값이 있다면 더 가까운 z만 저장
                if (current == 0 || Zr < current) {
                    current = static_cast<uint16_t>(Zr);
                }
            }

        }
    }

    // Z = 0인 빈 영역 3x3 평균으로 보간
    cv::Mat filled = transformed.clone();
    for (int y = 1; y < transformed.rows - 1; ++y) {
        for (int x = 1; x < transformed.cols - 1; ++x) {
            if (transformed.at<uint16_t>(y, x) == 0) {
                int sum = 0;
                int count = 0;

                // 주변 3x3 이웃 평균
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        uint16_t neighbor_z = transformed.at<uint16_t>(y + dy, x + dx);
                        if (neighbor_z > 0) {
                            sum += neighbor_z;
                            count++;
                        }
                    }
                }

                if (count > 0) {
                    filled.at<uint16_t>(y, x) = static_cast<uint16_t>(sum / count);
                }
            }
        }
    }

    current_depth_map_ = filled.clone();

}

void CamController::getCameraParam() {
    auto device = pipe_.getDevice();
    // Get the depth sensor
    auto depthSensor = device->getSensor(OB_SENSOR_DEPTH);

    // Get depth camera intrinsic parameters
    auto depthStreamProfile = depthSensor->getStreamProfileList()->getVideoStreamProfile(0);
    auto intrinsics = depthStreamProfile->getIntrinsic();

    std::cout << "Depth Camera Intrinsic Parameters:\n";
    std::cout << "fx: " << intrinsics.fx << ", fy: " << intrinsics.fy << "\n";
    std::cout << "cx: " << intrinsics.cx << ", cy: " << intrinsics.cy << "\n";
    std::cout << "width: " << intrinsics.width << ", height: " << intrinsics.height << "\n";

    auto rgbSensor = device->getSensor(OB_SENSOR_COLOR);
    auto rgbStreamProfile = rgbSensor->getStreamProfileList()->getVideoStreamProfile(0);
    auto rgbIntrinsics = rgbStreamProfile->getIntrinsic();

    std::cout << "RGB Camera Intrinsic Parameters:\n";
    std::cout << "fx: " << rgbIntrinsics.fx << ", fy: " << rgbIntrinsics.fy << "\n";
    std::cout << "cx: " << rgbIntrinsics.cx << ", cy: " << rgbIntrinsics.cy << "\n";
    std::cout << "width: " << rgbIntrinsics.width << ", height: " << rgbIntrinsics.height << "\n";

    auto depth_to_rgb_extrinsics = depthStreamProfile->getExtrinsicTo(rgbStreamProfile);

    std::cout << "Extrinsic Parameters (Depth to RGB):" << std::endl;
    std::cout << "Rotation Matrix:" << std::endl;
    for (int i = 0; i < 3; i++) {
        std::cout << depth_to_rgb_extrinsics.rot[i * 3] << " "
                    << depth_to_rgb_extrinsics.rot[i * 3 + 1] << " "
                    << depth_to_rgb_extrinsics.rot[i * 3 + 2] << std::endl;
    }

    std::cout << "Translation Vector: "
                << depth_to_rgb_extrinsics.trans[0] << " "
                << depth_to_rgb_extrinsics.trans[1] << " "
                << depth_to_rgb_extrinsics.trans[2] << std::endl;

    auto rgb_to_depth_extrinsics = rgbStreamProfile->getExtrinsicTo(depthStreamProfile);

    std::cout << "Extrinsic Parameters (RGB to Depth):" << std::endl;
    std::cout << "Rotation Matrix:" << std::endl;
    for (int i = 0; i < 3; i++) {
        std::cout << rgb_to_depth_extrinsics.rot[i * 3] << " "
                    << rgb_to_depth_extrinsics.rot[i * 3 + 1] << " "
                    << rgb_to_depth_extrinsics.rot[i * 3 + 2] << std::endl;
    }

    std::cout << "Translation Vector: "
                << rgb_to_depth_extrinsics.trans[0] << " "
                << rgb_to_depth_extrinsics.trans[1] << " "
                << rgb_to_depth_extrinsics.trans[2] << std::endl;
    
}

void CamController::startCameraPipeline() {
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR, 640, 480, 30, OB_FORMAT_MJPG);
    config->enableVideoStream(OB_STREAM_DEPTH, 640);

    // Start the pipeline with config
    pipe_.start(config);
    auto currentProfile = pipe_.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();

    std::cout << "Start Pipeline" << std::endl;
}

bool CamController::getFrameSet(int timeout_ms, int max_cnt) {
    int try_cnt = 0;
    
    while (try_cnt < max_cnt) {
        auto frameSet = pipe_.waitForFrames(timeout_ms);
        if (!frameSet) {
            std::cout << "Failed to get frameSet" << std::endl;
            try_cnt++;
            continue;
        }

        if (frameSet->colorFrame() && frameSet->depthFrame()) {
            current_frameset_ = frameSet;
            return true;
        }

        try_cnt++;
    }

    return false;
}

std::vector<uint8_t> CamController::getColorFrame() {
    std::cout << "Getting color data from colorFrame" << std::endl;
    auto color_frame = current_frameset_->colorFrame();

    uint8_t* data = (uint8_t*)color_frame->data();

    return std::vector<uint8_t> (data, data + color_frame->dataSize());    
}

cv::Mat CamController::getDepthFrame() {
    auto depth_frame = current_frameset_->depthFrame();

    int depth_width = depth_frame->width();
    int depth_height = depth_frame->height();
    uint16_t* depth_data = (uint16_t*)depth_frame->data();

    cv::Mat depth_image(depth_height, depth_width, CV_16UC1, depth_data);

    return depth_image.clone();        
}

void CamController::stopCameraPipeline() {
    pipe_.stop();
}
    
std::vector<float> CamController::pixelToCameraCoords(int u, int v) {
    updateDepthMap();

    float Z = current_depth_map_.at<uint16_t>(v, u);
    float X = (u - rgb_cx_) * Z / rgb_fx_;
    float Y = (v - rgb_cy_) * Z / rgb_fy_;
    return {X, Y, Z};
}