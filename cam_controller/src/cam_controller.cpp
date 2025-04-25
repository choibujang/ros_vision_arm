#include "cam_controller.hpp"



/*****************************
@brief
    Depth 이미지를 RGB 이미지 좌표계에 맞게 정렬하여
    특정 RGB 픽셀의 Depth 값을 조회할 수 있는 depth map 생성.
    RGB 이미지와 Depth 이미지의 해상도 차이로 인해
    Depth = 0인 빈 영역은 3x3 주변 평균으로 보간하여 채움.

@param  depth 원본 Depth 이미지 (640 * 400)
@return RGB 시점에 정렬된 depth 이미지 (640 * 480)
*******************************/

cv::Mat CamController::alighDepthToRGB(const cv::Mat& depth) {
    cv::Mat aligned = cv::Mat::zeros(this->rgb_height, this->rgb_width, CV_16UC1);

    for (int vd = 0; vd < depth.rows; ++vd) {
        for (int ud = 0; ud < depth.cols;) {
            uint16_t z = depth.at<uint16_t>(vd, ud);
            if (z == 0) continue;

            // Depth 픽셀 좌표를 카메라 좌표계의 3ㅇ 
            float Xd = (ud - this->depth_cx) * z / this->depth_fx;
            float Yd = (vd - this->depth_cy) * z / this->depth_fy;
            float Zd = z;

            // Depth → RGB 카메라 좌표계로 변환
            cv::Mat depth_3d = (cv::Mat_<float>(3,1) << Xd, Yd, Zd);
            cv::Mat rgb_3d = this->depth_to_rgb_rot * depth_3d + this->depth_to_rgb_trans;

            float Xr = rgb_3d.at<float>(0);
            float Yr = rgb_3d.at<float>(1);
            float Zr = rgb_3d.at<float>(2);

            if (Zr <= 0) continue; // 뒤에 있는 점 무시

            // RGB 카메라에서의 픽셀 위치
            int u_rgb = static_cast<int>(this->rgb_fx * Xr / Zr + this->rgb_cx);
            int v_rgb = static_cast<int>(this->rgb_fy * Yr / Zr + this->rgb_cy);

            // 범위 체크 후 저장
            if (u_rgb >= 0 && u_rgb < this->rgb_width && v_rgb >= 0 && v_rgb < this->rgb_height) {
                uint16_t& current = aligned.at<uint16_t>(v_rgb, u_rgb);
                // 이미 값이 있다면 더 가까운 z만 저장
                if (current == 0 || Zr < current) {
                    current = static_cast<uint16_t>(Zr);
                }
            }

        }
    }

    // Z = 0인 빈 영역 3x3 평균으로 보간
    cv::Mat filled = aligned.clone();
    for (int y = 1; y < aligned.rows - 1; ++y) {
        for (int x = 1; x < aligned.cols - 1; ++x) {
            if (aligned.at<uint16_t>(y, x) == 0) {
                int sum = 0;
                int count = 0;

                // 주변 3x3 이웃 평균
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        uint16_t neighbor_z = aligned.at<uint16_t>(y + dy, x + dx);
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

    return filled;

}

// void CamController::startCam() {
//     std::cout << "CamController::startColorStream started" << std::endl;
//     this->start_cam = true;

//     std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
//     config->enableVideoStream(OB_STREAM_COLOR);

//     this->pipe.start(config);

//     std::cout << "pipe started" << std::endl;

//     std::cout << "Waiting for camera to initialize..." << std::endl;
//     std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초 대기
//     std::cout << "Camera initialized. Requesting frames..." << std::endl;

//     uint32_t frame_id = 0; 

//     while (this->start_cam) {
//         auto frameSet = this->pipe.waitForFrames(100);
//         if(frameSet == nullptr) {
//             std::cerr << "Error: frameSet is nullptr!" << std::endl;
//             continue;
//         }

//         auto colorFrame = frameSet->colorFrame();
        
//         std::cout << colorFrame->width() << std::endl;
//         std::cout << colorFrame->height() << std::endl;

//         std::vector<uint8_t> mjpeg_data((uint8_t*)colorFrame->data(), (uint8_t*)colorFrame->data() + colorFrame->dataSize());

//         int total_size = mjpeg_data.size();
//         int num_chunks = (total_size + max_chunk_size - 1) / max_chunk_size;

//         for (int i = 0; i < num_chunks; i++) {
//             int offset = i * max_chunk_size;
//             int chunk_size = std::min(max_chunk_size, total_size - offset);

//             std::vector<uint8_t> packet(header_size + chunk_size);

//             memcpy(packet.data(), &frame_id, 4);
//             uint16_t chunk_idx = i;
//             uint16_t total_chunks = num_chunks;
//             memcpy(packet.data() + 4, &chunk_idx, 2);
//             memcpy(packet.data() + 6, &total_chunks, 2);

//             memcpy(packet.data() + header_size, mjpeg_data.data() + offset, chunk_size);

//             ssize_t sent_len = sendto(this->sock, packet.data(), packet.size(), 0,
//                                       (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));

//             if (sent_len < 0) {
//                 std::cerr << "Sending failed! Error: " << strerror(errno) << std::endl;
//                 break;
//             }
//         }
//         sendto(this->sock, frame_end, strlen(frame_end), 0,
//                (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
//         frame_id++;
//     }    
//     this->pipe.stop(); 
// }

// cv::Mat CamController::convertCoorPixToCam(int center_x, int center_y) {
//     std::cout << "CamController::getDepthValue started" << std::endl;

//     float v_d = center_y * (400.0/480.0);
//     float u_d = ((center_x - rgb_cx) * depth_fx / rgb_fx) + depth_cx;
//     v_d = ((v_d - rgb_cy) * depth_fy / rgb_fy) + depth_cy;
    
//     int center_depth;

//     std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
//     config->enableVideoStream(OB_STREAM_DEPTH);

//     this->pipe.start(config);
 
//     std::cout << "pipe started" << std::endl;

//     std::cout << "Waiting for camera to initialize..." << std::endl;
//     std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초 대기
//     std::cout << "Camera initialized. Requesting frames..." << std::endl;

//     while(true) {
//         // Wait for up to 100ms for a frameset in blocking mode.
//         auto frameSet = pipe.waitForFrames(100);
//         if(frameSet == nullptr) {
// 	    std::cout << "frameSet nullptr" << std::endl;
//             continue;
//         }

//         auto depthFrame = frameSet->depthFrame();

//         // for Y16 format depth frame, print the distance of the center pixel every 30 frames
//         if(depthFrame->index() % 30 == 0 && depthFrame->format() == OB_FORMAT_Y16) {
//             uint32_t  width  = depthFrame->width();
//             uint32_t  height = depthFrame->height();
//             float     scale  = depthFrame->getValueScale();
//             uint16_t *data   = (uint16_t *)depthFrame->data();

//             // pixel value multiplied by scale is the actual distance value in millimeters
//             center_depth = data[width * static_cast<int>(v_d) + static_cast<int>(u_d)] * scale;
// 	        std::cout << "centerDistance: " << center_depth << std::endl;

//             // attention: if the distance is 0, it means that the depth camera cannot detect the object（may be out of detection range）
//             if (center_depth) {
//                 this->pipe.stop();
//                 break;
//             }
//         }
//     }

//     float cam_x = (center_x - rgb_cx) * center_depth / rgb_fx;
//     float cam_y = (center_y - rgb_cy) * center_depth / rgb_fy;
//     float cam_z = static_cast<float>(center_depth);

//     cv::Mat P_cam = (cv::Mat_<float>(3,1) << cam_x, cam_y, cam_z);

//     std::cout << "cam x, y, z: " << cam_x << ", " << cam_y << ", " << cam_z << std::endl;
//     // X: -57.276, Y: 81.1239, Z: 254
//     // base frame xyz: 220, 60, 10
//     return P_cam;

// }

void CamController::getCameraParam() {
    auto device = this->pipe.getDevice();
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
    this->pipe.start(config);
    auto currentProfile = this->pipe.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();
}

bool CamController::getFrameSet(int timeout_ms, int max_cnt) {
    int try_cnt = 0;
    
    while (try_cnt < max_cnt) {
        auto frameSet = this->pipe.waitForFrames(timeout_ms);
        if (!frameSet) {
            try_cnt++;
            continue;
        }

        if (frameSet->colorFrame() && frameSet->depthFrame()) {
            this->current_frameset = frameSet;
            return true;
        }

        try_cnt++;
    }

    return false;
}

std::vector<uint8_t> CamController::getMjpegColorData() {
    auto colorFrame = this->current_frameset->colorFrame();

    if (colorFrame == nullptr) {
        throw std::runtime_error("No color frame");
    }

    uint8_t* data = (uint8_t*)colorFrame->data();

    return std::vector<uint8_t> (data, data + colorFrame->dataSize());    
}

cv::Mat CamController::getMatDepthData() {
    auto depthFrame = this->current_frameset->depthFrame();

    if (depthFrame == nullptr) {
        throw std::runtime_error("No depth frame");
    }

    int depthWidth = depthFrame->width();
    int depthHeight = depthFrame->height();
    uint16_t* depthData = (uint16_t*)depthFrame->data();

    cv::Mat depth_image(depthHeight, depthWidth, CV_16UC1, depthData);

    return depth_image;        
}

void CamController::stopCameraPipeline() {
    this->pipe.stop();
}
    
