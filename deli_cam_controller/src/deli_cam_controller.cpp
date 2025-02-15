#include "deli_cam_controller.hpp"

void DeliCamController::startCam() {
    std::cout << "DeliCamController::startColorStream started" << std::endl;
    this->start_cam = true;

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);

    this->pipe.start(config);

    std::cout << "pipe started" << std::endl;

    std::cout << "Waiting for camera to initialize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초 대기
    std::cout << "Camera initialized. Requesting frames..." << std::endl;

    uint32_t frame_id = 0; 

    while (this->start_cam) {
        auto frameSet = this->pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            std::cerr << "Error: frameSet is nullptr!" << std::endl;
            continue;
        }

        auto colorFrame = frameSet->colorFrame();
        
        std::cout << colorFrame->width() << std::endl;
        std::cout << colorFrame->height() << std::endl;

        std::vector<uint8_t> mjpeg_data((uint8_t*)colorFrame->data(), (uint8_t*)colorFrame->data() + colorFrame->dataSize());

        int total_size = mjpeg_data.size();
        int num_chunks = (total_size + max_chunk_size - 1) / max_chunk_size;

        for (int i = 0; i < num_chunks; i++) {
            int offset = i * max_chunk_size;
            int chunk_size = std::min(max_chunk_size, total_size - offset);

            std::vector<uint8_t> packet(header_size + chunk_size);

            memcpy(packet.data(), &frame_id, 4);
            uint16_t chunk_idx = i;
            uint16_t total_chunks = num_chunks;
            memcpy(packet.data() + 4, &chunk_idx, 2);
            memcpy(packet.data() + 6, &total_chunks, 2);

            memcpy(packet.data() + header_size, mjpeg_data.data() + offset, chunk_size);

            ssize_t sent_len = sendto(this->sock, packet.data(), packet.size(), 0,
                                      (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));

            if (sent_len < 0) {
                std::cerr << "Sending failed! Error: " << strerror(errno) << std::endl;
                break;
            }
        }
        sendto(this->sock, frame_end, strlen(frame_end), 0,
               (struct sockaddr *)&(this->server_addr), sizeof(this->server_addr));
        frame_id++;
    }    
    this->pipe.stop(); 
}

cv::Mat DeliCamController::convertCoorPixToCam(int center_x, int center_y) {
    std::cout << "DeliCamController::getDepthValue started" << std::endl;

    float rgb_x = (center_x - rgb_cx) / rgb_fx;
    float rgb_y = (center_y - rgb_cy) / rgb_fy;
    float rgb_z = 1.0;

    std::cout << "rgb x, y, z: " << rgb_x << ", " << rgb_y << ", " << rgb_z << std::endl;

    cv::Mat P_rgb = (cv::Mat_<float>(3,1) << rgb_x, rgb_y, rgb_z);

    cv::Mat P_depth = rgb_to_depth_rot * P_rgb + rgb_to_depth_trans;

    float depth_x = P_depth.at<float>(0,0);
    float depth_y = P_depth.at<float>(1,0);
    float depth_z = P_depth.at<float>(2,0);
    
    std::cout << "depth x, y, z: " << depth_x << ", " << depth_y << ", " << depth_z << std::endl;

    int depth_pix_x = static_cast<int>((depth_x * depth_fx) / depth_z + depth_cx);
    int depth_pix_y = static_cast<int>((depth_y * depth_fy) / depth_z + depth_cy);

    std::cout << "depth pix x, y: " << depth_pix_x << ", " << depth_pix_y << std::endl;
    
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_DEPTH, 640, 480, 30, OB_FORMAT_Y16);

    this->pipe.start(config);

    std::cout << "pipe started" << std::endl;

    std::cout << "Waiting for camera to initialize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));  // 2초 대기
    std::cout << "Camera initialized. Requesting frames..." << std::endl;

    while(true) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
	    std::cout << "frameSet nullptr" << std::endl;
            continue;
        }

        auto depthFrame = frameSet->depthFrame();

        // for Y16 format depth frame, print the distance of the center pixel every 30 frames
        if(depthFrame->index() % 30 == 0 && depthFrame->format() == OB_FORMAT_Y16) {
            uint32_t  width  = depthFrame->width();
            uint32_t  height = depthFrame->height();
            float     scale  = depthFrame->getValueScale();
            uint16_t *data   = (uint16_t *)depthFrame->data();

            // pixel value multiplied by scale is the actual distance value in millimeters
            depth_z = data[width * int(depth_pix_y) + int(depth_pix_x)] * scale;
	        std::cout << "centerDistance: " << depth_z << std::endl;

            // attention: if the distance is 0, it means that the depth camera cannot detect the object（may be out of detection range）
            if (depth_z) {
                this->pipe.stop();
                break;
            }
        }
    }

    float cam_x = (depth_pix_x - depth_cx) * depth_z / depth_fx;
    float cam_y = (depth_pix_y - depth_cy) * depth_z / depth_fy;
    float cam_z = depth_z;

    cv::Mat P_cam = (cv::Mat_<float>(3,1) << cam_x, cam_y, cam_z);

    return P_cam;

}

void DeliCamController::getCamParam() {
    auto device = pipe.getDevice();
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

// std::vector<float> DeliCamController::calcCamCoordinatePose(int u, int v, int z) {

//     float camera_x = 
// }

// int DeliCamController::sendImage() {
//     int cnt = 0;
//     int result;
  

//     return result;
// }
    
