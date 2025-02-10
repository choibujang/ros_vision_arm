#include <iostream>
#include "libobsensor/ObSensor.hpp"

int main() {
    // Pipeline 객체 생성
    ob::Pipeline pipe;

    // Device 객체 획득
    auto device = pipe.getDevice();

    // 카메라 파라미터 리스트 획득
    auto cameraParamList = device->getCalibrationCameraParamList();

    // 각 파라미터 출력
    for(int i = 0; i < cameraParamList->count(); i++) {
        OBCameraParam obParam = cameraParamList->getCameraParam(i);
        std::cout << "Camera " << i << " Depth Intrinsic Parameters:" << std::endl;
        std::cout << "fx: " << obParam.depthIntrinsic.fx << std::endl;
        std::cout << "fy: " << obParam.depthIntrinsic.fy << std::endl;
        std::cout << "cx: " << obParam.depthIntrinsic.cx << std::endl;
        std::cout << "cy: " << obParam.depthIntrinsic.cy << std::endl;
        std::cout << "width: " << obParam.depthIntrinsic.width << std::endl;
        std::cout << "height: " << obParam.depthIntrinsic.height << std::endl;
    }

    return 0;
}
