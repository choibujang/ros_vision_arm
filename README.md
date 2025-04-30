# ROS 기반 로봇 팔 자동화 프로젝트 🤖

## 📌 프로젝트 개요
이 프로젝트는 주어진 물체 목록을 기반으로 
Depth Camera를 이용해 대상 물체를 인식하고 3D 좌표를 추정한 뒤,
로봇 팔을 제어하여 지정된 위치로 물체를 이동시키는 자동화 시스템을 구축하는 것을 목표로 합니다.

## 🖼️ 시스템 구성도
![System Architecture](./assets/system_architecture.png)
## 📷 사용 장비
- RaspberryPi 4B
- [Orbbec] Astra Stereo S U3 3D Depth Camera
- 서보모터 MG996R * 6
- PCA9685 16채널 12비트 PWM 서보 드라이버
- USB C PD TO 터미널 단자 트리거 모듈 MALE [HPRO-0024]

## 🚀 Getting Started
### 1. 시스템 요구사항
- **운영체제**: Ubuntu 20.04
- **ROS2 버전**: Humble
- Orbbec Astra SDK
### 2. 프로젝트 다운로드

### 3. 환경변수 설정
```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:{path_to_orbbecSDK}
export LD_LIBRARY_PATH={path_to_OrbbecSDK}/lib/arm64:$LD_LIBRARY_PATH
```

### 4. 빌드 및 실행




## Setting
OrbbecSDK 받고
CMAKE_PREFIX_PATH에 OrbbecSDK 경로 넣고
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/OrbbecSDK

ld library path도 설정하고
export LD_LIBRARY_PATH=/.../OrbbecSDK/lib/arm64:$LD_LIBRARY_PATH

내 프로젝트 빌드
cd ros_vision_arm/robot_arm/cam_controller
# 1. 빌드 디렉토리 생성
mkdir build
cd build

# 2. cmake로 설정 (CMakeLists.txt 있는 상위 디렉토리를 지정)
cmake ..

# 3. make로 빌드
make
