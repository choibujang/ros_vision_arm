# Object Detection and Pick-and-Place Robot Arm Using ROS and Depth Camera
## Introduction
이 시스템은 사용자가 ROS 액션 서버를 통해 작업 명령을 요청하면, Depth 카메라로 실시간 물체 인식 및 3D 위치 추정을 수행한 뒤, 로봇 팔이 해당 물체를 집어 지정된 위치로 옮깁니다.
주요 구성 요소는 다음과 같습니다:

- ROS 
- Depth 카메라와 YOLO를 활용한 물체 인식 및 위치 추정
- 로봇 팔의 경로 계획 및 제어


객체 탐지 알고리즘과 **3D 좌표 변환(TF)**을 통한 정확한 위치 추정

MoveIt과 ROS 제어 노드를 통한 inverse kinematics 기반 경로 계획 및 동작 실행

ROS Action 서버/클라이언트 구조를 사용하여 비동기식 Pick-and-Place 명령 수행

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
