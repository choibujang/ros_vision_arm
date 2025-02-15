cmake_minimum_required(VERSION 3.8)
project(deli_cam_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(OrbbecSDK_DIR "/home/cook/my_ws/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(deli_cam_controller SHARED
    src/deli_cam_controller.cpp
)

add_executable(depth_test_cam test/depth_test_cam.cpp)
add_executable(cam_start_test test/cam_start_test.cpp)
add_executable(get_camera_param test/get_camera_param.cpp)

target_include_directories(deli_cam_controller PUBLIC include)
target_link_libraries(deli_cam_controller PUBLIC OrbbecSDK::OrbbecSDK ${OpenCV_LIBS})

target_link_libraries(depth_test_cam deli_cam_controller)
target_link_libraries(cam_start_test deli_cam_controller)
target_link_libraries(get_camera_param deli_cam_controller)


install(TARGETS deli_cam_controller
    EXPORT deli_cam_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

