cmake_minimum_required(VERSION 3.8)
project(cam_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(cam_controller SHARED
    src/cam_controller.cpp
)

add_executable(depth_test_cam test/depth_test_cam.cpp)
add_executable(camera_test test/camera_test.cpp)

target_include_directories(cam_controller PUBLIC include)
target_link_libraries(cam_controller PUBLIC OrbbecSDK::OrbbecSDK ${OpenCV_LIBS})

target_link_libraries(depth_test_cam cam_controller)
target_link_libraries(camera_test cam_controller)


install(TARGETS cam_controller
    EXPORT cam_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

