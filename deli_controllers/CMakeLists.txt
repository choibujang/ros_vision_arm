cmake_minimum_required(VERSION 3.8)
project(deli_arm_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(OrbbecSDK_DIR "/home/hunter/ws/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

include_directories(include)

find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/local/lib)

add_library(deli_controller SHARED
    src/deli_arm_controller.cpp
    src/I2CPeripheral.cpp
    src/pca9685_comm.cpp
    src/deli_cam_controller.cpp
)

add_executable(angle_test test/angle_test.cpp)
add_executable(move_test test/move_test.cpp)
add_executable(pp_test test/pp_test.cpp)
add_executable(camera_test test/camera_test.cpp)
add_executable(depth_test test/depth_test.cpp)

target_include_directories(deli_controller PUBLIC include)
target_link_libraries(deli_controller PUBLIC ${I2C_LIB})
target_link_libraries(deli_controller PUBLIC OrbbecSDK::OrbbecSDK)

target_link_libraries(angle_test deli_controller)
target_link_libraries(move_test deli_controller)
target_link_libraries(pp_test deli_controller)
target_link_libraries(camera_test deli_controller)
target_link_libraries(depth_test deli_controller)

install(TARGETS deli_controller
    EXPORT deli_controller_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

