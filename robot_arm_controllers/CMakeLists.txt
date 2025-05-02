cmake_minimum_required(VERSION 3.8)
project(robot_arm_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/local/lib)

# include_directories(${OpenCV_INCLUDE_DIRS})

add_library(arm_controller SHARED
    src/arm/arm_controller.cpp
    src/arm/I2CPeripheral.cpp
    src/arm/pca9685_comm.cpp
)
target_include_directories(arm_controller
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_controller PUBLIC ${I2C_LIB})


add_library(cam_controller SHARED
    src/cam/cam_controller.cpp
)
target_include_directories(cam_controller 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(cam_controller PUBLIC OrbbecSDK::OrbbecSDK ${OpenCV_LIBS})


add_library(net_controller SHARED
    src/net/net_controller.cpp
)
target_include_directories(net_controller 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(depth_test_cam test/depth_test_cam.cpp)
add_executable(camera_test test/camera_test.cpp)

target_link_libraries(depth_test_cam cam_controller)
target_link_libraries(camera_test cam_controller)


install(
  TARGETS 
    arm_controller
    cam_controller
    net_controller
  EXPORT robot_arm_controllers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

install(
  EXPORT robot_arm_controllers
  FILE robot_arm_controllersConfig.cmake
  NAMESPACE robot_arm_controllers::
  DESTINATION lib/cmake/robot_arm_controllers
)