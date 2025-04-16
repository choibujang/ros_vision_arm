cmake_minimum_required(VERSION 3.8)
project(ros_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(deli_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(OrbbecSDK_DIR "/home/hunter/ws/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

# export LD_LIBRARY_PATH=/home/hunter/ws/OrbbecSDK/lib/linux_x64:$LD_LIBRARY_PATH
# sudo usermod -aG i2c $USER

# set(ORBEC_SDK_LIB_PATH "/home/hunter/ws/OrbbecSDK/lib/arm64")
# set(ORBEC_SDK_INCLUDE_PATH "/home/hunter/ws/OrbbecSDK/include")

include_directories(include)

find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/local/lib)

add_library(deli_controller SHARED
    src/deli_arm_controller.cpp
    src/I2CPeripheral.cpp
    src/pca9685_comm.cpp
    src/deli_cam_controller.cpp
)

add_executable(deli_arm_ros src/ros2_nodes/deli_arm_ros.cpp)
add_executable(test_service_client src/ros2_nodes/test_service_client.cpp)
add_executable(angle_test test/angle_test.cpp)
add_executable(move_test test/move_test.cpp)
add_executable(pp_test test/pp_test.cpp)
add_executable(camera_test test/camera_test.cpp)
add_executable(depth_test test/depth_test.cpp)

target_include_directories(deli_controller PUBLIC include)
# target_include_directories(deli_controller PUBLIC ${ORBEC_SDK_INCLUDE_PATH})

target_link_libraries(deli_controller PUBLIC ${I2C_LIB})
target_link_libraries(deli_controller PUBLIC OrbbecSDK::OrbbecSDK)

target_link_libraries(deli_arm_ros deli_controller rclcpp::rclcpp)
target_link_libraries(test_service_client deli_controller rclcpp::rclcpp)
target_link_libraries(angle_test deli_controller)
target_link_libraries(move_test deli_controller)
target_link_libraries(pp_test deli_controller)
target_link_libraries(camera_test deli_controller)
target_link_libraries(depth_test deli_controller)

install(TARGETS deli_controller deli_arm_ros test_service_client
    EXPORT deli_controller_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_target_dependencies(deli_arm_ros
  "deli_interfaces"
  "rclcpp"
  "rclcpp_action")

ament_target_dependencies(test_service_client
  "deli_interfaces"
  "rclcpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()