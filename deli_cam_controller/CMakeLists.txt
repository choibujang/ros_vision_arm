cmake_minimum_required(VERSION 3.1.15)
project(deli_came_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#### for colcon build ###
#link_directories(../deli_arm_controller/lib)
#include_directories(../deli_arm_controller/include)
#
#add_library(deli_arm_library SHARED IMPORTED)
#set_target_properties(deli_arm_library PROPERTIES IMPORTED_LOCATION ../deli_arm_controller/lib/libdeli_arm_controller.so)
#
#add_library(deli_cam_library STATIC src/deli_cam_controller.cpp)
#
#target_link_libraries(deli_cam_library PRIVATE deli_arm_library)
#
#install(TARGETS deli_cam_library DESTINATION lib)
#install(FILES include/deli_cam_controller.hpp DESTINATION include)
#### for colcon build end ###

add_executable(get_intrinsic_parameter test/get_intrinsic_parameter.cpp)
add_executable(deli_cam_controller_test test/deli_cam_controller_test.cpp src/deli_cam_controller.cpp)
add_executable(cam_start_test test/cam_start_test.cpp)
add_executable(device_test test/device_test.cpp)

# find Orbbec SDK
set(OrbbecSDK_DIR "../../OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

target_link_libraries(get_intrinsic_parameter OrbbecSDK::OrbbecSDK)
target_link_libraries(deli_cam_controller_test OrbbecSDK::OrbbecSDK)
target_link_libraries(cam_start_test OrbbecSDK::OrbbecSDK)
target_link_libraries(device_test OrbbecSDK::OrbbecSDK)

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