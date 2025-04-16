cmake_minimum_required(VERSION 3.8)
project(arm_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/local/lib)

add_library(arm_controller SHARED
    src/deli_arm_controller.cpp
    src/I2CPeripheral.cpp
    src/pca9685_comm.cpp
)

add_executable(angle_test test/angle_test.cpp)
add_executable(move_test test/move_test.cpp)
add_executable(pp_test test/pp_test.cpp)
add_executable(camera_test test/camera_test.cpp)
add_executable(depth_test_arm test/depth_test_arm.cpp)

target_include_directories(deli_arm_controller PUBLIC include)
target_link_libraries(deli_arm_controller PUBLIC ${I2C_LIB})

target_link_libraries(angle_test deli_arm_controller)
target_link_libraries(move_test deli_arm_controller)
target_link_libraries(pp_test deli_arm_controller)
target_link_libraries(camera_test deli_arm_controller)
target_link_libraries(depth_test_arm deli_arm_controller)

install(TARGETS deli_arm_controller
    EXPORT deli_arm_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

