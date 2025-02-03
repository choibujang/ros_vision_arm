cmake_minimum_required(VERSION 3.16)
project(deli_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


# find_package(libi2c-dev REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  deli_hardware_interface
  SHARED
  src/pca9685_system.cpp
  src/pca9685_comm.cpp
  src/I2CPeripheral.cpp
)

target_compile_features(deli_hardware_interface PUBLIC cxx_std_17)
target_include_directories(deli_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/deli_hardware_interface>
)
target_link_libraries(deli_hardware_interface PUBLIC i2c)
ament_target_dependencies(
  deli_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "deli_hardware_interface_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface deli_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/deli_hardware_interface
)

install(TARGETS deli_hardware_interface
  EXPORT export_deli_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_deli_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
