#ifndef deli_hardware_interface__PCA9685_SYSTEM_HPP_
#define deli_hardware_interface__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "deli_hardware_interface/visibility_control.h"
#include <deli_hardware_interface/pca9685_comm.h>

namespace deli_hardware_interface
{
class Pca9685SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  deli_hardware_interface_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  deli_hardware_interface_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  deli_hardware_interface_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  deli_hardware_interface_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  deli_hardware_interface_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  deli_hardware_interface_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  deli_hardware_interface_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> vel_commands_;
  std::vector<double> pos_commands_;
  PiPCA9685::PCA9685 pca;
  double command_to_duty_cycle(double command);
};

}  // namespace deli_hardware_interface

#endif  // deli_hardware_interface__PCA9685_SYSTEM_HPP_
