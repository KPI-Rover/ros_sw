#ifndef KPI_ROVER_DIFF_DRIVE_HW__KPI_ROVER_DIFF_DRIVE_HW_HPP_
#define KPI_ROVER_DIFF_DRIVE_HW__KPI_ROVER_DIFF_DRIVE_HW_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kpi_rover_diff_drive_hw
{
class KPIRoverDiffDriveHW : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(KPIRoverDiffDriveHW)

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  double hw_positions_[4];
  double hw_velocities_[4];
  double hw_commands_[4];

};
}  // namespace kpi_rover_diff_drive_hw

#endif  // KPI_ROVER_DIFF_DRIVE_HW__KPI_ROVER_DIFF_DRIVE_HW_HPP_