#ifndef KPI_ROVER_DIFF_DRIVE_HW__KPI_ROVER_DIFF_DRIVE_HW_HPP_
#define KPI_ROVER_DIFF_DRIVE_HW__KPI_ROVER_DIFF_DRIVE_HW_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kpi_rover/ecu_bridge/ecu_bridge.hpp"  

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

  // Add ECUBridge member
  std::unique_ptr<kpi_rover::ECUBridge> ecu_bridge_;

  // Convert rad/s to RPM*100 (which ECU expects)
  static constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);  // rad/s to RPM conversion factor
  inline int32_t convertToRPM100(double rad_s) {
      return static_cast<int32_t>(rad_s * RAD_S_TO_RPM * 100.0);
  }

};
}  // namespace kpi_rover_diff_drive_hw

#endif  // KPI_ROVER_DIFF_DRIVE_HW__KPI_ROVER_DIFF_DRIVE_HW_HPP_