#ifndef KPI_ROVER_SYSTEM_HW_HPP
#define KPI_ROVER_SYSTEM_HW_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kpi_rover/ecu_bridge/ecu_bridge.hpp"

namespace kpi_rover_system_hw
{
class KPIRoverSystemHW : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(KPIRoverSystemHW)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Drive States
    double hw_positions_[4];
    double hw_velocities_[4];
    double hw_commands_[4];

    // IMU States
    double imu_orientation_[4]; // w, x, y, z
    double imu_angular_velocity_[3]; // x, y, z
    double imu_linear_acceleration_[3]; // x, y, z
    double initial_imu_orientation_[4]; // w, x, y, z
    bool first_imu_read_{true};

    // ECU Bridge
    std::unique_ptr<kpi_rover::ECUBridge> ecu_bridge_;

    // Parameters
    int32_t encoder_ticks_per_rev_{4096};
    double wheel_radius_{0.04};

    // Encoder previous values
    long long previous_encoder_values_[4] = {0, 0, 0, 0};
    bool first_encoder_read_{true};

    // Utils
    static constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
    inline int32_t convertToRPM100(double rad_s) {
        return static_cast<int32_t>(rad_s * RAD_S_TO_RPM * 100.0);
    }
};
} // namespace kpi_rover_system_hw

#endif // KPI_ROVER_SYSTEM_HW_HPP
