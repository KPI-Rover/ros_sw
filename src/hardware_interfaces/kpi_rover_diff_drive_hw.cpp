#include "kpi_rover/hardware_interfaces/kpi_rover_diff_drive_hw.hpp"

namespace kpi_rover_diff_drive_hw
{
    const char * LOGGER_NAME = "KPIRoverDiffDriveHW";

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_init(const hardware_interface::HardwareInfo & info)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_init()");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_configure()");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_cleanup()");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_activate()");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_deactivate()");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KPIRoverDiffDriveHW::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "read()");
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KPIRoverDiffDriveHW::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "write()");
        for (int i = 0; i < 4; ++i)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "hw_commands_[%d]: %f", i, hw_commands_[i]);
            
        }
        return hardware_interface::return_type::OK;
    }
    
    std::vector<hardware_interface::StateInterface> KPIRoverDiffDriveHW::export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "export_state_interfaces()");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.push_back(hardware_interface::StateInterface("front_left_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
        state_interfaces.push_back(hardware_interface::StateInterface("front_left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
        state_interfaces.push_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
        state_interfaces.push_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_left_wheel_joint",   hardware_interface::HW_IF_POSITION, &hw_positions_[2]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_left_wheel_joint",   hardware_interface::HW_IF_VELOCITY, &hw_velocities_[2]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_right_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[3]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_right_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[3]));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> KPIRoverDiffDriveHW::export_command_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "export_command_interfaces()");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.push_back(hardware_interface::CommandInterface("front_left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
        command_interfaces.push_back(hardware_interface::CommandInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));
        command_interfaces.push_back(hardware_interface::CommandInterface("rear_left_wheel_joint",   hardware_interface::HW_IF_VELOCITY, &hw_commands_[2]));
        command_interfaces.push_back(hardware_interface::CommandInterface("rear_right_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[3]));
        return command_interfaces;
    }

}  // namespace kpi_rover_diff_drive_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kpi_rover_diff_drive_hw::KPIRoverDiffDriveHW,
    hardware_interface::SystemInterface
)