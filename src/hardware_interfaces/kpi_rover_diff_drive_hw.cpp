#include "kpi_rover/hardware_interfaces/kpi_rover_diff_drive_hw.hpp"
#include "kpi_rover/ecu_bridge/tcp_transport.hpp"

namespace kpi_rover_diff_drive_hw
{
    const char * LOGGER_NAME = "KPIRoverDiffDriveHW";

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_init(const hardware_interface::HardwareInfo & info)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_init()");
        
        if (!info_.hardware_parameters.count("ecu_ip") ||
            !info_.hardware_parameters.count("ecu_port")) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Missing ECU connection parameters");
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::string ecu_ip = info_.hardware_parameters["ecu_ip"];
        int ecu_port = std::stoi(info_.hardware_parameters["ecu_port"]);
        
        // Read encoder ticks parameter
        if (info_.hardware_parameters.count("encoder_ticks_per_rev") > 0) {
            encoder_ticks_per_rev_ = std::stoi(info_.hardware_parameters.at("encoder_ticks_per_rev"));
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                        "Using encoder ticks per revolution: %d", encoder_ticks_per_rev_);
        }

        // Create transport with parameters from config
        auto transport = std::make_unique<kpi_rover::TCPTransport>(
            ecu_ip,
            ecu_port,
            kpi_rover::DEFAULT_RECONNECT_INTERVAL_MS
        );
        ecu_bridge_ = std::make_unique<kpi_rover::ECUBridge>(std::move(transport));
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverDiffDriveHW::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_configure()");
        
        // Initialize ECU caches
        if (ecu_bridge_) {
            ecu_bridge_->updateCash(1);  // TODO: Make driver version configurable
        }
        
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
        if (!ecu_bridge_) {
            return hardware_interface::return_type::ERROR;
        }

        // Get all encoder values as a vector
        auto encoder_values = ecu_bridge_->getAllEncoders(true);
        
        if (encoder_values.size() != 4) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
                         "getAllEncoders returned wrong number of values");
            return hardware_interface::return_type::ERROR;
        }

        // Log raw encoder values
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "Raw encoder values: [%d, %d, %d, %d]",
                    encoder_values[0], encoder_values[1], 
                    encoder_values[2], encoder_values[3]);

        // Convert encoder values to radians and angular velocity
        for (size_t i = 0; i < 4; ++i) {
            // Convert encoder ticks to radians
            double position_rad = (2.0 * M_PI * encoder_values[i]) / encoder_ticks_per_rev_;
            
            // Calculate velocity (rad/s) using position difference and period
            double velocity_rad_s = (position_rad - hw_positions_[i]) / period.seconds();
            
            hw_positions_[i] = position_rad;
            hw_velocities_[i] = velocity_rad_s;
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                    "Joint positions [rad]: [%.2f, %.2f, %.2f, %.2f]",
                    hw_positions_[0], hw_positions_[1], 
                    hw_positions_[2], hw_positions_[3]);
        
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                    "Joint velocities [rad/s]: [%.2f, %.2f, %.2f, %.2f]",
                    hw_velocities_[0], hw_velocities_[1], 
                    hw_velocities_[2], hw_velocities_[3]);
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KPIRoverDiffDriveHW::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "write()");
        
        // Use ECUBridge to set motor speeds
        if (ecu_bridge_) {
            uint8_t result = ecu_bridge_->setAllMotorsSpeed(
                convertToRPM100(hw_commands_[0]),
                convertToRPM100(hw_commands_[1]),
                convertToRPM100(hw_commands_[2]),
                convertToRPM100(hw_commands_[3])
            );
            
            if (result != 0) {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
                           "setAllMotorsSpeed failed with code: %d", result);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "Commands [rad/s]: [%.2f, %.2f, %.2f, %.2f]",
                    hw_commands_[0], hw_commands_[1], 
                    hw_commands_[2], hw_commands_[3]);

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "Commands [RPM]: [%.2f, %.2f, %.2f, %.2f]",
                    hw_commands_[0] * RAD_S_TO_RPM,
                    hw_commands_[1] * RAD_S_TO_RPM,
                    hw_commands_[2] * RAD_S_TO_RPM,
                    hw_commands_[3] * RAD_S_TO_RPM);
        
        return hardware_interface::return_type::OK;
    }
    
    std::vector<hardware_interface::StateInterface> KPIRoverDiffDriveHW::export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "export_state_interfaces()");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.push_back(hardware_interface::StateInterface("front_left_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
        state_interfaces.push_back(hardware_interface::StateInterface("front_left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_left_wheel_joint",   hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_left_wheel_joint",   hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));
        state_interfaces.push_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[2]));
        state_interfaces.push_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[2]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_right_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[3]));
        state_interfaces.push_back(hardware_interface::StateInterface("rear_right_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[3]));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> KPIRoverDiffDriveHW::export_command_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "export_command_interfaces()");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.push_back(hardware_interface::CommandInterface("front_left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
        command_interfaces.push_back(hardware_interface::CommandInterface("rear_left_wheel_joint",   hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));
        command_interfaces.push_back(hardware_interface::CommandInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[2]));
        command_interfaces.push_back(hardware_interface::CommandInterface("rear_right_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[3]));
        return command_interfaces;
    }
}  // namespace kpi_rover_diff_drive_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kpi_rover_diff_drive_hw::KPIRoverDiffDriveHW,
    hardware_interface::SystemInterface
)