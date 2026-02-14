#define _USE_MATH_DEFINES
#include "kpi_rover/hardware_interfaces/kpi_rover_system_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cstring>
#include <memory>
#include <cmath>

namespace kpi_rover_system_hw
{
    const char * LOGGER_NAME = "KPIRoverSystemHW";

    hardware_interface::CallbackReturn KPIRoverSystemHW::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_init()");

        if (!info.hardware_parameters.count("serial_device")) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Missing serial_device parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::string serial_device = info.hardware_parameters.at("serial_device");
        int baud_rate = info.hardware_parameters.count("baud_rate") ? std::stoi(info.hardware_parameters.at("baud_rate")) : 115200;

        if (info.hardware_parameters.count("encoder_ticks_per_rev") > 0) {
            encoder_ticks_per_rev_ = std::stoi(info.hardware_parameters.at("encoder_ticks_per_rev"));
        }

        if (info.hardware_parameters.count("wheel_radius") > 0) {
            wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));
        }

        // Initialize ECU Bridge
        ecu_bridge_ = std::make_unique<kpi_rover::ECUBridge>(serial_device, baud_rate);

        // Initialize buffers
        memset(hw_positions_, 0, sizeof(hw_positions_));
        memset(hw_velocities_, 0, sizeof(hw_velocities_));
        memset(hw_commands_, 0, sizeof(hw_commands_));
        memset(imu_orientation_, 0, sizeof(imu_orientation_));
        imu_orientation_[0] = 1.0; // w=1.0 for valid identity quaternion
        memset(imu_angular_velocity_, 0, sizeof(imu_angular_velocity_));
        memset(imu_linear_acceleration_, 0, sizeof(imu_linear_acceleration_));

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> KPIRoverSystemHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Joint state interfaces
        state_interfaces.emplace_back("front_left_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[3]);
        state_interfaces.emplace_back("front_left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[3]);
        state_interfaces.emplace_back("rear_left_wheel_joint",   hardware_interface::HW_IF_POSITION, &hw_positions_[2]);
        state_interfaces.emplace_back("rear_left_wheel_joint",   hardware_interface::HW_IF_VELOCITY, &hw_velocities_[2]);
        state_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
        state_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);
        state_interfaces.emplace_back("rear_right_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[1]);
        state_interfaces.emplace_back("rear_right_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);

        // IMU state interfaces
        state_interfaces.emplace_back("imu", "orientation.x", &imu_orientation_[1]);
        state_interfaces.emplace_back("imu", "orientation.y", &imu_orientation_[2]);
        state_interfaces.emplace_back("imu", "orientation.z", &imu_orientation_[3]);
        state_interfaces.emplace_back("imu", "orientation.w", &imu_orientation_[0]);
        state_interfaces.emplace_back("imu", "angular_velocity.x", &imu_angular_velocity_[0]);
        state_interfaces.emplace_back("imu", "angular_velocity.y", &imu_angular_velocity_[1]);
        state_interfaces.emplace_back("imu", "angular_velocity.z", &imu_angular_velocity_[2]);
        state_interfaces.emplace_back("imu", "linear_acceleration.x", &imu_linear_acceleration_[0]);
        state_interfaces.emplace_back("imu", "linear_acceleration.y", &imu_linear_acceleration_[1]);
        state_interfaces.emplace_back("imu", "linear_acceleration.z", &imu_linear_acceleration_[2]);

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> KPIRoverSystemHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back("front_left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[3]);
        command_interfaces.emplace_back("rear_left_wheel_joint",   hardware_interface::HW_IF_VELOCITY, &hw_commands_[2]);
        command_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
        command_interfaces.emplace_back("rear_right_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
        return command_interfaces;
    }

    hardware_interface::CallbackReturn KPIRoverSystemHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_configure()");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverSystemHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_cleanup()");
        if (ecu_bridge_) {
            ecu_bridge_->stop();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverSystemHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_activate()");
        if (ecu_bridge_) {
            if (!ecu_bridge_->start()) {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to start ECU bridge");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KPIRoverSystemHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_deactivate()");
        if (ecu_bridge_) {
            ecu_bridge_->stop();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KPIRoverSystemHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        if (!ecu_bridge_) return hardware_interface::return_type::ERROR;

        // 1. Read Encoders
        auto enc = ecu_bridge_->getEncoderData();
        for (int i = 0; i < 4; i++) {
            double position_diff_rad = (2.0 * M_PI * enc.values[i] * -1.0) / encoder_ticks_per_rev_;
            hw_positions_[i] += position_diff_rad;
            hw_velocities_[i] = (position_diff_rad / period.seconds()) * wheel_radius_;
        }

        // 2. Read IMU
        auto imu = ecu_bridge_->getIMUData();
        for (int i = 0; i < 4; i++) imu_orientation_[i] = imu.quat[i];
        for (int i = 0; i < 3; i++) {
            imu_angular_velocity_[i] = imu.gyro[i];
            imu_linear_acceleration_[i] = imu.accel[i];
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KPIRoverSystemHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!ecu_bridge_) return hardware_interface::return_type::ERROR;

        int32_t speeds[4] = {
            convertToRPM100(hw_commands_[0] * -1.0),
            convertToRPM100(hw_commands_[1] * -1.0),
            convertToRPM100(hw_commands_[2] * -1.0),
            convertToRPM100(hw_commands_[3] * -1.0)
        };
        ecu_bridge_->setMotorSpeeds(speeds);

        return hardware_interface::return_type::OK;
    }

} // namespace kpi_rover_system_hw

PLUGINLIB_EXPORT_CLASS(
    kpi_rover_system_hw::KPIRoverSystemHW,
    hardware_interface::SystemInterface
)
