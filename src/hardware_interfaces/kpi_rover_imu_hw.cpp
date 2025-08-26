#include "pluginlib/class_list_macros.hpp"
#include "kpi_rover/hardware_interfaces/kpi_rover_imu_hw.hpp"
#include "kpi_rover/ecu_bridge/udp_transport.hpp"

namespace kpi_rover_imu_hw
{
  const char * LOGGER_NAME = "KPIRoverIMUHW";

  hardware_interface::CallbackReturn KPIRoverIMUHW::on_init(const hardware_interface::HardwareInfo & info)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_init()");

    if (!info_.hardware_parameters.count("rpi_port")) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Missing RPI port parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }

    rpi_port_ = std::stoi(info_.hardware_parameters["rpi_port"]);
    

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn KPIRoverIMUHW::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_configure()");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn KPIRoverIMUHW::on_cleanup(const rclcpp_lifecycle::State & previous_state)
  {
      RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_cleanup()");
      
      return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn KPIRoverIMUHW::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_activate()");
    auto transport = std::make_unique<kpi_rover::UDPTransport>(
      rpi_port_
    );
    ecu_bridge_ = std::make_unique<kpi_rover::ECUBridgeIMU>(std::move(transport));

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn KPIRoverIMUHW::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "on_deactivate()");
    ecu_bridge_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> KPIRoverIMUHW::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "export_state_interfaces()");

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "orientation.w", &imu_data_.orient[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "orientation.x", &imu_data_.orient[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "orientation.y", &imu_data_.orient[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "orientation.z", &imu_data_.orient[3]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "angular_velocity.x", &imu_data_.ang_vel[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "angular_velocity.y", &imu_data_.ang_vel[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "angular_velocity.z", &imu_data_.ang_vel[2]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "linear_acceleration.x", &imu_data_.accel[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "linear_acceleration.y", &imu_data_.accel[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "linear_acceleration.z", &imu_data_.accel[2]));

    return state_interfaces;
  }

  hardware_interface::return_type KPIRoverIMUHW::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    if (!ecu_bridge_) {
        return hardware_interface::return_type::ERROR;
    }

    if (!ecu_bridge_->getIMUdata(imu_data_, true, 100)) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "getIMUData failed");
        return hardware_interface::return_type::ERROR;
    }

    //Convert angular velocity from deg/s to rad/s
    imu_data_.ang_vel[0] = imu_data_.ang_vel[0] * 180 / M_PI;
    imu_data_.ang_vel[1] = imu_data_.ang_vel[1] * 180 / M_PI;
    imu_data_.ang_vel[2] = imu_data_.ang_vel[2] * 180 / M_PI;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), \
                "[%ld]IMU Data:\n\t \
                  Orientation[units]:\tx=%.3f, y=%.3f, z=%.3f, w=%.3f\n\t \
                  Angular velocity[rad/s]:\tx=%.3f, y=%.3f, z=%.3f\n\t \
                  Linear acceleration[m/s^2]:\tx=%.3f, y=%.3f, z=%.3f", \
                (static_cast<long>(time.nanoseconds())), \
                imu_data_.orient[0], imu_data_.orient[1], imu_data_.orient[2], imu_data_.orient[3], \
                imu_data_.ang_vel[0], imu_data_.ang_vel[1], imu_data_.ang_vel[2], \
                imu_data_.accel[0], imu_data_.accel[1], imu_data_.accel[2]); \

    return hardware_interface::return_type::OK;
  }

};

// Register the hardware interface as a plugin
PLUGINLIB_EXPORT_CLASS(
    kpi_rover_imu_hw::KPIRoverIMUHW, 
    hardware_interface::SensorInterface
)
