#pragma once
#include "kpi_rover/ecu_bridge/serial_transport.hpp"
#include "kpi_rover/ecu_bridge/ecu_protocol.hpp"
#include "kpi_rover/ecu_bridge/cache.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <vector>
#include <mutex>

namespace kpi_rover
{
    struct IMUData {
        float accel[3];
        float gyro[3];
        float mag[3];
        float quat[4];
    };

    struct EncoderData {
        int32_t values[4];
    };

    /**
     * @brief The ECUBridge class manages serial communication, state caching, and periodic tasks.
     */
    class ECUBridge
    {
    public:
        /**
         * @brief Constructor.
         * @param device Serial device path.
         * @param baud_rate Serial baud rate.
         */
        ECUBridge(const std::string &device, int baud_rate);

        /**
         * @brief Destructor.
         */
        ~ECUBridge();

        /**
         * @brief Starts the bridge and its periodic tasks.
         * @return true if successful, false otherwise.
         */
        bool start();

        /**
         * @brief Stops the bridge and its periodic tasks.
         */
        void stop();

        /**
         * @brief Sets the polling period for IMU data.
         * @param ms Period in milliseconds.
         */
        void setIMUPeriod(std::chrono::milliseconds ms) { imu_period_ = ms; }

        /**
         * @brief Sets the polling period for encoder data.
         * @param ms Period in milliseconds.
         */
        void setEncoderPeriod(std::chrono::milliseconds ms) { encoder_period_ = ms; }

        /**
         * @brief Sets the period for pushing motor commands.
         * @param ms Period in milliseconds.
         */
        void setMotorPeriod(std::chrono::milliseconds ms) { motor_period_ = ms; }

        /**
         * @brief Retrieves the latest cached IMU data.
         * @return IMUData struct.
         */
        IMUData getIMUData() const { return imu_cache_.get(); }

        /**
         * @brief Retrieves the latest cached encoder data.
         * @return EncoderData struct.
         */
        EncoderData getEncoderData() const { return encoder_cache_.get(); }

        /**
         * @brief Retrieves the cached API version of the ECU.
         * @return API version number.
         */
        uint8_t getAPIVersion() const { return api_version_cache_.get(); }

        /**
         * @brief Sets the target motor speeds to be sent to the ECU.
         * @param speeds Array of 4 speeds (one for each motor).
         */
        void setMotorSpeeds(const int32_t speeds[4]);

        /**
         * @brief Checks if the bridge is currently connected to the ECU.
         * @return true if connected (recent successful communication), false otherwise.
         */
        bool isConnected() const;

        /**
         * @brief Opens the serial transport without starting the periodic manager thread.
         * Useful for testing or manual control.
         * @return true if opened successfully.
         */
        bool open();

        /**
         * @brief Synchronizes the API version with the ECU.
         * Sends a request to get the API version and updates the cache.
         * @return true if successful, false otherwise.
         */
        bool syncAPIVersion();

        /**
         * @brief Polls IMU data from the ECU immediately.
         * Updates the IMU cache on success.
         * @return true if successful, false otherwise.
         */
        bool pollIMU();

        /**
         * @brief Polls encoder data from the ECU immediately.
         * Updates the encoder cache on success.
         * @return true if successful, false otherwise.
         */
        bool pollEncoders();

        /**
         * @brief Pushes the current motor setpoints to the ECU immediately.
         * @return true if successful, false otherwise.
         */
        bool pushMotors();

        /**
         * @brief Sends a raw custom request to the ECU.
         * Useful for testing new commands or debugging.
         * @param cmd_id The command ID of the request.
         * @param payload The payload data for the request.
         * @param resp Reference to store the response message.
         * @return true if successful, false otherwise.
         */
        bool sendRawRequest(uint8_t cmd_id, const std::vector<uint8_t>& payload, ECUProtocol::Message& resp);

    private:
        // Components
        SerialTransport transport_;
        std::string logger_name_;
        
        // Caches
        ValueCache<IMUData> imu_cache_;
        ValueCache<EncoderData> encoder_cache_;
        ValueCache<uint8_t> api_version_cache_;
        ValueCache<std::vector<int32_t>> motor_setpoints_cache_;

        // Periods
        std::atomic<std::chrono::milliseconds> imu_period_{std::chrono::milliseconds(20)}; 
        std::atomic<std::chrono::milliseconds> encoder_period_{std::chrono::milliseconds(100)};
        std::atomic<std::chrono::milliseconds> motor_period_{std::chrono::milliseconds(100)};

        // Execution control
        std::atomic<bool> running_{false};
        std::atomic<bool> is_connected_{false};
        std::chrono::steady_clock::time_point last_success_time_;
        std::thread manager_thread_;
        std::mutex serial_mutex_;
        rclcpp::Clock clock_{RCL_STEADY_TIME};
        unsigned int request_counter_total_{0};
        unsigned int request_counter_success_{0};

        void managerLoop();
        
        // Generic request-response helper
        bool sendRequest(const ECUProtocol::Message &req, ECUProtocol::Message &resp, int timeout_ms = 1000);
        
        // Frame reading helper
        bool readFrame(std::vector<uint8_t> &frame, int timeout_ms);
    };
} // namespace kpi_rover
