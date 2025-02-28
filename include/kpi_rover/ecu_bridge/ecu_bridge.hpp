#pragma once
#include <cstdint>
#include <vector>
#include <queue>
#include <memory>
#include <future>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <functional>
#include "kpi_rover/ecu_bridge/transport.hpp"
#include "kpi_rover/ecu_bridge/cache.hpp"  // included cache class

namespace kpi_rover
{
    static constexpr int TIMEOUT_MS = 1000;
    static constexpr uint8_t CMD_GET_API_VERSION      = 0x01;
    static constexpr uint8_t CMD_SET_MOTOR_SPEED      = 0x02;
    static constexpr uint8_t CMD_SET_ALL_MOTORS_SPEED = 0x03;
    static constexpr uint8_t CMD_GET_ENCODER          = 0x04;
    static constexpr uint8_t CMD_GET_ALL_ENCODERS     = 0x05;

    static constexpr size_t RESP_LEN_GET_API_VERSION      = 2;
    static constexpr size_t RESP_LEN_SET_MOTOR_SPEED      = 1;
    static constexpr size_t RESP_LEN_SET_ALL_MOTORS_SPEED = 1;
    static constexpr size_t RESP_LEN_GET_ENCODER          = 5;
    static constexpr size_t RESP_LEN_GET_ALL_ENCODERS     = 17;

    /**
     * @brief ECUBridge implements the protocol layer with a queued request mechanism.
     *
     * This class provides synchronous get/set functions which return cached values immediately
     * while initiating asynchronous refreshes.
     */
    class ECUBridge
    {
    public:
        /**
         * @brief Constructor.
         * @param transport Pointer to a Transport implementation.
         */
        ECUBridge(std::unique_ptr<Transport> transport);

        /**
         * @brief Destructor.
         */
        ~ECUBridge();

        /**
         * @brief Retrieves the API version.
         * @param driver_version The ROS2 driver version.
         * @param sync If true, the function returns the value synchronously (waiting for response, or 0 on timeout).
         * @return API version (0 if timeout or not cached).
         */
        uint8_t getAPIVersion(uint8_t driver_version, bool sync = false);

        /**
         * @brief Retrieves the encoder value for a motor.
         * @param motor_id The motor identifier (0-3).
         * @param sync If true, the function waits for a response and returns the value (or 0 on timeout).
         * @return Encoder value (0 if timeout or not cached).
         */
        int32_t getEncoder(uint8_t motor_id, bool sync = false);

        /**
         * @brief Retrieves all encoder values.
         * @param sync If true, waits for a response and returns the values (or a vector of 0s on timeout).
         * @return Vector of encoder values (size 4; defaults to 0 if timeout or not cached).
         */
        std::vector<int32_t> getAllEncoders(bool sync = false);

        /**
         * @brief Sets the speed for a single motor synchronously.
         *
         * This method returns the cached result for the previous command, then triggers an asynchronous update.
         *
         * @param motor_id The motor identifier (0-3).
         * @param speed The desired speed (in RPM*100, signed).
         * @return 0 if the previous call was successful, nonzero error code otherwise.
         */
        uint8_t setMotorSpeed(uint8_t motor_id, int32_t speed);

        /**
         * @brief Sets speeds for all four motors synchronously.
         *
         * This method returns the cached result for the previous command, then triggers an asynchronous update.
         *
         * @param speed1 Speed for motor 1 (in RPM*100, signed).
         * @param speed2 Speed for motor 2 (in RPM*100, signed).
         * @param speed3 Speed for motor 3 (in RPM*100, signed).
         * @param speed4 Speed for motor 4 (in RPM*100, signed).
         * @return 0 if the previous call was successful, nonzero error code otherwise.
         */
        uint8_t setAllMotorsSpeed(int32_t speed1, int32_t speed2, int32_t speed3, int32_t speed4);

        /**
         * @brief Updates all caches asynchronously.
         *
         * This function triggers the public getters (which update caches asynchronously)
         * for the API version and encoders. It should be called on system initialization.
         *
         * @param driver_version The ROS2 driver version.
         */
        void updateCash(uint8_t driver_version);

    private:
        bool sendCommand(const std::vector<uint8_t> &request, std::vector<uint8_t> &response, int timeout_ms);
        void processQueue();
        size_t getExpectedResponseLength(uint8_t cmd_id);
        std::future<std::vector<uint8_t>> enqueueCommand(const std::vector<uint8_t> &request);

        std::unique_ptr<Transport> transport_;
        std::queue<std::pair<std::vector<uint8_t>, std::promise<std::vector<uint8_t>>>> request_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        bool stop_;
        std::thread worker_thread_;

        // Get caches.
        ValueCache<uint8_t> apiVersionCache_;
        ValueCache<std::vector<int32_t>> encodersCache_;

        // Set caches: 0 means OK, nonzero error code.
        // For per-motor speed result (vector size = 4).
        ValueCache<std::vector<uint8_t>> motorSpeedResultCache_;
        // For all motors speed result.
        ValueCache<uint8_t> allMotorsSpeedResultCache_;

        uint8_t asyncGetAPIVersionSync(uint8_t driver_version);
        int32_t asyncGetEncoderSync(uint8_t motor_id);
        std::vector<int32_t> asyncGetAllEncodersSync();
    };

} // namespace kpi_rover
