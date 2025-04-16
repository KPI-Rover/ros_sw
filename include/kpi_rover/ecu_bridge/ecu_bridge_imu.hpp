#pragma once
#include <cstdint>
#include <vector>
#include <memory>
#include <future>
#include <mutex>
#include <condition_variable>
#include <string>
#include <functional>
#include <atomic>
#include <chrono>
#include "kpi_rover/ecu_bridge/transport.hpp"
#include "kpi_rover/ecu_bridge/ecu_bridge.hpp"

#define ACCEL_SIZE 3
#define ANG_VEL_SIZE 3
#define ORIENT_SIZE 4

namespace kpi_rover
{
    static constexpr uint8_t CMD_SET_UDP_PORT      = 0x06;
    static constexpr size_t IMU_DATA_MSG_LEN      = 1+2+(4+3+3)*sizeof(float);
    static constexpr int RECIEVE_DATA_TIMEOUT_MS     = 1000;

    struct IMUData {
        double orient[ORIENT_SIZE];
        double ang_vel[ANG_VEL_SIZE];
        double accel[ACCEL_SIZE];
    };

    
    
    class ECUBridgeIMU: public ECUBridge
    {
        public:
            /**
            * @brief Constructor.
            * @param transport Pointer to a Transport implementation.
            */
            explicit ECUBridgeIMU(std::unique_ptr<Transport> transport);
            
            /**
            * @brief Destructor.
            */
            ~ECUBridgeIMU() override;

            /**
            * @brief Retrieves orientation, angular velocity and acceleration.
            * @param data Pointer to IMU data structure to write retrieved value.
            * @param sync If true, the function waits for a response and give the value (or take latest on timeout).
            * @return true if successfuly read, false in case of error or timeout.
            */
            bool getIMUdata(IMUData& data, bool sync = false, int timeout_ms = 100);

            

        private:
            std::mutex imu_data_mutex_;
            std::thread worker_thread_;
            std::condition_variable newIMUData_cv_;
            struct IMUData latestIMUData_;
            bool IMUDataUpdated_ = false;  
            uint16_t last_msg_seq_num_ = -1;
            std::chrono::time_point<std::chrono::system_clock> last_msg_timestamp_;
            void processIMUData();
            
            template <typename ValueType>
            ValueType uint32FloatToValueType(uint32_t value);
    };

}
