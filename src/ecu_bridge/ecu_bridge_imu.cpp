#include "kpi_rover/ecu_bridge/ecu_bridge_imu.hpp"
#include <iostream>
#include <chrono>
#include <arpa/inet.h>
#include <cstring>
#include <functional>
#include <limits>
#include <type_traits>

namespace kpi_rover
{

    ECUBridgeIMU::ECUBridgeIMU(std::unique_ptr<Transport> transport): ECUBridge(std::move(transport))
    {
        memset(&latestIMUData_, 0, sizeof(latestIMUData_));
        worker_thread_ = std::thread(&ECUBridgeIMU::processIMUData, this);
        LOGGER_NAME = "ECUBridgeIMU";
    }

    ECUBridgeIMU::~ECUBridgeIMU()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "In destructor");
        stop_ = true; //signal worker thread to stop processing

        if (worker_thread_.joinable())
            worker_thread_.join();
    }

    template <typename ValueType>
    ValueType ECUBridgeIMU::uint32FloatToValueType(uint32_t value)
    {
        uint32_t val_host;
        float val_host_f;
        val_host = ntohl(value); // works only for network transport, TODO: intergrate other transport (eg UART)
        memcpy(&val_host_f, &val_host, 4);
        return static_cast<ValueType>(val_host_f);
    }

    void ECUBridgeIMU::processIMUData()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Start listeting to new data from IMU");
        std::vector<uint8_t> data;
        while (!stop_)
        {
            maintainConnection(); // Check connection
            bool status = transport_->receive(data, IMU_DATA_MSG_LEN, RECIEVE_DATA_TIMEOUT_MS);
            if (status)
            {
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Recieved a message");
                auto new_msg_timestamp = std::chrono::system_clock::now();

                decltype(last_msg_seq_num_) new_msg_seq_num;
                memcpy(&new_msg_seq_num, &data[1], sizeof(new_msg_seq_num));

                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Command %u", data[0]);
                
                new_msg_seq_num = static_cast<decltype(last_msg_seq_num_)>(ntohs(new_msg_seq_num));
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Packet num %u", new_msg_seq_num);

                
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Lost messages: %d", 
                            (std::numeric_limits<decltype(last_msg_seq_num_)>::max() + new_msg_seq_num - last_msg_seq_num_) % (std::numeric_limits<decltype(last_msg_seq_num_)>::max() + 1));
                
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Time since previous msg: %lu ns", (new_msg_timestamp - last_msg_timestamp_)/1000);
    

                last_msg_timestamp_ =  new_msg_timestamp;
                last_msg_seq_num_ = new_msg_seq_num;

                std::lock_guard<std::mutex> imu_data_lock(imu_data_mutex_);


                //Unpack message to IMUData
                uint32_t val_net; //client sends float 
                
                for (size_t i = 0; i < ACCEL_SIZE; i++) {
                    memcpy(&val_net, &data[1 + sizeof(last_msg_seq_num_) + i * sizeof(val_net)], sizeof(val_net));
                    latestIMUData_.accel[i] = uint32FloatToValueType<std::remove_reference_t<decltype(latestIMUData_.accel[0])>>(val_net);
                }           
                
                for (size_t i = 0; i < ANG_VEL_SIZE; i++) {
                    memcpy(&val_net, &data[1 + sizeof(last_msg_seq_num_) + sizeof(val_net)*(ACCEL_SIZE + i)], sizeof(val_net));
                    latestIMUData_.ang_vel[i] = uint32FloatToValueType<std::remove_reference_t<decltype(latestIMUData_.ang_vel[0])>>(val_net);
                }
                              
                    
                for (size_t i = 0; i < ORIENT_SIZE; i++) {
                    memcpy(&val_net, &data[1 + sizeof(last_msg_seq_num_) + sizeof(val_net)*(ACCEL_SIZE + ANG_VEL_SIZE + i)], sizeof(val_net));
                    latestIMUData_.orient[i] = uint32FloatToValueType<std::remove_reference_t<decltype(latestIMUData_.orient[0])>>(val_net);
                }
                
                IMUDataUpdated_ = true;
                newIMUData_cv_.notify_one();
            }
            else
            {
                is_connected_ = false;
                RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Failed to recieve a message");
            }
        }
    }

    bool ECUBridgeIMU::getIMUdata(IMUData& data, bool sync, int timeout_ms)
    {
        std::unique_lock<std::mutex> lck(imu_data_mutex_);
        if (sync){
            newIMUData_cv_.wait_for(lck, std::chrono::milliseconds(timeout_ms), [this] {
                return IMUDataUpdated_;
            });
            
            IMUDataUpdated_ = false;
        }
        
        memcpy(&data, &latestIMUData_, sizeof(IMUData));
        return true;
    }

}
