#include "kpi_rover/ecu_bridge/ecu_bridge.hpp"
#include <iostream>
#include <chrono>
#include <arpa/inet.h>
#include <cstring>
#include <functional>

namespace kpi_rover
{

    // Constructor implementation moved from header
    ECUBridge::ECUBridge(std::unique_ptr<Transport> transport)
        : transport_(std::move(transport))
        , stop_(false)
        , is_connected_(false)
    {
        worker_thread_ = std::thread(&ECUBridge::processQueue, this);
    }

    ECUBridge::~ECUBridge()
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        queue_cv_.notify_all();
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

    size_t ECUBridge::getExpectedResponseLength(uint8_t cmd_id)
    {
        switch (cmd_id)
        {
        case CMD_GET_API_VERSION:
            return RESP_LEN_GET_API_VERSION; // get_api_version response
        case CMD_SET_MOTOR_SPEED:
            return RESP_LEN_SET_MOTOR_SPEED; // set_motor_speed response
        case CMD_SET_ALL_MOTORS_SPEED:
            return RESP_LEN_SET_ALL_MOTORS_SPEED; // set_all_motors_speed response
        case CMD_GET_ENCODER:
            return RESP_LEN_GET_ENCODER; // get_encoder response: cmd + 4 bytes
        case CMD_GET_ALL_ENCODERS:
            return RESP_LEN_GET_ALL_ENCODERS; // get_all_encoders response: cmd + 4*4 bytes
        default:
            return 0;
        }
    }

    bool ECUBridge::sendCommand(const std::vector<uint8_t> &request, std::vector<uint8_t> &response, int timeout_ms)
    {
        std::promise<std::vector<uint8_t>> prom;
        auto fut = prom.get_future();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push({request, std::move(prom)});
        }
        queue_cv_.notify_one();
        // Wait for response or timeout
        if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready)
        {
            response = fut.get();
            return !response.empty(); // empty response indicates error
        }
        return false;
    }

    bool ECUBridge::tryConnect()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Attempting to connect...");
        if (!transport_->connect()) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Connection attempt failed");
            return false;
        }
        is_connected_ = true;
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Successfully connected");
        return true;
    }

    void ECUBridge::maintainConnection()
    {
        while (!is_connected_ && !stop_) {
            if (tryConnect()) {
                break;
            }
            RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), 
                "Connection failed, retrying in %d ms...", RECONNECTION_TIMEOUT_MS);
            std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECTION_TIMEOUT_MS));
        }
    }

    void ECUBridge::processQueue()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Starting queue processor");
        
        // First establish connection
        maintainConnection();

        while (true)
        {
            std::pair<std::vector<uint8_t>, std::promise<std::vector<uint8_t>>> item;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [&]
                               { return stop_ || !request_queue_.empty(); });
                if (stop_ && request_queue_.empty())
                    break;
                item = std::move(request_queue_.front());
                request_queue_.pop();
            }

            // Check/restore connection before processing
            if (!is_connected_) {
                RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Connection lost, attempting to reconnect...");
                maintainConnection();
                if (!is_connected_) {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to reconnect");
                    item.second.set_value({}); // Return empty response on connection failure
                    continue;
                }
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Successfully reconnected");
            }

            const auto &req = item.first;
            uint8_t cmd_id = req.empty() ? 0 : req[0];
            size_t expected_len = getExpectedResponseLength(cmd_id);
            if (!transport_->send(req))
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to send command 0x%02X", cmd_id);
                is_connected_ = false; // Mark as disconnected
                item.second.set_value({});
                continue;
            }
            std::vector<uint8_t> resp;
            // Use a fixed timeout for receiving response (e.g., 1000ms)
            if (!transport_->receive(resp, expected_len, 1000))
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to receive response for command 0x%02X", cmd_id);
                is_connected_ = false; // Mark as disconnected
                item.second.set_value({});
                continue;
            }
            item.second.set_value(resp);
        }
    }

    /**
     * @brief Enqueues a command request.
     *
     * @param request The command request as a vector of bytes.
     * @return std::future<std::vector<uint8_t>> Future resolving to the raw response.
     */
    std::future<std::vector<uint8_t>> ECUBridge::enqueueCommand(const std::vector<uint8_t> &request)
    {
        std::promise<std::vector<uint8_t>> prom;
        auto fut = prom.get_future();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push({request, std::move(prom)});
        }
        queue_cv_.notify_one();
        return fut;
    }

    uint8_t ECUBridge::asyncGetAPIVersionSync(uint8_t driver_version)
    {
        auto fut = this->enqueueCommand({CMD_GET_API_VERSION, driver_version});
        auto resp = fut.get();
        if (resp.size() >= RESP_LEN_GET_API_VERSION) {
            apiVersionCache_.update(resp[1]);
            return resp[1];
        }
        return 0;
    }

    int32_t ECUBridge::asyncGetEncoderSync(uint8_t motor_id)
    {
        auto fut = this->enqueueCommand({CMD_GET_ENCODER, motor_id});
        auto resp = fut.get();
        if (resp.size() >= RESP_LEN_GET_ENCODER) {
            int32_t encoder_net;
            memcpy(&encoder_net, &resp[1], 4);
            int32_t encoder_val = static_cast<int32_t>(ntohl(encoder_net));
            std::vector<int32_t> enc = encodersCache_.get();
            if (enc.size() < 4)
                enc.resize(4, 0);
            enc[motor_id] = encoder_val;
            encodersCache_.update(enc);
            return encoder_val;
        }
        return 0;
    }

    std::vector<int32_t> ECUBridge::asyncGetAllEncodersSync()
    {
        auto fut = this->enqueueCommand({CMD_GET_ALL_ENCODERS});
        auto resp = fut.get();
        std::vector<int32_t> enc;
        if (resp.size() >= RESP_LEN_GET_ALL_ENCODERS) {
            for (size_t i = 0; i < 4; i++) {
                int32_t val_net;
                memcpy(&val_net, &resp[1 + i * 4], 4);
                enc.push_back(static_cast<int32_t>(ntohl(val_net)));
            }
            encodersCache_.update(enc);
            return enc;
        }
        return std::vector<int32_t>(4, 0);
    }

    uint8_t ECUBridge::getAPIVersion(uint8_t driver_version, bool sync)
    {
        if (sync)
        {
            return asyncGetAPIVersionSync(driver_version);
        }
        else
        {
            uint8_t value = apiVersionCache_.get();
            (void) std::async(std::launch::async, [this, driver_version]()
            {
                asyncGetAPIVersionSync(driver_version);
            });
            return value;
        }
    }

    int32_t ECUBridge::getEncoder(uint8_t motor_id, bool sync)
    {
        if (sync)
        {
            return asyncGetEncoderSync(motor_id);
        }
        else
        {
            std::vector<int32_t> enc = encodersCache_.get();
            int32_t value = (enc.size() > motor_id) ? enc[motor_id] : 0;
            (void) std::async(std::launch::async, [this, motor_id]()
            {
                asyncGetEncoderSync(motor_id);
            });
            return value;
        }
    }

    std::vector<int32_t> ECUBridge::getAllEncoders(bool sync)
    {
        if (sync)
        {
            return asyncGetAllEncodersSync();
        }
        else
        {
            std::vector<int32_t> values = encodersCache_.get();
            if(values.empty() || values.size() < 4)
                values = std::vector<int32_t>(4, 0);
            (void) std::async(std::launch::async, [this]()
            {
                asyncGetAllEncodersSync();
            });
            return values;
        }
    }

    uint8_t ECUBridge::setMotorSpeed(uint8_t motor_id, int32_t speed)
    {
        std::vector<uint8_t> vec = motorSpeedResultCache_.get();
        if(vec.size() < 4)
            vec.resize(4, 1); // default error code 1
        uint8_t cached_result = vec[motor_id];
        // Trigger asynchronous update.
        (void) std::async(std::launch::async, [this, motor_id, speed]()
        {
            std::vector<uint8_t> req;
            req.push_back(CMD_SET_MOTOR_SPEED);
            req.push_back(motor_id);
            int32_t speed_net = htonl(speed);
            uint8_t* p = reinterpret_cast<uint8_t*>(&speed_net);
            req.insert(req.end(), p, p + 4);
            auto fut_raw = this->enqueueCommand(req);
            auto resp = fut_raw.get();
            uint8_t result = (!resp.empty() && resp[0] == CMD_SET_MOTOR_SPEED) ? 0 : 1;
            std::vector<uint8_t> vec = motorSpeedResultCache_.get();
            if(vec.size() < 4)
                vec.resize(4, 1);
            vec[motor_id] = result;
            motorSpeedResultCache_.update(vec);
        });
        return cached_result;
    }

    uint8_t ECUBridge::setAllMotorsSpeed(int32_t speed1, int32_t speed2, int32_t speed3, int32_t speed4)
    {
        uint8_t cached_result = allMotorsSpeedResultCache_.get();
        // Trigger asynchronous update.
        (void) std::async(std::launch::async, [this, speed1, speed2, speed3, speed4]()
        {
            std::vector<uint8_t> req;
            req.push_back(CMD_SET_ALL_MOTORS_SPEED);
            int32_t speeds[4] = {speed1, speed2, speed3, speed4};
            for (int i = 0; i < 4; i++)
            {
                int32_t net = htonl(speeds[i]);
                uint8_t* p = reinterpret_cast<uint8_t*>(&net);
                req.insert(req.end(), p, p + 4);
            }
            auto fut_raw = this->enqueueCommand(req);
            auto resp = fut_raw.get();
            uint8_t result = (!resp.empty() && resp[0] == CMD_SET_ALL_MOTORS_SPEED) ? 0 : 1;
            allMotorsSpeedResultCache_.update(result);
        });
        return cached_result;
    }

    void ECUBridge::updateCash(uint8_t driver_version)
    {
        getAPIVersion(driver_version, true);
        getAllEncoders(true);
    }

} // namespace kpi_rover
