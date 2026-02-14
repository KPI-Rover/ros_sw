#include "kpi_rover/ecu_bridge/ecu_bridge.hpp"
#include <cstring>
#include <algorithm>
#include <endian.h>
#include <sstream>
#include <iomanip>

namespace kpi_rover
{
    namespace {
        std::string to_hex(const uint8_t* data, size_t len) {
            std::stringstream ss;
            ss << std::hex << std::setfill('0');
            for (size_t i = 0; i < len; ++i) {
                ss << std::setw(2) << static_cast<int>(data[i]) << (i < len - 1 ? " " : "");
            }
            return ss.str();
        }
        
        std::string to_hex(const std::vector<uint8_t>& data) {
            return to_hex(data.data(), data.size());
        }
    }

    ECUBridge::ECUBridge(const std::string &device, int baud_rate)
        : transport_(device, baud_rate), logger_name_("ECUBridge")
    {
        // Initialize motor setpoints with zeros
        std::vector<int32_t> zero_setpoints(4, 0);
        motor_setpoints_cache_.update(zero_setpoints);
    }

    ECUBridge::~ECUBridge()
    {
        stop();
    }

    bool ECUBridge::start()
    {
        if (!open()) return false;

        manager_thread_ = std::thread(&ECUBridge::managerLoop, this);
        return true;
    }

    bool ECUBridge::open()
    {
        if (!transport_.open())
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "Failed to open serial transport");
            return false;
        }

        transport_.flush();
        running_ = true;
        return true;
    }

    void ECUBridge::stop()
    {
        running_ = false;
        transport_.close();
        if (manager_thread_.joinable())
        {
            manager_thread_.join();
        }
    }

    void ECUBridge::setMotorSpeeds(const int32_t speeds[4])
    {
        std::vector<int32_t> s(speeds, speeds + 4);
        motor_setpoints_cache_.update(s);
    }

    bool ECUBridge::isConnected() const
    {
        return is_connected_;
    }

    void ECUBridge::managerLoop()
    {
        RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Manager loop started");
        
        // Brief delay to allow serial port stabilization
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto last_imu_poll = std::chrono::steady_clock::now();
        auto last_encoder_poll = std::chrono::steady_clock::now();
        auto last_motor_push = std::chrono::steady_clock::now();
        auto last_heartbeat = std::chrono::steady_clock::now();
        last_success_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);

        // Initial sync
        if (syncAPIVersion())
        {
            last_success_time_ = std::chrono::steady_clock::now();
        }

        while (running_)
        {
            auto now = std::chrono::steady_clock::now();

            if (now - last_heartbeat >= std::chrono::seconds(5))
            {
                RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Manager loop heartbeat [is_connected=%s]", is_connected_ ? "true" : "false");
                RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Request success rate: %u/%u", request_counter_success_, request_counter_total_);
                last_heartbeat = now;
            }

            // Update is_connected_ based on time since last global success
            is_connected_ = (now - last_success_time_) < std::chrono::milliseconds(500);

            // IMU Polling
            if (now - last_imu_poll >= imu_period_.load())
            {
                last_imu_poll = now;
                if (pollIMU())
                {
                    last_success_time_ = std::chrono::steady_clock::now();
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(logger_name_), "IMU poll failed");
                }
            }

            // Encoder Polling
            if (now - last_encoder_poll >= encoder_period_.load())
            {
                last_encoder_poll = now;
                if (pollEncoders())
                {
                    last_success_time_ = std::chrono::steady_clock::now();
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Encoder poll failed");
                }
            }

            // Motor Pushing
            if (now - last_motor_push >= motor_period_.load())
            {
                last_motor_push = now;
                if (pushMotors())
                {
                    last_success_time_ = std::chrono::steady_clock::now();
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Motor push failed");
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Manager loop exiting");
    }

    bool ECUBridge::syncAPIVersion()
    {
        ECUProtocol::Message req;
        req.command_id = ECUProtocol::CommandId::GET_API_VERSION; // CMD_GET_API_VERSION
        req.payload = {0x01}; // Driver version 1

        ECUProtocol::Message resp;
        if (sendRequest(req, resp))
        {
            if (resp.payload.size() >= 1)
            {
                api_version_cache_.update(resp.payload[0]);
                RCLCPP_INFO(rclcpp::get_logger(logger_name_), "API Version: %d", resp.payload[0]);
                return true;
            }
        }
        return false;
    }

    bool ECUBridge::pollIMU()
    {
        ECUProtocol::Message req;
        req.command_id = ECUProtocol::CommandId::GET_IMU; // CMD_GET_IMU
        
        ECUProtocol::Message resp;
        if (sendRequest(req, resp))
        {
            if (resp.payload.size() >= 44) // 11 floats * 4 bytes
            {
                IMUData data;
                const uint8_t* p = resp.payload.data();
                // Assumes Little Endian as per srs.md and protocol.md
                std::memcpy(data.accel, p, 12); p += 12;
                std::memcpy(data.gyro, p, 12); p += 12;
                std::memcpy(data.mag, p, 12); p += 12;
                std::memcpy(data.quat, p, 16);
                
                imu_cache_.update(data);
                return true;
            }
        }
        return false;
    }

    bool ECUBridge::pollEncoders()
    {
        ECUProtocol::Message req;
        req.command_id = ECUProtocol::CommandId::GET_ALL_ENCODERS; // CMD_GET_ALL_ENCODERS
        
        ECUProtocol::Message resp;
        if (sendRequest(req, resp))
        {
            if (resp.payload.size() >= 16) // 4 int32 * 4 bytes
            {
                EncoderData data;
                for (int i = 0; i < 4; ++i)
                {
                    int32_t val;
                    std::memcpy(&val, resp.payload.data() + i * 4, 4);
                    // Match server logic (which uses htonl/ntohl for ints, so it's Big Endian on wire)
                    data.values[i] = be32toh(val);
                }
                encoder_cache_.update(data);
                return true;
            }
        }
        return false;
    }

    bool ECUBridge::pushMotors()
    {
        auto setpoints = motor_setpoints_cache_.get();
        ECUProtocol::Message req;
        req.command_id = ECUProtocol::CommandId::SET_ALL_MOTORS_SPEED; // CMD_SET_ALL_MOTORS_SPEED
        
        for (int i = 0; i < 4; ++i)
        {
            // Match server logic (Big Endian for ints on wire)
            uint32_t val = htobe32(setpoints[i]);
            uint8_t bytes[4];
            std::memcpy(bytes, &val, 4);
            req.payload.insert(req.payload.end(), bytes, bytes + 4);
        }

        ECUProtocol::Message resp;
        if (sendRequest(req, resp))
        {
            return (resp.payload.size() >= 1 && resp.payload[0] == 0);
        }
        return false;
    }

    bool ECUBridge::sendRequest(const ECUProtocol::Message &req, ECUProtocol::Message &resp, int timeout_ms)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        
        transport_.flush();
        
        request_counter_total_ ++;

        auto frame = ECUProtocol::encode(req);
        RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "[TX] %s", to_hex(frame).c_str());

        if (!transport_.write(frame)) 
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "Serial write failed");
            return false;
        }

        auto start_time = std::chrono::steady_clock::now();
        while (running_)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            
            int remaining = timeout_ms - static_cast<int>(elapsed);

            if (remaining <= 0) {
                RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Request timeout");
                break;
            }

            std::vector<uint8_t> resp_frame;
            if (readFrame(resp_frame, remaining))
            {
                auto msg = ECUProtocol::decode(resp_frame);
                if (msg)
                {
                    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "[RX] %s", to_hex(resp_frame).c_str());
                    if (msg->command_id == req.command_id)
                    {
                        resp = *msg;
                        request_counter_success_ ++;
                        return true;
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Stale response discarded: CMD %02x (expected %02x)", 
                                    static_cast<uint8_t>(msg->command_id), static_cast<uint8_t>(req.command_id));
                        // Continue reading
                    }
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Protocol decode failed for RX frame");
                }
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Frame read timeout");
                break;
            }
        }
        return false;
    }

    bool ECUBridge::readFrame(std::vector<uint8_t> &frame, int timeout_ms)
    {
        uint8_t b;
        auto start_search = std::chrono::steady_clock::now();
        
        // Find Start Byte
        bool found_start = false;
        std::vector<uint8_t> skipped;
        while (running_)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_search).count();
            int remaining = timeout_ms - static_cast<int>(elapsed);
            
            if (remaining <= 0) break;

            if (transport_.read(&b, 1, remaining) == 1) 
            {
                if (b == ECUProtocol::START_BYTE) 
                {
                    found_start = true;
                    break;
                }
                skipped.push_back(b);
            }
            else
            {
                break;
            }
        }

        if (!skipped.empty()) {
            RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "[RX Skipped] %s", to_hex(skipped).c_str());
        }

        if (!running_ || !found_start){
             return false;
        }

        frame.clear();
        frame.push_back(ECUProtocol::START_BYTE);

        // Read Length Byte
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_search).count();
        int remaining = timeout_ms - static_cast<int>(elapsed);

        if (remaining <= 0 || transport_.read(&b, 1, remaining) != 1) {
            if (running_) {
                RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Timeout/Error reading length byte. Frame: %s", to_hex(frame).c_str());
            }
            return false;
        }
        
        frame.push_back(b);
        uint8_t len = b;
        if (len < ECUProtocol::MIN_FRAME_SIZE || len > 64) {
            RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Invalid/Garbage length byte: %d. Frame: %s", len, to_hex(frame).c_str());
            return false;
        }

        size_t to_read = len - 1;
        std::vector<uint8_t> buffer(to_read);
        size_t read_so_far = 0;
        
        while (read_so_far < to_read && running_)
        {
            now = std::chrono::steady_clock::now();
            elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_search).count();
            remaining = timeout_ms - static_cast<int>(elapsed);

            if (remaining <= 0) break;

            size_t n = transport_.read(buffer.data() + read_so_far, to_read - read_so_far, remaining);
            if (n > 0)
            {
                read_so_far += n;
                if (read_so_far < to_read) 
                {
                    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Reading frame body... %zu/%zu bytes", read_so_far, to_read);
                }
            }
            else
            {
                break;
            }
        }

        if (!running_) {
            return false;
        } 
        
        if (read_so_far < to_read) {
             frame.insert(frame.end(), buffer.begin(), buffer.begin() + read_so_far);
             RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Timeout (%d ms) reading frame body. Read %zu/%zu bytes. Total so far: %s", 
                         timeout_ms, read_so_far, to_read, to_hex(frame).c_str());
             return false;
        }

        frame.insert(frame.end(), buffer.begin(), buffer.end());
        return true;
    }

    bool ECUBridge::sendRawRequest(uint8_t cmd_id, const std::vector<uint8_t>& payload, ECUProtocol::Message& resp)
    {
        ECUProtocol::Message req;
        req.command_id = static_cast<ECUProtocol::CommandId>(cmd_id);
        req.payload = payload;
        return sendRequest(req, resp);
    }
} // namespace kpi_rover
