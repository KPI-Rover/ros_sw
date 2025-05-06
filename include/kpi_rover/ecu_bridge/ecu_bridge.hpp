#include "kpi_rover/ecu_bridge/transport.hpp"
#include "rclcpp/logging.hpp"
#include <thread>
#include <chrono>

namespace kpi_rover
{
    static constexpr int RECONNECTION_TIMEOUT_MS = 1000;
    class ECUBridge
    {
        public:
            /**
             * @brief Constructor.
             * @param transport Pointer to a Transport implementation.
             */
            explicit ECUBridge(std::unique_ptr<Transport> transport)
            : transport_(std::move(transport))
            , stop_(false)
            , is_connected_(false)
            {};

            /**
             * @brief Destructor.
             */
            virtual ~ECUBridge() {};

        protected:
            std::unique_ptr<Transport> transport_;
            std::string LOGGER_NAME;
            std::atomic<bool> stop_;
            std::atomic<bool> is_connected_;

            bool tryConnect()
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

            void maintainConnection()
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
    };
}
