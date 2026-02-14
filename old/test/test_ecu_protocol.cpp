#include "kpi_rover/ecu_bridge/ecu_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <chrono>
#include <functional>
#include <thread>

using namespace kpi_rover;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_ecu_protocol");

    node->declare_parameter("serial_device", "/dev/ttyAMA2");
    node->declare_parameter("baud_rate", 115200);
    node->declare_parameter("load_test_iterations", 10000);

    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        std::cout << "Usage: ros2 run kpi_rover test_ecu_protocol --ros-args "
                  << "[-p serial_device:=/dev/ttyAMA2] "
                  << "[-p baud_rate:=115200] "
                  << "[-p load_test_iterations:=1000]" << std::endl;
        return 0;
    }

    std::string device = node->get_parameter("serial_device").as_string();
    int baud = node->get_parameter("baud_rate").as_int();
    int number_of_iterations = node->get_parameter("load_test_iterations").as_int();

    // Use ECUBridge directly - no code duplication!
    ECUBridge bridge(device, baud);
    
    // Open manually (do not start the automatic manager thread)
    if (!bridge.open()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open serial device %s", device.c_str());
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "=== ECU Protocol Stress Test (Using ECUBridge) ===");
    RCLCPP_INFO(node->get_logger(), "Device: %s @ %d baud", device.c_str(), baud);

    struct TestCmd {
        uint8_t id;
        std::string name;
        std::function<bool()> func;
    };

    std::vector<TestCmd> commands = {
        {0x01, "GET_API_VER", [&](){ return bridge.syncAPIVersion(); }},
        {0x03, "SET_MOTORS",  [&](){ return bridge.pushMotors(); }},
        {0x05, "GET_ENCODERS", [&](){ return bridge.pollEncoders(); }},
        {0x06, "GET_IMU",     [&](){ return bridge.pollIMU(); }}
    };

    RCLCPP_INFO(node->get_logger(), "\n--- Phase 1: Command Verification (10 iterations each) ---");
    for (const auto& cmd : commands) {
        int success = 0;
        for (int i = 0; i < 10; ++i) {
            ECUProtocol::Message resp; // Dummy
            if (cmd.func()) {
                success++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(node->get_logger(), "  [0x%02x] %-12s: %d/10 success", cmd.id, cmd.name.c_str(), success);
    }

    RCLCPP_INFO(node->get_logger(), "\n--- Phase 2: Load Test (%d iterations) ---", number_of_iterations);
    int success_counter = 0;
    
    double min_latency = 1e9;
    double max_latency = 0;
    double sum_latency = 0;

    auto start_time = std::chrono::steady_clock::now();
    
    for (int i = 0; i < number_of_iterations; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        bool success = bridge.pollIMU();
        auto t1 = std::chrono::steady_clock::now();

        if (success) {
            success_counter++;
            double latency = std::chrono::duration<double, std::milli>(t1 - t0).count();
            if (latency < min_latency) min_latency = latency;
            if (latency > max_latency) max_latency = latency;
            sum_latency += latency;
        }

        if ((i + 1) % (number_of_iterations / 10) == 0) {
            RCLCPP_INFO(node->get_logger(), "  Progress: %d%%...", ((i + 1) * 100) / number_of_iterations);
        }
        if (!rclcpp::ok()) break;
    }
    
    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    RCLCPP_INFO(node->get_logger(), "\n=== Summary ===");
    RCLCPP_INFO(node->get_logger(), "Success Rate: %d/%d (%d%%)", success_counter, number_of_iterations, (success_counter * 100) / number_of_iterations);
    
    if (success_counter > 0) {
        RCLCPP_INFO(node->get_logger(), "Response Time:    Min=%.2f ms, Max=%.2f ms, Avg=%.2f ms", 
                    min_latency, max_latency, sum_latency / success_counter);
    }
    
    RCLCPP_INFO(node->get_logger(), "Test Duration:    %.2f seconds", duration);
    RCLCPP_INFO(node->get_logger(), "Avg Frequency:    %.2f Hz", number_of_iterations / duration);

    bridge.stop(); // Safe shutdown
    rclcpp::shutdown();
    return 0;
}
