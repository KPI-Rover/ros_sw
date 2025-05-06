#include "kpi_rover/ecu_bridge/ecu_bridge_motors.hpp"
#include "kpi_rover/ecu_bridge/tcp_transport.hpp"
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>

// Helper function template to run an operation and return elapsed time in ms
template <typename Func>
double measure_time(Func f)
{
    auto start = std::chrono::steady_clock::now();
    f();
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

int main(int argc, char* argv[])
{
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <server_ip> <port>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string server_ip = argv[1];
    uint16_t port = static_cast<uint16_t>(std::atoi(argv[2]));

    // Create a TCPTransport with configuration
    auto transport = std::make_unique<kpi_rover::TCPTransport>(
        server_ip,  // host
        port,       // port
        1000        // reconnection interval
    );
    
    // Create ECUBridge instance
    kpi_rover::ECUBridgeMotors ecu_bridge(std::move(transport));
    
    // Measure time for updateCash(1)
    double elapsed = measure_time([&]() {
        ecu_bridge.updateCash(1);
    });
    std::cout << "updateCash took: " << elapsed << " ms" << std::endl << std::endl;

    // Test getAPIVersion with default (non-sync)
    elapsed = measure_time([&]() {
        uint8_t api = ecu_bridge.getAPIVersion(1);
        std::cout << "getAPIVersion: " << static_cast<int>(api) << std::endl;
    });
    std::cout << "getAPIVersion took: " << elapsed << " ms" << std::endl << std::endl;

    // Test setMotorSpeed
    elapsed = measure_time([&]() {
        uint8_t res = ecu_bridge.setMotorSpeed(0, 1000);
        std::cout << "setMotorSpeed (motor 0) result: " << static_cast<int>(res) << std::endl;
    });
    std::cout << "setMotorSpeed took: " << elapsed << " ms" << std::endl << std::endl;

    // Test setAllMotorsSpeed
    elapsed = measure_time([&]() {
        uint8_t res = ecu_bridge.setAllMotorsSpeed(1000, 1100, 1200, 1300);
        std::cout << "setAllMotorsSpeed result: " << static_cast<int>(res) << std::endl;
    });
    std::cout << "setAllMotorsSpeed took: " << elapsed << " ms" << std::endl << std::endl;

    // Test getEncoder
    elapsed = measure_time([&]() {
        int32_t enc = ecu_bridge.getEncoder(0);
        std::cout << "getEncoder (motor 0): " << enc << std::endl;
    });
    std::cout << "getEncoder took: " << elapsed << " ms" << std::endl << std::endl;

    // Test getAllEncoders
    elapsed = measure_time([&]() {
        auto encoders = ecu_bridge.getAllEncoders();
        std::cout << "getAllEncoders:";
        for(auto e : encoders)
            std::cout << " " << e;
        std::cout << std::endl;
    });
    std::cout << "getAllEncoders took: " << elapsed << " ms" << std::endl << std::endl;

    std::cout << "All tests completed." << std::endl;
    return EXIT_SUCCESS;
}
