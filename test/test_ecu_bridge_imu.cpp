#include "kpi_rover/ecu_bridge/ecu_bridge_imu.hpp"
#include "kpi_rover/ecu_bridge/udp_transport.hpp"
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <unistd.h>

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
    if(argc < 1 || argc > 2)
    {
        std::cerr << "Usage: " << argv[0] << " <port> [address]" << argc << std::endl;
        return EXIT_FAILURE;
    }

    
    uint16_t port = static_cast<uint16_t>(std::atoi(argv[1]));
    
    // Create a UDPTransport with configuration
    auto transport = std::make_unique<kpi_rover::UDPTransport>(
        port
    );
    
    // Create ECUBridge instance
    kpi_rover::ECUBridgeIMU ecu_bridge(std::move(transport));

    struct kpi_rover::IMUData data;

    // Wait UDP server to start
    sleep(1); 

    // Test getIMUdata with default (non-sync)
    double elapsed = measure_time([&]() {
        ecu_bridge.getIMUdata(data);
    });
    printf("IMU Data async:\n\t \
            Orientation[units]:\t\tx=%.3f, y=%.3f, z=%.3f, w=%.3f\n\t \
            Angular velocity[rad/s]:\tx=%.3f, y=%.3f, z=%.3f\n\t \
            Linear acceleration[m/s^2]:\tx=%.3f, y=%.3f, z=%.3f\n",
                data.orient[0], data.orient[1], data.orient[2], data.orient[3],
                data.ang_vel[0], data.ang_vel[1], data.ang_vel[2],
                data.accel[0], data.accel[1], data.accel[2]); 
    std::cout << "getIMUdata async took: " << elapsed << " ms" << std::endl << std::endl;

    // Test getIMUdata with sync
    elapsed = measure_time([&]() {
        ecu_bridge.getIMUdata(data, true, 1000);
    });
    printf("IMU Data sync:\n\t \
        Orientation[units]:\t\tx=%.3f, y=%.3f, z=%.3f, w=%.3f\n\t \
        Angular velocity[rad/s]:\tx=%.3f, y=%.3f, z=%.3f\n\t \
        Linear acceleration[m/s^2]:\tx=%.3f, y=%.3f, z=%.3f\n",
            data.orient[0], data.orient[1], data.orient[2], data.orient[3],
            data.ang_vel[0], data.ang_vel[1], data.ang_vel[2],
            data.accel[0], data.accel[1], data.accel[2]); 
    std::cout << "getIMUdata sync took: " << elapsed << " ms" << std::endl << std::endl;

    
    std::cout << "All tests completed." << std::endl;
    return EXIT_SUCCESS;
}
