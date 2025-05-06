#pragma once
#include "kpi_rover/ecu_bridge/transport.hpp"

namespace kpi_rover {

static constexpr int DEFAULT_RECONNECT_INTERVAL_MS = 1000;

class TCPTransport : public Transport {
public:
    explicit TCPTransport(
        const std::string& host = "127.0.0.1",
        uint16_t connection_port = 6000,
        uint16_t bind_port = 9999,
        int reconnect_interval_ms = DEFAULT_RECONNECT_INTERVAL_MS);
    ~TCPTransport() override;
    bool connect() override;
    bool send(const std::vector<uint8_t>& data) override;
    bool receive(std::vector<uint8_t>& data, size_t length, int timeout_ms) override;

private:
    std::string host_;
    uint16_t connection_port_;
    uint16_t bind_port_;
    int reconnect_interval_ms_;
    int sockfd_;
};

} // namespace kpi_rover
