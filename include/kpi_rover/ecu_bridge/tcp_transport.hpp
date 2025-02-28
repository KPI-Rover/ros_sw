#pragma once
#include "kpi_rover/ecu_bridge/transport.hpp"

namespace kpi_rover
{

    class TCPTransport : public Transport
    {
    public:
        TCPTransport();
        virtual ~TCPTransport();
        bool connect(const std::string &host, uint16_t port) override;
        bool send(const std::vector<uint8_t> &data) override;
        bool receive(std::vector<uint8_t> &data, size_t length, int timeout_ms) override;

    private:
        int sockfd;
    };

} // namespace kpi_rover
