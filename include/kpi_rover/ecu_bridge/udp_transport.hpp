#pragma once
#include "kpi_rover/ecu_bridge/transport.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>

namespace kpi_rover {

    class UDPTransport : public Transport {
        public:
            explicit UDPTransport(
                uint16_t port = 9999,
                const std::string& host = "0.0.0.0");
            ~UDPTransport() override;
            bool connect() override;
            bool send(const std::vector<uint8_t>& data) override;
            bool receive(std::vector<uint8_t>& data, size_t length, int timeout_ms) override;

        private:
            std::string host_;
            uint16_t port_;
            int sockfd_;
            sockaddr_in cli_addr_;
            socklen_t cli_addr_len_;
    };

}
