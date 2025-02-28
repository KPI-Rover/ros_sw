#include "kpi_rover/ecu_bridge/tcp_transport.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>

namespace kpi_rover
{

    TCPTransport::TCPTransport() : sockfd(-1)
    {
        // ...existing code...
    }

    TCPTransport::~TCPTransport()
    {
        if (sockfd != -1)
            close(sockfd);
    }

    bool TCPTransport::connect(const std::string &host, uint16_t port)
    {
        sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
            return false;
        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &serv_addr.sin_addr) <= 0)
            return false;
        if (::connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
            return false;
        return true;
    }

    bool TCPTransport::send(const std::vector<uint8_t> &data)
    {
        ssize_t total = 0;
        while (total < (ssize_t)data.size())
        {
            ssize_t sent = ::write(sockfd, data.data() + total, data.size() - total);
            if (sent <= 0)
                return false;
            total += sent;
        }
        return true;
    }

    bool TCPTransport::receive(std::vector<uint8_t> &data, size_t length, int timeout_ms)
    {
        data.clear();
        data.resize(length);
        size_t received = 0;
        while (received < length)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sockfd, &readfds);
            timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
            int ret = select(sockfd + 1, &readfds, nullptr, nullptr, &tv);
            if (ret <= 0)
                return false; // timeout or error
            ssize_t r = ::read(sockfd, data.data() + received, length - received);
            if (r <= 0)
                return false;
            received += r;
        }
        return true;
    }

} // namespace kpi_rover
