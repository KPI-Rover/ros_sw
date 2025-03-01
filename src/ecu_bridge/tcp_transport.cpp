#include "kpi_rover/ecu_bridge/tcp_transport.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>

namespace kpi_rover
{

    TCPTransport::TCPTransport(const std::string& host, uint16_t port, int reconnect_interval_ms)
        : host_(host)
        , port_(port)
        , reconnect_interval_ms_(reconnect_interval_ms)
        , sockfd_(-1)
    {
    }

    TCPTransport::~TCPTransport()
    {
        if (sockfd_ != -1)
            close(sockfd_);
    }

    bool TCPTransport::connect()
    {
        sockfd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0)
            return false;

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port_);
        if (::inet_pton(AF_INET, host_.c_str(), &serv_addr.sin_addr) <= 0)
            return false;
            
        return ::connect(sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0;
    }

    bool TCPTransport::send(const std::vector<uint8_t> &data)
    {
        ssize_t total = 0;
        while (total < (ssize_t)data.size())
        {
            ssize_t sent = ::write(sockfd_, data.data() + total, data.size() - total);
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
            FD_SET(sockfd_, &readfds);
            timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
            int ret = select(sockfd_ + 1, &readfds, nullptr, nullptr, &tv);
            if (ret <= 0)
                return false; // timeout or error
            ssize_t r = ::read(sockfd_, data.data() + received, length - received);
            if (r <= 0)
                return false;
            received += r;
        }
        return true;
    }

} // namespace kpi_rover
