#include "kpi_rover/ecu_bridge/udp_transport.hpp"
#include <unistd.h>
#include <sys/select.h>
#include <cstring>

namespace kpi_rover
{

    UDPTransport::UDPTransport(uint16_t port, const std::string& host)
        : port_(port)
        , host_(host)
        , sockfd_(-1)
    {
        memset(&cli_addr_, 0, sizeof(cli_addr_));
    }

    

    UDPTransport::~UDPTransport()
    {
        if (sockfd_ != -1)
            close(sockfd_);
    }

    bool UDPTransport::connect()
    {
        if (sockfd_!=-1) {
            int optval;
            socklen_t optlen = sizeof(optval);

            if (getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &optval, &optlen) == 0 && optval == 0) {
                return true;
            }

            close(sockfd_);
        }

        sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0)
            return false;

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port_);
        if (::inet_pton(AF_INET, host_.c_str(), &serv_addr.sin_addr) <= 0)
            return false;
            
        return ::bind(sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0;
    }

    bool UDPTransport::send(const std::vector<uint8_t> &data)
    {
        if (cli_addr_.sin_port == 0) // Check if client address and port was received
            return false;

        ssize_t sent = ::sendto(sockfd_, data.data(), data.size(), MSG_CONFIRM, (const struct sockaddr *) &cli_addr_, cli_addr_len_);
        cli_addr_.sin_port = 0;
        if (sent <= 0)
            return false;
        return true;
    }

    bool UDPTransport::receive(std::vector<uint8_t> &data, size_t length, int timeout_ms)
    {
        data.clear();
        data.resize(length);
        timeval read_timeout{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
        ::setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
        ssize_t received = ::recvfrom(sockfd_, data.data(), length, MSG_WAITALL, ( struct sockaddr *) &cli_addr_, &cli_addr_len_);
        if (received <= 0)
            return false;
        return true;
    }

} // namespace kpi_rover
