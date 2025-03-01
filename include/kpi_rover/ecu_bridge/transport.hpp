#pragma once
#include <cstdint>
#include <vector>
#include <string>

namespace kpi_rover
{
    class Transport
    {
    public:
        virtual ~Transport() {}
        virtual bool connect() = 0;
        virtual bool send(const std::vector<uint8_t> &data) = 0;
        virtual bool receive(std::vector<uint8_t> &data, size_t length, int timeout_ms) = 0;
    };
} // namespace kpi_rover
