#include "kpi_rover/ecu_bridge/ecu_protocol.hpp"
#include <cstring>

namespace kpi_rover
{
    std::vector<uint8_t> ECUProtocol::encode(const Message &msg)
    {
        // Payload consists of command_id + payload data
        size_t payload_size = 1 + msg.payload.size();
        // Frame: [AA] [Length] [CommandID] [Data...] [CRC_L] [CRC_H]
        // Length = LengthByte(1) + Payload(N) + CRC(2) = N + 3
        uint8_t frame_len = static_cast<uint8_t>(payload_size + 3);

        std::vector<uint8_t> frame;
        frame.reserve(1 + frame_len);
        
        frame.push_back(START_BYTE);
        frame.push_back(frame_len);
        frame.push_back(static_cast<uint8_t>(msg.command_id));
        frame.insert(frame.end(), msg.payload.begin(), msg.payload.end());

        // CRC is calculated over Length + Payload (starts at index 1)
        uint16_t crc = calculateCRC(frame.data() + 1, frame.size() - 1);
        frame.push_back(static_cast<uint8_t>(crc & 0xFF));        // Low byte
        frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // High byte

        return frame;
    }

    std::optional<ECUProtocol::Message> ECUProtocol::decode(const std::vector<uint8_t> &frame)
    {
        if (frame.size() < MIN_FRAME_SIZE) return std::nullopt; // Min size: AA, Len, Cmd, CRC_L, CRC_H
        if (frame[0] != START_BYTE) return std::nullopt;

        uint8_t frame_len = frame[1];
        if (frame.size() != (size_t)(frame_len + 1)) return std::nullopt;

        // Verify CRC
        uint16_t received_crc = frame[frame.size() - 2] | (frame[frame.size() - 1] << 8);
        uint16_t calculated_crc = calculateCRC(frame.data() + 1, frame_len - 2);

        if (received_crc != calculated_crc) return std::nullopt;

        Message msg;
        msg.command_id = static_cast<CommandId>(frame[2]);
        if (frame.size() > MIN_FRAME_SIZE)
        {
            msg.payload.assign(frame.begin() + 3, frame.end() - 2);
        }

        return msg;
    }

    uint16_t ECUProtocol::calculateCRC(const uint8_t *data, size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (size_t pos = 0; pos < length; pos++)
        {
            crc ^= (uint16_t)data[pos];
            for (int i = 8; i != 0; i--)
            {
                if ((crc & 0x0001) != 0)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
} // namespace kpi_rover
