#pragma once
#include <vector>
#include <cstdint>
#include <optional>

namespace kpi_rover
{
    /**
     * @brief ECUProtocol handles framing and CRC for the ECU communication.
     */
    class ECUProtocol
    {
    public:
        enum class CommandId : uint8_t {
            GET_API_VERSION = 0x01,
            SET_ALL_MOTORS_SPEED = 0x03,
            GET_ALL_ENCODERS = 0x05,
            GET_IMU = 0x06,
            UNKNOWN = 0xFF
        };

        struct Message
        {
            CommandId command_id;
            std::vector<uint8_t> payload;
        };

        /**
         * @brief Encapsulates a command into a protocol frame.
         * @param msg The message to encapsulate.
         * @return Vector containing the complete frame.
         */
        static std::vector<uint8_t> encode(const Message &msg);

        /**
         * @brief Decapsulates a protocol frame into a message.
         * @param frame The raw frame data (starting with 0xAA).
         * @return The message if successful, nullopt otherwise.
         */
        static std::optional<Message> decode(const std::vector<uint8_t> &frame);

        /**
         * @brief Calculates CRC16 (Modbus).
         * @param data Pointer to data.
         * @param length Length of data.
         * @return Calculated CRC16.
         */
        static uint16_t calculateCRC(const uint8_t *data, size_t length);

        static constexpr uint8_t START_BYTE = 0xAA;
        static constexpr size_t MIN_FRAME_SIZE = 5; // AA Len Cmd CRC_L CRC_H
        static constexpr size_t HEADER_SIZE = 3; // AA Len Cmd
    };
} // namespace kpi_rover
