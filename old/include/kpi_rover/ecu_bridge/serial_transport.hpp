#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>

namespace kpi_rover
{
    /**
     * @brief SerialTransport provides low-level access to the serial port.
     */
    class SerialTransport
    {
    public:
        /**
         * @brief Constructor.
         * @param device Serial device path (e.g., "/dev/ttyAMA0").
         * @param baud_rate Baud rate (e.g., 115200).
         */
        SerialTransport(const std::string &device, int baud_rate);

        /**
         * @brief Destructor.
         */
        ~SerialTransport();

        /**
         * @brief Opens and configures the serial port.
         * @return true if successful, false otherwise.
         */
        bool open();

        /**
         * @brief Closes the serial port.
         */
        void close();

        /**
         * @brief Sends data over the serial port.
         * @param data Vector of bytes to send.
         * @return true if successful, false otherwise.
         */
        bool write(const std::vector<uint8_t> &data);

        /**
         * @brief Reads data from the serial port.
         * @param buffer Pointer to the buffer to store data.
         * @param size Number of bytes to read.
         * @param timeout_ms Timeout in milliseconds (0 for non-blocking).
         * @return Number of bytes actually read.
         */
        size_t read(uint8_t *buffer, size_t size, int timeout_ms = 0);

        /**
         * @brief Checks if the serial port is open.
         * @return true if open, false otherwise.
         */
        bool isOpen() const { return fd_ != -1; }

        /**
         * @brief Flushes the serial port buffers.
         */
        void flush();

    private:
        std::string device_;
        int baud_rate_;
        int fd_;

        /**
         * @brief Configures the serial port settings.
         * @return true if successful, false otherwise.
         */
        bool configure();
    };
} // namespace kpi_rover
