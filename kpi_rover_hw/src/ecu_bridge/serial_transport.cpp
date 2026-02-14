#include "kpi_rover/ecu_bridge/serial_transport.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/select.h>
#include <errno.h>
#include "rclcpp/logging.hpp"

namespace kpi_rover
{
    SerialTransport::SerialTransport(const std::string &device, int baud_rate)
        : device_(device), baud_rate_(baud_rate), fd_(-1)
    {
    }

    SerialTransport::~SerialTransport()
    {
        close();
    }

    bool SerialTransport::open()
    {
        close();

        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SerialTransport"), "Error opening %s: %s", device_.c_str(), strerror(errno));
            return false;
        }

        if (!configure())
        {
            close();
            return false;
        }

        return true;
    }

    void SerialTransport::close()
    {
        if (fd_ != -1)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void SerialTransport::flush()
    {
        if (fd_ != -1)
        {
            tcflush(fd_, TCIOFLUSH);
        }
    }

    bool SerialTransport::configure()
    {
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SerialTransport"), "Error from tcgetattr: %s", strerror(errno));
            return false;
        }

        speed_t speed;
        switch (baud_rate_)
        {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 500000: speed = B500000; break;
            case 576000: speed = B576000; break;
            case 921600: speed = B921600; break;
            case 1000000: speed = B1000000; break;
            case 1152000: speed = B1152000; break;
            case 1500000: speed = B1500000; break;
            case 2000000: speed = B2000000; break;
            case 2500000: speed = B2500000; break;
            case 3000000: speed = B3000000; break;
            case 3500000: speed = B3500000; break;
            case 4000000: speed = B4000000; break;
            default:
                RCLCPP_WARN(rclcpp::get_logger("SerialTransport"), "Unsupported baudrate %d, defaulting to 115200", baud_rate_);
                speed = B115200;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;  // Read returns as soon as any data is available (up to VTIME)
        tty.c_cc[VTIME] = 1; // 0.1s inter-character timeout

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SerialTransport"), "Error from tcsetattr: %s", strerror(errno));
            return false;
        }

        return true;
    }

    bool SerialTransport::write(const std::vector<uint8_t> &data)
    {
        if (fd_ == -1) return false;

        ssize_t written = ::write(fd_, data.data(), data.size());
        return (written == static_cast<ssize_t>(data.size()));
    }

    size_t SerialTransport::read(uint8_t *buffer, size_t size, int timeout_ms)
    {
        if (fd_ == -1) return 0;

        if (timeout_ms > 0)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd_, &readfds);

            struct timeval tv;
            tv.tv_sec = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;

            int ret = select(fd_ + 1, &readfds, NULL, NULL, &tv);
            if (ret <= 0)
            {
                // Timeout or error
                return 0;
            }
        }

        ssize_t n = ::read(fd_, buffer, size);
        if (n > 0) return static_cast<size_t>(n);
        
        return 0;
    }
} // namespace kpi_rover
