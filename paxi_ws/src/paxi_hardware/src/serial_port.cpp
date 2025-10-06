
#include "paxi_hardware/serial_port.hpp"

namespace paxi_hardware
{

  SerialPort::SerialPort() : port_("/dev/ttyUSB0"), baud_rate_(115200), fd_(-1), connected_(false) {}

  SerialPort::SerialPort(const std::string & port, std::uint32_t baud_rate)
  : port_(port), baud_rate_(baud_rate), fd_(-1), connected_(false)
  {
  }

  SerialPort::SerialPort(SerialPort && other) noexcept
  : port_(std::move(other.port_)),
    baud_rate_(other.baud_rate_),
    fd_(other.fd_),
    connected_(other.connected_)
  {
    other.fd_ = -1;
  }

  SerialPort & SerialPort::operator=(SerialPort && other) noexcept
  {  
    if (this != &other) {
      close_port();
      port_ = std::move(other.port_);
      baud_rate_ = other.baud_rate_;
      fd_ = other.fd_;
      connected_ = other.connected_;

      other.fd_ = -1;
    }
    return *this;
  }

  SerialPort::~SerialPort(){
      if (is_open()) {
        close_port();
      }
  }

  bool SerialPort::open_port(){
      fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
      if (fd_ == -1) {
          RCLCPP_FATAL(
              rclcpp::get_logger(LOGGER_SERIAL), "Failed to open serial port [%s]: %s", 
              port_.c_str(),
              strerror(errno)
          );
          return false;
      }

      struct termios tty;
      memset(&tty, 0, sizeof(tty));
      if (tcgetattr(fd_, &tty) != 0) {
          RCLCPP_FATAL(
              rclcpp::get_logger(LOGGER_SERIAL),
              "Failed to get terminal attributes for serial port [%s]: %s", 
              port_.c_str(), 
              strerror(errno)
          );
          return false;
      }

      speed_t speed;
      switch (baud_rate_) {
          case 9600:
              speed = B9600;
              break;
          case 19200:
              speed = B19200;
              break;
          case 38400:
              speed = B38400;
              break;
          case 57600:
              speed = B57600;
              break;
          case 115200:
              speed = B115200;
              break;
          default:
              speed = B9600;
      }

      cfsetospeed(&tty, speed);
      cfsetispeed(&tty, speed);

      tty.c_cflag &= ~PARENB;         // No parity
      tty.c_cflag &= ~CSTOPB;         // 1 stop bit
      tty.c_cflag &= ~CSIZE;          // Clear data size bits
      tty.c_cflag |= CS8;             // 8 data bits
      tty.c_cflag &= ~CRTSCTS;        // Disable hardware flow control
      tty.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control lines

      tty.c_lflag &= ~ICANON;  // Disable canonical mode
      tty.c_lflag &= ~ECHO;    // Disable echo
      tty.c_lflag &= ~ECHOE;   // Disable erasure
      tty.c_lflag &= ~ECHONL;  // Disable new-line echo
      tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT, and SUSP

      tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

      tty.c_oflag &= ~OPOST;  // Disable output processing
      tty.c_oflag &= ~ONLCR;  // Disable conversion of newline to carriage return/line feed

      // Set timeouts
      tty.c_cc[VMIN] = 0;  // Non-blocking read, return immediately if no data
      tty.c_cc[VTIME] = 1;  
      // 1 second timeout, this makes read return 0 if no data in 1 second, -1 if an error occurs (like usb disconnect)

      if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
          RCLCPP_FATAL(
              rclcpp::get_logger(LOGGER_SERIAL),
              "Failed to set terminal attributes, closing serial port [%s]: %s", 
              port_.c_str(),
              strerror(errno)
          );

          ::close(fd_);
          fd_ = -1;
          return false;
      }

      connected_ = true;  //assume it starts connected -> will shortly change

      RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_SERIAL), "Successfully opened serial port [%s] with baud rate [%u]",
        port_.c_str(), baud_rate_);
      return true;
  }

  void SerialPort::close_port()
  {
      if (is_open()) {
          ::close(fd_);
          fd_ = -1;

          RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_SERIAL), 
            "Serial port [%s] is now closed", 
            port_.c_str()
          );
      }
  }

  ssize_t SerialPort::write_port(const std::string & data) const
  {
    if (!is_open()) {
        RCLCPP_WARN(
            rclcpp::get_logger(LOGGER_SERIAL),
            "Serial port [%s] is closed, unable to write to closed port", 
            port_.c_str()
        );
        return -1;
    }

    ssize_t num_bytes_written = ::write(fd_, data.c_str(), data.size());
    if (num_bytes_written != static_cast<ssize_t>(data.size())) {
        RCLCPP_ERROR(
            rclcpp::get_logger(LOGGER_SERIAL), "Incomplete write to port [%s]: %ld of %zu bytes written",
            port_.c_str(), 
            num_bytes_written, 
            data.size()
        );
    }

    return num_bytes_written;
  }

  ssize_t SerialPort::write_port(const SerialCommand & cmd) const
  {
    if (!is_open() || !is_connected()) {
        RCLCPP_WARN(
          rclcpp::get_logger(LOGGER_SERIAL),
          "Serial port [%s] is closed or disconnected, unable to write to closed port", 
          port_.c_str()
        );

        return -1;
    }

    ssize_t num_bytes_written = ::write(fd_, static_cast<const void *>(&cmd), sizeof(cmd));
    if (num_bytes_written != sizeof(cmd)) {
      RCLCPP_ERROR(
          rclcpp::get_logger(LOGGER_SERIAL),
              "Incomplete write to port of struct SerialCommand [%s]: %ld of %zu bytes written",
              port_.c_str(), 
              num_bytes_written, 
              sizeof(cmd)
        );
    }

    return num_bytes_written;
  }

  ssize_t SerialPort::read_into_uint8_buf(uint8_t * buffer, std::size_t max_len) const
  {
      if (!is_open() || !is_connected()) {
          RCLCPP_WARN(
              rclcpp::get_logger(LOGGER_SERIAL),
              "Serial Port [%s] is closed or disconnected, unable to read port", port_.c_str()
          );

        return -1;
      }
      return ::read(fd_, buffer, max_len);
  }

  bool SerialPort::set_port(const std::string & port_name)
  {
      port_ = port_name;
      return true;
  }

  bool SerialPort::set_baud(const std::uint32_t & baud_rate)
  {
      baud_rate_ = baud_rate;
      return true;
  }
  

  void SerialPort::update_connection()
  {
      if (!is_open()) {
          connected_ = false;
      }

      struct pollfd pfd;
      pfd.fd = fd_;
      pfd.events = POLLIN | POLLHUP | POLLERR | POLLNVAL;

      int poll_result = poll(&pfd, 1, 0);
      if (poll_result < 0) {
          connected_ = false;
      }

      if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
          connected_ = false;
      }
  }
}  // end of namespace paxi_hardware