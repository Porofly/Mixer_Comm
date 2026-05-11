#include "mixer_comm/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace mixer_comm
{

namespace
{

speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud));
  }
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & device, int baud)
{
  close();

  int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    throw std::runtime_error(
      "open(" + device + ") failed: " + std::strerror(errno));
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    int e = errno;
    ::close(fd);
    throw std::runtime_error(
      std::string("tcgetattr failed: ") + std::strerror(e));
  }

  const speed_t speed = to_speed(baud);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 100ms inter-byte timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    int e = errno;
    ::close(fd);
    throw std::runtime_error(
      std::string("tcsetattr failed: ") + std::strerror(e));
  }

  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::read(void * buf, std::size_t buf_size)
{
  return ::read(fd_, buf, buf_size);
}

void SerialPort::write_all(const void * buf, std::size_t buf_size)
{
  const auto * p = static_cast<const unsigned char *>(buf);
  std::size_t remaining = buf_size;
  while (remaining > 0) {
    ssize_t n = ::write(fd_, p, remaining);
    if (n < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(
        std::string("write failed: ") + std::strerror(errno));
    }
    p += n;
    remaining -= static_cast<std::size_t>(n);
  }
}

}  // namespace mixer_comm
