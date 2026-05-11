#pragma once

#include <cstddef>
#include <string>

namespace mixer_comm
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens device for read+write in raw, blocking-with-timeout mode.
  // Throws std::runtime_error on failure.
  void open(const std::string & device, int baud);

  void close();

  bool is_open() const { return fd_ >= 0; }

  // Reads up to buf_size bytes. Returns bytes read, 0 on timeout, -1 on error.
  // Blocks up to ~100ms.
  ssize_t read(void * buf, std::size_t buf_size);

  // Writes exactly buf_size bytes (loops on partial writes). Throws on error.
  void write_all(const void * buf, std::size_t buf_size);

private:
  int fd_ = -1;
};

}  // namespace mixer_comm
