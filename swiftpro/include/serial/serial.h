#ifndef SWIFTPRO_SERIAL_POSIX_H
#define SWIFTPRO_SERIAL_POSIX_H

#include <string>
#include <cstddef>
#include <chrono>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>

namespace serial {

struct Timeout {
  // timeout in milliseconds
  int ms{1000};
  static Timeout simpleTimeout(int m) { Timeout t; t.ms = m; return t; }
};

class Serial {
public:
  Serial() : fd_(-1), baud_(115200) {}
  explicit Serial(const std::string &port, unsigned long baud = 115200) : fd_(-1), port_(port), baud_(baud) {}

  void setPort(const std::string &p) { port_ = p; }
  void setBaudrate(unsigned long b) { baud_ = b; }
  void setTimeout(const Timeout &t) { timeout_ = t; }

  void open() {
    if (port_.empty()) throw std::runtime_error("serial: port not set");
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY );
    if (fd_ < 0) throw std::runtime_error("serial: open failed");

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      ::close(fd_); fd_ = -1; throw std::runtime_error("serial: tcgetattr failed");
    }

    cfmakeraw(&tty);

    speed_t speed = baudToSpeed(baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // Set minimal blocking reads with timeout via VMIN/VTIME
    tty.c_cc[VMIN] = 0; // return immediately with what is available
    tty.c_cc[VTIME] = static_cast<unsigned char>((timeout_.ms + 99) / 100); // deciseconds

    tty.c_cflag |= CLOCAL | CREAD;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_); fd_ = -1; throw std::runtime_error("serial: tcsetattr failed");
    }
  }

  bool isOpen() const { return fd_ >= 0; }

  std::size_t available() {
    if (fd_ < 0) return 0;
    int count = 0;
    if (ioctl(fd_, FIONREAD, &count) == -1) return 0;
    return static_cast<std::size_t>(count);
  }

  std::string read(std::size_t size) {
    if (fd_ < 0) return std::string();
    if (size == 0) return std::string();
    std::string buf;
    buf.resize(size);
    ssize_t r = ::read(fd_, &buf[0], static_cast<size_t>(size));
    if (r <= 0) return std::string();
    buf.resize(static_cast<size_t>(r));
    return buf;
  }

  void write(const std::string &data) {
    if (fd_ < 0) throw std::runtime_error("serial: not open");
    const char *p = data.data();
    size_t remaining = data.size();
    while (remaining > 0) {
      ssize_t w = ::write(fd_, p, remaining);
      if (w <= 0) throw std::runtime_error("serial: write failed");
      remaining -= static_cast<size_t>(w);
      p += w;
    }
  }

  ~Serial() {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  int fd_;
  std::string port_;
  unsigned long baud_;
  Timeout timeout_;

  static speed_t baudToSpeed(unsigned long baud) {
    switch (baud) {
      case 115200: return B115200;
      case 57600: return B57600;
      case 38400: return B38400;
      case 19200: return B19200;
      case 9600: return B9600;
      case 4800: return B4800;
      case 2400: return B2400;
      case 1200: return B1200;
      default: return B115200;
    }
  }
};

} // namespace serial

#endif // SWIFTPRO_SERIAL_POSIX_H
