#ifndef SWIFTPRO_SERIAL_FALLBACK_H
#define SWIFTPRO_SERIAL_FALLBACK_H

#include <string>
#include <cstddef>

namespace serial {

struct Timeout {
  static Timeout simpleTimeout(int /*ms*/) { return Timeout(); }
};

class Serial {
public:
  Serial() {}
  void setPort(const std::string &) {}
  void setBaudrate(unsigned long) {}
  void setTimeout(const Timeout &) {}
  void open() {}
  bool isOpen() const { return false; }
  std::size_t available() { return 0; }
  std::string read(std::size_t) { return std::string(); }
  void write(const std::string &) {}
};

} // namespace serial

#endif // SWIFTPRO_SERIAL_FALLBACK_H
