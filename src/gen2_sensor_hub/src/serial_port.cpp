#include "gen2_sensor_hub/serial_port.hpp"

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace gen2_sensor_hub
{

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & path, std::string & err)
{
  close();
  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    err = path + ": " + std::strerror(errno);
    return false;
  }
  if (::ioctl(fd, TIOCEXCL) != 0) {
    // Not fatal: exclusive access just prevents a second reader from stealing bytes.
  }
  termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    err = path + ": tcgetattr: " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~HUPCL;  // do not toggle DTR/RTS on close (ESP32 auto-reset lines)
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, B921600);  // ignored by CDC-ACM, kept for USB-UART bridges
  ::cfsetospeed(&tio, B921600);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    err = path + ": tcsetattr: " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  ::tcflush(fd, TCIFLUSH);
  fd_ = fd;
  path_ = path;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

long SerialPort::read_some(uint8_t * buf, std::size_t cap, int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }
  pollfd p{fd_, POLLIN, 0};
  const int r = ::poll(&p, 1, timeout_ms);
  if (r < 0) {
    return errno == EINTR ? 0 : -1;
  }
  if (r == 0) {
    return 0;
  }
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return -1;
  }
  const ssize_t n = ::read(fd_, buf, cap);
  if (n > 0) {
    return n;
  }
  if (n == 0) {
    return -1;  // readable but EOF: device vanished
  }
  return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
}

std::vector<std::string> discover_ports(
  const std::string & port, const std::vector<std::string> & by_id_patterns)
{
  if (port != "auto") {
    return {port};
  }
  std::vector<std::string> out;
  auto add_glob = [&out](const std::string & pattern) {
      glob_t g{};
      if (::glob(pattern.c_str(), 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; ++i) {
          if (std::find(out.begin(), out.end(), g.gl_pathv[i]) == out.end()) {
            out.emplace_back(g.gl_pathv[i]);
          }
        }
      }
      ::globfree(&g);
    };
  for (const auto & p : by_id_patterns) {
    add_glob(p);
  }
  add_glob("/dev/ttyACM*");
  return out;
}

}  // namespace gen2_sensor_hub
