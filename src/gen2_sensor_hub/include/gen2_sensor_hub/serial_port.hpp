// Minimal non-blocking raw serial port for USB CDC-ACM devices.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gen2_sensor_hub
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open(const std::string & path, std::string & err);
  void close();
  bool is_open() const {return fd_ >= 0;}
  const std::string & path() const {return path_;}

  // Waits up to timeout_ms for data. Returns bytes read (>=0), or -1 if the device is gone.
  long read_some(uint8_t * buf, std::size_t cap, int timeout_ms);

private:
  int fd_ = -1;
  std::string path_;
};

// Resolve "auto" into candidate device paths: /dev/serial/by-id entries matching any of
// `by_id_patterns` (glob), then /dev/ttyACM*. Explicit paths are returned as-is.
std::vector<std::string> discover_ports(
  const std::string & port, const std::vector<std::string> & by_id_patterns);

}  // namespace gen2_sensor_hub
