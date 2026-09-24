#include "gen2_hardware/can_socket.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>

namespace gen2_hardware
{

int64_t mono_now_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

std::string can_operstate(const std::string & ifname)
{
  std::ifstream f("/sys/class/net/" + ifname + "/operstate");
  if (!f) {
    return "missing";
  }
  std::string s;
  f >> s;
  return s;
}

CanSocket::~CanSocket()
{
  close();
}

bool CanSocket::open(const std::string & ifname, std::string & err, bool receive_own)
{
  close();
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) {
    err = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    err = ifname + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  can_err_mask_t emask = CAN_ERR_MASK;  // all error classes
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &emask, sizeof(emask));
  const int own = receive_own ? 1 : 0;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof(own));
  const int on = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));
  const int rcvbuf = 1 << 20;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    err = std::string("bind: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

void CanSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int CanSocket::read(RxFrame & out, int timeout_ms)
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
  can_frame cf{};
  iovec iov{&cf, sizeof(cf)};
  alignas(cmsghdr) char ctrl[CMSG_SPACE(sizeof(timespec))];
  msghdr msg{};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = ctrl;
  msg.msg_controllen = sizeof(ctrl);
  const ssize_t n = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
  out.mono_ns = mono_now_ns();
  if (n < 0) {
    return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
  }
  if (n < static_cast<ssize_t>(sizeof(can_frame))) {
    return 0;
  }
  out.kernel_realtime_ns = 0;
  for (cmsghdr * c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
    if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPNS) {
      timespec ts{};
      std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
      out.kernel_realtime_ns = static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    }
  }
  out.error_frame = (cf.can_id & CAN_ERR_FLAG) != 0;
  out.error_class = out.error_frame ? (cf.can_id & CAN_ERR_MASK) : 0;
  out.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
  out.frame.id = out.extended ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
  out.frame.len = cf.len > 8 ? 8 : cf.len;
  std::memcpy(out.frame.data, cf.data, 8);
  return 1;
}

bool CanSocket::write_ext(const cubemars::Frame & f, std::string & err)
{
  if (fd_ < 0) {
    err = "socket closed";
    return false;
  }
  can_frame cf{};
  cf.can_id = (f.id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  cf.len = f.len > 8 ? 8 : f.len;
  std::memcpy(cf.data, f.data, 8);
  const ssize_t n = ::send(fd_, &cf, sizeof(cf), MSG_DONTWAIT);
  if (n != static_cast<ssize_t>(sizeof(cf))) {
    err = std::string("send: ") + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace gen2_hardware
