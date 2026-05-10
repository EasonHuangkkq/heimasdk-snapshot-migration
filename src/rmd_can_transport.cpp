#include "rmd_can_sdk/rmd_can_transport.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace RmdCanSdk {

SocketCanTransport::~SocketCanTransport() {
    close();
}

int SocketCanTransport::open(std::string const& device) {
    close();
    device_ = device;
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        return -1;
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) != 0) {
        close();
        return -1;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close();
        return -1;
    }

    int loopback = 0;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    }
    return 0;
}

void SocketCanTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SocketCanTransport::send(int id, unsigned char const* data, int length) {
    if (fd_ < 0 || length < 0 || length > 8) {
        return -1;
    }
    can_frame frame{};
    frame.can_id = static_cast<canid_t>(id);
    frame.can_dlc = static_cast<__u8>(length);
    std::memcpy(frame.data, data, static_cast<std::size_t>(length));
    ssize_t ret = write(fd_, &frame, sizeof(frame));
    return ret == static_cast<ssize_t>(sizeof(frame)) ? id : -1;
}

int SocketCanTransport::receive(CanFrame& frame, int timeoutMs) {
    if (fd_ < 0) {
        return -1;
    }
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int ready = poll(&pfd, 1, timeoutMs);
    if (ready <= 0) {
        return ready;
    }
    can_frame raw{};
    ssize_t ret = read(fd_, &raw, sizeof(raw));
    if (ret <= 0) {
        return -1;
    }
    frame.id = static_cast<int>(raw.can_id & CAN_SFF_MASK);
    frame.length = raw.can_dlc;
    frame.data = {};
    std::memcpy(frame.data.data(), raw.data, raw.can_dlc);
    return frame.length;
}

} // namespace RmdCanSdk

