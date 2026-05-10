#pragma once

#include <array>
#include <string>

namespace RmdCanSdk {

struct CanFrame {
    int id = 0;
    int length = 0;
    std::array<unsigned char, 8> data{};
};

class SocketCanTransport {
public:
    SocketCanTransport() = default;
    ~SocketCanTransport();

    SocketCanTransport(SocketCanTransport const&) = delete;
    SocketCanTransport& operator=(SocketCanTransport const&) = delete;

    int open(std::string const& device);
    void close();
    bool isOpen() const { return fd_ >= 0; }
    int send(int id, unsigned char const* data, int length);
    int receive(CanFrame& frame, int timeoutMs);

private:
    int fd_ = -1;
    std::string device_;
};

} // namespace RmdCanSdk

