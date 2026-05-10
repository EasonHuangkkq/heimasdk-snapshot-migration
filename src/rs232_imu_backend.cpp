#include "rmd_can_sdk/rs232_imu_backend.h"

#include "rmd_can_sdk/rmd_types.h"
#include "rmd_can_sdk/yesense_imu_decoder.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace RmdCanSdk {
namespace {

struct ImuProtocol {
    int frameLength = 0;
    unsigned char header0 = 0;
    unsigned char header1 = 0;
};

speed_t baudToSpeed(int baudrate) {
    switch (baudrate) {
    case 921600: return B921600;
    case 576000: return B576000;
    case 460800: return B460800;
    case 230400: return B230400;
    case 115200: return B115200;
    case 57600: return B57600;
    case 38400: return B38400;
    case 19200: return B19200;
    case 9600: return B9600;
    case 4800: return B4800;
    case 2400: return B2400;
    case 1200: return B1200;
    case 300: return B300;
    default: return 0;
    }
}

int openSerial(char const* device, int baudrate) {
    speed_t const speed = baudToSpeed(baudrate);
    if (speed == 0) {
        return -1;
    }
    int fd = open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }

    termios opt{};
    if (tcgetattr(fd, &opt) != 0) {
        close(fd);
        return -1;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CRTSCTS;
    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~INPCK;
    opt.c_iflag &= ~(ICRNL | INLCR);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_oflag &= ~OPOST;
    opt.c_oflag &= ~(OCRNL | ONLCR);
    opt.c_cc[VTIME] = 1;
    opt.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &opt) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

std::string normalizeType(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

bool protocolForType(std::string const& type, ImuProtocol& protocol) {
    std::string const normalized = normalizeType(type);
    if (normalized == "XSENS") {
        protocol.frameLength = 50;
        protocol.header0 = 0xfa;
        protocol.header1 = 0xff;
        return true;
    }
    if (normalized == "HIPNUC") {
        protocol.frameLength = 82;
        protocol.header0 = 0x5a;
        protocol.header1 = 0xa5;
        return true;
    }
    if (normalized == "YESENSE") {
        protocol.frameLength = 0;
        protocol.header0 = 0x59;
        protocol.header1 = 0x53;
        return true;
    }
    return false;
}

float bigEndianFloat(unsigned char const* bytes) {
    float value = 0.0f;
    unsigned char* out = reinterpret_cast<unsigned char*>(&value);
    out[0] = bytes[3];
    out[1] = bytes[2];
    out[2] = bytes[1];
    out[3] = bytes[0];
    return value;
}

float littleEndianFloat(unsigned char const* bytes) {
    float value = 0.0f;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

void crcUpdate(unsigned short& currentCrc, unsigned char const* src, int length) {
    unsigned int crc = currentCrc;
    for (int i = 0; i < length; ++i) {
        unsigned int byte = src[i];
        crc ^= byte << 8;
        for (int j = 0; j < 8; ++j) {
            unsigned int temp = crc << 1;
            if ((crc & 0x8000) != 0) {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    currentCrc = static_cast<unsigned short>(crc);
}

bool validXsens(unsigned char const* frame) {
    bool const dataIdsValid =
        ((frame[4] == 0x20 && frame[5] == 0x10 && frame[6] == 0x10) ||
         (frame[4] == 0x20 && frame[5] == 0x30 && frame[6] == 0x0c)) &&
        frame[19] == 0x40 && frame[20] == 0x20 && frame[21] == 0x0c &&
        frame[34] == 0x80 && frame[35] == 0x20 && frame[36] == 0x0c;
    if (!dataIdsValid) {
        return false;
    }

    int sum = 0;
    for (int i = 1; i < 49; ++i) {
        sum += frame[i];
    }
    if (sum > 0xff) {
        sum = ~sum;
        sum += 1;
    }
    sum &= 0xff;
    return sum == frame[49];
}

bool validHiPNUC(unsigned char const* frame) {
    unsigned short crc = 0;
    crcUpdate(crc, frame, 4);
    crcUpdate(crc, frame + 6, 76);
    return crc == static_cast<unsigned short>((frame[5] << 8) | frame[4]);
}

bool decodeXsens(unsigned char const* frame, DriverSDK::imuStruct& out) {
    if (!validXsens(frame)) {
        return false;
    }
    out.rpy[0] = bigEndianFloat(frame + 7) * Pi / 180.0f;
    out.rpy[1] = bigEndianFloat(frame + 11) * Pi / 180.0f;
    out.rpy[2] = bigEndianFloat(frame + 15) * Pi / 180.0f;
    out.acc[0] = bigEndianFloat(frame + 22);
    out.acc[1] = bigEndianFloat(frame + 26);
    out.acc[2] = bigEndianFloat(frame + 30);
    out.gyr[0] = bigEndianFloat(frame + 37);
    out.gyr[1] = bigEndianFloat(frame + 41);
    out.gyr[2] = bigEndianFloat(frame + 45);
    return true;
}

bool decodeHiPNUC(unsigned char const* frame, DriverSDK::imuStruct& out) {
    if (!validHiPNUC(frame)) {
        return false;
    }
    out.acc[0] = littleEndianFloat(frame + 18) * 9.81f;
    out.acc[1] = littleEndianFloat(frame + 22) * 9.81f;
    out.acc[2] = littleEndianFloat(frame + 26) * 9.81f;
    out.gyr[0] = littleEndianFloat(frame + 30) * Pi / 180.0f;
    out.gyr[1] = littleEndianFloat(frame + 34) * Pi / 180.0f;
    out.gyr[2] = littleEndianFloat(frame + 38) * Pi / 180.0f;
    out.rpy[0] = littleEndianFloat(frame + 58) * Pi / 180.0f;
    out.rpy[1] = littleEndianFloat(frame + 54) * Pi / 180.0f;
    out.rpy[2] = littleEndianFloat(frame + 62) * Pi / 180.0f;
    return true;
}

bool decodeFrame(std::string const& type, unsigned char const* frame, DriverSDK::imuStruct& out) {
    std::string const normalized = normalizeType(type);
    if (normalized == "XSENS") {
        return decodeXsens(frame, out);
    }
    if (normalized == "HIPNUC") {
        return decodeHiPNUC(frame, out);
    }
    return false;
}

} // namespace

bool isSupportedImuType(char const* type) {
    if (type == nullptr) {
        return false;
    }
    ImuProtocol protocol;
    return protocolForType(type, protocol);
}

Rs232ImuBackend::Rs232ImuBackend(ImuSnapshotBuffer& buffer) : buffer_(buffer) {}

Rs232ImuBackend::~Rs232ImuBackend() {
    stop();
}

int Rs232ImuBackend::start(char const* device, int baudrate) {
    return start(device, baudrate, "HiPNUC");
}

int Rs232ImuBackend::start(char const* device, int baudrate, char const* type) {
    if (device == nullptr || type == nullptr) {
        return -1;
    }
    ImuProtocol protocol;
    if (!protocolForType(type, protocol)) {
        return -1;
    }
    if (running()) {
        return 0;
    }
    int fd = openSerial(device, baudrate);
    if (fd < 0) {
        return -1;
    }
    device_ = device;
    type_ = normalizeType(type);
    baudrate_ = baudrate;
    serialFd_ = fd;
    DriverSDK::imuStruct zero;
    buffer_.publish(zero);
    stop_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&Rs232ImuBackend::readLoop, this);
    return 0;
}

void Rs232ImuBackend::stop() {
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
}

void Rs232ImuBackend::readLoop() {
    ImuProtocol protocol;
    if (!protocolForType(type_, protocol)) {
        running_.store(false, std::memory_order_release);
        return;
    }

    int const fd = serialFd_;
    if (fd < 0) {
        running_.store(false, std::memory_order_release);
        return;
    }

    if (type_ == "YESENSE") {
        YesenseImuDecoder decoder;
        std::array<unsigned char, 512> input{};
        while (!stop_.load(std::memory_order_acquire)) {
            ssize_t const n = read(fd, input.data(), input.size());
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            DriverSDK::imuStruct decoded;
            if (decoder.feed(input.data(), static_cast<std::size_t>(n), decoded)) {
                buffer_.publish(decoded);
            }
        }
        close(fd);
        serialFd_ = -1;
        running_.store(false, std::memory_order_release);
        return;
    }

    std::array<unsigned char, 128> frame{};
    std::array<unsigned char, 512> input{};
    int frameIndex = 0;

    while (!stop_.load(std::memory_order_acquire)) {
        ssize_t const n = read(fd, input.data(), input.size());
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            unsigned char const byte = input[static_cast<std::size_t>(i)];
            if (frameIndex == 0) {
                if (byte != protocol.header0) {
                    continue;
                }
                frame[0] = byte;
                frameIndex = 1;
                continue;
            }
            if (frameIndex == 1) {
                if (byte != protocol.header1) {
                    frameIndex = byte == protocol.header0 ? 1 : 0;
                    frame[0] = byte == protocol.header0 ? byte : 0;
                    continue;
                }
                frame[1] = byte;
                frameIndex = 2;
                continue;
            }

            frame[static_cast<std::size_t>(frameIndex)] = byte;
            frameIndex++;
            if (frameIndex == protocol.frameLength) {
                DriverSDK::imuStruct decoded;
                if (decodeFrame(type_, frame.data(), decoded)) {
                    buffer_.publish(decoded);
                }
                frameIndex = 0;
            }
        }
    }

    close(fd);
    serialFd_ = -1;
    running_.store(false, std::memory_order_release);
}

} // namespace RmdCanSdk
