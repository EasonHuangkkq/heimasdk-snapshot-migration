#pragma once

#include <string>

namespace RmdCanSdk {

class ProcessLock {
public:
    ProcessLock() = default;
    ~ProcessLock();

    ProcessLock(ProcessLock const&) = delete;
    ProcessLock& operator=(ProcessLock const&) = delete;

    bool acquire(std::string const& path);
    void release();

    bool acquired() const { return fd_ >= 0; }
    int lastError() const { return lastError_; }

private:
    int fd_ = -1;
    int lastError_ = 0;
};

} // namespace RmdCanSdk
