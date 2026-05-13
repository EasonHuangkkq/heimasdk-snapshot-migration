#include "rmd_can_sdk/rmd_process_lock.h"

#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace RmdCanSdk {

ProcessLock::~ProcessLock() {
    release();
}

bool ProcessLock::acquire(std::string const& path) {
    release();
    lastError_ = 0;
    int const fd = open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (fd < 0) {
        lastError_ = errno;
        return false;
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        lastError_ = errno;
        close(fd);
        return false;
    }
    fd_ = fd;
    return true;
}

void ProcessLock::release() {
    if (fd_ < 0) {
        return;
    }
    flock(fd_, LOCK_UN);
    close(fd_);
    fd_ = -1;
}

} // namespace RmdCanSdk
