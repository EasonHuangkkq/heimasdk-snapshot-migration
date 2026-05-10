#include "rmd_can_sdk/rmd_types.h"

namespace RmdCanSdk {

float clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void MaskTracker::setExpectedFromSlaveIds(std::vector<int> const& slaveIds) {
    expectedMask_ = 0;
    currentMask_ = 0;
    for (int slaveId : slaveIds) {
        expectedMask_ |= bitForSlaveId(slaveId);
    }
}

bool MaskTracker::markReceived(int slaveId) {
    std::uint64_t const bit = bitForSlaveId(slaveId);
    if ((expectedMask_ & bit) == 0) {
        return false;
    }
    currentMask_ |= bit;
    if (expectedMask_ != 0 && currentMask_ == expectedMask_) {
        currentMask_ = 0;
        return true;
    }
    return false;
}

bool MaskTracker::expects(int slaveId) const {
    std::uint64_t const bit = bitForSlaveId(slaveId);
    return bit != 0 && (expectedMask_ & bit) != 0;
}

bool MaskTracker::isReceived(int slaveId) const {
    std::uint64_t const bit = bitForSlaveId(slaveId);
    return bit != 0 && (currentMask_ & bit) != 0;
}

void MaskTracker::resetCurrent() {
    currentMask_ = 0;
}

std::uint64_t MaskTracker::bitForSlaveId(int slaveId) {
    if (slaveId < 0 || slaveId >= 64) {
        return 0;
    }
    return std::uint64_t{1} << slaveId;
}

void FeedbackFrameTracker::setExpectedFromSlaveIds(std::vector<int> const& slaveIds) {
    mask_.setExpectedFromSlaveIds(slaveIds);
    active_ = false;
}

bool FeedbackFrameTracker::markReceived(int slaveId, Clock::time_point now) {
    if (!mask_.expects(slaveId)) {
        return false;
    }
    if (!active_) {
        started_ = now;
        active_ = true;
    }
    if (mask_.markReceived(slaveId)) {
        active_ = false;
        return true;
    }
    return false;
}

bool FeedbackFrameTracker::shouldPublishTimeout(Clock::time_point now, Clock::duration timeout) const {
    return active_ && mask_.currentMask() != 0 && now - started_ >= timeout;
}

bool FeedbackFrameTracker::isReceived(int slaveId) const {
    return mask_.isReceived(slaveId);
}

void FeedbackFrameTracker::reset() {
    mask_.resetCurrent();
    active_ = false;
}

} // namespace RmdCanSdk
