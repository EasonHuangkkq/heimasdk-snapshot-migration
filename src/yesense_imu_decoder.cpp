#include "rmd_can_sdk/yesense_imu_decoder.h"

#include "rmd_can_sdk/rmd_types.h"
#include "yesense/yesense_decoder_comm.h"

namespace RmdCanSdk {

DriverSDK::imuStruct yesenseOutputToImu(yesense::yis_out_data_t const& output) {
    DriverSDK::imuStruct imu;
    if (output.content.euler) {
        imu.rpy[0] = output.euler.roll * Pi / 180.0f;
        imu.rpy[1] = output.euler.pitch * Pi / 180.0f;
        imu.rpy[2] = output.euler.yaw * Pi / 180.0f;
    }
    if (output.content.gyro) {
        imu.gyr[0] = output.gyro.x * Pi / 180.0f;
        imu.gyr[1] = output.gyro.y * Pi / 180.0f;
        imu.gyr[2] = output.gyro.z * Pi / 180.0f;
    }
    if (output.content.acc) {
        imu.acc[0] = output.acc.x;
        imu.acc[1] = output.acc.y;
        imu.acc[2] = output.acc.z;
    }
    return imu;
}

bool YesenseImuDecoder::feed(unsigned char const* data, std::size_t length, DriverSDK::imuStruct& out) {
    if (data == nullptr || length == 0) {
        return false;
    }
    int const rc = decoder_.data_proc(const_cast<unsigned char*>(data), static_cast<unsigned int>(length), &output_);
    if (rc != analysis_ok || !output_.content.valid_flg) {
        return false;
    }
    out = yesenseOutputToImu(output_);
    return true;
}

} // namespace RmdCanSdk
