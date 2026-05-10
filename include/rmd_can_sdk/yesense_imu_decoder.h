#pragma once

#include "rmd_can_sdk/heima_driver_sdk.h"

#include "yesense/yesense_decoder.h"
#include "yesense/yesense_std_out_decoder.h"

#include <cstddef>

namespace RmdCanSdk {

DriverSDK::imuStruct yesenseOutputToImu(yesense::yis_out_data_t const& output);

class YesenseImuDecoder {
public:
    bool feed(unsigned char const* data, std::size_t length, DriverSDK::imuStruct& out);

private:
    yesense::yesense_decoder decoder_;
    yesense::yis_out_data_t output_{};
};

} // namespace RmdCanSdk
