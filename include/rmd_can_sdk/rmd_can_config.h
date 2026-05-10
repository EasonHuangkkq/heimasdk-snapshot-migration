#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <string>

namespace RmdCanSdk {

Config loadConfig(std::string const& path);

} // namespace RmdCanSdk

