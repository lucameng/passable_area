#ifndef PASSABLE_AREA_CORE_CONFIG_VALIDATION_HPP_
#define PASSABLE_AREA_CORE_CONFIG_VALIDATION_HPP_

#include "passable_area/core/types/config_types.hpp"

#include <string>
#include <vector>

namespace passable_area::core {

std::vector<std::string> ValidateConfig(const Config &config);
void ValidateConfigOrThrow(const Config &config);

} // namespace passable_area::core

#endif
