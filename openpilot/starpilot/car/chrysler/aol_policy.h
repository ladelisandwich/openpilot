#pragma once

#include "selfdrive/pandad/aol_protocol.h"

inline bool chrysler_aol_param(uint16_t param) {
  return (param == 0U) || (param == 1U) || (param == 4U) || (param == 8U);
}

inline constexpr AolSafetyProfile CHRYSLER_AOL_PROFILE{9U, chrysler_aol_param, true};
