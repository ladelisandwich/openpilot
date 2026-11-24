#pragma once

#include "openpilot/selfdrive/pandad/aol_protocol.h"

inline bool rivian_aol_param(uint16_t param) {
  return (param == 0U) || (param == 1U);
}

// Native capability is additionally required. RELEASE param1 advertises none.
inline constexpr AolSafetyProfile RIVIAN_AOL_PROFILE{33U, rivian_aol_param, true};
