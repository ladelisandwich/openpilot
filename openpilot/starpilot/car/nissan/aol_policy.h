#pragma once
#include "openpilot/selfdrive/pandad/aol_protocol.h"
inline bool nissan_aol_param(uint16_t param) {
  return (param == 0U) || (param == 1U) || (param == 4U);
}
inline constexpr AolSafetyProfile NISSAN_AOL_PROFILE{14U, nissan_aol_param, true};
