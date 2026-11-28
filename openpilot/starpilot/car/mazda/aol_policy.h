#pragma once

#include "selfdrive/pandad/aol_protocol.h"

// An unmodified GEN1 (0), or the ladelisandwich fork's GEN1 with the Torque Interceptor and radar emulation
// (1 | 8 | 256 = 265; panda mazda.h mazda_aol_ti_re).
inline bool mazda_aol_param(uint16_t param) {
  return param == 0U || param == 265U;
}

inline constexpr AolSafetyProfile MAZDA_AOL_PROFILE{13U, mazda_aol_param, true};
