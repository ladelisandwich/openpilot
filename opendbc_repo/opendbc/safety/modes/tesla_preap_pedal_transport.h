#pragma once
#include "opendbc/safety/modes/tesla_preap_pedal.h"

// Startup transport API: included by the actual Panda USB owner and native
// library bridge. The CAN safety translation unit owns the shared state only.
static inline bool tesla_preap_pedal_config_word(uint16_t index, uint16_t value, bool may_configure) {
  bool accepted = false;
  if (may_configure && !tesla_preap_pedal_configured) {
    if (index == 0U) {
      tesla_preap_pedal_startup.next_word = 0U;
      tesla_preap_pedal_startup.staged_ready = false;
    }
    if (index == 0xFFFFU) {
      tesla_preap_pedal_startup.staged_ready = (value == 0xC051U) &&
        (tesla_preap_pedal_startup.next_word == TESLA_PREAP_PEDAL_CONFIG_WORDS) &&
        tesla_preap_pedal_config_valid(tesla_preap_pedal_startup.staged);
      accepted = tesla_preap_pedal_startup.staged_ready;
    } else if ((index != tesla_preap_pedal_startup.next_word) || (index >= TESLA_PREAP_PEDAL_CONFIG_WORDS)) {
      tesla_preap_pedal_startup.next_word = 0U;
      tesla_preap_pedal_startup.staged_ready = false;
    } else {
      tesla_preap_pedal_startup.staged[index] = value;
      tesla_preap_pedal_startup.next_word++;
      accepted = true;
    }
  }
  return accepted;
}

static inline unsigned int tesla_preap_pedal_config_read(uint8_t *out) {
  unsigned int length = 0U;
  if (tesla_preap_pedal_configured || tesla_preap_pedal_startup.staged_ready) {
    const uint16_t *words = tesla_preap_pedal_configured ? tesla_preap_pedal_startup.active_words : tesla_preap_pedal_startup.staged;
    for (unsigned int i = 0U; i < TESLA_PREAP_PEDAL_CONFIG_WORDS; i++) {
      out[2U * i] = words[i] & 0xFFU;
      out[(2U * i) + 1U] = words[i] >> 8U;
    }
    length = TESLA_PREAP_PEDAL_CONFIG_WORDS * 2U;
  }
  return length;
}
