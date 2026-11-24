#pragma once

// Optional Nissan stock-cruise profile. The core RX checks and all angle limits stay upstream.
static bool nissan_aol_enabled = false;
static bool nissan_aol_leaf = false;
static bool nissan_aol_alt = false;
static bool nissan_aol_main = false;
static bool nissan_aol_drive = false;
static bool nissan_aol_eps = false;
static bool nissan_aol_stock_lkas = false;
static bool nissan_aol_seen[4] = {false, false, false, false};
static uint32_t nissan_aol_ts[4] = {0U, 0U, 0U, 0U};

static void nissan_aol_reset(void) {
  nissan_aol_main = false;
  nissan_aol_drive = false;
  nissan_aol_eps = false;
  nissan_aol_stock_lkas = false;
  for (unsigned int i = 0U; i < 4U; i++) {
    nissan_aol_seen[i] = false;
    nissan_aol_ts[i] = 0U;
  }
}

static uint8_t nissan_aol_request(void) {
  uint8_t request = 0U;
  if (heartbeat_engaged && aol_rx_healthy() && !safety_rx_checks_invalid && !relay_malfunction &&
      (safety_get_ts_elapsed(microsecond_timer_get(), aol_host_request_ts) <= AOL_HOST_REQUEST_TIMEOUT_US)) {
    request = aol_host_axis_mask;
  }
  return request;
}

static uint8_t nissan_aol_permission(void) {
  const uint8_t request = nissan_aol_request();
  bool fresh = true;
  for (unsigned int i = 0U; i < 4U; i++) {
    // New optional vehicle-source bound; never renew a source from a host request.
    fresh &= nissan_aol_seen[i] && (safety_get_ts_elapsed(microsecond_timer_get(), nissan_aol_ts[i]) <= 300000U);
  }
  uint8_t permission = 0U;
  if (fresh && nissan_aol_main && nissan_aol_drive && nissan_aol_eps && !nissan_aol_stock_lkas) {
    permission = request & 1U;
    // This describes stock cruise eligibility, never adds Nissan longitudinal CAN messages.
    if (controls_allowed && !brake_pressed && !gas_pressed) {
      permission |= request & 2U;
    }
  }
  return permission;
}

static void nissan_aol_observe(const CANPacket_t *msg) {
  if (nissan_aol_enabled) {
    const unsigned int pt = nissan_aol_alt ? 1U : 0U;
    const unsigned int main_addr = nissan_aol_leaf ? 0x239U : 0x1B6U;
    unsigned int main_bus = 0U;
    if (!nissan_aol_leaf) {
      main_bus = nissan_aol_alt ? 2U : 1U;
    }
    const unsigned int gear_addr = nissan_aol_leaf ? 0x421U : 0x41FU;
    unsigned int index = 4U;
    unsigned int len = 0U;
    if (msg_matches(msg, main_addr, main_bus)) {
      index = 0U;
      len = 8U;
    } else if (msg_matches(msg, gear_addr, pt)) {
      index = 1U;
      len = nissan_aol_leaf ? 3U : 2U;
    } else if (msg_matches(msg, 0x185U, 0U)) {
      index = 2U;
      len = 8U;
    } else if (msg_matches(msg, 0x4CBU, nissan_aol_alt ? 1U : 2U)) {
      index = 3U;
      len = 8U;
    } else {
    }
    if (index < 4U) {
      nissan_aol_seen[index] = GET_LEN(msg) == len;
      if (nissan_aol_seen[index]) {
        nissan_aol_ts[index] = microsecond_timer_get();
        if (index == 0U) {
          nissan_aol_main = GET_BIT(msg, nissan_aol_leaf ? 17U : 36U);
        } else if (index == 1U) {
          const unsigned int gear = (msg->data[0] >> 3) & 7U;
          nissan_aol_drive = (gear == 4U) || (gear == (nissan_aol_leaf ? 7U : 6U));
        } else if (index == 2U) {
          const unsigned int status = msg->data[6] >> 4;
          nissan_aol_eps = (status == 5U) || (status == 6U);
        } else {
          nissan_aol_stock_lkas = GET_BIT(msg, 51U);
        }
      }
    }
  }
}
