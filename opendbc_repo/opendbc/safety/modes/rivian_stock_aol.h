#pragma once

// A normal stock engagement grants one GEN1 torque session. Standby is
// availability only; host requests cannot manufacture a physical grant.
static bool rivian_aol_enabled = false;
static bool rivian_aol_longitudinal = false;
static bool rivian_aol_token = false;
static bool rivian_aol_session = false;
static bool rivian_aol_requested = false;
static bool rivian_aol_pending = false;
static bool rivian_aol_drive = false;
static bool rivian_aol_eps_ok = false;
static bool rivian_aol_toi_ok = false;
static uint8_t rivian_aol_feature = 0U;
static uint32_t rivian_aol_grant_ts = 0U;

static void rivian_aol_clear(void) {
  rivian_aol_token = false;
  rivian_aol_pending = false;
  rivian_aol_session = false;
}

static void rivian_aol_reset(void) {
  rivian_aol_clear();
  rivian_aol_enabled = false;
  rivian_aol_longitudinal = false;
  rivian_aol_requested = false;
  rivian_aol_drive = false;
  rivian_aol_eps_ok = false;
  rivian_aol_toi_ok = false;
  rivian_aol_feature = 0U;
  rivian_aol_grant_ts = 0U;
}

static void rivian_aol_request(uint8_t mask) {
  rivian_aol_requested |= mask != 0U;
}

static uint8_t rivian_aol_request_mask(void) {
  const uint32_t now = microsecond_timer_get();
  const uint32_t grant_age = safety_get_ts_elapsed(now, rivian_aol_grant_ts);
  const bool request_expired = rivian_aol_requested &&
    (safety_get_ts_elapsed(now, aol_host_request_ts) > AOL_HOST_REQUEST_TIMEOUT_US);
  const bool pending_expired = rivian_aol_pending && (grant_age > AOL_HOST_REQUEST_TIMEOUT_US);
  uint8_t request = 0U;
  if (relay_malfunction || safety_rx_checks_invalid || !aol_rx_healthy() || request_expired || pending_expired ||
      (rivian_aol_session && !heartbeat_engaged)) {
    rivian_aol_clear();
  } else if (heartbeat_engaged) {
    // Pending activation needs a strictly later host request, inside the
    // original physical grant deadline. Neither observation renews that grant.
    const bool joined = !rivian_aol_pending || ((aol_host_axis_mask != 0U) &&
      (safety_get_ts_elapsed(now, aol_host_request_ts) < grant_age));
    if (joined) {
      rivian_aol_pending = false;
      rivian_aol_session = rivian_aol_token;
      request = aol_host_axis_mask;
    }
  } else {
  }
  return request;
}

static uint8_t rivian_aol_permission(void) {
  const uint8_t request = rivian_aol_request_mask();
  uint8_t permission = 0U;
  if (acc_main_on && rivian_aol_drive && rivian_aol_eps_ok && rivian_aol_toi_ok) {
    if (rivian_aol_token && !rivian_aol_pending) {
      permission = request & 1U;
    }
    if (rivian_aol_longitudinal && controls_allowed && !brake_pressed && !gas_pressed) {
      permission |= request & 2U;
    }
  }
  return permission;
}

static void rivian_aol_observe(const CANPacket_t *msg) {
  if (rivian_aol_enabled) {
    if (msg_matches(msg, 0x150U, 0U)) {
      const uint8_t gear = msg->data[2] & 0xFU;
      rivian_aol_drive = gear == 4U;
      if ((gear == 0U) || (gear == 1U)) {
        rivian_aol_clear();
      }
    }
    if (msg_matches(msg, 0x380U, 0U)) {
      rivian_aol_toi_ok = (msg->data[4] & 0x28U) == 0U;
    }
    if (msg_matches(msg, 0x390U, 0U)) {
      const uint8_t state = (msg->data[4] >> 1U) & 7U;
      const uint8_t error = msg->data[4] >> 4U;
      rivian_aol_eps_ok = (error == 0U) && (state <= 3U);
      if (state == 4U) {
        rivian_aol_clear();
      }
    }
    if (msg_matches(msg, 0x100U, 2U)) {
      const uint8_t feature = msg->data[2] >> 5U;
      if ((feature == 1U) && (rivian_aol_feature != 1U) && controls_allowed && rivian_aol_drive &&
          !brake_pressed && !gas_pressed && rivian_aol_eps_ok && rivian_aol_toi_ok) {
        rivian_aol_token = true;
        rivian_aol_pending = true;
        rivian_aol_grant_ts = microsecond_timer_get();
      }
      rivian_aol_feature = feature;
      acc_main_on = (feature == 0U) || (feature == 1U);
      if (!acc_main_on) {
        rivian_aol_clear();
      }
    }
  }
}
