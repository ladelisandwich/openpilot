#pragma once

// Optional stock-ACC owner; existing steering limits and stock safety words stay.
static bool chrysler_aol_enabled;
static bool chrysler_aol_set_guard;
static bool chrysler_aol_set_seen;
static bool chrysler_aol_armed;
static bool chrysler_aol_cancelled;
static bool chrysler_aol_main;
static bool chrysler_aol_drive;
static bool chrysler_aol_eps_temporary;
static bool chrysler_aol_eps_permanent;
static bool chrysler_aol_initialized[3];
static bool chrysler_aol_seen[3];  // gear, Ram EPS3, physical buttons
static uint8_t chrysler_aol_counter[3];
static uint32_t chrysler_aol_ts[3];
static bool chrysler_aol_button_neutral;
static bool chrysler_aol_button_held;
static bool chrysler_aol_gesture_seen;
static uint32_t chrysler_aol_gesture_ts;

static void chrysler_aol_reset(void) {
  chrysler_aol_armed = false;
  chrysler_aol_set_seen = false;
  chrysler_aol_cancelled = false;
  chrysler_aol_main = false;
  chrysler_aol_drive = false;
  chrysler_aol_eps_temporary = false;
  chrysler_aol_eps_permanent = false;
  chrysler_aol_button_neutral = false;
  chrysler_aol_button_held = false;
  chrysler_aol_gesture_seen = false;
  chrysler_aol_gesture_ts = 0U;
  for (unsigned int i = 0U; i < 3U; i++) {
    chrysler_aol_initialized[i] = false;
    chrysler_aol_seen[i] = false;
    chrysler_aol_counter[i] = 0U;
    chrysler_aol_ts[i] = 0U;
  }
}

static bool chrysler_aol_current(void) {
  const uint32_t now = microsecond_timer_get();
  return chrysler_aol_seen[0] && chrysler_aol_main && chrysler_aol_drive &&
    !chrysler_aol_eps_temporary && !chrysler_aol_eps_permanent && aol_rx_healthy() &&
    (safety_get_ts_elapsed(now, chrysler_aol_ts[0]) <= 100000U) &&
    ((chrysler_platform != CHRYSLER_RAM_DT) || (chrysler_aol_seen[1] &&
      (safety_get_ts_elapsed(now, chrysler_aol_ts[1]) <= 100000U)));
}

static uint8_t chrysler_aol_request(void) {
  uint8_t request = 0U;
  const uint32_t now = microsecond_timer_get();
  if (!heartbeat_engaged || relay_malfunction) { chrysler_aol_armed = false; }
  if (heartbeat_engaged && !relay_malfunction && aol_rx_healthy() &&
      (safety_get_ts_elapsed(now, aol_host_request_ts) <= AOL_HOST_REQUEST_TIMEOUT_US)) {
    // Stock ACC remains the only longitudinal owner.
    request = aol_host_axis_mask & 1U;
    const bool gesture = chrysler_aol_gesture_seen &&
      (safety_get_ts_elapsed(now, chrysler_aol_gesture_ts) <= AOL_HOST_REQUEST_TIMEOUT_US);
    if ((request != 0U) && chrysler_aol_current() &&
        ((controls_allowed && !chrysler_aol_cancelled) || (gesture && (!chrysler_aol_set_guard || chrysler_aol_set_seen)))) {
      if (controls_allowed) { chrysler_aol_set_seen = true; }
      chrysler_aol_cancelled = false;
      chrysler_aol_armed = true;
      chrysler_aol_gesture_seen = false;
    }
  }
  return request;
}

static uint8_t chrysler_aol_permission(void) {
  const uint8_t request = chrysler_aol_request();
  return (!chrysler_aol_cancelled && chrysler_aol_current() &&
          (controls_allowed || chrysler_aol_armed || !chrysler_aol_set_guard)) ? request : 0U;
}

static void chrysler_aol_rx(const CANPacket_t *msg) {
  if (chrysler_aol_enabled) {
    const unsigned int cruise_bus = (chrysler_platform == CHRYSLER_RAM_DT) ? 2U : 0U;
    if (msg_matches(msg, CHRYSLER_ADDR(DAS_3), cruise_bus, 8U)) {
      if (GET_BIT(msg, 21U) && !cruise_engaged_prev) { chrysler_aol_cancelled = false; }
      chrysler_aol_main = GET_BIT(msg, 20U) && ((msg->data[5] & 0xC0U) == 0U);
      if (!chrysler_aol_main) {
        chrysler_aol_set_seen = false;
        chrysler_aol_armed = false;
        chrysler_aol_gesture_seen = false;
      }
    }
    if ((chrysler_platform == CHRYSLER_PACIFICA) && msg_matches(msg, CHRYSLER_EPS_2, 0U, 8U)) {
      chrysler_aol_eps_temporary = GET_BIT(msg, 38U);
      chrysler_aol_eps_permanent = (msg->data[2] >> 4) == 4U;
      if (chrysler_aol_eps_permanent) {
        chrysler_aol_set_seen = false;
        chrysler_aol_cancelled = true;
        chrysler_aol_armed = false;
        chrysler_aol_gesture_seen = false;
      }
    }
  }
}

static bool chrysler_aol_advance(unsigned int source, uint8_t counter, uint32_t now) {
  const bool advanced = !chrysler_aol_initialized[source] ||
    (counter == ((chrysler_aol_counter[source] + 1U) & 0xFU));
  chrysler_aol_initialized[source] = true;
  chrysler_aol_seen[source] = advanced;
  if (advanced) { chrysler_aol_ts[source] = now; }
  chrysler_aol_counter[source] = counter;
  return advanced;
}

static void chrysler_aol_optional_rx(const CANPacket_t *msg) {
  if (chrysler_aol_enabled) {
    const uint32_t now = microsecond_timer_get();
    const bool ram = chrysler_platform == CHRYSLER_RAM_DT;
    if (msg_matches(msg, ram ? 147U : 746U, 0U)) {
      // Classic GEAR_CHECKSUM is not CHECKSUM in the upstream DBC/parser.
      // Do not guess its algorithm; use the existing decoded gear only as a
      // restrictive gate, with the original advancing counter and bounded age.
      const bool valid = (GET_LEN(msg) == (ram ? 8U : 5U)) &&
        (!ram || (chrysler_get_checksum(msg) == chrysler_compute_checksum(msg)));
      if (!valid) { chrysler_aol_seen[0] = false; }
      else {
        const uint8_t counter = ram ? (msg->data[6] >> 4) : (msg->data[3] >> 4);
        const bool advanced = chrysler_aol_advance(0U, counter, now);
        const unsigned int gear = ram ? ((msg->data[0] >> 2) & 7U) : (msg->data[0] & 7U);
        const bool drive = (gear == 4U) || (!ram && (gear == 5U));
        if (advanced || !drive) { chrysler_aol_drive = drive; }
      }
    }
    if (ram && msg_matches(msg, 164U, 0U)) {
      const bool valid = (GET_LEN(msg) == 8U) &&
        (chrysler_get_checksum(msg) == chrysler_compute_checksum(msg));
      if (!valid) { chrysler_aol_seen[1] = false; }
      else {
        const bool advanced = chrysler_aol_advance(1U, msg->data[6] >> 4, now);
        const bool fault = GET_BIT(msg, 34U);
        if (advanced || fault) { chrysler_aol_eps_temporary = fault; }
      }
    }
    if (msg_matches(msg, CHRYSLER_ADDR(CRUISE_BUTTONS), 0U)) {
      const bool valid = (GET_LEN(msg) == 3U) &&
        (chrysler_get_checksum(msg) == chrysler_compute_checksum(msg));
      if (!valid) {
        chrysler_aol_cancelled = true;
        chrysler_aol_armed = false;
        chrysler_aol_gesture_seen = false;
        chrysler_aol_seen[2] = false;
        chrysler_aol_button_neutral = false;
      } else {
        const bool advanced = chrysler_aol_advance(2U, msg->data[1] >> 4, now);
        const bool press = (msg->data[0] & 0xDEU) != 0U;
        if (GET_BIT(msg, 0U)) {
          chrysler_aol_cancelled = true;
          chrysler_aol_armed = false;
          chrysler_aol_gesture_seen = false;
        } else if (advanced && !press) {
          chrysler_aol_button_neutral = true;
        } else if (advanced && press && !chrysler_aol_button_held && chrysler_aol_button_neutral) {
          chrysler_aol_gesture_seen = true;
          chrysler_aol_gesture_ts = now;
          chrysler_aol_button_neutral = false;
        } else {
        }
        chrysler_aol_button_held = press;
      }
    }

  }
}

static void chrysler_aol_configure(uint16_t param) {
  chrysler_aol_reset();
  chrysler_aol_set_guard = param == 8U;
  chrysler_aol_enabled = (alternative_experience == 32) && ((param == 0U) || (param == 1U) || (param == 4U) || (param == 8U));
  if (chrysler_aol_enabled) {
    static const AolSafetyPolicy policy = {
      .reset = chrysler_aol_reset,
      .request_mask = chrysler_aol_request,
      .permission_mask = chrysler_aol_permission,
      .rx_invalid = chrysler_aol_reset,
    };
    aol_policy = &policy;
  }
}
