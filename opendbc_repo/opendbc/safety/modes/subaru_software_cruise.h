#pragma once

// Stock ACC owns engagement and all pedal actuation. This exact optional owner
// can emit only Dom's Resume/Set pair from one current neutral physical frame.
static bool subaru_software_enabled = false;
static bool subaru_software_main = false;
static bool subaru_software_fault = false;
static bool subaru_software_drive = false;
static bool subaru_software_gear_seen = false;
static bool subaru_software_gear_valid = false;
static uint8_t subaru_software_gear_counter = 0U;
static uint32_t subaru_software_gear_ts = 0U;
static bool subaru_software_buttons_seen = false;
static bool subaru_software_buttons_valid = false;
static uint8_t subaru_software_buttons_counter = 0U;
static uint8_t subaru_software_buttons[8];
static uint32_t subaru_software_buttons_ts = 0U;
static uint8_t subaru_software_copies = 0U;
static uint8_t subaru_software_button = 0U;
static bool subaru_software_burst_seen = false;
static uint32_t subaru_software_burst_ts = 0U;

static void subaru_software_reset(bool enabled) {
  subaru_software_enabled = enabled;
  subaru_software_main = false;
  subaru_software_fault = false;
  subaru_software_drive = false;
  subaru_software_gear_seen = false;
  subaru_software_gear_valid = false;
  subaru_software_gear_counter = 0U;
  subaru_software_gear_ts = 0U;
  subaru_software_buttons_seen = false;
  subaru_software_buttons_valid = false;
  subaru_software_buttons_counter = 0U;
  subaru_software_buttons_ts = 0U;
  subaru_software_copies = 0U;
  subaru_software_button = 0U;
  subaru_software_burst_seen = false;
  subaru_software_burst_ts = 0U;
}

static bool subaru_software_checksum(const CANPacket_t *msg) {
  uint8_t checksum = (uint8_t)msg->addr + (uint8_t)(msg->addr >> 8U);
  for (int i = 1; i < 8; i++) {
    checksum += msg->data[i];
  }
  return msg->data[0] == checksum;
}

static void subaru_software_optional_rx(const CANPacket_t *msg) {
  if (subaru_software_enabled && ((msg->addr == 0x48U) || (msg->addr == 0x146U))) {
    if (msg->bus == 0U) {
      const bool gear = msg->addr == 0x48U;
      const bool seen = gear ? subaru_software_gear_seen : subaru_software_buttons_seen;
      const uint8_t previous = gear ? subaru_software_gear_counter : subaru_software_buttons_counter;
      const uint8_t counter = msg->data[1] & 15U;
      const bool valid = (GET_LEN(msg) == 8U) && subaru_software_checksum(msg);
      const bool progress = !seen || (counter != previous);
      const bool sequential = valid && seen && (counter == ((previous + 1U) & 15U));
      if (gear) {
        if (!valid) {
          subaru_software_gear_seen = false;
          subaru_software_gear_valid = false;
        } else if (progress) {
          subaru_software_gear_seen = true;
          subaru_software_gear_valid = sequential;
          subaru_software_gear_counter = counter;
          subaru_software_gear_ts = microsecond_timer_get();
          subaru_software_drive = msg->data[3] == 121U;
        } else {
          if (((msg->data[3] == 121U) && !subaru_software_drive) ||
              ((msg->data[3] != 121U) && subaru_software_drive)) {
            subaru_software_gear_valid = false;
          }
        }
      } else {
        if (!valid) {
          subaru_software_buttons_seen = false;
          subaru_software_buttons_valid = false;
        } else if (progress) {
          subaru_software_buttons_seen = true;
          subaru_software_buttons_valid = sequential;
          subaru_software_buttons_counter = counter;
          subaru_software_buttons_ts = microsecond_timer_get();
          subaru_software_copies = 0U;
          for (int i = 0; i < 8; i++) {
            subaru_software_buttons[i] = msg->data[i];
          }
        } else {
          for (int i = 0; i < 8; i++) {
            if (subaru_software_buttons[i] != msg->data[i]) {
              subaru_software_buttons_valid = false;
            }
          }
        }
      }
    }
  }
}

static bool subaru_software_tx(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  const uint8_t button = msg->data[5] & 0x1CU;
  const uint8_t copy = subaru_software_copies;
  bool allowed = subaru_software_enabled && controls_allowed && subaru_software_main && !subaru_software_fault &&
    !brake_pressed && !gas_pressed && vehicle_moving && !relay_malfunction && !safety_rx_checks_invalid && aol_rx_healthy() &&
    subaru_software_gear_valid && subaru_software_drive &&
    (safety_get_ts_elapsed(now, subaru_software_gear_ts) <= 100000U) &&
    subaru_software_buttons_valid && ((subaru_software_buttons[5] & 0x1CU) == 0U) &&
    (safety_get_ts_elapsed(now, subaru_software_buttons_ts) <= 100000U) &&
    (msg->bus == 0U) && (GET_LEN(msg) == 8U) && subaru_software_checksum(msg) &&
    ((button == 8U) || (button == 16U)) && (copy < 2U) &&
    ((msg->data[1] & 15U) == ((subaru_software_buttons_counter + copy + 1U) & 15U));
  if (copy == 0U) {
    allowed = allowed && (!subaru_software_burst_seen ||
      (safety_get_ts_elapsed(now, subaru_software_burst_ts) >= 100000U));
  } else {
    allowed = allowed && (button == subaru_software_button) && (safety_get_ts_elapsed(now, subaru_software_burst_ts) <= 10000U);
  }
  for (int i = 1; i < 8; i++) {
    const uint8_t mask = (i == 1) ? 0xF0U : ((i == 5) ? 0xE3U : 0xFFU);
    allowed = allowed && ((msg->data[i] & mask) == (subaru_software_buttons[i] & mask));
  }
  if (allowed) {
    subaru_software_copies++;
    subaru_software_button = button;
    subaru_software_burst_seen = true;
    if (copy == 0U) {
      subaru_software_burst_ts = now;
    }
  }
  return allowed;
}
