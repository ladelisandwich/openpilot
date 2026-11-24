#pragma once

static bool volkswagen_meb_stock_acc = false;

static bool vw_meb_aol_enabled = false;
static bool vw_meb_aol_alt_gear = false;
static bool vw_meb_aol_initial_main = true;
static bool vw_meb_aol_token = false;
static bool vw_meb_aol_long_armed = false;
static bool vw_meb_aol_cruise_engaged = false;
static bool vw_meb_aol_main_neutral = false;
static bool vw_meb_aol_requested = false;
static bool vw_meb_aol_gear_drive = false;
static bool vw_meb_aol_eps_ready = false;
static bool vw_meb_aol_eps_initialized = false;
static uint8_t vw_meb_aol_seen = 0U;
static uint8_t vw_meb_aol_buttons = 0U;
static bool vw_meb_aol_counter_seen = false;
static uint8_t vw_meb_aol_gear_counter = 0U;
static uint32_t vw_meb_aol_main_ts = 0U;
static uint32_t vw_meb_aol_gear_ts = 0U;
static uint32_t vw_meb_aol_eps_ts = 0U;
static uint32_t vw_meb_aol_eps_init_ts = 0U;

static void vw_meb_aol_clear(void) {
  vw_meb_aol_token = false;
  vw_meb_aol_long_armed = false;
  vw_meb_aol_main_neutral = false;
  vw_meb_aol_buttons = 0U;
}

static void vw_meb_aol_reset(void) {
  vw_meb_aol_clear();
  vw_meb_aol_enabled = false;
  vw_meb_aol_alt_gear = false;
  vw_meb_aol_initial_main = true;
  vw_meb_aol_cruise_engaged = false;
  vw_meb_aol_requested = false;
  vw_meb_aol_gear_drive = false;
  vw_meb_aol_eps_ready = false;
  vw_meb_aol_eps_initialized = false;
  vw_meb_aol_seen = 0U;
  vw_meb_aol_counter_seen = false;
  vw_meb_aol_gear_counter = 0U;
  vw_meb_aol_main_ts = 0U;
  vw_meb_aol_gear_ts = 0U;
  vw_meb_aol_eps_ts = 0U;
  vw_meb_aol_eps_init_ts = 0U;
}

static bool vw_meb_aol_current(void) {
  const uint32_t now = microsecond_timer_get();
  return (vw_meb_aol_seen == 7U) &&
    (safety_get_ts_elapsed(now, vw_meb_aol_main_ts) <= 300000U) &&
    (safety_get_ts_elapsed(now, vw_meb_aol_gear_ts) <= 300000U) &&
    (safety_get_ts_elapsed(now, vw_meb_aol_eps_ts) <= 100000U);
}

static void vw_meb_aol_request(uint8_t mask) {
  vw_meb_aol_requested |= mask != 0U;
}

static uint8_t vw_meb_aol_request_mask(void) {
  const bool expired = vw_meb_aol_requested &&
    (safety_get_ts_elapsed(microsecond_timer_get(), aol_host_request_ts) > AOL_HOST_REQUEST_TIMEOUT_US);
  uint8_t request = 0U;
  if (!vw_meb_aol_enabled || relay_malfunction || safety_rx_checks_invalid || !heartbeat_engaged ||
      !aol_rx_healthy() || !vw_meb_aol_current() || expired) {
    vw_meb_aol_clear();
  } else {
    request = aol_host_axis_mask;
  }
  return request;
}

static uint8_t vw_meb_aol_permission(void) {
  const uint8_t request = vw_meb_aol_request_mask();
  uint8_t permission = 0U;
  if (acc_main_on && vw_meb_aol_gear_drive && vw_meb_aol_eps_ready && vw_meb_aol_token) {
    permission = request & 1U;
  }
  if (!volkswagen_meb_stock_acc && vw_meb_aol_long_armed && controls_allowed && acc_main_on &&
      vw_meb_aol_gear_drive && !brake_pressed && !gas_pressed) {
    permission |= request & 2U;
  }
  return permission;
}

static void vw_meb_aol_optional_rx(const CANPacket_t *msg) {
  if (vw_meb_aol_enabled) {
    unsigned int gear = 0U;
    bool accepted = false;
    if (vw_meb_aol_alt_gear && msg_matches(msg, 0x3DCU, 0U, 8U)) {
      gear = msg->data[5] & 15U;
      accepted = true;  // Gateway_73 has no checksum/counter in the current DBC.
    } else if (!vw_meb_aol_alt_gear && msg_matches(msg, 0xADU, 0U, 8U)) {
      const uint8_t counter = volkswagen_mxb_get_counter(msg);
      const uint8_t salt[16] = {0x3FU, 0x69U, 0x39U, 0xDCU, 0x94U, 0xF9U, 0x14U, 0x64U,
                                0xD8U, 0x6AU, 0x34U, 0xCEU, 0xA2U, 0x55U, 0xB5U, 0x2CU};
      const bool checksum = volkswagen_compute_crc(msg, 8, salt[counter]) == msg->data[0];
      const bool pair = vw_meb_aol_counter_seen &&
        (safety_get_ts_elapsed(microsecond_timer_get(), vw_meb_aol_gear_ts) <= 300000U) &&
        (counter == ((vw_meb_aol_gear_counter + 1U) & 15U));
      vw_meb_aol_counter_seen = checksum;
      vw_meb_aol_gear_counter = counter;
      if (checksum) {
        vw_meb_aol_gear_ts = microsecond_timer_get();
      }
      if (checksum && pair) {
        gear = (msg->data[5] >> 2) & 15U;
        accepted = true;
      } else {
        vw_meb_aol_seen &= 5U;
        vw_meb_aol_gear_drive = false;
      }
    } else {
    }
    if (accepted) {
      vw_meb_aol_seen |= 2U;
      vw_meb_aol_gear_ts = microsecond_timer_get();
      vw_meb_aol_gear_drive = (gear == 8U) || (gear == 9U) || (gear == 10U) || (gear == 13U) || (gear == 14U);
    }
    if (!vw_meb_aol_gear_drive) {
      vw_meb_aol_long_armed = false;
    }
  }
}

static void vw_meb_aol_rx(const CANPacket_t *msg) {
  if (vw_meb_aol_enabled) {
    if (brake_pressed || !vw_meb_aol_gear_drive) {
      vw_meb_aol_long_armed = false;
    }
    if (msg_matches(msg, MSG_QFK_01, 0U, 32U)) {
      const unsigned int eps = (msg->data[2] >> 4) & 7U;
      if ((vw_meb_aol_seen & 4U) == 0U) {
        vw_meb_aol_eps_init_ts = microsecond_timer_get();
      }
      vw_meb_aol_seen |= 4U;
      vw_meb_aol_eps_ts = microsecond_timer_get();
      vw_meb_aol_eps_ready = (eps == 2U) || (eps == 4U);
      // Match CarState's existing 600-frame initialization window; no permission is granted while EPS is unavailable.
      vw_meb_aol_eps_initialized |= (eps == 0U) || (eps == 2U) || (eps == 4U) ||
        (safety_get_ts_elapsed(microsecond_timer_get(), vw_meb_aol_eps_init_ts) > 6000000U);
      if (((eps == 0U) && vw_meb_aol_gear_drive) ||
          (vw_meb_aol_eps_initialized && ((eps == 3U) || (eps == 6U)))) {
        vw_meb_aol_initial_main = false;
        vw_meb_aol_clear();
      }
    } else if (msg_matches(msg, MSG_Motor_51, 0U)) {
      const unsigned int status = msg->data[11] & 7U;
      const bool engaged = (status == 3U) || (status == 4U) || (status == 5U);
      const bool edge = engaged && !vw_meb_aol_cruise_engaged;
      vw_meb_aol_cruise_engaged = engaged;
      vw_meb_aol_seen |= 1U;
      vw_meb_aol_main_ts = microsecond_timer_get();
      if (!heartbeat_engaged || !aol_rx_healthy() || !vw_meb_aol_current() || safety_rx_checks_invalid) {
        vw_meb_aol_clear();
      } else if (!acc_main_on) {
        vw_meb_aol_initial_main = false;
        vw_meb_aol_clear();
        vw_meb_aol_main_neutral = status < 2U;
      } else if (vw_meb_aol_gear_drive && vw_meb_aol_eps_ready && !brake_pressed &&
                 (vw_meb_aol_initial_main || vw_meb_aol_main_neutral ||
                  (volkswagen_meb_stock_acc && edge && controls_allowed))) {
        vw_meb_aol_initial_main = false;
        vw_meb_aol_main_neutral = false;
        vw_meb_aol_token = true;
      } else {
      }
    } else if (msg_matches(msg, MSG_GRA_ACC_01, 0U, 8U)) {
      if (GET_BIT(msg, 13U)) {
        vw_meb_aol_initial_main = false;
        vw_meb_aol_clear();
      } else if (!volkswagen_meb_stock_acc) {
        const bool healthy = heartbeat_engaged && aol_rx_healthy() && vw_meb_aol_current() && !safety_rx_checks_invalid;
        const uint8_t buttons = (GET_BIT(msg, 16U) ? 1U : 0U) | (GET_BIT(msg, 19U) ? 2U : 0U);
        if (!healthy) {
          vw_meb_aol_clear();
        } else if (buttons == 0U) {
          if (((vw_meb_aol_buttons & 3U) != 0U) && controls_allowed && acc_main_on &&
              vw_meb_aol_gear_drive && vw_meb_aol_eps_ready && !brake_pressed) {
            vw_meb_aol_token = true;
            vw_meb_aol_long_armed = true;
          }
          vw_meb_aol_buttons = 4U;
        } else if ((vw_meb_aol_buttons & 4U) != 0U) {
          vw_meb_aol_buttons = 4U | buttons;
        } else {
        }
      } else {
      }
    } else {
    }
  }
}
