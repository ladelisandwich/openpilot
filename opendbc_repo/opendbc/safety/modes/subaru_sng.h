#pragma once

// Private candidate. Only a current physical frame can authorize one camera copy.
typedef struct {
  bool seen;
  bool valid;
  bool used;
  uint8_t counter;
  uint8_t data[8];
  uint32_t timestamp;
} SubaruSngSource;

static bool subaru_sng_enabled = false;
static SubaruSngSource subaru_sng_throttle;
static SubaruSngSource subaru_sng_brake;
static SubaruSngSource subaru_sng_gear;
static SubaruSngSource subaru_sng_lead;
static bool subaru_sng_throttle_lease = false;
static bool subaru_sng_brake_lease = false;
static uint32_t subaru_sng_throttle_tx_ts = 0U;
static uint32_t subaru_sng_brake_tx_ts = 0U;
static bool subaru_sng_departure = false;
static bool subaru_sng_started = false;
static uint32_t subaru_sng_departure_ts = 0U;
static uint8_t subaru_sng_resume_count = 0U;
static bool subaru_sng_standstill_seen = false;
static uint32_t subaru_sng_standstill_ts = 0U;
static bool subaru_sng_pulse_seen = false;
static uint32_t subaru_sng_pulse_ts = 0U;

static void subaru_sng_reset(bool enabled) {
  const SubaruSngSource empty = {0};
  subaru_sng_enabled = enabled;
  subaru_sng_throttle = empty;
  subaru_sng_brake = empty;
  subaru_sng_gear = empty;
  subaru_sng_lead = empty;
  subaru_sng_throttle_lease = false;
  subaru_sng_brake_lease = false;
  subaru_sng_throttle_tx_ts = 0U;
  subaru_sng_brake_tx_ts = 0U;
  subaru_sng_departure = false;
  subaru_sng_started = false;
  subaru_sng_departure_ts = 0U;
  subaru_sng_resume_count = 0U;
  subaru_sng_standstill_seen = false;
  subaru_sng_standstill_ts = 0U;
  subaru_sng_pulse_seen = false;
  subaru_sng_pulse_ts = 0U;
}

static bool subaru_sng_source(SubaruSngSource *source, const CANPacket_t *msg) {
  const uint8_t counter = msg->data[1] & 15U;
  bool advanced = false;
  if ((GET_LEN(msg) != 8U) || !subaru_software_checksum(msg)) {
    source->seen = false;
    source->valid = false;
  } else if (!source->seen || (source->counter != counter)) {
    source->valid = source->seen && (counter == ((source->counter + 1U) & 15U));
    source->seen = true;
    source->counter = counter;
    source->timestamp = microsecond_timer_get();
    source->used = false;
    for (int i = 0; i < 8; i++) {
      source->data[i] = msg->data[i];
    }
    advanced = source->valid;
  } else {
    for (int i = 0; i < 8; i++) {
      if (source->data[i] != msg->data[i]) {
        source->valid = false;
      }
    }
  }
  return advanced;
}

static bool subaru_sng_fresh(const SubaruSngSource *source, uint32_t age) {
  return source->valid && (safety_get_ts_elapsed(microsecond_timer_get(), source->timestamp) <= age);
}

static bool subaru_sng_ready(void) {
  return controls_allowed && subaru_software_main && !subaru_software_fault &&
    !vehicle_moving && !brake_pressed && !gas_pressed && !relay_malfunction &&
    !safety_rx_checks_invalid && aol_rx_healthy() &&
    subaru_sng_fresh(&subaru_sng_throttle, 20000U) && (subaru_sng_throttle.data[4] == 0U) &&
    subaru_sng_fresh(&subaru_sng_brake, 40000U) &&
    ((((uint32_t)subaru_sng_brake.data[4] >> 4U) | ((uint32_t)subaru_sng_brake.data[5] << 4U)) == 0U) &&
    subaru_sng_fresh(&subaru_sng_gear, 100000U) && (subaru_sng_gear.data[3] == 121U) &&
    subaru_sng_fresh(&subaru_sng_lead, 100000U) && ((subaru_sng_lead.data[4] & 1U) != 0U) &&
    ((subaru_sng_lead.data[1] & 0x80U) == 0U);
}

static void subaru_sng_rx(const CANPacket_t *msg) {
  if (subaru_sng_enabled) {
    if ((msg->addr == 0x13AU) && (msg->bus == 0U)) {
      if (vehicle_moving) {
        subaru_sng_standstill_seen = false;
      } else if (!subaru_sng_standstill_seen) {
        subaru_sng_standstill_seen = true;
        subaru_sng_standstill_ts = microsecond_timer_get();
      } else {
      }
    }
    if (msg->bus == 0U) {
      if (msg->addr == 0x40U) {
        (void)subaru_sng_source(&subaru_sng_throttle, msg);
      } else if (msg->addr == 0x139U) {
        (void)subaru_sng_source(&subaru_sng_brake, msg);
      } else if (msg->addr == 0x48U) {
        (void)subaru_sng_source(&subaru_sng_gear, msg);
      } else {
      }
    }
    if ((msg->bus == 2U) && (msg->addr == 0x221U)) {
      const bool previous_valid = subaru_sng_lead.valid;
      const uint8_t previous_distance = subaru_sng_lead.data[5];
      const bool advanced = subaru_sng_source(&subaru_sng_lead, msg);
      const uint8_t distance = subaru_sng_lead.data[5];
      if (advanced && previous_valid && !subaru_sng_started && subaru_sng_ready() &&
          (distance > previous_distance) && (distance > 153U) && (distance < 230U)) {
        subaru_sng_departure = true;
        subaru_sng_departure_ts = microsecond_timer_get();
        subaru_sng_resume_count = 0U;
      }
    }
    if (!subaru_sng_ready()) {
      subaru_sng_departure = false;
      subaru_sng_started = false;
      subaru_sng_resume_count = 0U;
    }
  }
}

static bool subaru_sng_tx(const CANPacket_t *msg) {
  const bool throttle = msg->addr == 0x40U;
  SubaruSngSource *source = throttle ? &subaru_sng_throttle : &subaru_sng_brake;
  const uint32_t now = microsecond_timer_get();
  bool allowed = subaru_sng_enabled && aol_rx_healthy() && (msg->bus == 2U) && (GET_LEN(msg) == 8U) &&
    subaru_software_checksum(msg) && subaru_sng_fresh(source, throttle ? 20000U : 40000U) && !source->used &&
    ((msg->data[1] & 15U) == ((source->counter + 1U) & 15U));
  bool changed;
  if (throttle) {
    changed = msg->data[4] != source->data[4];
  } else {
    changed = (msg->data[2] != source->data[2]) || ((msg->data[3] & 15U) != (source->data[3] & 15U));
  }
  for (int i = 1; i < 8; i++) {
    uint8_t mask = (i == 1) ? 0xF0U : 0xFFU;
    if (throttle && (i == 4)) {
      mask = 0U;
    } else if (!throttle && (i == 2)) {
      mask = 0U;
    } else if (!throttle && (i == 3)) {
      mask = 0xF0U;
    } else {
    }
    allowed = allowed && ((msg->data[i] & mask) == (source->data[i] & mask));
  }
  if (changed) {
    allowed = allowed && subaru_sng_ready();
    if (throttle) {
      allowed = allowed && (source->data[4] == 0U) && (msg->data[4] == 5U) &&
        subaru_sng_departure && (subaru_sng_resume_count < 15U) &&
        (subaru_sng_lead.data[5] > 153U) && (subaru_sng_lead.data[5] < 230U) &&
        (safety_get_ts_elapsed(now, subaru_sng_departure_ts) <= (subaru_sng_started ? 150000U : 100000U));
    } else {
      const uint32_t age = safety_get_ts_elapsed(now, subaru_sng_pulse_ts);
      const uint32_t physical_speed = ((uint32_t)(source->data[3] & 15U) << 8U) | source->data[2];
      allowed = allowed && subaru_sng_standstill_seen &&
        (safety_get_ts_elapsed(now, subaru_sng_standstill_ts) >= 500000U) && (physical_speed <= 53U) && (msg->data[2] == 53U) && ((msg->data[3] & 15U) == 0U) &&
        (!subaru_sng_pulse_seen || (age <= 50000U) || (age >= 550000U));
    }
  }
  if (allowed) {
    source->used = true;
    if (throttle) {
      subaru_sng_throttle_lease = true;
      subaru_sng_throttle_tx_ts = now;
      if (changed) {
        if (!subaru_sng_started) {
          subaru_sng_started = true;
          subaru_sng_departure_ts = now;
        }
        subaru_sng_resume_count++;
      }
    } else {
      subaru_sng_brake_lease = true;
      subaru_sng_brake_tx_ts = now;
      if (changed && (!subaru_sng_pulse_seen ||
          (safety_get_ts_elapsed(now, subaru_sng_pulse_ts) >= 550000U))) {
        subaru_sng_pulse_seen = true;
        subaru_sng_pulse_ts = now;
      }
    }
  }
  return allowed;
}

static bool subaru_sng_fwd(int bus, int addr) {
  const uint32_t now = microsecond_timer_get();
  const bool driver_brake = brake_pressed ||
    ((((uint32_t)subaru_sng_brake.data[4] >> 4U) | ((uint32_t)subaru_sng_brake.data[5] << 4U)) != 0U);
  return subaru_sng_enabled && aol_rx_healthy() && (bus == 0) &&
    (((addr == 0x40) && !gas_pressed && (subaru_sng_throttle.data[4] == 0U) &&
      subaru_sng_throttle.valid && subaru_sng_throttle_lease &&
      (safety_get_ts_elapsed(now, subaru_sng_throttle_tx_ts) < 20000U)) ||
     ((addr == 0x139) && !driver_brake && subaru_sng_brake.valid && subaru_sng_brake_lease &&
      (safety_get_ts_elapsed(now, subaru_sng_brake_tx_ts) < 40000U)));
}
