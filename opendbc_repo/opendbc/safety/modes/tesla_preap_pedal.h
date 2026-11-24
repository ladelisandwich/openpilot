#pragma once
#include <stdint.h>
#include <stdbool.h>

static bool tesla_preap_pedal_finite(float value) {
  union { float value; uint32_t bits; } word = {.value = value};
  return (word.bits & 0x7F800000U) != 0x7F800000U;
}
static float tesla_preap_pedal_abs(float value) { return (value < 0.0F) ? -value : value; }

// Fork-local startup USB envelope. No CAN packet configures these values.
#define TESLA_PREAP_PEDAL_CONFIG_WORDS 24U
// The startup capsule is shared by native mode admission and its USB owner.
typedef struct {
  uint16_t staged[24];
  uint16_t active_words[24];
  unsigned int next_word;
  bool staged_ready;
} TeslaPreapPedalStartupState;
static TeslaPreapPedalStartupState tesla_preap_pedal_startup = {0};
static bool tesla_preap_pedal_configured = false;
static float tesla_preap_pedal_factor = 0.0F;
static float tesla_preap_pedal_zero = 0.0F;
static unsigned int tesla_preap_pedal_bus = 0U;
static bool tesla_preap_pedal_feedback_seen = false;
static bool tesla_preap_pedal_feedback_healthy = false;
static uint32_t tesla_preap_pedal_feedback_ts = 0U;
static float tesla_preap_pedal_feedback_di = 0.0F;
static unsigned int tesla_preap_pedal_feedback_raw_one = 0U;
static unsigned int tesla_preap_pedal_feedback_raw_two = 0U;
static bool tesla_preap_pedal_tx_seen = false;
static bool tesla_preap_pedal_tx_enabled = false;
static unsigned int tesla_preap_pedal_last_raw_one = 0U;
static unsigned int tesla_preap_pedal_last_raw_two = 0U;

static float tesla_preap_pedal_word_float(const uint16_t *words) {
  union { uint32_t bits; float value; } decoded;
  decoded.bits = (uint32_t)words[0] | ((uint32_t)words[1] << 16U);
  return decoded.value;
}

static bool tesla_preap_pedal_config_valid(const uint16_t *words) {
  bool hash_present = false;
  for (unsigned int i = 8U; i < TESLA_PREAP_PEDAL_CONFIG_WORDS; i++) { hash_present |= words[i] != 0U; }
  const float factor = tesla_preap_pedal_word_float(&words[4]);
  const float zero = tesla_preap_pedal_word_float(&words[6]);
  bool valid = (words[0] == 1U) && ((words[1] == 0U) || (words[1] == 2U)) &&
    ((words[2] == 1U) || (words[2] == 3U) || (words[2] == 7U)) && (words[3] == 0xD051U) &&
    tesla_preap_pedal_finite(factor) && (tesla_preap_pedal_abs(factor) > 0.000001F) && tesla_preap_pedal_finite(zero) && hash_present;
  if (valid) {
    const float demands[2] = {-5.0F, 90.0F};
    for (unsigned int d = 0U; d < 2U; d++) {
      const float physical = zero + (demands[d] / factor) + 22.85856576F;
      const float raw_one = physical / 0.050796813F;
      const float raw_two = physical / 0.101593626F;
      const unsigned int one = (raw_one < 0.0F) ? 0U : ((raw_one > 65534.0F) ? 65534U : (unsigned int)raw_one);
      const unsigned int two = (raw_two < 0.0F) ? 0U : ((raw_two > 65534.0F) ? 65534U : (unsigned int)raw_two);
      const int difference = (int)one - (2 * (int)two);
      valid &= (difference >= 0) && (difference <= 1);
    }
  }
  return valid;
}

static void tesla_preap_pedal_mode_reset(uint16_t mode) {
  tesla_preap_pedal_configured = false;
  tesla_preap_pedal_feedback_seen = false;
  tesla_preap_pedal_feedback_healthy = false;
  tesla_preap_pedal_tx_seen = false;
  tesla_preap_pedal_tx_enabled = false;
  tesla_preap_pedal_last_raw_one = 0U;
  tesla_preap_pedal_last_raw_two = 0U;
  if (mode != 39U) {
    tesla_preap_pedal_startup.staged_ready = false;
    tesla_preap_pedal_startup.next_word = 0U;
  }
}

static bool tesla_preap_pedal_profile_init(uint16_t param) {
  const bool good = tesla_preap_pedal_startup.staged_ready && (tesla_preap_pedal_startup.staged[2] == param) &&
    tesla_preap_pedal_config_valid(tesla_preap_pedal_startup.staged);
  tesla_preap_pedal_startup.staged_ready = false;  // a startup capsule is consumed exactly once
  if (good) {
    for (unsigned int i = 0U; i < TESLA_PREAP_PEDAL_CONFIG_WORDS; i++) {
      tesla_preap_pedal_startup.active_words[i] = tesla_preap_pedal_startup.staged[i];
    }
    tesla_preap_pedal_factor = tesla_preap_pedal_word_float(&tesla_preap_pedal_startup.active_words[4]);
    tesla_preap_pedal_zero = tesla_preap_pedal_word_float(&tesla_preap_pedal_startup.active_words[6]);
    tesla_preap_pedal_bus = tesla_preap_pedal_startup.active_words[1];
  }
  tesla_preap_pedal_configured = good;
  return good;
}

static float tesla_preap_pedal_di(unsigned int raw, float scale) {
  return (((float)raw * scale) - 22.85856576F - tesla_preap_pedal_zero) * tesla_preap_pedal_factor;
}

static bool tesla_preap_pedal_feedback(const uint8_t *data, unsigned int bus, uint32_t now) {
  bool next = false;
  if (tesla_preap_pedal_configured && (bus == tesla_preap_pedal_bus)) {
    static uint8_t tesla_preap_pedal_feedback_counter = 0U;
    const unsigned int one = ((unsigned int)data[0] << 8U) | data[1];
    const unsigned int two = ((unsigned int)data[2] << 8U) | data[3];
    const uint8_t counter = data[4] & 15U;
    const bool monotonic = !tesla_preap_pedal_feedback_seen || ((uint32_t)(now - tesla_preap_pedal_feedback_ts) > 0U);
    const bool channels = (one < 65535U) && (two < 65535U) &&
      (tesla_preap_pedal_abs(((float)one * 0.050796813F) - ((float)two * 0.101593626F)) <= (2.0F * 0.101593626F));
    const bool valid = ((data[4] >> 4U) == 0U) && channels && monotonic;
    next = tesla_preap_pedal_feedback_seen && valid &&
      (counter == ((tesla_preap_pedal_feedback_counter + 1U) & 15U));
    tesla_preap_pedal_feedback_healthy = next;
    tesla_preap_pedal_feedback_seen = valid;
    tesla_preap_pedal_feedback_counter = counter;
    tesla_preap_pedal_feedback_ts = now;
    const float a = tesla_preap_pedal_di(one, 0.050796813F);
    const float b = tesla_preap_pedal_di(two, 0.101593626F);
    tesla_preap_pedal_feedback_raw_one = one;
    tesla_preap_pedal_feedback_raw_two = two;
    tesla_preap_pedal_feedback_di = (a > b) ? a : b;
  }
  return next;
}

static bool tesla_preap_pedal_feedback_ready(uint32_t now) {
  return tesla_preap_pedal_configured && tesla_preap_pedal_feedback_healthy &&
    ((uint32_t)(now - tesla_preap_pedal_feedback_ts) <= 500000U);
}

static float tesla_preap_pedal_max_di(float speed) {
  const float bp[6] = {0.0F, 5.0F, 12.0F, 20.0F, 30.0F, 40.0F};
  const float values[6] = {50.0F, 58.0F, 66.0F, 74.0F, 82.0F, 90.0F};
  float result = values[5];
  for (unsigned int i = 1U; i < 6U; i++) {
    if (speed <= bp[i]) {
      const float bounded = (speed < 0.0F) ? 0.0F : speed;
      result = values[i - 1U] + ((bounded - bp[i - 1U]) / (bp[i] - bp[i - 1U])) * (values[i] - values[i - 1U]);
      break;
    }
  }
  return result;
}

static bool tesla_preap_pedal_channel_bounds(unsigned int raw, unsigned int previous, float scale, float speed) {
  const float first = (tesla_preap_pedal_zero + (-5.0F / tesla_preap_pedal_factor) + 22.85856576F) / scale;
  const float second = (tesla_preap_pedal_zero + (tesla_preap_pedal_max_di(speed) / tesla_preap_pedal_factor) + 22.85856576F) / scale;
  // Match the working Dom codec's0..65534 clamp, including a valid floor
  // saturation. Config admission separately rejects incompatible channel tops.
  const float bounded_first = (first < 0.0F) ? 0.0F : ((first > 65534.0F) ? 65534.0F : first);
  const float bounded_second = (second < 0.0F) ? 0.0F : ((second > 65534.0F) ? 65534.0F : second);
  const float low = (bounded_first < bounded_second) ? bounded_first : bounded_second;
  const float high = (bounded_first > bounded_second) ? bounded_first : bounded_second;
  // One ADC count covers Float32-vs-host quotient rounding without a DI-based
  // tolerance growing with calibration factor.
  const unsigned int floor_low = (unsigned int)low;
  const unsigned int lower = (floor_low > 0U) ? (floor_low - 1U) : 0U;
  const unsigned int upper = ((unsigned int)high < 65534U) ? ((unsigned int)high + 1U) : 65534U;
  const float unit = tesla_preap_pedal_abs(tesla_preap_pedal_factor) * scale;
  const float up = 5.0F / unit;
  const float down = 2.5F / unit;
  const unsigned int rate_up = (up >= 65534.0F) ? 65534U : ((unsigned int)up + 1U);
  const unsigned int rate_down = (down >= 65534.0F) ? 65534U : ((unsigned int)down + 1U);
  const int delta = (tesla_preap_pedal_factor > 0.0F) ? ((int)raw - (int)previous) : ((int)previous - (int)raw);
  return (raw >= lower) && (raw <= upper) && (delta <= (int)rate_up) && (delta >= -(int)rate_down);
}

static bool tesla_preap_pedal_tx(const uint8_t *data, unsigned int bus, uint32_t now,
                                 float speed, bool longitudinal_allowed) {
  bool good = false;
  if (tesla_preap_pedal_configured && (bus == tesla_preap_pedal_bus)) {
    static uint8_t tesla_preap_pedal_tx_counter = 0U;
    const unsigned int one = ((unsigned int)data[0] << 8U) | data[1];
    const unsigned int two = ((unsigned int)data[2] << 8U) | data[3];
    unsigned int sum = 0x51U + 5U;
    for (unsigned int i = 0U; i < 5U; i++) { sum += data[i]; }
    const bool enabled = (data[4] & 128U) != 0U;
    const uint8_t counter = data[4] & 15U;
    good = ((data[4] & 112U) == 0U) && ((sum & 255U) == data[5]) &&
      (!tesla_preap_pedal_tx_seen || (counter == ((tesla_preap_pedal_tx_counter + 1U) & 15U)));
    if (!enabled) {
      // A valid zero/disabled command may resynchronize a dropped TX counter.
      good = ((data[4] & 112U) == 0U) && ((sum & 255U) == data[5]) && (one == 0U) && (two == 0U);
    } else {
      const int difference = (int)one - (2 * (int)two);
      const unsigned int previous = tesla_preap_pedal_tx_enabled ? tesla_preap_pedal_last_raw_one : tesla_preap_pedal_feedback_raw_one;
      const unsigned int previous_two = tesla_preap_pedal_tx_enabled ? tesla_preap_pedal_last_raw_two : tesla_preap_pedal_feedback_raw_two;
      good &= tesla_preap_pedal_tx_seen && longitudinal_allowed && tesla_preap_pedal_feedback_ready(now) &&
        (tesla_preap_pedal_feedback_di <= 2.0F) && tesla_preap_pedal_finite(speed) && (speed >= 0.0F) &&
        (difference >= 0) && (difference <= 1) && (one < 65535U) && (two < 65535U) &&
        tesla_preap_pedal_channel_bounds(one, previous, 0.050796813F, speed) &&
        tesla_preap_pedal_channel_bounds(two, previous_two, 0.101593626F, speed);
      if (good) { tesla_preap_pedal_last_raw_one = one; tesla_preap_pedal_last_raw_two = two; }
    }
    if (good) {
      tesla_preap_pedal_tx_seen = true;
      tesla_preap_pedal_tx_counter = counter;
      tesla_preap_pedal_tx_enabled = enabled;
    }
    // Rejected TX does not alter the last accepted rate/counter baseline.
  }
  return good;
}
