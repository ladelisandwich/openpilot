#pragma once

#include "board/drivers/drivers.h"

struct fan_state_t fan_state;

static const uint8_t FAN_TICK_FREQ = 8U;
static const uint8_t FAN_STALL_THRESHOLD_MIN = 3U;

void fan_set_power(uint8_t percentage) {
  if (percentage > 0U) {
    fan_state.power = CLAMP(percentage, 20U, 100U);
  } else {
    fan_state.power = 0U;
  }
}

void fan_init(void) {
  fan_state.stall_threshold = FAN_STALL_THRESHOLD_MIN;
  fan_state.cooldown_counter = current_board->fan_enable_cooldown_time * FAN_TICK_FREQ;
  llfan_init();
}

// Call this at FAN_TICK_FREQ
void fan_tick(void) {
  if (current_board->has_fan) {
    // Measure fan RPM
    uint16_t fan_rpm_fast = fan_state.tach_counter * (60U * FAN_TICK_FREQ / 4U);   // 4 interrupts per rotation
    fan_state.tach_counter = 0U;
    fan_state.rpm = (fan_rpm_fast + (3U * fan_state.rpm)) / 4U;

    bool fan_stalled = false;
    if (current_board->fan_stall_recovery) {
      if (fan_state.power > 0U) {
        fan_state.stall_counter = (fan_rpm_fast == 0U) ? MIN(fan_state.stall_counter + 1U, 254U) : 0U;
        if (fan_state.stall_counter > (fan_state.stall_threshold * FAN_TICK_FREQ)) {
          fan_stalled = true;
          fan_state.stall_counter = 0U;
          fan_state.stall_threshold = CLAMP(fan_state.stall_threshold + 2U, FAN_STALL_THRESHOLD_MIN, 8U);
        }
      } else {
        fan_state.stall_counter = 0U;
        fan_state.stall_threshold = FAN_STALL_THRESHOLD_MIN;
      }
    }

    // Cooldown counter to prevent noise on tachometer line.
    if (fan_state.power > 0U) {
      fan_state.cooldown_counter = current_board->fan_enable_cooldown_time * FAN_TICK_FREQ;
    } else {
      if (fan_state.cooldown_counter > 0U) {
        fan_state.cooldown_counter--;
      }
    }

    // Set PWM and enable line
    pwm_set(TIM3, 3, fan_state.power);
    // A one-tick disable resets the tici fan driver even while cooldown is active.
    current_board->set_fan_enabled(!fan_stalled && ((fan_state.power > 0U) || (fan_state.cooldown_counter > 0U)));
  }
}
