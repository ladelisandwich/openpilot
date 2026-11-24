import json
from pathlib import Path
import re
import subprocess

import pytest


PANDA = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def fan_simulator(tmp_path_factory):
  tmp = tmp_path_factory.mktemp("fan")
  declarations = (PANDA / "board/drivers/drivers.h").read_text()
  state = re.search(r"struct fan_state_t \{.*?\};", declarations, re.S).group()
  board_header = PANDA / "board/boards/board_declarations.h"
  boards = []
  for name in ("dos", "tres", "cuatro", "red"):
    definition = (PANDA / f"board/boards/{name}.h").read_text().split(f"board board_{name} = {{", 1)[1]
    fields = []
    for field in ("has_fan", "fan_enable_cooldown_time", "fan_stall_recovery"):
      match = re.search(rf"\.{field}\s*=\s*(\w+)", definition)
      if match:
        fields.append(f".{field} = {match[1]}")
    boards.append("{" + ", ".join(fields + [".set_fan_enabled = set_enabled"]) + "}")
  header = tmp / "board/drivers/drivers.h"
  header.parent.mkdir(parents=True)
  header.write_text(f"""#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct harness_configuration harness_configuration;
typedef struct GPIO_TypeDef GPIO_TypeDef;
#include "{board_header}"
{state}
void llfan_init(void);
""")
  source = tmp / "fan.c"
  source.write_text(r"""
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "board/drivers/drivers.h"
#define MIN(a,b) ((a)<(b)?(a):(b))
#define CLAMP(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define TIM3 0
static bool enabled;
static unsigned tick, pwm, resets[64], reset_count;
static void set_enabled(bool value) {
  enabled = value;
  if (!value && pwm > 0U) resets[reset_count++] = tick;
}
void llfan_init(void) {}
static void pwm_set(int timer, int channel, uint8_t value) {
  (void)timer; (void)channel; pwm = value;
}
static struct board boards[] = {
""" + ",\n".join(boards) + r"""
};
static struct board *current_board;
#include "board/drivers/fan.h"
static void step(unsigned pulses) {
  tick++;
  fan_state.tach_counter = pulses;
  fan_tick();
}
int main(int argc, char **argv) {
  assert(argc == 3);
  current_board = &boards[atoi(argv[1])];
  unsigned scenario = (unsigned)atoi(argv[2]);
  fan_init(); fan_set_power(30U);
  if (scenario == 0U) {
    for (unsigned i = 0; i < 240U; i++) step(0U);
  } else if (scenario == 1U) {
    for (unsigned i = 1; i <= 240U; i++) step(i % 24U == 0U ? 1U : 0U);
  } else if (scenario == 2U) {
    fan_state.rpm = 6500U;
    for (unsigned i = 0; i < 25U; i++) step(0U);
  } else if (scenario == 3U) {
    for (unsigned i = 0; i < 25U; i++) step(0U);
    fan_set_power(0U);
    for (unsigned i = 0; i < 23U; i++) {step(0U); assert(enabled);}
    step(0U); assert(!enabled);
    fan_set_power(30U);
    for (unsigned i = 0; i < 25U; i++) step(0U);
  }
  printf("{\"power\":%u,\"rpm\":%u,\"resets\":[", pwm, fan_state.rpm);
  for (unsigned i = 0; i < reset_count; i++) printf("%s%u", i ? "," : "", resets[i]);
  puts("]}");
}
""")
  binary = tmp / "fan"
  subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(tmp), "-I", str(PANDA),
                  str(source), "-o", str(binary)], check=True)

  def run(board, scenario):
    return json.loads(subprocess.check_output([str(binary), str(board), str(scenario)], text=True))

  return run


def test_tici_retries_a_stopped_fan_with_bounded_backoff(fan_simulator):
  result = fan_simulator(0, 0)
  assert result["resets"] == [25, 66, 123, 188]
  assert result["power"] == 30


@pytest.mark.parametrize("board", [1, 2, 3])
def test_other_boards_do_not_gain_enable_resets(fan_simulator, board):
  assert fan_simulator(board, 0)["resets"] == []


def test_tach_pulses_clear_consecutive_stall_time(fan_simulator):
  assert fan_simulator(0, 1)["resets"] == []


def test_recovery_uses_raw_tach_even_while_filtered_rpm_is_nonzero(fan_simulator):
  result = fan_simulator(0, 2)
  assert result["resets"] == [25]
  assert result["rpm"] > 0


def test_stopping_preserves_cooldown_and_resets_retry_interval(fan_simulator):
  assert fan_simulator(0, 3)["resets"] == [25, 74]
