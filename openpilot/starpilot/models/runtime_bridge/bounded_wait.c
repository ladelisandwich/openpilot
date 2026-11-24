#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>

static _Thread_local uint64_t wait_calls;
uint64_t starpilot_wait_calls(void) { return wait_calls; }

/* Private compatibility prototype. A failure exits before another submit. */
uint64_t starpilot_bounded_wait(const volatile uint64_t *signal, uint64_t target) {
  wait_calls++;
  uint64_t done = *signal;
  if (done >= target) return done;
  struct timespec start, now;
  if (clock_gettime(CLOCK_MONOTONIC, &start) != 0) _exit(74);
  do {
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) _exit(74);
    int64_t elapsed = (now.tv_sec - start.tv_sec) * INT64_C(1000000000) + now.tv_nsec - start.tv_nsec;
    if (elapsed >= INT64_C(30000000000)) _exit(73);
    sched_yield();
    done = *signal;
  } while (done < target);
  return done;
}
