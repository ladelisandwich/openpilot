#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

_Static_assert(sizeof(unsigned long) == 8 && sizeof(void *) == 8, "64-bit native host ABI required");
static _Thread_local uint64_t deadlines[256];
static _Thread_local void *pending[16];
static _Thread_local uint64_t guarded_calls;

static uint64_t now_ns(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts)) _exit(74);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

uint64_t sp_loop_start(uint64_t slot, uint64_t timeout_ns) {
  if (slot >= 256 || !timeout_ns || timeout_ns > 30000000000ULL) _exit(82);
  deadlines[slot] = now_ns() + timeout_ns;
  return 0;
}

uint64_t sp_loop_touch(uint64_t slot, uint64_t timeout_ns, uint64_t loop_token) {
  (void)loop_token;
  if (slot >= 256 || !timeout_ns || timeout_ns > 30000000000ULL) _exit(82);
  if (!deadlines[slot]) return sp_loop_start(slot, timeout_ns);
  if (now_ns() >= deadlines[slot]) _exit(73);
  return 0;
}

uint64_t sp_loop_check(uint64_t condition, uint64_t slot) {
  guarded_calls++;
  if (slot >= 256 || !deadlines[slot]) _exit(82);
  if (now_ns() >= deadlines[slot]) _exit(73);
  if (!condition) deadlines[slot] = 0;
  return condition;
}

int sp_control(uint64_t function, uint64_t handle, uint64_t request_type, uint64_t request,
               uint64_t value, uint64_t index, uint64_t data, uint64_t length, uint64_t timeout, uint64_t order_marker) {
  (void)order_marker;
  typedef int (*fn)(void *, uint8_t, uint8_t, uint16_t, uint16_t, unsigned char *, uint16_t, unsigned int);
  if (request_type > UINT8_MAX || request > UINT8_MAX || value > UINT16_MAX || index > UINT16_MAX || length > UINT16_MAX || timeout > UINT32_MAX) _exit(76);
  int rc = ((fn)(uintptr_t)function)((void *)(uintptr_t)handle, (uint8_t)request_type, (uint8_t)request, (uint16_t)value, (uint16_t)index, (unsigned char *)(uintptr_t)data, (uint16_t)length, (unsigned int)timeout);
  guarded_calls++;
  if (length > UINT16_MAX || timeout > UINT32_MAX || rc < 0 || (uint64_t)rc != length) _exit(76);
  return rc;
}

int sp_bulk(uint64_t function, uint64_t handle, uint64_t endpoint, uint64_t data,
            uint64_t length, uint64_t transferred, uint64_t timeout, uint64_t order_marker) {
  (void)order_marker;
  typedef int (*fn)(void *, unsigned char, unsigned char *, int, int *, unsigned int);
  if (endpoint > UINT8_MAX || length > INT32_MAX || timeout > UINT32_MAX) _exit(77);
  int actual = 0;
  int rc = ((fn)(uintptr_t)function)((void *)(uintptr_t)handle, (unsigned char)endpoint, (unsigned char *)(uintptr_t)data, (int)length, &actual, (unsigned int)timeout);
  guarded_calls++;
  if (rc || length > INT32_MAX || timeout > UINT32_MAX || actual < 0 || (uint64_t)actual != length) _exit(77);
  if (transferred) *(int *)(uintptr_t)transferred = actual;
  return rc;
}

/* Offsets are asserted against the bound libusb ABI before installing this adapter. */
static int field(void *transfer, size_t offset) {
  int value;
  memcpy(&value, (unsigned char *)transfer + offset, sizeof(value));
  return value;
}

int sp_submit(uint64_t function, uint64_t transfer_address, uint64_t order_marker) {
  (void)order_marker;
  typedef int (*fn)(void *);
  void *transfer = (void *)(uintptr_t)transfer_address;
  int rc = ((fn)(uintptr_t)function)(transfer);
  guarded_calls++;
  if (rc) _exit(78);
  for (size_t i = 0; i < 16; i++) if (pending[i] == transfer) return rc;
  for (size_t i = 0; i < 16; i++) {
    if (!pending[i]) { pending[i] = transfer; return rc; }
  }
  _exit(79);
}

int sp_events(uint64_t function, uint64_t context, uint64_t timeout, uint64_t order_marker) {
  (void)order_marker;
  typedef int (*fn)(void *, void *);
  int rc = ((fn)(uintptr_t)function)((void *)(uintptr_t)context, (void *)(uintptr_t)timeout);
  guarded_calls++;
  /* LIBUSB_ERROR_INTERRUPTED is retryable, as in the new native runtime. */
  if (rc < 0 && rc != -10) _exit(80);
  for (size_t i = 0; i < 16; i++) if (pending[i]) {
    int status = field(pending[i], 16);
    if (status == 0xff) continue;
    if (status || field(pending[i], 24) != field(pending[i], 20)) _exit(81);
    pending[i] = NULL;
  }
  return rc;
}

uint64_t sp_guard_calls(void) { return guarded_calls; }
