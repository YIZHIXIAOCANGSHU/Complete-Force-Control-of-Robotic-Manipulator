#include "main_stm.h"

#include <time.h>

static int g_host_hooks_ready = 0;

static double host_now_ms(void *user_ctx) {
  struct timespec now;
  (void)user_ctx;

  clock_gettime(CLOCK_MONOTONIC, &now);
  return (double)now.tv_sec * 1000.0 + (double)now.tv_nsec / 1000000.0;
}

static void ensure_host_hooks(void) {
  stm_platform_hooks_t hooks;

  if (g_host_hooks_ready) {
    return;
  }

  hooks.now_ms = host_now_ms;
  hooks.user_ctx = NULL;
  stm_controller_set_platform_hooks(&hooks);
  g_host_hooks_ready = 1;
}

void stm_init(void) {
  ensure_host_hooks();
  stm_controller_init();
}

void stm_step(const stm_input_t *in, stm_output_t *out) {
  ensure_host_hooks();
  stm_controller_step(in, out);
}

void stm_reset(void) {
  stm_controller_reset();
}

void stm_set_platform_hooks(const stm_platform_hooks_t *hooks) {
  g_host_hooks_ready = 1;
  stm_controller_set_platform_hooks(hooks);
}
