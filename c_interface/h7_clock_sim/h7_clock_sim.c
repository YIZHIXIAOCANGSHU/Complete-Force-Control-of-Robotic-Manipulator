#include "h7_clock_sim.h"

#include <math.h>
#include <time.h>

static double h7_clock_sim_default_now_ms(void *user_ctx) {
  struct timespec now;
  (void)user_ctx;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (double)now.tv_sec * 1000.0 + (double)now.tv_nsec / 1000000.0;
}

void h7_clock_sim_init(h7_clock_sim_t *clock, double control_dt_s) {
  if (clock == 0) {
    return;
  }
  clock->control_dt_s = control_dt_s > 0.0 ? control_dt_s : 0.001;
  clock->max_catchup_s = 0.02;
  clock->last_tick_ms = 0.0;
  clock->initialized = 0;
  clock->now_ms = h7_clock_sim_default_now_ms;
  clock->user_ctx = 0;
}

void h7_clock_sim_set_now_fn(h7_clock_sim_t *clock, h7_clock_now_ms_fn now_ms,
                             void *user_ctx) {
  if (clock == 0) {
    return;
  }
  clock->now_ms = now_ms != 0 ? now_ms : h7_clock_sim_default_now_ms;
  clock->user_ctx = user_ctx;
}

void h7_clock_sim_set_max_catchup(h7_clock_sim_t *clock, double max_catchup_s) {
  if (clock == 0 || max_catchup_s <= 0.0 || !isfinite(max_catchup_s)) {
    return;
  }
  clock->max_catchup_s = max_catchup_s;
}

int h7_clock_sim_due_ticks(h7_clock_sim_t *clock) {
  double now_ms;
  double elapsed_s;
  double tick_s;
  int ticks;
  int max_ticks;

  if (clock == 0 || clock->now_ms == 0) {
    return 1;
  }

  now_ms = clock->now_ms(clock->user_ctx);
  if (!isfinite(now_ms)) {
    return 1;
  }

  if (!clock->initialized) {
    clock->last_tick_ms = now_ms;
    clock->initialized = 1;
    return 1;
  }

  elapsed_s = (now_ms - clock->last_tick_ms) / 1000.0;
  tick_s = clock->control_dt_s;
  if (elapsed_s <= 0.0 || tick_s <= 0.0) {
    return 0;
  }

  ticks = (int)floor(elapsed_s / tick_s);
  if (ticks < 1) {
    return 0;
  }

  max_ticks = (int)ceil(clock->max_catchup_s / tick_s);
  if (max_ticks < 1) {
    max_ticks = 1;
  }
  if (ticks > max_ticks) {
    ticks = max_ticks;
  }

  clock->last_tick_ms += (double)ticks * tick_s * 1000.0;
  return ticks;
}
