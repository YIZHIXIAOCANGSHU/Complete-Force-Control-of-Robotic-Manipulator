#include "h7_clock_sim.h"

#include <math.h>
#include <time.h>

static uint64_t h7_clock_sim_default_now_us(void *user_ctx) {
  struct timespec now;
  (void)user_ctx;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_nsec / 1000ULL;
}

void h7_clock_sim_init(h7_clock_sim_t *clock) {
  if (clock == 0) {
    return;
  }
  clock->max_elapsed_us = 20000ULL;
  clock->last_sample_us = 0ULL;
  clock->initialized = 0;
  clock->now_us = h7_clock_sim_default_now_us;
  clock->user_ctx = 0;
}

void h7_clock_sim_set_now_fn(h7_clock_sim_t *clock, h7_clock_now_us_fn now_us,
                             void *user_ctx) {
  if (clock == 0) {
    return;
  }
  clock->now_us = now_us != 0 ? now_us : h7_clock_sim_default_now_us;
  clock->user_ctx = user_ctx;
}

void h7_clock_sim_set_max_elapsed(h7_clock_sim_t *clock, double max_elapsed_s) {
  if (clock == 0 || max_elapsed_s <= 0.0 || !isfinite(max_elapsed_s)) {
    return;
  }
  clock->max_elapsed_us = (uint64_t)llround(max_elapsed_s * 1000000.0);
  if (clock->max_elapsed_us < 1ULL) {
    clock->max_elapsed_us = 1ULL;
  }
}

uint64_t h7_clock_sim_elapsed_us(h7_clock_sim_t *clock) {
  uint64_t now_us;
  uint64_t elapsed_us;

  if (clock == 0 || clock->now_us == 0) {
    return 0ULL;
  }

  now_us = clock->now_us(clock->user_ctx);

  if (!clock->initialized) {
    clock->last_sample_us = now_us;
    clock->initialized = 1;
    return 0ULL;
  }

  if (now_us <= clock->last_sample_us) {
    clock->last_sample_us = now_us;
    return 0ULL;
  }

  elapsed_us = now_us - clock->last_sample_us;
  clock->last_sample_us = now_us;

  if (clock->max_elapsed_us > 0ULL && elapsed_us > clock->max_elapsed_us) {
    elapsed_us = clock->max_elapsed_us;
  }

  return elapsed_us;
}

double h7_clock_sim_elapsed_s(h7_clock_sim_t *clock) {
  return (double)h7_clock_sim_elapsed_us(clock) / 1000000.0;
}
