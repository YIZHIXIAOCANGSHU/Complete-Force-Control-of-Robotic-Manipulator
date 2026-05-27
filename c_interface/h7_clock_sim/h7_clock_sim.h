#ifndef H7_CLOCK_SIM_H
#define H7_CLOCK_SIM_H

#include <stdint.h>

typedef uint64_t (*h7_clock_now_us_fn)(void *user_ctx);

typedef struct {
  uint64_t max_elapsed_us;
  uint64_t last_sample_us;
  int initialized;
  h7_clock_now_us_fn now_us;
  void *user_ctx;
} h7_clock_sim_t;

void h7_clock_sim_init(h7_clock_sim_t *clock);
void h7_clock_sim_set_now_fn(h7_clock_sim_t *clock, h7_clock_now_us_fn now_us,
                             void *user_ctx);
void h7_clock_sim_set_max_elapsed(h7_clock_sim_t *clock, double max_elapsed_s);
uint64_t h7_clock_sim_elapsed_us(h7_clock_sim_t *clock);
double h7_clock_sim_elapsed_s(h7_clock_sim_t *clock);

#endif /* H7_CLOCK_SIM_H */
