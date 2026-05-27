#ifndef H7_CLOCK_SIM_H
#define H7_CLOCK_SIM_H

typedef double (*h7_clock_now_ms_fn)(void *user_ctx);

typedef struct {
  double control_dt_s;
  double max_catchup_s;
  double last_tick_ms;
  int initialized;
  h7_clock_now_ms_fn now_ms;
  void *user_ctx;
} h7_clock_sim_t;

void h7_clock_sim_init(h7_clock_sim_t *clock, double control_dt_s);
void h7_clock_sim_set_now_fn(h7_clock_sim_t *clock, h7_clock_now_ms_fn now_ms,
                             void *user_ctx);
void h7_clock_sim_set_max_catchup(h7_clock_sim_t *clock, double max_catchup_s);
int h7_clock_sim_due_ticks(h7_clock_sim_t *clock);

#endif /* H7_CLOCK_SIM_H */
