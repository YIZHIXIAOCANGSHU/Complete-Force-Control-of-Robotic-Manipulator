#ifndef STM_CONTROLLER_H
#define STM_CONTROLLER_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double q[NUM_JOINTS];
  double qd[NUM_JOINTS];
  double target_pos[3];
  double target_quat[4];
} stm_input_t;

typedef struct {
  double tau[NUM_JOINTS];
  int status;
  double ee_pos[3];
  double ee_quat[4];
  double traj_t;
  int step_count;
  double calc_time_ms;
} stm_output_t;

typedef double (*stm_now_ms_fn)(void *user_ctx);

typedef struct {
  stm_now_ms_fn now_ms;
  void *user_ctx;
} stm_platform_hooks_t;

void stm_controller_init(void);
void stm_controller_step(const stm_input_t *in, stm_output_t *out);
void stm_controller_reset(void);
void stm_controller_set_platform_hooks(const stm_platform_hooks_t *hooks);

#ifdef __cplusplus
}
#endif

#endif /* STM_CONTROLLER_H */
