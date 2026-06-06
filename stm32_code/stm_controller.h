#ifndef STM_CONTROLLER_H
#define STM_CONTROLLER_H

#include "config.h"
#include <stdint.h>

#define STM_STATUS_OK 0
#define STM_STATUS_SAFETY_LATCHED -1
#define STM_STATUS_WAITING_ZERO -2

#define STM_ARM_MASK_LEFT (1u << ARM_LEFT)
#define STM_ARM_MASK_RIGHT (1u << ARM_RIGHT)
#define STM_ARM_MASK_BOTH (STM_ARM_MASK_LEFT | STM_ARM_MASK_RIGHT)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double q[NUM_JOINTS];
  double qd[NUM_JOINTS];
  /* Runtime arm selection. 0 is treated as STM_ARM_MASK_BOTH. */
  uint8_t active_arm_mask;
  /* Sim-only Body0422 torso command: Waist01, Waist02, Body0422. */
  double body_q[NUM_BODY_JOINTS];
  /* Target TCP pose in the Body0422 dynamic frame, including relative rotation. */
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
} stm_input_t;

typedef struct {
  double tau[NUM_JOINTS];
  double tau_gravity[NUM_JOINTS];
  double tau_gc[NUM_JOINTS];
  int status;
  double ee_pos[NUM_ARMS][3];
  double ee_quat[NUM_ARMS][4];
  double ref_pos[NUM_ARMS][3];
  double ref_quat[NUM_ARMS][4];
  double ee_twist[NUM_ARMS][6];
  double traj_t;
  int step_count;
} stm_output_t;

void stm_controller_init(void);

/**
 * @brief Execute one dual-arm control update.
 *
 * elapsed_s is supplied by the platform layer and represents the real time
 * elapsed since the previous control call, in seconds. stm32_code does not own
 * or assume any hardware timer frequency.
 */
void stm_controller_step_elapsed(const stm_input_t *in, stm_output_t *out,
                                 double elapsed_s);
void stm_controller_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* STM_CONTROLLER_H */
