#include "control_logic.h"
#include "stm_controller.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define REAL_CONTROLLER_MAGIC 0xAA55u
#define REAL_MODE_CONTROL 1u
#define REAL_MODE_FK_ONLY 2u

#pragma pack(push, 1)
typedef struct {
  uint16_t magic;
  uint8_t mode;
  uint8_t active_arm_mask;
  double elapsed_s;
  double q[NUM_JOINTS];
  double qd[NUM_JOINTS];
  double body_q[NUM_BODY_JOINTS];
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
} real_input_t;

typedef struct {
  uint16_t magic;
  int32_t status;
  double tau[NUM_JOINTS];
  double ee_pos[NUM_ARMS][3];
  double ee_quat[NUM_ARMS][4];
  double traj_t;
  int32_t step_count;
} real_output_t;
#pragma pack(pop)

static uint8_t normalize_arm_mask(uint8_t mask) {
  mask &= STM_ARM_MASK_BOTH;
  return mask == 0u ? STM_ARM_MASK_BOTH : mask;
}

static void run_fk_only(const real_input_t *in, real_output_t *out) {
  uint8_t active_arm_mask = normalize_arm_mask(in->active_arm_mask);
  control_arm_kinematics_t kin;

  memset(out->tau, 0, sizeof(out->tau));
  out->status = STM_STATUS_OK;

  if (in->body_q[0] == in->body_q[0] && in->body_q[1] == in->body_q[1] &&
      in->body_q[2] == in->body_q[2]) {
    control_update_body_gravity(in->body_q);
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((active_arm_mask & (1u << arm)) == 0u) {
      continue;
    }
    const int offset = arm * ARM_JOINTS;
    control_get_arm_kinematics_with_offset(arm, in->q + offset, &kin);
    memcpy(out->ee_pos[arm], kin.pos, sizeof(double) * 3);
    memcpy(out->ee_quat[arm], kin.quat_wxyz, sizeof(double) * 4);
  }
}

int main(void) {
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  stm_controller_init();

  real_input_t in;
  real_output_t out;
  stm_input_t stm_in;
  stm_output_t stm_out;

  while (fread(&in, sizeof(in), 1, stdin) == 1) {
    memset(&out, 0, sizeof(out));
    out.magic = REAL_CONTROLLER_MAGIC;

    if (in.magic != REAL_CONTROLLER_MAGIC) {
      out.status = STM_STATUS_SAFETY_LATCHED;
      fwrite(&out, sizeof(out), 1, stdout);
      continue;
    }

    if (in.mode == REAL_MODE_FK_ONLY) {
      run_fk_only(&in, &out);
    } else if (in.mode == REAL_MODE_CONTROL) {
      memset(&stm_in, 0, sizeof(stm_in));
      memcpy(stm_in.q, in.q, sizeof(stm_in.q));
      memcpy(stm_in.qd, in.qd, sizeof(stm_in.qd));
      memcpy(stm_in.body_q, in.body_q, sizeof(stm_in.body_q));
      memcpy(stm_in.target_pos, in.target_pos, sizeof(stm_in.target_pos));
      memcpy(stm_in.target_quat, in.target_quat, sizeof(stm_in.target_quat));
      stm_in.active_arm_mask = in.active_arm_mask;

      stm_controller_step_elapsed(&stm_in, &stm_out, in.elapsed_s);
      out.status = (int32_t)stm_out.status;
      memcpy(out.tau, stm_out.tau, sizeof(out.tau));
      memcpy(out.ee_pos, stm_out.ee_pos, sizeof(out.ee_pos));
      memcpy(out.ee_quat, stm_out.ee_quat, sizeof(out.ee_quat));
      out.traj_t = stm_out.traj_t;
      out.step_count = (int32_t)stm_out.step_count;
    } else {
      out.status = STM_STATUS_SAFETY_LATCHED;
    }

    fwrite(&out, sizeof(out), 1, stdout);
  }

  return 0;
}
