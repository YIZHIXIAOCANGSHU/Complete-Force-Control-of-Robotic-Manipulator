#include "sim_bridge.h"

#include "control_logic.h"

void control_mujoco_to_rbdl(const double mj_pos[3], const double mj_quat[4],
                            double rbdl_pos[3], double rbdl_quat[4]) {
  rbdl_pos[0] = mj_pos[0];
  rbdl_pos[1] = mj_pos[1];
  rbdl_pos[2] = mj_pos[2];

  rbdl_quat[0] = mj_quat[0];
  rbdl_quat[1] = mj_quat[1];
  rbdl_quat[2] = mj_quat[2];
  rbdl_quat[3] = mj_quat[3];
}

void control_step_v2_mujoco(const double mj_target_pos[3],
                            const double mj_target_quat[4],
                            const double current_q[7],
                            const double current_qd[7], double tau_out[7]) {
  double rbdl_pos[3];
  double rbdl_quat[4];

  control_mujoco_to_rbdl(mj_target_pos, mj_target_quat, rbdl_pos, rbdl_quat);
  control_step_v2(rbdl_pos, rbdl_quat, current_q, current_qd, tau_out);
}
