#include "sim_bridge.h"

#include "config.h"
#include "control_logic.h"

void control_mujoco_to_rbdl(const double mj_pos[3], const double mj_quat[4],
                            double rbdl_pos[3], double rbdl_quat[4]) {
  double dx = mj_pos[0];
  double dy = mj_pos[1];
  double dz = mj_pos[2] - MUJOCO_Z_OFFSET;

  rbdl_pos[0] = dy;
  rbdl_pos[1] = dz;
  rbdl_pos[2] = dx;

  double q_mj_xyzw[4] = {mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]};
  double q_b2w_inv[4] = {-0.5, -0.5, -0.5, 0.5};
  double q_res_xyzw[4];

  quat_mul(q_b2w_inv, q_mj_xyzw, q_res_xyzw);

  rbdl_quat[0] = q_res_xyzw[3];
  rbdl_quat[1] = q_res_xyzw[0];
  rbdl_quat[2] = q_res_xyzw[1];
  rbdl_quat[3] = q_res_xyzw[2];
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
