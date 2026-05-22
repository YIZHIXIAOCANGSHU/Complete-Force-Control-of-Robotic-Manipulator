#include "sim_bridge.h"

#include "control_logic.h"

void control_mujoco_to_rbdl(const double base_pos[3], const double base_quat[4],
                            double control_pos[3], double control_quat[4]) {
  control_pos[0] = base_pos[0];
  control_pos[1] = base_pos[1];
  control_pos[2] = base_pos[2];

  control_quat[0] = base_quat[0];
  control_quat[1] = base_quat[1];
  control_quat[2] = base_quat[2];
  control_quat[3] = base_quat[3];
}

void control_step_v2_mujoco(const double base_target_pos[3],
                            const double base_target_quat[4],
                            const double current_q[7],
                            const double current_qd[7], double tau_out[7]) {
  double control_pos[3];
  double control_quat[4];

  control_mujoco_to_rbdl(base_target_pos, base_target_quat, control_pos,
                         control_quat);
  control_step_v2(control_pos, control_quat, current_q, current_qd, tau_out);
}
