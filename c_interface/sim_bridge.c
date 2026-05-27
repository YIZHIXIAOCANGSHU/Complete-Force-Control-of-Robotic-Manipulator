#include "sim_bridge.h"

#include "control_logic.h"

void control_mujoco_to_rbdl(const double target_pos[3], const double target_quat[4],
                            double control_pos[3], double control_quat[4]) {
  control_pos[0] = target_pos[0];
  control_pos[1] = target_pos[1];
  control_pos[2] = target_pos[2];

  control_quat[0] = target_quat[0];
  control_quat[1] = target_quat[1];
  control_quat[2] = target_quat[2];
  control_quat[3] = target_quat[3];
}

void control_step_v2_mujoco(const double target_pos[3],
                            const double target_quat[4],
                            const double current_q[ARM_JOINTS],
                            const double current_qd[ARM_JOINTS],
                            double tau_out[ARM_JOINTS]) {
  control_step_v2_mujoco_arm(ARM_LEFT, target_pos, target_quat,
                             current_q, current_qd, tau_out);
}

void control_step_v2_mujoco_arm(int side, const double target_pos[3],
                                const double target_quat[4],
                                const double current_q[ARM_JOINTS],
                                const double current_qd[ARM_JOINTS],
                                double tau_out[ARM_JOINTS]) {
  double control_pos[3];
  double control_quat[4];

  control_mujoco_to_rbdl(target_pos, target_quat, control_pos,
                         control_quat);
  control_step_v2_arm(side, control_pos, control_quat, current_q, current_qd,
                      tau_out);
}
