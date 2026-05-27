#ifndef SIM_BRIDGE_H
#define SIM_BRIDGE_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void control_mujoco_to_rbdl(const double base_pos[3], const double base_quat[4],
                            double control_pos[3], double control_quat[4]);

void control_step_v2_mujoco(const double base_target_pos[3],
                            const double base_target_quat[4],
                            const double current_q[ARM_JOINTS],
                            const double current_qd[ARM_JOINTS],
                            double tau_out[ARM_JOINTS]);

void control_step_v2_mujoco_arm(int side, const double base_target_pos[3],
                                const double base_target_quat[4],
                                const double current_q[ARM_JOINTS],
                                const double current_qd[ARM_JOINTS],
                                double tau_out[ARM_JOINTS]);

#ifdef __cplusplus
}
#endif

#endif /* SIM_BRIDGE_H */
