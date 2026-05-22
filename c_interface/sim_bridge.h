#ifndef SIM_BRIDGE_H
#define SIM_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void control_mujoco_to_rbdl(const double base_pos[3], const double base_quat[4],
                            double control_pos[3], double control_quat[4]);

void control_step_v2_mujoco(const double base_target_pos[3],
                            const double base_target_quat[4],
                            const double current_q[7],
                            const double current_qd[7], double tau_out[7]);

#ifdef __cplusplus
}
#endif

#endif /* SIM_BRIDGE_H */
