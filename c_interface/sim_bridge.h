#ifndef SIM_BRIDGE_H
#define SIM_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void control_mujoco_to_rbdl(const double mj_pos[3], const double mj_quat[4],
                            double rbdl_pos[3], double rbdl_quat[4]);

#ifdef __cplusplus
}
#endif

#endif /* SIM_BRIDGE_H */
