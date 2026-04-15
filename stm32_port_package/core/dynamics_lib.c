#include "dynamics_lib.h"

/**
 * @brief 使用递归牛顿-欧拉法 (RNEA) 计算逆向动力学
 * @param model RBDL模型
 * @param q 关节角度
 * @param qd 关节角速度 (若传NULL则视为0)
 * @param qdd 关节角加速度 (若传NULL则视为0)
 * @param tau 输出的关节力矩
 */
void rbdl_inverse_dynamics(const RBDLModel *model, const double *q,
                           const double *qd, const double *qdd, double *tau) {
  int N = model->num_bodies;

  double v[MAX_BODIES][6];
  double a[MAX_BODIES][6];
  double f[MAX_BODIES][6];
  double E_lambda[MAX_BODIES][9];
  double r_lambda[MAX_BODIES][3];

  /* ---- 前向传播 (Forward Pass) ----
   * 从基座向末端传播连杆的运动学信息 (速度, 加速度) 并计算每个连杆的惯性力。
   */
  for (int i = 0; i < N; i++) {
    const Body *b = &model->bodies[i];

    /* X_J: spatial transform for joint rotation E_J = R(axis, -q) */
    double E_J[9];
    axis_angle_to_rotmat(b->axis, -q[i], E_J);
    double r_J[3] = {0, 0, 0};

    /* X_lambda = X_J * X_T (parent->joint * joint->child) */
    st_compose(E_J, r_J, b->E_p, b->r_p, E_lambda[i], r_lambda[i]);

    /* Parent velocity/acceleration in world frame */
    double v_parent[6], a_parent[6];
    if (i == 0) {
      sv_zero(v_parent);
      /* a_0 = -gravity (Featherstone convention) */
      a_parent[0] = 0;
      a_parent[1] = 0;
      a_parent[2] = 0;
      a_parent[3] = -model->gravity[0];
      a_parent[4] = -model->gravity[1];
      a_parent[5] = -model->gravity[2];
    } else {
      sv_copy(v[i - 1], v_parent);
      sv_copy(a[i - 1], a_parent);
    }

    /* 计算父连杆到当前连杆的速度: v_i = X_i * v_parent + S_i * qd_i */
    double v_temp[6];
    st_apply(E_lambda[i], r_lambda[i], v_parent, v_temp);
    double qd_val = qd ? qd[i] : 0.0;
    for (int k = 0; k < 6; k++)
      v[i][k] = v_temp[k] + b->S[k] * qd_val;

    /* 计算速度叉乘项 (Coriolis term): c_i = v_i x (S_i * qd_i) */
    double Sqd[6], c_J[6];
    for (int k = 0; k < 6; k++)
      Sqd[k] = b->S[k] * qd_val;
    sv_crossm(v[i], Sqd, c_J);

    /* 计算连杆的绝对加速度: a_i = X_i * a_parent + S_i * qdd_i + c_i */
    double a_temp[6];
    st_apply(E_lambda[i], r_lambda[i], a_parent, a_temp);
    double qdd_val = qdd ? qdd[i] : 0.0;
    for (int k = 0; k < 6; k++)
      a[i][k] = a_temp[k] + b->S[k] * qdd_val + c_J[k];

    /* 计算施加在连杆质心上的惯性力: f_i = I_i * a_i + v_i x* (I_i * v_i) */
    double Iv[6], vxIv[6], Ia[6];
    si_mul_vec(b->mass, b->mhc, b->inertia, v[i], Iv);
    sv_crossf(v[i], Iv, vxIv);
    si_mul_vec(b->mass, b->mhc, b->inertia, a[i], Ia);
    for (int k = 0; k < 6; k++)
      f[i][k] = Ia[k] + vxIv[k];
  }

  /* ---- 反向传播 (Backward Pass) ----
   * 从末端向基座反向累加力，计算出每个关节需要的驱动力矩 (tau_i)
   */
  for (int i = N - 1; i >= 0; i--) {
    const Body *b = &model->bodies[i];

    /* 当前关节所需的力矩等于全空间反作用力在关节运动轴 (S_i) 上的投影: tau_i =
     * S_i^T * f_i */
    tau[i] = 0.0;
    for (int k = 0; k < 6; k++)
      tau[i] += b->S[k] * f[i][k];

    /* f_parent += X_i^T * f_i */
    if (i > 0) {
      double f_temp[6];
      st_apply_transpose(E_lambda[i], r_lambda[i], f[i], f_temp);
      for (int k = 0; k < 6; k++)
        f[i - 1][k] += f_temp[k];
    }
  }
}

/**
 * @brief 计算纯重力项力矩
 * @param model RBDL模型
 * @param q 关节角度
 * @param G 输出的重力补偿力矩
 */
void rbdl_calc_gravity(const RBDLModel *model, const double *q, double *G) {
  rbdl_inverse_dynamics(model, q, NULL, NULL, G);
}

/**
 * @brief 计算重力与科里奥利/离心力的耦合项力矩 (Gravity and Coriolis)
 * @param model RBDL模型
 * @param q 关节角度
 * @param qd 关节角速度
 * @param tau_gc 输出的所需力矩
 */
void rbdl_calc_gc(const RBDLModel *model, const double *q, const double *qd,
                  double *tau_gc) {
  rbdl_inverse_dynamics(model, q, qd, NULL, tau_gc);
}
