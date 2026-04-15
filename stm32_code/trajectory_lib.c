#include "trajectory_lib.h"
#include "math_lib.h"
#include <math.h>

void trajectory_init(TrajectoryPlanner *tp, const double start_pos[3], int axis,
                     double amplitude, double speed, double accel) {
  vec3_copy(start_pos, tp->start_pos);
  tp->amplitude = amplitude;
  tp->speed = speed;
  tp->accel = accel;
  tp->axis = axis;
  tp->direction = 1;

  tp->L = amplitude;
  tp->v_max = speed;
  tp->a = accel;

  /*
   * 梯形速度规划 (Trapezoidal Velocity Profile):
   * 计算加速段能达到的理想距离: d_a_ideal = 0.5 * v_max^2 / a
   * 如果 d_a_ideal 超过了总距离 L 的一半，说明还没加速到 v_max 就需要减速了
   * (退化为三角形速度曲线)
   */
  double d_a_ideal = 0.5 * tp->v_max * tp->v_max / tp->a;
  if (2.0 * d_a_ideal > tp->L) {
    tp->v_max = sqrt(tp->a * tp->L);
    tp->t_a = tp->v_max / tp->a;
    tp->t_c = 0.0;
    tp->d_a = tp->L / 2.0;
    tp->d_c = 0.0;
  } else {
    tp->t_a = tp->v_max / tp->a;
    tp->d_a = d_a_ideal;
    tp->d_c = tp->L - 2.0 * tp->d_a;
    tp->t_c = tp->d_c / tp->v_max;
  }

  tp->half_period = 2.0 * tp->t_a + tp->t_c; /* 单程运动时间 */
  tp->period = 2.0 * tp->half_period;        /* 往复一次的总周期 */
}

/* 评估单程(半周期)内的时间 t 对应的位置和速度 */
static void evaluate_half(TrajectoryPlanner *tp, double t, double *p,
                          double *v) {
  if (t < tp->t_a) {
    *v = tp->a * t;
    *p = 0.5 * tp->a * t * t;
  } else if (t < tp->t_a + tp->t_c) {
    double dt = t - tp->t_a;
    *v = tp->v_max;
    *p = tp->d_a + tp->v_max * dt;
  } else {
    double dt = t - (tp->t_a + tp->t_c);
    *v = tp->v_max - tp->a * dt;
    *p = tp->d_a + tp->d_c + tp->v_max * dt - 0.5 * tp->a * dt * dt;
    if (*p > tp->L)
      *p = tp->L;
    if (*v < 0)
      *v = 0.0;
  }
}

void trajectory_evaluate(TrajectoryPlanner *tp, double t, double pos[3],
                         double vel[3]) {
  double t_mod = fmod(t, tp->period);
  double p, v;

  if (t_mod < tp->half_period) {
    evaluate_half(tp, t_mod, &p, &v);
  } else {
    evaluate_half(tp, t_mod - tp->half_period, &p, &v);
    p = tp->L - p;
    v = -v;
  }

  vec3_copy(tp->start_pos, pos);
  pos[tp->axis] += p;

  vec3_zero(vel);
  vel[tp->axis] = v;
}

/* ================================================================
 *  LinearPathPlanner: 3D 直线路径 + 梯形速度曲线
 * ================================================================ */

void linear_path_init(LinearPathPlanner *lp, const double start_pos[3],
                      const double start_quat[4], const double end_pos[3],
                      const double end_quat[4], double speed, double accel) {
  vec3_copy(start_pos, lp->start_pos);
  vec3_copy(end_pos, lp->end_pos);
  memcpy(lp->start_quat, start_quat, sizeof(double) * 4);
  memcpy(lp->end_quat, end_quat, sizeof(double) * 4);

  /* 计算距离和方向 */
  double dx = end_pos[0] - start_pos[0];
  double dy = end_pos[1] - start_pos[1];
  double dz = end_pos[2] - start_pos[2];
  double L = sqrt(dx * dx + dy * dy + dz * dz);
  lp->L = L;

  if (L < 1e-6) {
    /* 起点与终点几乎重合，设为短暂原地保持 */
    lp->dir[0] = 0.0;
    lp->dir[1] = 0.0;
    lp->dir[2] = 0.0;
    lp->v_max = 0.0;
    lp->a = accel;
    lp->t_a = 0.0;
    lp->t_c = 0.02;
    lp->d_a = 0.0;
    lp->total_time = 0.02;
    return;
  }

  lp->dir[0] = dx / L;
  lp->dir[1] = dy / L;
  lp->dir[2] = dz / L;
  lp->a = accel;

  /* 梯形速度规划 */
  double d_a_ideal = 0.5 * speed * speed / accel;
  if (2.0 * d_a_ideal > L) {
    /* 退化为三角形 */
    lp->v_max = sqrt(accel * L);
    lp->t_a = lp->v_max / accel;
    lp->d_a = L / 2.0;
    lp->t_c = 0.0;
  } else {
    lp->v_max = speed;
    lp->t_a = speed / accel;
    lp->d_a = d_a_ideal;
    lp->t_c = (L - 2.0 * d_a_ideal) / speed;
  }
  lp->total_time = 2.0 * lp->t_a + lp->t_c;
}

void linear_path_evaluate(const LinearPathPlanner *lp, double t, double pos[3],
                          double quat[4]) {
  if (lp->L < 1e-6) {
    vec3_copy(lp->end_pos, pos);
    double ratio = t / lp->total_time;
    if (ratio > 1.0)
      ratio = 1.0;
    quat_slerp(lp->start_quat, lp->end_quat, ratio, quat);
    return;
  }

  /* 超出路径时间则停在终点 */
  if (t >= lp->total_time) {
    vec3_copy(lp->end_pos, pos);
    memcpy(quat, lp->end_quat, sizeof(double) * 4);
    return;
  }

  double s; /* 沿路径方向的位移 (m) */
  if (t < lp->t_a) {
    /* 加速段 */
    s = 0.5 * lp->a * t * t;
  } else if (t < lp->t_a + lp->t_c) {
    /* 匀速段 */
    double dt = t - lp->t_a;
    s = lp->d_a + lp->v_max * dt;
  } else {
    /* 减速段 */
    double dt = t - lp->t_a - lp->t_c;
    s = lp->d_a + lp->v_max * lp->t_c + lp->v_max * dt - 0.5 * lp->a * dt * dt;
    if (s > lp->L)
      s = lp->L;
  }

  pos[0] = lp->start_pos[0] + s * lp->dir[0];
  pos[1] = lp->start_pos[1] + s * lp->dir[1];
  pos[2] = lp->start_pos[2] + s * lp->dir[2];

  double ratio = s / lp->L;
  if (ratio > 1.0)
    ratio = 1.0;
  quat_slerp(lp->start_quat, lp->end_quat, ratio, quat);
}

int linear_path_reached(const LinearPathPlanner *lp, const double ee_pos[3],
                        double threshold) {
  double dx = ee_pos[0] - lp->end_pos[0];
  double dy = ee_pos[1] - lp->end_pos[1];
  double dz = ee_pos[2] - lp->end_pos[2];
  double dist = sqrt(dx * dx + dy * dy + dz * dz);
  return (dist < threshold) ? 1 : 0;
}
