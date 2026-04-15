/*
 * AM-D02 机器人控制器数学库
 * 针对 STM32H7 DP-FPU 兼容性，统一使用 'double' 类型。
 * 整合了 math_utils 和 rbdl_math。
 */

#ifndef MATH_LIB_H
#define MATH_LIB_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef fmax
#define fmax(a, b) ((a) > (b) ? (a) : (b))
#endif

/* ================================================================
 *  3D 向量操作 (double[3])
 * ================================================================ */

static inline void vec3_zero(double v[3]) { v[0] = v[1] = v[2] = 0.0; }

static inline void vec3_set(double v[3], double x, double y, double z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

static inline void vec3_copy(const double s[3], double d[3]) {
  d[0] = s[0];
  d[1] = s[1];
  d[2] = s[2];
}

static inline void vec3_add(const double a[3], const double b[3], double c[3]) {
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
}

static inline void vec3_sub(const double a[3], const double b[3], double c[3]) {
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
}

static inline void vec3_scale(const double v[3], double s, double out[3]) {
  out[0] = v[0] * s;
  out[1] = v[1] * s;
  out[2] = v[2] * s;
}

static inline double vec3_dot(const double a[3], const double b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline void vec3_cross(const double a[3], const double b[3],
                              double c[3]) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

static inline double vec3_norm(const double v[3]) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static inline void vec3_normalize(double v[3]) {
  double n = vec3_norm(v);
  if (n > 1e-15) {
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
  }
}

/* ================================================================
 *  3x3 矩阵操作 (double[9], 行主序)
 * ================================================================ */

static inline void mat3_identity(double M[9]) {
  M[0] = 1;
  M[1] = 0;
  M[2] = 0;
  M[3] = 0;
  M[4] = 1;
  M[5] = 0;
  M[6] = 0;
  M[7] = 0;
  M[8] = 1;
}

static inline void mat3_zero(double M[9]) { memset(M, 0, 9 * sizeof(double)); }

static inline void mat3_copy(const double s[9], double d[9]) {
  memcpy(d, s, 9 * sizeof(double));
}

static inline void mat3_transpose(const double M[9], double Mt[9]) {
  Mt[0] = M[0];
  Mt[1] = M[3];
  Mt[2] = M[6];
  Mt[3] = M[1];
  Mt[4] = M[4];
  Mt[5] = M[7];
  Mt[6] = M[2];
  Mt[7] = M[5];
  Mt[8] = M[8];
}

static inline void mat3_mul_vec3(const double M[9], const double v[3],
                                 double out[3]) {
  out[0] = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
  out[1] = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
  out[2] = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
}

static inline void mat3T_mul_vec3(const double M[9], const double v[3],
                                  double out[3]) {
  out[0] = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
  out[1] = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
  out[2] = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
}

/**
 * @brief 3x3矩阵乘法：C = A * B
 */
void mat3_mul(const double A[9], const double B[9], double C[9]);

/**
 * @brief 3x3矩阵求逆
 * @return 1 表示成功，0 表示矩阵不可逆（奇异）
 */
uint8_t mat3_inverse(const double m[9], double inv[9]);

/* ================================================================
 *  4x4 齐次变换矩阵操作 (double[16], 行主序)
 * ================================================================ */

static inline void mat4_identity(double M[16]) {
  memset(M, 0, 16 * sizeof(double));
  M[0] = 1;
  M[5] = 1;
  M[10] = 1;
  M[15] = 1;
}

static inline void mat4_from_rot_trans(const double R[9], const double t[3],
                                       double T[16]) {
  T[0] = R[0];
  T[1] = R[1];
  T[2] = R[2];
  T[3] = t[0];
  T[4] = R[3];
  T[5] = R[4];
  T[6] = R[5];
  T[7] = t[1];
  T[8] = R[6];
  T[9] = R[7];
  T[10] = R[8];
  T[11] = t[2];
  T[12] = 0;
  T[13] = 0;
  T[14] = 0;
  T[15] = 1;
}

/**
 * @brief 4x4齐次变换矩阵乘法：C = A * B
 */
void mat4_mul(const double A[16], const double B[16], double C[16]);

/**
 * @brief 4x4矩阵转置
 */
void mat4_transpose(const double a[16], double result[16]);

static inline void mat4_get_rotation(const double T[16], double R[9]) {
  R[0] = T[0];
  R[1] = T[1];
  R[2] = T[2];
  R[3] = T[4];
  R[4] = T[5];
  R[5] = T[6];
  R[6] = T[8];
  R[7] = T[9];
  R[8] = T[10];
}

static inline void mat4_get_position(const double T[16], double p[3]) {
  p[0] = T[3];
  p[1] = T[7];
  p[2] = T[11];
}

static inline void mat4_rot_vec3(const double T[16], const double v[3],
                                 double out[3]) {
  out[0] = T[0] * v[0] + T[1] * v[1] + T[2] * v[2];
  out[1] = T[4] * v[0] + T[5] * v[1] + T[6] * v[2];
  out[2] = T[8] * v[0] + T[9] * v[1] + T[10] * v[2];
}

/* ================================================================
 *  四元数操作 (double[4]: [x, y, z, w])
 * ================================================================ */

/**
 * @brief 欧拉角 (Roll, Pitch, Yaw) 转 四元数
 */
void quat_from_euler(double roll, double pitch, double yaw, double q[4]);

/**
 * @brief 四元数 转 欧拉角 (Roll, Pitch, Yaw)
 */
void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw);

/**
 * @brief 四元数乘法：out = q1 * q2
 */
void quat_mul(const double q1[4], const double q2[4], double out[4]);

/**
 * @brief 四元数共轭：out = [-x, -y, -z, w]
 */
void quat_conjugate(const double q[4], double out[4]);

/**
 * @brief 四元数求模
 */
double quat_norm(const double q[4]);

/**
 * @brief 四元数归一化
 */
void quat_normalize(double q[4]);

/**
 * @brief 3x3旋转矩阵转四元数
 */
void rotmat_to_quat(const double R[9], double q[4]);

/**
 * @brief 四元数球面线性插值
 */
void quat_slerp(const double q1[4], const double q2[4], double t,
                double out[4]);

/* ================================================================
 *  旋转与转换
 * ================================================================ */

/**
 * @brief 欧拉角 (Roll, Pitch, Yaw) 转旋转矩阵 (3x3)
 */
void rpy_to_rotmat(double roll, double pitch, double yaw, double R[9]);

/**
 * @brief 轴角 (Axis-Angle) 转旋转矩阵 (3x3)
 * @param axis 旋转轴向量
 * @param angle 旋转角度 (弧度)
 */
void axis_angle_to_rotmat(const double axis[3], double angle, double R[9]);

/* ================================================================
 *  6D 空间向量操作 (Featherstone)
 * ================================================================ */

static inline void sv_zero(double v[6]) { memset(v, 0, 6 * sizeof(double)); }

static inline void sv_copy(const double s[6], double d[6]) {
  memcpy(d, s, 6 * sizeof(double));
}

static inline void sv_add(const double a[6], const double b[6], double c[6]) {
  for (int i = 0; i < 6; i++)
    c[i] = a[i] + b[i];
}

/**
 * @brief 空间向量运动叉积 (Motion Cross Product) v1 x_m v2
 */
void sv_crossm(const double v1[6], const double v2[6], double out[6]);

/**
 * @brief 空间向量力叉积 (Force Cross Product) v x_f f
 */
void sv_crossf(const double v[6], const double f[6], double out[6]);

/**
 * @brief 空间变换矩阵对运动向量的作用 (Spatial Transform Apply)
 * @param E 3x3 旋转矩阵
 * @param r 3D 平移向量
 */
void st_apply(const double E[9], const double r[3], const double v[6],
              double out[6]);

/**
 * @brief 空间变换矩阵转置对力向量的作用
 */
void st_apply_transpose(const double E[9], const double r[3], const double f[6],
                        double out[6]);

/**
 * @brief 复合两个空间变换 E_out = E1 * E2, r_out = r2 + (E2^T * r1)
 */
void st_compose(const double E1[9], const double r1[3], const double E2[9],
                const double r2[3], double E_out[9], double r_out[3]);

/**
 * @brief 空间惯性矩阵乘以运动向量得到力向量：f = I * v
 * @param mass 连杆质量
 * @param h 质心相对于连杆坐标原点的坐标乘以质量 (h = mass * com)
 * @param Io 原点处的惯性张量 (6个独立元素，表示 3x3 对称矩阵)
 */
void si_mul_vec(double mass, const double h[3], const double Io[6],
                const double v[6], double out[6]);

/* ================================================================
 *  6x6 矩阵操作 (固定大小)
 * ================================================================ */

/**
 * @brief 6x6 矩阵求逆 (使用高斯消元法)
 * @return 1 表示成功，0 表示矩阵奇异
 */
uint8_t mat6_inverse(const double a[36], double inv[36]);

/* ================================================================
 *  1D 卡尔曼滤波器 (用于速度滤波)
 * ================================================================ */

typedef struct {
  double x;           /* 状态估计值 */
  double P;           /* 估计协方差 */
  double Q;           /* 过程噪声协方差 */
  double R;           /* 测量噪声协方差 */
  uint8_t initialized;
} KalmanFilter1D;

/**
 * @brief 初始化 1D 卡尔曼滤波器
 */
void kalman_filter1d_init(KalmanFilter1D *f, double Q, double R);

/**
 * @brief 更新 1D 卡尔曼滤波器
 * @param z 测量值
 * @return 滤波后的估计值
 */
double kalman_filter1d_update(KalmanFilter1D *f, double z);

#ifdef __cplusplus
}
#endif

#endif /* MATH_LIB_H */
