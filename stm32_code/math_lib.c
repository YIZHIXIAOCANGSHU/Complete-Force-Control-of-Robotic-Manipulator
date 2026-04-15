#include "math_lib.h"

/* ================================================================
 *  3x3 Matrix Operations (3x3 矩阵操作)
 * ================================================================ */

void mat3_mul(const double A[9], const double B[9], double C[9]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C[i * 3 + j] = A[i * 3 + 0] * B[0 * 3 + j] + A[i * 3 + 1] * B[1 * 3 + j] +
                     A[i * 3 + 2] * B[2 * 3 + j];
    }
  }
}

uint8_t mat3_inverse(const double m[9], double inv[9]) {
  double det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);

  if (fabs(det) < 1e-15)
    return 0;

  double inv_det = 1.0 / det;

  inv[0] = (m[4] * m[8] - m[7] * m[5]) * inv_det;
  inv[1] = (m[2] * m[7] - m[1] * m[8]) * inv_det;
  inv[2] = (m[1] * m[5] - m[2] * m[4]) * inv_det;

  inv[3] = (m[5] * m[6] - m[3] * m[8]) * inv_det;
  inv[4] = (m[0] * m[8] - m[2] * m[6]) * inv_det;
  inv[5] = (m[2] * m[3] - m[0] * m[5]) * inv_det;

  inv[6] = (m[3] * m[7] - m[4] * m[6]) * inv_det;
  inv[7] = (m[1] * m[6] - m[0] * m[7]) * inv_det;
  inv[8] = (m[0] * m[4] - m[1] * m[3]) * inv_det;

  return 1;
}

/* ================================================================
 *  4x4 Matrix Operations (4x4 齐次变换矩阵操作)
 * ================================================================ */

void mat4_mul(const double A[16], const double B[16], double C[16]) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      double s = 0.0;
      for (int k = 0; k < 4; k++)
        s += A[i * 4 + k] * B[k * 4 + j];
      C[i * 4 + j] = s;
    }
  }
}

void mat4_transpose(const double a[16], double result[16]) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++)
      result[j * 4 + i] = a[i * 4 + j];
  }
}

/* ================================================================
 *  Quaternion Operations (四元数操作)
 * ================================================================ */

void quat_from_euler(double roll, double pitch, double yaw, double q[4]) {
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q[0] = sr * cp * cy - cr * sp * sy;
  q[1] = cr * sp * cy + sr * cp * sy;
  q[2] = cr * cp * sy - sr * sp * cy;
  q[3] = cr * cp * cy + sr * sp * sy;
}

void quat_to_euler(const double q[4], double *roll, double *pitch,
                   double *yaw) {
  double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  *roll = atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1.0) {
    *pitch = copysign(M_PI / 2.0, sinp);
  } else {
    *pitch = asin(sinp);
  }

  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  *yaw = atan2(siny_cosp, cosy_cosp);
}

void quat_mul(const double q1[4], const double q2[4], double out[4]) {
  double q1x = q1[0], q1y = q1[1], q1z = q1[2], q1w = q1[3];
  double q2x = q2[0], q2y = q2[1], q2z = q2[2], q2w = q2[3];
  out[0] = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y;
  out[1] = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x;
  out[2] = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w;
  out[3] = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z;
}

void quat_conjugate(const double q[4], double out[4]) {
  out[0] = -q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = q[3];
}

double quat_norm(const double q[4]) {
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void quat_normalize(double q[4]) {
  double n = quat_norm(q);
  if (n > 1e-15) {
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  }
}

void rotmat_to_quat(const double R[9], double q[4]) {
  double trace = R[0] + R[4] + R[8];
  double w = 0.5 * sqrt(fmax(1e-12, 1.0 + trace));
  double inv4w = 1.0 / (4.0 * w + 1e-15);
  q[0] = (R[7] - R[5]) * inv4w;
  q[1] = (R[2] - R[6]) * inv4w;
  q[2] = (R[3] - R[1]) * inv4w;
  q[3] = w;
}

void quat_slerp(const double q1[4], const double q2[4], double t,
                double out[4]) {
  double cosHalfTheta =
      q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  double q2_adj[4] = {q2[0], q2[1], q2[2], q2[3]};
  if (cosHalfTheta < 0.0) {
    q2_adj[0] = -q2[0];
    q2_adj[1] = -q2[1];
    q2_adj[2] = -q2[2];
    q2_adj[3] = -q2[3];
    cosHalfTheta = -cosHalfTheta;
  }
  if (cosHalfTheta >= 1.0) {
    out[0] = q1[0];
    out[1] = q1[1];
    out[2] = q1[2];
    out[3] = q1[3];
    return;
  }
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
  if (fabs(sinHalfTheta) < 1e-3) {
    out[0] = q1[0] * (1.0 - t) + q2_adj[0] * t;
    out[1] = q1[1] * (1.0 - t) + q2_adj[1] * t;
    out[2] = q1[2] * (1.0 - t) + q2_adj[2] * t;
    out[3] = q1[3] * (1.0 - t) + q2_adj[3] * t;
    quat_normalize(out);
    return;
  }
  double ratioA = sin((1.0 - t) * halfTheta) / sinHalfTheta;
  double ratioB = sin(t * halfTheta) / sinHalfTheta;
  out[0] = q1[0] * ratioA + q2_adj[0] * ratioB;
  out[1] = q1[1] * ratioA + q2_adj[1] * ratioB;
  out[2] = q1[2] * ratioA + q2_adj[2] * ratioB;
  out[3] = q1[3] * ratioA + q2_adj[3] * ratioB;
}

/* ================================================================
 *  Rotations (旋转转换)
 * ================================================================ */

void rpy_to_rotmat(double roll, double pitch, double yaw, double R[9]) {
  double cr = cos(roll), sr = sin(roll);
  double cp = cos(pitch), sp = sin(pitch);
  double cy = cos(yaw), sy = sin(yaw);
  R[0] = cy * cp;
  R[1] = cy * sp * sr - sy * cr;
  R[2] = cy * sp * cr + sy * sr;
  R[3] = sy * cp;
  R[4] = sy * sp * sr + cy * cr;
  R[5] = sy * sp * cr - cy * sr;
  R[6] = -sp;
  R[7] = cp * sr;
  R[8] = cp * cr;
}

void axis_angle_to_rotmat(const double axis[3], double angle, double R[9]) {
  double ax = axis[0], ay = axis[1], az = axis[2];
  double n = sqrt(ax * ax + ay * ay + az * az);
  if (n < 1e-15) {
    mat3_identity(R);
    return;
  }
  ax /= n;
  ay /= n;
  az /= n;

  double c = cos(angle), s = sin(angle), t = 1.0 - c;
  R[0] = t * ax * ax + c;
  R[1] = t * ax * ay - s * az;
  R[2] = t * ax * az + s * ay;
  R[3] = t * ax * ay + s * az;
  R[4] = t * ay * ay + c;
  R[5] = t * ay * az - s * ax;
  R[6] = t * ax * az - s * ay;
  R[7] = t * ay * az + s * ax;
  R[8] = t * az * az + c;
}

/* ================================================================
 *  Spatial Vector Operations (6D 空间向量操作 - Featherstone 算法)
 * ================================================================ */

void sv_crossm(const double v1[6], const double v2[6], double out[6]) {
  out[0] = -v1[2] * v2[1] + v1[1] * v2[2];
  out[1] = v1[2] * v2[0] - v1[0] * v2[2];
  out[2] = -v1[1] * v2[0] + v1[0] * v2[1];
  out[3] = -v1[5] * v2[1] + v1[4] * v2[2] - v1[2] * v2[4] + v1[1] * v2[5];
  out[4] = v1[5] * v2[0] - v1[3] * v2[2] + v1[2] * v2[3] - v1[0] * v2[5];
  out[5] = -v1[4] * v2[0] + v1[3] * v2[1] - v1[1] * v2[3] + v1[0] * v2[4];
}

void sv_crossf(const double v[6], const double f[6], double out[6]) {
  out[0] = -v[2] * f[1] + v[1] * f[2] - v[5] * f[4] + v[4] * f[5];
  out[1] = v[2] * f[0] - v[0] * f[2] + v[5] * f[3] - v[3] * f[5];
  out[2] = -v[1] * f[0] + v[0] * f[1] - v[4] * f[3] + v[3] * f[4];
  out[3] = -v[2] * f[4] + v[1] * f[5];
  out[4] = v[2] * f[3] - v[0] * f[5];
  out[5] = -v[1] * f[3] + v[0] * f[4];
}

void st_apply(const double E[9], const double r[3], const double v[6],
              double out[6]) {
  double rxw[3];
  vec3_cross(r, v, rxw);
  double temp[3] = {v[3] - rxw[0], v[4] - rxw[1], v[5] - rxw[2]};
  mat3_mul_vec3(E, v, out);
  mat3_mul_vec3(E, temp, out + 3);
}

void st_apply_transpose(const double E[9], const double r[3], const double f[6],
                        double out[6]) {
  double Et_fl[3], Et_fa[3], rxEtfl[3];
  mat3T_mul_vec3(E, f + 3, Et_fl);
  mat3T_mul_vec3(E, f, Et_fa);
  vec3_cross(r, Et_fl, rxEtfl);
  out[0] = Et_fa[0] + rxEtfl[0];
  out[1] = Et_fa[1] + rxEtfl[1];
  out[2] = Et_fa[2] + rxEtfl[2];
  out[3] = Et_fl[0];
  out[4] = Et_fl[1];
  out[5] = Et_fl[2];
}

void st_compose(const double E1[9], const double r1[3], const double E2[9],
                const double r2[3], double E_out[9], double r_out[3]) {
  mat3_mul(E1, E2, E_out);
  double Et2_r1[3];
  mat3T_mul_vec3(E2, r1, Et2_r1);
  vec3_add(r2, Et2_r1, r_out);
}

void si_mul_vec(double mass, const double h[3], const double Io[6],
                const double v[6], double out[6]) {
  out[0] = Io[0] * v[0] + Io[1] * v[1] + Io[2] * v[2];
  out[1] = Io[1] * v[0] + Io[3] * v[1] + Io[4] * v[2];
  out[2] = Io[2] * v[0] + Io[4] * v[1] + Io[5] * v[2];
  out[0] += h[1] * v[5] - h[2] * v[4];
  out[1] += h[2] * v[3] - h[0] * v[5];
  out[2] += h[0] * v[4] - h[1] * v[3];
  out[3] = mass * v[3] + v[1] * h[2] - v[2] * h[1];
  out[4] = mass * v[4] + v[2] * h[0] - v[0] * h[2];
  out[5] = mass * v[5] + v[0] * h[1] - v[1] * h[0];
}

/* ================================================================
 *  Matrix 6x6 Operations (6x6 矩阵操作)
 * ================================================================ */

uint8_t mat6_inverse(const double a[36], double inv[36]) {
  double temp[36];
  memcpy(temp, a, 36 * sizeof(double));
  memset(inv, 0, 36 * sizeof(double));
  for (int i = 0; i < 6; i++)
    inv[i * 6 + i] = 1.0;

  for (int i = 0; i < 6; i++) {
    double pivot = temp[i * 6 + i];
    int pivotRow = i;
    for (int j = i + 1; j < 6; j++) {
      if (fabs(temp[j * 6 + i]) > fabs(pivot)) {
        pivot = temp[j * 6 + i];
        pivotRow = j;
      }
    }
    if (fabs(pivot) < 1e-15)
      return 0;

    if (pivotRow != i) {
      for (int k = 0; k < 6; k++) {
        double t = temp[i * 6 + k];
        temp[i * 6 + k] = temp[pivotRow * 6 + k];
        temp[pivotRow * 6 + k] = t;
        t = inv[i * 6 + k];
        inv[i * 6 + k] = inv[pivotRow * 6 + k];
        inv[pivotRow * 6 + k] = t;
      }
    }

    double div = temp[i * 6 + i];
    for (int k = 0; k < 6; k++) {
      temp[i * 6 + k] /= div;
      inv[i * 6 + k] /= div;
    }

    for (int j = 0; j < 6; j++) {
      if (j != i) {
        double factor = temp[j * 6 + i];
        for (int k = 0; k < 6; k++) {
          temp[j * 6 + k] -= factor * temp[i * 6 + k];
          inv[j * 6 + k] -= factor * inv[i * 6 + k];
        }
      }
    }
  }
  return 1;
}

/* ================================================================
 *  1D 卡尔曼滤波器实现
 * ================================================================ */

void kalman_filter1d_init(KalmanFilter1D *f, double Q, double R) {
  f->Q = Q;
  f->R = R;
  f->P = 1.0;
  f->x = 0.0;
  f->initialized = 0;
}

double kalman_filter1d_update(KalmanFilter1D *f, double z) {
  if (!f->initialized) {
    f->x = z;
    f->initialized = 1;
    return f->x;
  }

  /* 1. 预测 */
  /* x_p = x, P_p = P + Q */
  f->P = f->P + f->Q;

  /* 2. 更新 (测量反馈) */
  /* K = P_p / (P_p + R) */
  double K = f->P / (f->P + f->R);
  /* x = x_p + K * (z - x_p) */
  f->x = f->x + K * (z - f->x);
  /* P = (1 - K) * P_p */
  f->P = (1.0 - K) * f->P;

  return f->x;
}
