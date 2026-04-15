/*
 * AM-D02 机器人模型库
 * 整合了 am_d02_model, rbdl_model 和 rbdl_lite。
 */

#ifndef MODEL_LIB_H
#define MODEL_LIB_H

#include "config.h"
#include "math_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  基础类型 (DH, 位姿, 变换)
 * ================================================================ */

/**
 * @brief DH 参数结构体
 */
typedef struct {
  double d;     /* 连杆偏移 (沿着上一 Z 轴的距离) */
  double theta; /* 关节角偏移 (绕着上一 Z 轴的旋转角) */
  double a;     /* 连杆长度 (沿着 X 轴的距离) */
  double alpha; /* 连杆扭角 (绕着 X 轴的旋转角) */
} DHParameters;

/**
 * @brief 4x4 变换矩阵
 */
typedef struct {
  double m[4][4];
} TransformMatrix;

/**
 * @brief 四元数: [x, y, z, w]
 */
typedef struct {
  double x;
  double y;
  double z;
  double w;
} Quaternion;

/**
 * @brief 空间位姿 (包含平移与四元数旋转)
 */
typedef struct {
  double position[3];     /* 位置 [x, y, z] */
  Quaternion orientation; /* 姿态四元数 [x, y, z, w] */
} Pose;

/* ================================================================
 *  RBDL-Lite 结构体定义
 * ================================================================ */

#define MAX_BODIES 10

/**
 * @brief 刚体 (Link) 动力学参数与运行状态缓存
 */
typedef struct {
  int id;     /* 连杆 ID */
  int parent; /* 父节点 (连杆) ID */

  /* 惯性属性 */
  double mass;       /* 质量 */
  double mhc[3];     /* 质量与质心坐标的乘积 (mass * COM) */
  double inertia[6]; /* 在关节原点处的惯性张量 (Ixx, Ixy, Ixz, Iyy, Iyz, Izz) */

  /* 坐标系变换 (父节点 -> 子节点, 当关节角 q=0 时) */
  double E_p[9]; /* E_p: 旋转矩阵 */
  double r_p[3]; /* r_p: 平移向量 */

  /* 关节属性 */
  double axis[3]; /* 子坐标系下的关节转轴向量 */
  double S[6];    /* 运动子空间 (表示关节在空间内的运动轴) */

  /* 运动学缓存数据 (每次前向运动学计算更新) */
  double X_p[6]; /* 空间变换矩阵 父 -> 子 */
  double X_0[6]; /* 空间变换矩阵 World -> 子 */
  double E_0[9]; /* 绝对旋转矩阵 World -> 子 */
  double r_0[3]; /* 绝对平移向量 (如有需要) */

  /* 动力学缓存数据 */
  double v[6]; /* 空间速度 */
  double a[6]; /* 空间加速度 */
  double f[6]; /* 空间力 */
} Body;

/**
 * @brief RBDL 树形拓扑动力学模型
 */
typedef struct {
  int num_bodies;          /* 模型刚体(连杆)数量 */
  Body bodies[MAX_BODIES]; /* 刚体数组列表 */
  double gravity[3];       /* 系统重力向量 [x, y, z] */
} RBDLModel;

/* ================================================================
 *  AM-D02-AemLURDF0413 模型常量
 * ================================================================ */

extern const double AM_D02_JOINT_XYZ[NUM_JOINTS][3];
extern const double AM_D02_JOINT_RPY[NUM_JOINTS][3];
extern const double AM_D02_JOINT_AXIS[NUM_JOINTS][3];
extern const double AM_D02_LINK_MASS[NUM_JOINTS];
extern const double AM_D02_LINK_COM[NUM_JOINTS][3];
extern const double AM_D02_LINK_INERTIA[NUM_JOINTS][6];

/* ================================================================
 *  机器人特定模型
 * ================================================================ */

/**
 * @brief 构建 AM-D02 机器人的原厂 URDF 对应的 RBDL 动力学模型
 */
void build_am_d02_model(RBDLModel *model);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_LIB_H */
