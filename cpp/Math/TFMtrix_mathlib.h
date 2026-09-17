#ifndef __TFMatrix_mathlib_H__
#define __TFMatrix_mathlib_H__

#include "arm_math.h"
#include "kinematics_types.h"  // Vector3D_t, Coordinates_t

typedef struct{
	float64_t thita;
	float64_t d;
	float64_t a;
	float64_t alpha;
	
	float64_t offset;
}DH_Param_t;

extern const DH_Param_t DH_Param_I;

/* ==================== 位姿结构体 ==================== */

/**
 * @brief 6自由度位姿结构体 (ZYX 欧拉角, 弧度制)
 * @details 用于描述空间中的完整位姿
 *          位置: x, y, z (单位: m)
 *          姿态: yaw, pitch, roll (单位: rad)
 *          旋转顺序: Z(yaw) → Y(pitch) → X(roll)
 *          成员顺序与旋转应用顺序一致
 */
typedef struct {
    float64_t x;        // X 位置 (m)
    float64_t y;        // Y 位置 (m)
    float64_t z;        // Z 位置 (m)
    float64_t yaw;      // 绕 Z 轴旋转 (rad) - 第1步
    float64_t pitch;    // 绕 Y 轴旋转 (rad) - 第2步
    float64_t roll;     // 绕 X 轴旋转 (rad) - 第3步
} Pose6D_ZYX_t;

/**
 * @brief 6自由度位姿结构体 (ZYX 欧拉角, 角度制)
 * @details 成员顺序与旋转应用顺序一致: Z(yaw) → Y(pitch) → X(roll)
 */
typedef struct {
    float64_t x;            // X 位置 (m)
    float64_t y;            // Y 位置 (m)
    float64_t z;            // Z 位置 (m)
    float64_t yaw_deg;      // 绕 Z 轴旋转 (deg) - 第1步
    float64_t pitch_deg;    // 绕 Y 轴旋转 (deg) - 第2步
    float64_t roll_deg;     // 绕 X 轴旋转 (deg) - 第3步
} Pose6D_ZYX_Deg_t;

/**
 * @brief 6自由度位姿结构体 (ZYZ 欧拉角)
 * @details 旋转顺序: Z → Y → Z (yaw1 → pitch → yaw2)
 */
typedef struct {
    float64_t x;        // X 位置 (m)
    float64_t y;        // Y 位置 (m)
    float64_t z;        // Z 位置 (m)
    float64_t yaw1;     // 第一次绕 Z 轴旋转 (rad)
    float64_t pitch;    // 绕 Y 轴旋转 (rad)
    float64_t yaw2;     // 第二次绕 Z 轴旋转 (rad)
} Pose6D_ZYZ_t;

/**
 * @brief 6自由度位姿结构体 (ZYZ 欧拉角, 角度制)
 */
typedef struct {
    float64_t x;            // X 位置 (m)
    float64_t y;            // Y 位置 (m)
    float64_t z;            // Z 位置 (m)
    float64_t yaw1_deg;     // 第一次绕 Z 轴旋转 (deg)
    float64_t pitch_deg;    // 绕 Y 轴旋转 (deg)
    float64_t yaw2_deg;     // 第二次绕 Z 轴旋转 (deg)
} Pose6D_ZYZ_Deg_t;

/**
 * @brief 6自由度位姿结构体 (ZXZ 欧拉角)
 * @details 旋转顺序: Z → X → Z (yaw1 → roll → yaw2)
 *          常用于机械臂末端姿态描述
 */
typedef struct {
    float64_t x;        // X 位置 (m)
    float64_t y;        // Y 位置 (m)
    float64_t z;        // Z 位置 (m)
    float64_t yaw1;     // 第一次绕 Z 轴旋转 (rad)
    float64_t roll;     // 绕 X 轴旋转 (rad)
    float64_t yaw2;     // 第二次绕 Z 轴旋转 (rad)
} Pose6D_ZXZ_t;

/**
 * @brief 6自由度位姿结构体 (ZXZ 欧拉角, 角度制)
 */
typedef struct {
    float64_t x;            // X 位置 (m)
    float64_t y;            // Y 位置 (m)
    float64_t z;            // Z 位置 (m)
    float64_t yaw1_deg;     // 第一次绕 Z 轴旋转 (deg)
    float64_t roll_deg;     // 绕 X 轴旋转 (deg)
    float64_t yaw2_deg;     // 第二次绕 Z 轴旋转 (deg)
} Pose6D_ZXZ_Deg_t;

void my_mat_mult_f64(
  const arm_matrix_instance_f64 * pSrcA,
  const arm_matrix_instance_f64 * pSrcB,
        arm_matrix_instance_f64 * pDst);

/* ==================== 齐次变换点/向量操作 ==================== */

/**
 * @brief 将点从局部坐标系变换到世界坐标系
 * @details p_world = T * p_local (齐次坐标)
 *          p_world = R * p_local + t
 * @param T         4x4 齐次变换矩阵
 * @param p_local   局部坐标系中的点
 * @param p_world   输出: 世界坐标系中的点
 */
void TfMatrix_TransformPoint(const arm_matrix_instance_f64* T,
                             const Coordinates_t* p_local,
                             Coordinates_t* p_world);

/**
 * @brief 将向量从局部坐标系变换到世界坐标系（只旋转，不平移）
 * @details v_world = R * v_local (只取旋转部分)
 * @param T         4x4 齐次变换矩阵
 * @param v_local   局部坐标系中的向量
 * @param v_world   输出: 世界坐标系中的向量
 */
void TfMatrix_TransformVector(const arm_matrix_instance_f64* T,
                              const Vector3D_t* v_local,
                              Vector3D_t* v_world);

/**
 * @brief 将向量从世界坐标系变换到局部坐标系（只旋转，不平移）
 * @details v_local = R^T * v_world (利用旋转矩阵的正交性，R^T = R^-1)
 * @param T_local_to_world  4x4 齐次变换矩阵 (局部系到世界系)
 * @param v_world           世界坐标系中的向量
 * @param v_local           输出: 局部坐标系中的向量
 */
void TfMatrix_TransformVector_Inverse(const arm_matrix_instance_f64* T_local_to_world,
                                      const Vector3D_t* v_world, 
                                      Vector3D_t* v_local);

void DH_Param_Init(DH_Param_t*DH, float64_t thita, float64_t d, float64_t a, float64_t alpha);
void DH_Standard_TfMatrix_Create(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH);
void DH_Standard_Eye_TfMatrix_Create(arm_matrix_instance_f64* joint_tfmat);

/**
 * @brief 根据 DH 参数更新变换矩阵内容（不分配内存）
 * @param joint_tfmat 已初始化的矩阵结构体指针
 * @param DH          DH 参数
 * @note 矩阵必须已通过 TfMatrix_Init() 初始化
 */
void DH_Standard_TfMatrix_Update(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH);

/**
 * @brief 创建改进型 (Modified/Craig) DH 矩阵
 * @details 公式: T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
 *          坐标系建立在连杆始端（关节轴上），质心向量填正数更直观
 * @param joint_tfmat 已初始化的矩阵结构体指针
 * @param DH          DH 参数 (alpha_{i-1}, a_{i-1}, theta_i, d_i)
 */
void DH_Modified_TfMatrix_Create(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH);

/**
 * @brief 更新改进型 (Modified/Craig) DH 矩阵 (不分配内存)
 * @details 用于运行时更新关节角度
 * @param joint_tfmat 已初始化的矩阵结构体指针
 * @param DH          DH 参数
 */
void DH_Modified_TfMatrix_Update(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH);
  
void TfMatrix_For_Rotation_Around_X(arm_matrix_instance_f64* tfmatB, float64_t angle);
void TfMatrix_For_Rotation_Around_Y(arm_matrix_instance_f64* tfmatB, float64_t angle);
void TfMatrix_For_Rotation_Around_Z(arm_matrix_instance_f64* tfmatB, float64_t angle);
void TfMatrix_For_Translation_Along_X(arm_matrix_instance_f64* tfmatB, float64_t dis);
void TfMatrix_For_Translation_Along_Y(arm_matrix_instance_f64* tfmatB, float64_t dis);
void TfMatrix_For_Translation_Along_Z(arm_matrix_instance_f64* tfmatB, float64_t dis);

/* ==================== MATLAB 风格辅助函数 ==================== */

/**
 * @brief 统一的 4x4 变换矩阵初始化函数 (推荐使用)
 * @details 同时完成:
 *   1. arm_mat_init_f64() 初始化矩阵结构
 *   2. 设置为单位矩阵 eye(4)
 * @param tfmat 矩阵结构体指针
 * @param data  存储矩阵数据的 float64_t[16] 数组
 * @note 这是所有变换矩阵初始化的统一入口
 *       等效于 MATLAB: T = eye(4);
 */
void TfMatrix_Init(arm_matrix_instance_f64* tfmat, float64_t* data);

/**
 * @brief 初始化为单位矩阵 (对应 MATLAB eye(4))
 * @param tfmat 矩阵指针 (需已初始化 pData)
 * @note 仅设置为单位阵，不做 arm_mat_init_f64
 *       如需完整初始化请使用 TfMatrix_Init()
 */
void TfMatrix_Eye(arm_matrix_instance_f64* tfmat);
void TfMatrix_Transl(arm_matrix_instance_f64* start_tfmat, arm_matrix_instance_f64* end_tfmat, float64_t x, float64_t y, float64_t z);
void TfMatrix_Rotate_zyx(arm_matrix_instance_f64* start_tfmat, arm_matrix_instance_f64* end_tfmat, float64_t roll, float64_t pitch, float64_t yaw);
void TfMatrix_Rotate_zyz(arm_matrix_instance_f64* start_tfmat, arm_matrix_instance_f64* end_tfmat, float64_t yaw, float64_t pitch, float64_t roll);
void TfMatrix_Copy(const arm_matrix_instance_f64* src, arm_matrix_instance_f64* dst);

/**
 * @brief 从 ZYX 欧拉角位姿结构体设置变换矩阵 (弧度制)
 * @details 操作顺序: eye(4) → 平移(x,y,z) → 旋转ZYX(yaw,pitch,roll)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param pose  位姿参数 (rad)
 */
void TfMatrix_SetPose_ZYX(arm_matrix_instance_f64* tfmat, const Pose6D_ZYX_t* pose);

/**
 * @brief 从 ZYX 欧拉角位姿结构体设置变换矩阵 (角度制)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param pose  位姿参数 (deg)
 */
void TfMatrix_SetPose_ZYX_Deg(arm_matrix_instance_f64* tfmat, const Pose6D_ZYX_Deg_t* pose);

/**
 * @brief 从变换矩阵提取 ZYX 欧拉角位姿 (角度制)
 * @details 旋转提取顺序: Z(yaw) → Y(pitch) → X(roll)
 *          含 Gimbal Lock (pitch ~= ±90°) 基本处理
 * @param tfmat 已初始化的 4x4 变换矩阵 (pData 必须有效)
 * @param pose  输出位姿参数 (deg)
 */
void TfMatrix_GetPose_ZYX_Deg(const arm_matrix_instance_f64* tfmat, Pose6D_ZYX_Deg_t* pose);

/**
 * @brief 从变换矩阵提取 ZYX 欧拉角位姿到分立参数 (角度制)
 * @details 旋转提取顺序: Z(yaw) → Y(pitch) → X(roll)
 *          含 Gimbal Lock (pitch ~= ±90°) 基本处理
 * @param tfmat     已初始化的 4x4 变换矩阵 (pData 必须有效)
 * @param x, y, z   输出位置参数 (m) 的指针
 * @param yaw_deg, pitch_deg, roll_deg  输出 ZYX 欧拉角 (deg) 的指针
 */
void TfMatrix_Decompose_ZYX_Deg(const arm_matrix_instance_f64* tfmat,
                                 float64_t* x, float64_t* y, float64_t* z,
                                 float64_t* yaw_deg, float64_t* pitch_deg, float64_t* roll_deg);

/**
 * @brief 从分立参数组合 ZYX 变换矩阵 (弧度制)
 * @details ZYX 欧拉角旋转顺序: Z(yaw) → Y(pitch) → X(roll)
 *          参数顺序与旋转应用顺序一致，更符合直觉
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param x, y, z           位置 (m)
 * @param yaw, pitch, roll  ZYX 欧拉角 (rad)，按旋转应用顺序排列
 */
void TfMatrix_Compose_ZYX(arm_matrix_instance_f64* tfmat,
                           float64_t x, float64_t y, float64_t z,
                           float64_t yaw, float64_t pitch, float64_t roll);

/**
 * @brief 从分立参数组合 ZYX 变换矩阵 (角度制)
 * @details ZYX 欧拉角旋转顺序: Z(yaw) → Y(pitch) → X(roll)
 *          参数顺序与旋转应用顺序一致，更符合直觉
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param x, y, z                       位置 (m)
 * @param yaw_deg, pitch_deg, roll_deg  ZYX 欧拉角 (deg)，按旋转应用顺序排列
 */
void TfMatrix_Compose_ZYX_Deg(arm_matrix_instance_f64* tfmat,
                               float64_t x, float64_t y, float64_t z,
                               float64_t yaw_deg, float64_t pitch_deg, float64_t roll_deg);

/**
 * @brief 从 ZYZ 欧拉角位姿结构体设置变换矩阵 (弧度制)
 * @details 操作顺序: eye(4) → 平移(x,y,z) → 旋转ZYZ(yaw1,pitch,yaw2)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param pose  位姿参数 (rad)
 */
void TfMatrix_SetPose_ZYZ(arm_matrix_instance_f64* tfmat, const Pose6D_ZYZ_t* pose);

/**
 * @brief 从 ZYZ 欧拉角位姿结构体设置变换矩阵 (角度制)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param pose  位姿参数 (deg)
 */
void TfMatrix_SetPose_ZYZ_Deg(arm_matrix_instance_f64* tfmat, const Pose6D_ZYZ_Deg_t* pose);

/**
 * @brief 从分立参数组合 ZYZ 变换矩阵 (弧度制)
 * @details ZYZ 欧拉角旋转顺序: Z(yaw1) → Y(pitch) → Z(yaw2)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param x, y, z               位置 (m)
 * @param yaw1, pitch, yaw2     ZYZ 欧拉角 (rad)，按旋转应用顺序排列
 */
void TfMatrix_Compose_ZYZ(arm_matrix_instance_f64* tfmat,
                           float64_t x, float64_t y, float64_t z,
                           float64_t yaw1, float64_t pitch, float64_t yaw2);

/**
 * @brief 从分立参数组合 ZYZ 变换矩阵 (角度制)
 * @details ZYZ 欧拉角旋转顺序: Z(yaw1) → Y(pitch) → Z(yaw2)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param x, y, z                           位置 (m)
 * @param yaw1_deg, pitch_deg, yaw2_deg     ZYZ 欧拉角 (deg)，按旋转应用顺序排列
 */
void TfMatrix_Compose_ZYZ_Deg(arm_matrix_instance_f64* tfmat,
                               float64_t x, float64_t y, float64_t z,
                               float64_t yaw1_deg, float64_t pitch_deg, float64_t yaw2_deg);

/* ==================== ZXZ 欧拉角位姿转变换矩阵函数 ==================== */

/**
 * @brief 从 ZXZ 欧拉角位姿结构体设置变换矩阵 (弧度制)
 * @details 操作顺序: eye(4) → 平移(x,y,z) → 旋转ZXZ(yaw1,roll,yaw2)
 *          旋转顺序: 先绕 Z 轴旋转 yaw1, 再绕 X 轴旋转 roll, 最后绕 Z 轴旋转 yaw2
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param pose  位姿参数 (rad)
 */
void TfMatrix_SetPose_ZXZ(arm_matrix_instance_f64* tfmat, const Pose6D_ZXZ_t* pose);

/**
 * @brief 从 ZXZ 欧拉角位姿结构体设置变换矩阵 (角度制)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param pose  位姿参数 (deg)
 */
void TfMatrix_SetPose_ZXZ_Deg(arm_matrix_instance_f64* tfmat, const Pose6D_ZXZ_Deg_t* pose);

/**
 * @brief 从分立参数组合 ZXZ 变换矩阵 (弧度制)
 * @details ZXZ 欧拉角旋转顺序: Z(yaw1) → X(roll) → Z(yaw2)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param x, y, z               位置 (m)
 * @param yaw1, roll, yaw2      ZXZ 欧拉角 (rad)，按旋转应用顺序排列
 */
void TfMatrix_Compose_ZXZ(arm_matrix_instance_f64* tfmat,
                           float64_t x, float64_t y, float64_t z,
                           float64_t yaw1, float64_t roll, float64_t yaw2);

/**
 * @brief 从分立参数组合 ZXZ 变换矩阵 (角度制)
 * @details ZXZ 欧拉角旋转顺序: Z(yaw1) → X(roll) → Z(yaw2)
 * @param tfmat 已分配内存的矩阵结构体指针 (pData 必须有效)
 * @param x, y, z                           位置 (m)
 * @param yaw1_deg, roll_deg, yaw2_deg      ZXZ 欧拉角 (deg)，按旋转应用顺序排列
 */
void TfMatrix_Compose_ZXZ_Deg(arm_matrix_instance_f64* tfmat,
                               float64_t x, float64_t y, float64_t z,
                               float64_t yaw1_deg, float64_t roll_deg, float64_t yaw2_deg);

/**
 * @brief SE(3) 矩阵求逆 (齐次变换矩阵求逆)
 * @details 对于 SE(3) 矩阵 T = [R, t; 0, 1], 其逆为:
 *          T^(-1) = [R^T, -R^T * t; 0, 1]
 *          此函数利用 SE(3) 的特殊结构，避免通用矩阵求逆的计算开销
 * @param T     输入的 4x4 齐次变换矩阵
 * @param T_inv 输出的逆矩阵
 */
void TfMatrix_Inverse_SE3(const arm_matrix_instance_f64* T, arm_matrix_instance_f64* T_inv);

/* ==================== 位姿误差计算函数 ==================== */

/**
 * @brief 计算两个变换矩阵的位置误差 (欧氏距离)
 * @details 比较两个 4x4 齐次变换矩阵的平移向量差
 *          error = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
 * @param T1 第一个变换矩阵
 * @param T2 第二个变换矩阵
 * @return 位置误差 (m)
 */
float64_t TfMatrix_PositionError(const arm_matrix_instance_f64* T1, const arm_matrix_instance_f64* T2);

/**
 * @brief 计算两个变换矩阵的旋转误差 (Frobenius 范数)
 * @details 比较两个 4x4 齐次变换矩阵的 3x3 旋转部分
 *          error = sqrt(sum((R1[i,j] - R2[i,j])^2))
 * @param T1 第一个变换矩阵
 * @param T2 第二个变换矩阵
 * @return 旋转误差 (Frobenius 范数)
 * @note 对于接近相同的旋转矩阵，误差接近 0
 *       对于相差 180° 的旋转，误差约为 2*sqrt(2) ≈ 2.83
 */
float64_t TfMatrix_RotationError(const arm_matrix_instance_f64* T1, const arm_matrix_instance_f64* T2);

/**
 * @brief 同时计算位置误差和旋转误差
 * @param T1 第一个变换矩阵
 * @param T2 第二个变换矩阵
 * @param pos_error  输出: 位置误差 (m)
 * @param rot_error  输出: 旋转误差 (Frobenius 范数)
 */
void TfMatrix_PoseError(const arm_matrix_instance_f64* T1, const arm_matrix_instance_f64* T2,
                        float64_t* pos_error, float64_t* rot_error);

/* ==================== 绕轴旋转与平移操作 ==================== */

/**
 * @brief 绕定点旋转变换 (罗德里格斯公式)
 * @details 
 *   使用罗德里格斯公式: R = I + sin(θ)*K + (1-cos(θ))*K?
 *   绕点旋转: T_action = [R, (I-R)*pivot; 0 0 0 1]
 *   结果: T_result = T_action * T_current
 * @param T_current    当前变换矩阵
 * @param axis_vec     旋转轴向量 (会自动归一化)
 * @param pivot_point  旋转轴上的点 (定点)
 * @param angle        旋转角度 (rad)
 * @param T_result     输出结果矩阵
 */
void TfMatrix_RotateAroundAxis(const arm_matrix_instance_f64* T_current,
                               const Vector3D_t* axis_vec,
                               const Coordinates_t* pivot_point,
                               float64_t angle,
                               arm_matrix_instance_f64* T_result);

/**
 * @brief 创建沿向量平移的变换矩阵
 * @details 
 *   创建一个沿指定方向平移指定距离的 4x4 齐次变换矩阵
 *   T = [I, d*dir; 0 0 0 1]
 * @param T_result   输出结果矩阵 (必须已初始化)
 * @param direction  平移方向向量 (会自动归一化)
 * @param distance   平移距离
 */
void TfMatrix_TranslateAlongVector(arm_matrix_instance_f64* T_result,
                                   const Vector3D_t* direction,
                                   float64_t distance);

void TFMatrix_NormalizeRotation(arm_matrix_instance_f64* T);
void TFMatrix_NormalizeRotation_PreserveZ(arm_matrix_instance_f64* T);

#endif
