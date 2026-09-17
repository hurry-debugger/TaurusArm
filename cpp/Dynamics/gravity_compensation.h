#ifndef __GRAVITY_COMPENSATION_H__
#define __GRAVITY_COMPENSATION_H__

#include "main.h"
#include "kinematics_types.h"
#include "vector_mathlib.h"
#include "TFMtrix_mathlib.h"
#include "forward_kinematics.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

/* ==================== 常量定义 ==================== */
#ifndef GRAVITY_ACC
	#define GRAVITY_ACC 9.80665
#endif


/* ==================== 数据类型定义 ==================== */

/**
 * @brief 单个连杆的动力学参数
 * @details DH 参数统一从 forward_kinematics 模块获取，此处仅保留质量和质心
 *          质心用 MM2M() 宏填入, 直接从 SolidWorks 抄 mm 值
 */
typedef struct {
	float64_t mass;          // 连杆质量 (kg)
	Vector3D_t com;          // 质心位置，相对于当前连杆 MDH 坐标系 (m)
} LinkParam_t;

/**
 * @brief 重力补偿计算结构体
 * @details 正运动学部分复用 forward_kinematics 模块的 T_0_to_i[]
 */
typedef struct {
	// 输入: 关节角度
	float64_t q[NUM_JOINTS];
	
	// 中间变量: 从 FK 的 T_0_to_i 中提取
	Vector3D_t P_com_global[NUM_JOINTS];   // 每个质心的绝对坐标
	Vector3D_t Z_axis[NUM_JOINTS];         // 每个关节轴的Z方向 (旋转轴)
	Vector3D_t P_joint[NUM_JOINTS];        // 每个关节轴的原点位置
	
	// 输出: 补偿力矩
	float64_t tau[NUM_JOINTS];
	int8_t tau_dirt[NUM_JOINTS];
} GravComp_t;

/* ==================== 外部变量声明 ==================== */
extern DTCM_RAM GravComp_t g_gravcomp;
extern DTCM_RAM LinkParam_t g_link_params[NUM_JOINTS];
extern const Vector3D_t GRAVITY_VEC;

/* ==================== API函数声明 ==================== */

/**
 * @brief 重力补偿计算 (7自由度求和法) - float32接口
 * @param q_rad 关节角度数组 [7] (单位: 弧度, float32)
 * @param tau_out 输出补偿力矩数组 [7] (单位: N·m, float32)
 * @note 力矩方向为抵消重力所需的力矩
 */
void Gravity_Compensation_Calcu(float q_rad[NUM_JOINTS], float tau_out[NUM_JOINTS]);

/**
 * @brief 重力补偿计算 (7自由度求和法) - float64接口
 * @param q_rad 关节角度数组 [7] (单位: 弧度, float64)
 * @param tau_out 输出补偿力矩数组 [7] (单位: N·m, float64)
 */
void Gravity_Compensation_Calcu_f64(float64_t q_rad[NUM_JOINTS], float64_t tau_out[NUM_JOINTS]);

/**
 * @brief 夹爪夹取 — 将被夹物体质量加到 Link7
 * @note 夹到物体时调用，Link7 质量 += 物体质量
 */
void GravComp_Gripper_Grab(void);

/**
 * @brief 夹爪松开 — 恢复 Link7 原始质量
 * @note 松开物体时调用，Link7 质量恢复为 LINK7_MASS
 */
void GravComp_Gripper_Release(void);

#endif /* __GRAVITY_COMPENSATION_H__ */
