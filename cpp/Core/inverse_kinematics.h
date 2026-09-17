/**
 * @file inverse_kinematics.h
 * @brief 7DOF 机械臂逆运动学计算模块
 */

#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

/* 向后兼容：保留旧的保护宏 */
#ifndef __Inverse_Kinematics_H__
#define __Inverse_Kinematics_H__
#endif

#include "main.h"
#include "kinematics_types.h"
#include "arm_math.h"
#include "TFMtrix_mathlib.h"

/**
 * @brief 7DOF逆解工作结构体
 * @details 存储逆解计算过程中的中间变量和结果
 */
typedef struct {
	// 工具坐标系相关 
	arm_matrix_instance_f64 T_end_to_tool_inv;	  // 工具变换矩阵的逆
	arm_matrix_instance_f64 T_base_to_wrist;      // 去掉工具后的变换矩阵 (Base->Wrist)
	
	// 坐标系转换相关
	arm_matrix_instance_f64 T_world_to_base_inv;  // World->Base 变换矩阵的逆
	arm_matrix_instance_f64 T_base_to_target;     // Base->Target 变换矩阵
	
	// 姿态解算相关矩阵
	arm_matrix_instance_f64 R0_4;           // 前四轴旋转矩阵
	arm_matrix_instance_f64 R0_4_inv;       // 前四轴旋转矩阵的逆 (转置)
	arm_matrix_instance_f64 R4_7;           // 后三轴旋转矩阵 R4_7 = R0_4' * R_target
	
	// ===== 矩阵数据存储 =====
	float64_t data_T_end_to_tool_inv[16];
	float64_t data_T_base_to_wrist[16];
	float64_t data_T_world_to_base_inv[16];
	float64_t data_T_base_to_target[16];
	float64_t data_R0_4[16];
	float64_t data_R0_4_inv[16];
	float64_t data_R4_7[16];
	
	// 目标腕心位置
	float64_t gx, gy, gz;
	
	// 七轴关节角度 (DH 坐标系下)
	float64_t q_dh[NUM_JOINTS];
	
	// 七轴关节角度 (物理坐标系，减去 offset)
	float64_t q[NUM_JOINTS];
	
} Inverse_Kinematics_7DOF_t;

/**
 * @brief IK连续性优化上下文 (用于帧间解的平滑选择)
 * @details 7DOF: 关心 J5, J6, J7 的连续性
 */
typedef struct {
    float64_t last_q[NUM_JOINTS];   // 上一帧关节角度
    uint8_t has_last;               // 是否有上一帧解 (0=无, 1=有)
} IK_Continuity_7DOF_t;

/* ==================== 单例访问函数 ==================== */

/**
 * @brief 获取全局逆解实例
 * @return Inverse_Kinematics_7DOF_t* 逆解实例指针
 */
Inverse_Kinematics_7DOF_t* IK_GetInstance(void);

/**
 * @brief 获取全局连续性上下文实例
 */
IK_Continuity_7DOF_t* IK_GetContinuity(void);

/* ==================== 逆运动学主要函数 ==================== */

/**
 * @brief 初始化逆运动学模块
 */
void IK_Init(void);

/**
 * @brief 7DOF 机械臂逆解计算 (单解极简版)
 * @param T_world_to_target 目标末端在世界系中的变换矩阵
 * @param joint3_angle_deg  固定的 J3 角度 (度)
 * @param last_q_sol        上一帧解 (用于奇异处理，可为 NULL)
 * @param q_out             输出的关节角度 (7个元素, rad)
 * @return CHECK_PASS: 成功, CHECK_FAIL: 失败/无解
 * 
 * @note 特性：
 *   1. 固定 J3
 *   2. J1/J2 双分支代数法求解，选择限位内最优解
 *   3. J5-J7 双解提取 (sin(q6)>0 / sin(q6)<0)，选择限位内最优解
 */
CheckResult_e IK_Solve_7DOF(const arm_matrix_instance_f64* T_world_to_target,
                            float64_t joint3_angle_deg,
                            const float64_t* last_q_sol,
                            float64_t q_out[NUM_JOINTS]);

/**
 * @brief 执行逆解计算并检查有效性（含数值有效性 + 限位检查）
 * @param T_world_to_target 目标变换矩阵
 * @param joint3_angle_deg  固定的 J3 角度 (度)
 * @param q_out 输出关节角度 (7个元素, rad)
 * @return CHECK_PASS: 成功, CHECK_FAIL: 失败
 */
CheckResult_e IK_SolveAndCheck(const arm_matrix_instance_f64* T_world_to_target,
                               float64_t joint3_angle_deg,
                               float64_t q_out[NUM_JOINTS]);

/* ==================== 关节限位检查函数 ==================== */

/**
 * @brief 检查关节角度是否有效（数值有限 + 限位内）
 * @param q 7个关节角度 (rad)
 * @return CHECK_PASS: 有效, CHECK_FAIL: 无效
 */
CheckResult_e IK_CheckJointLimits(const float64_t q[NUM_JOINTS]);

/* ==================== 连续性优化函数 ==================== */

/**
 * @brief 更新连续性上下文 (保存当前解用于下一帧)
 * @param continuity  连续性上下文
 * @param q           当前帧关节角度 (7个元素)
 */
void IK_UpdateContinuity(IK_Continuity_7DOF_t* continuity, const float64_t q[NUM_JOINTS]);

/**
 * @brief 重置连续性上下文 (清除上一帧解)
 */
void IK_ResetContinuity(IK_Continuity_7DOF_t* continuity);

#endif /* __INVERSE_KINEMATICS_H__ */

