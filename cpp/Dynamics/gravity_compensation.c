#include "gravity_compensation.h"

/*
 * =============================================================================
 * 7自由度机械臂重力补偿 - 求和法 (Summation Method)
 * =============================================================================
 * 
 * 原理：对每一级关节，把后面所有连杆产生的重力矩加起来
 *       τ_i = Σ_{k=i}^{7} ( r_k × F_k ) · Z_i
 * 
 * 复用 vector_mathlib 和 TFMtrix_mathlib 中的运算
 * =============================================================================
 */

/* ==================== 全局变量 ==================== */

// 重力向量 (假设基座Z轴垂直向上)
const Vector3D_t GRAVITY_VEC = {.x = 0.0, .y = 0.0, .z = -GRAVITY_ACC};
// 重力补偿计算结构体
DTCM_RAM GravComp_t g_gravcomp = 
{	//电机力方向	
	.tau_dirt = {1, -1, 1, -1, 1, -1, 1},
};
DTCM_RAM LinkParam_t g_link_params[NUM_JOINTS] = {};
/* ==================== API函数实现 ==================== */

/**
 * @brief 夹爪夹取 — 将被夹物体质量加到 Link7
 */
void GravComp_Gripper_Grab(void) {
	g_link_params[6].mass = LINK7_MASS + ENERGY_UNIT_MASS;
}

/**
 * @brief 夹爪松开 — 恢复 Link7 原始质量
 */
void GravComp_Gripper_Release(void) {
	g_link_params[6].mass = LINK7_MASS;
}

/**
 * @brief 重力补偿计算核心 (float64版本) [复用 forward_kinematics]
 * 
 * [MDH 关键说明]:
 *   在 Modified DH 中，关节 i 的旋转轴是 Frame i 的 Z 轴。
 *   累积变换矩阵 T_0_to_i 由 forward_kinematics 模块计算并缓存。
 */
void Gravity_Compensation_Calcu_f64(float64_t q_rad[NUM_JOINTS], float64_t tau_out[NUM_JOINTS]) {
	// 保存输入
	memcpy(g_gravcomp.q, q_rad, NUM_JOINTS * sizeof(float64_t));
	
	Robot_t* robot = Robot_GetInstance();
	
	// ==================== 1. 复用 FK 正运动学 ====================
	// 更新关节矩阵并计算所有累积变换矩阵 T_0_to_i[0..6]
	Robotic_Arm_T0i_TFMatrix_Calcu(q_rad);
	
	// ==================== 2. 提取 Z_axis / P_joint / P_com ====================
	for (int i = 0; i < NUM_JOINTS; i++) {
		float64_t* T = robot->T_0_to_i[i].pData;
		
		// Z轴 (第3列) - Frame i 的旋转轴
		g_gravcomp.Z_axis[i].x = T[0 * 4 + 2];
		g_gravcomp.Z_axis[i].y = T[1 * 4 + 2];
		g_gravcomp.Z_axis[i].z = T[2 * 4 + 2];
		
		// 关节位置 (第4列) - Frame i 的原点
		g_gravcomp.P_joint[i].x = T[0 * 4 + 3];
		g_gravcomp.P_joint[i].y = T[1 * 4 + 3];
		g_gravcomp.P_joint[i].z = T[2 * 4 + 3];
		
		// 质心全局坐标 = T_0_to_i * com_local
		TfMatrix_TransformPoint(&robot->T_0_to_i[i], &g_link_params[i].com, &g_gravcomp.P_com_global[i]);
	}
	
	// ==================== 3. 力矩求和 (Summation Loop) ====================
	for (int i = 0; i < NUM_JOINTS; i++) {
		float64_t tau_i = 0.0;
		
		// 关节i需要承担Link i, i+1, ..., 6的所有重力
		for (int k = i; k < NUM_JOINTS; k++) {
			// 1. 力臂 r = 质心位置 - 关节轴位置
			Vector3D_t r;
			Vector3D_Sub(&g_gravcomp.P_com_global[k], &g_gravcomp.P_joint[i], &r);
			
			// 2. 重力 F = m * g
			Vector3D_t F_gravity;
			Vector3D_Scale(&GRAVITY_VEC, g_link_params[k].mass, &F_gravity);
			
			// 3. 力矩 = r × F
			Vector3D_t torque;
			Vector3D_Cross(&r, &F_gravity, &torque);
			
			// 4. 投影到电机轴 (加负号产生"反"重力的力矩)
			tau_i += -1.0 * Vector3D_Dot(&torque, &g_gravcomp.Z_axis[i]);
		}
		
		g_gravcomp.tau[i] = tau_i;
		tau_out[i] = g_gravcomp.tau[i] * g_gravcomp.tau_dirt[i];
	}
}

/**
 * @brief 重力补偿计算 (float32接口，内部转换为float64计算)
 */
void Gravity_Compensation_Calcu(float q_rad[NUM_JOINTS], float tau_out[NUM_JOINTS]) {
	float64_t q_rad_f64[NUM_JOINTS] = {0};
	float64_t tau_f64[NUM_JOINTS] = {0};
	
	// 输入转换: float32 -> float64
	Float32ToFloat64(q_rad, q_rad_f64, NUM_JOINTS);
	
	// 核心计算
	Gravity_Compensation_Calcu_f64(q_rad_f64, tau_f64);
	
	// 输出转换: float64 -> float32
	Float64ToFloat32(tau_f64, tau_out, NUM_JOINTS);
}

