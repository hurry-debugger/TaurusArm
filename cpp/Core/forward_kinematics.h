/**
 * @file forward_kinematics.h
 * @brief 正运动学计算模块
 * @note 原文件名位于 Core/Forward_Kinematics/forward_kinematics.h
 */

#ifndef __FORWARD_KINEMATICS_H__
#define __FORWARD_KINEMATICS_H__

/* 向后兼容：保留旧的保护宏 */
#ifndef __Forward_Kinematics_H__
#define __Forward_Kinematics_H__
#endif

#include "main.h"
#include "kinematics_types.h"
#include "arm_math.h"
#include "TFMtrix_mathlib.h"

/**
 * @brief 关节限位结构体
 * @details 每个关节的最小和最大角度限制 (弧度)
 */
typedef struct {
	float64_t min;  // 关节最小角度 (rad)
	float64_t max;  // 关节最大角度 (rad)
} JointLimit_t;

/**
 * @brief 机器人模型结构体 
 * @details 包含 DH 参数、变换矩阵、关节限位等
 *          
 *          矩阵数据直接存储在结构体内部的 data_XXX 数组中，
 *          使用 TfMatrix_Init() 初始化，便于调试查看
 */
typedef struct {
	DH_Param_t DH_Param_joint[NUM_JOINTS];              // 7个关节DH参数
	arm_matrix_instance_f64 joint_tfmatrix[NUM_JOINTS]; // 7个关节变换矩阵 T_{i-1}^{i}
	arm_matrix_instance_f64 T_0_to_i[NUM_JOINTS];      // 7个累积变换矩阵 T_0^{i} (基座系到 Frame i)
	arm_matrix_instance_f64 T_world_to_base;
	arm_matrix_instance_f64 T_end_to_tool;
	arm_matrix_instance_f64 T_world;
	
	// ===== 矩阵数据存储 =====
	float64_t data_joint_tfmatrix[NUM_JOINTS][16];   // 7个关节变换矩阵数据
	float64_t data_T_0_to_i[NUM_JOINTS][16];         // 7个累积变换矩阵数据
	float64_t data_T_world_to_base[16];              // 世界到基座变换矩阵数据
	float64_t data_T_end_to_tool[16];                // 末端到工具变换矩阵数据
	float64_t data_T_world[16];                      // 临时世界变换矩阵数据
	
	// ===== 关节限位 =====
	JointLimit_t qlim[NUM_JOINTS];
} Robot_t;

/* ==================== 单例访问函数 ==================== */

/**
 * @brief 获取全局机器人模型实例
 * @return Robot_t* 机器人模型指针
 */
Robot_t* Robot_GetInstance(void);

typedef enum {
	JOINT1,
	JOINT2,
	JOINT3,
	JOINT4,
	JOINT5,
	JOINT6,
	JOINT7,
} TF_Matrix_Name_e;
	
/* ==================== 正运动学函数声明 ==================== */

/**
 * @brief 初始化正运动学模块
 * @details 初始化 DH 参数、关节变换矩阵、关节限位等
 */
void Robotic_Arm_FK_Init(void);

/**
 * @brief 计算前四个关节的复合变换矩阵 T0_4
 * @param thita 关节角度数组 (7个元素, 单位: rad)
 * @details 内部自动更新关节矩阵，结果存入 Robot_t.T_0_to_i[3]
 *          IK 可通过 Robot_GetInstance()->T_0_to_i[3] 直接读取
 */
void Robotic_Arm_T04_TFMatrix_Calcu(const float64_t* thita);

/**
 * @brief 计算所有关节的累积变换矩阵 T_0^{i}
 * @param thita 关节角度数组 (7个元素, 单位: rad)，传 NULL 时使用零角度
 * @details 内部自动更新关节矩阵，然后计算 T_0_to_i[0..6]
 *          结果缓存在 Robot_t.T_0_to_i[] 中，可直接读取
 */
void Robotic_Arm_T0i_TFMatrix_Calcu(const float64_t* thita);

/**
 * @brief 一步完成正运动学计算 (FK)
 * @param tfmat 输出的末端位姿矩阵 (必须已初始化)
 * @param thita 关节角度数组 (7个元素, 单位: rad)
 * @details 封装了关节矩阵更新和正解链计算，这是 FK 的主要入口函数
 */
void Robotic_Arm_T_World_to_Tool_Update(arm_matrix_instance_f64* tfmat, const float64_t* thita);

/* ==================== 基座变换设置函数 ==================== */

/**
 * @brief 设置世界系到基座系的变换矩阵（通过位置和ZYX欧拉角，角度单位为度）
 * @param x, y, z      基座原点在世界系中的位置 (单位: m)
 * @param yaw_deg      绕Z轴旋转角度 (单位: deg) - 第1步
 * @param pitch_deg    绕Y轴旋转角度 (单位: deg) - 第2步
 * @param roll_deg     绕X轴旋转角度 (单位: deg) - 第3步
 * @details 变换顺序: T = transl(x,y,z) * RotZ(yaw) * RotY(pitch) * RotX(roll)
 */
void Robot_SetWorldToBase_Deg(float64_t x, float64_t y, float64_t z, float64_t yaw_deg, float64_t pitch_deg, float64_t roll_deg);


#endif /* __FORWARD_KINEMATICS_H__ */
