/**
 * @file forward_kinematics.c
 * @brief 正运动学计算模块实现
 */

#include "forward_kinematics.h"
#include "arm_math.h"
#include "kinematics_types.h"
/*
	正解
*/
static Robot_t g_robot;

/* ==================== 单例访问函数 ==================== */

/**
 * @brief 获取全局机器人模型实例
 * @return Robot_t* 机器人模型指针
 */
Robot_t* Robot_GetInstance(void)
{
    return &g_robot;
}

/* ==================== 静态函数前置声明 ==================== */
static void Robotic_Arm_TFMatrix_Init(void);
static void Robotic_Arm_DH_Param_Init(void);
static void Robotic_Arm_TFMatrix_Update(const float64_t* thita);

void Robotic_Arm_FK_Init(void)
{
	Robotic_Arm_TFMatrix_Init();  // 先初始化矩阵结构
	Robotic_Arm_DH_Param_Init();
	Robotic_Arm_TFMatrix_Update(NULL);  // 用零角度初始化一次
}

/**
 * @brief 初始化机器人所有矩阵结构体（绑定数据存储）
 * @details 使用 TfMatrix_Init 将矩阵结构体与内部数据数组绑定
 *          同时初始化固定不变的变换矩阵（基座、工具）
 */
static void Robotic_Arm_TFMatrix_Init(void)
{
	// 初始化7个关节变换矩阵
	for (uint8_t i = 0; i < NUM_JOINTS; i++) {
		TfMatrix_Init(&g_robot.joint_tfmatrix[i], g_robot.data_joint_tfmatrix[i]);
	}
	
	// 初始化7个累积变换矩阵 T_0^{i}
	for (uint8_t i = 0; i < NUM_JOINTS; i++) {
		TfMatrix_Init(&g_robot.T_0_to_i[i], g_robot.data_T_0_to_i[i]);
	}
	
	// 初始化其他变换矩阵
	TfMatrix_Init(&g_robot.T_world_to_base, g_robot.data_T_world_to_base);
	TfMatrix_Init(&g_robot.T_end_to_tool, g_robot.data_T_end_to_tool);
	TfMatrix_Init(&g_robot.T_world, g_robot.data_T_world);
	
	// 基座变换矩阵初始化为单位阵（后续通过 Robot_SetWorldToBase() 设置实际值）
	TfMatrix_Eye(&g_robot.T_world_to_base);
	
	// 工具变换矩阵（固定不变）
	// T_end_to_tool = transl(0, 0, gripper_len) * troty(-90°)
	TfMatrix_Eye(&g_robot.T_end_to_tool);
	TfMatrix_For_Translation_Along_Z(&g_robot.T_end_to_tool, gripper_len);
	TfMatrix_For_Rotation_Around_Y(&g_robot.T_end_to_tool, DEG2RAD(-90.0));
}

/**
 * @brief 更新所有关节的 DH 变换矩阵
 * @param thita 关节角度数组 (7个元素, 单位: rad)，传 NULL 时使用零角度
 * @details 遍历7个关节，将输入角度加上偏移后更新到 DH 参数，
 *          然后重建每个关节的齐次变换矩阵
 */
static void Robotic_Arm_TFMatrix_Update(const float64_t* thita)
{
	// 更新 DH 参数中的 theta 并重建关节变换矩阵
	for(uint8_t i = 0; i < NUM_JOINTS; i++) {
		g_robot.DH_Param_joint[i].thita = (thita ? thita[i] : 0) + g_robot.DH_Param_joint[i].offset;
		DH_Modified_TfMatrix_Update(&g_robot.joint_tfmatrix[i], g_robot.DH_Param_joint[i]);
	}
}

/**
 * @brief 计算前四个关节的复合变换矩阵 T0_4，结果存入 g_robot.T_0_to_i[3]
 * @param thita 关节角度数组 (7个元素, 单位: rad)
 * @details 内部自动调用 TFMatrix_Update 更新关节矩阵，
 *          然后计算 T_0_to_i[0..3]，其中 T_0_to_i[3] 即 T0_4
 *          供 IK 通过 Robot_GetInstance()->T_0_to_i[3] 直接读取
 */
void Robotic_Arm_T04_TFMatrix_Calcu(const float64_t* thita)
{
	// 1. 更新所有关节变换矩阵
	Robotic_Arm_TFMatrix_Update(thita);
	
	// 2. 计算累积变换: T_0_to_i[0] ~ T_0_to_i[3]
	TfMatrix_Copy(&g_robot.joint_tfmatrix[0], &g_robot.T_0_to_i[0]);
	for (uint8_t i = 1; i <= 3; i++) {
		my_mat_mult_f64(&g_robot.T_0_to_i[i - 1], &g_robot.joint_tfmatrix[i], &g_robot.T_0_to_i[i]);
	}
}

/**
 * @brief 计算所有关节的累积变换矩阵 T_0^{i} (基座系到 Frame i)
 * @param thita 关节角度数组 (7个元素, 单位: rad)，传 NULL 时使用零角度
 * @details 内部自动调用 TFMatrix_Update 更新关节矩阵，然后计算：
 *          T_0_to_i[0] = T_{0}^{1}
 *          T_0_to_i[1] = T_{0}^{1} * T_{1}^{2}
 *          ...
 *          T_0_to_i[6] = T_{0}^{1} * T_{1}^{2} * ... * T_{6}^{7}
 *       结果缓存在 Robot_t.T_0_to_i[] 中，可通过 Robot_GetInstance() 访问
 */
void Robotic_Arm_T0i_TFMatrix_Calcu(const float64_t* thita)
{
	// 1. 更新所有关节变换矩阵
	Robotic_Arm_TFMatrix_Update(thita);
	
	// 2. 计算累积变换: T_0_to_i[0] ~ T_0_to_i[6]
	// T_0_to_i[0] = joint_tfmatrix[0]
	TfMatrix_Copy(&g_robot.joint_tfmatrix[0], &g_robot.T_0_to_i[0]);
	
	// T_0_to_i[i] = T_0_to_i[i-1] * joint_tfmatrix[i]
	for (uint8_t i = 1; i < NUM_JOINTS; i++) {
		my_mat_mult_f64(&g_robot.T_0_to_i[i - 1], &g_robot.joint_tfmatrix[i], &g_robot.T_0_to_i[i]);
	}
}

/**
 * @brief 一步完成正运动学计算 (FK)
 * @param tfmat 输出的末端位姿矩阵 (必须已初始化)
 * @param thita 关节角度数组 (7个元素, 单位: rad)
 * @details 封装了关节矩阵更新和正解链计算:
 *          1. 更新所有关节变换矩阵
 *          2. 计算 T_world_to_end = T_world_to_base * T0_7 * T_end_to_tool
 * @note 这是正解的主要入口函数，直接传入关节角度即可获得末端位姿
 */
void Robotic_Arm_T_World_to_Tool_Update(arm_matrix_instance_f64* tfmat, const float64_t* thita)
{
	// 更新所有关节的变换矩阵
	Robotic_Arm_TFMatrix_Update(thita);
	// 重新初始化末端变换矩阵为单位矩阵（注意: tfmat 必须已经初始化）
	TfMatrix_Eye(tfmat);
	
	// 计算完整正解链: T_world_to_base * T0_1 * ... * T6_7 * T_end_to_tool
	my_mat_mult_f64(&g_robot.T_end_to_tool, tfmat, tfmat);
	
	for(int8_t i = NUM_JOINTS - 1; i >= 0; i--)
		my_mat_mult_f64(&g_robot.joint_tfmatrix[i], tfmat, tfmat);
	
	my_mat_mult_f64(&g_robot.T_world_to_base, tfmat, tfmat);
}

static void Robotic_Arm_DH_Param_Init(void)
{
	/*
		7轴机械臂 DH参数
		Link([thita, d, a, alpha], 'modified')   <-- Modified DH (Craig Convention)
		
		注意: h (底座高度) 已移动到 robot.base 中，关节1的 d=0
		注意: Link5 采用U形管分解建模，d=l3(轴向分量), a=l2(干涉偏移分量)
	*/

	// L(1) = Link([0, 0, 0, 0], 'modified');             offset = 0
	DH_Param_Init(&g_robot.DH_Param_joint[0], 0, 0,  0,  0);
	
	// L(2) = Link([0, 0, 0, -pi/2], 'modified');         offset = 0
	DH_Param_Init(&g_robot.DH_Param_joint[1], 0, 0,  0,  -pi/2.0);
	
	// L(3) = Link([0, l1, 0, pi/2], 'modified');         offset = pi
	DH_Param_Init(&g_robot.DH_Param_joint[2], 0, l1, 0,  pi/2.0);
	g_robot.DH_Param_joint[2].offset = pi;
	
	// L(4) = Link([0, 0, 0, pi/2], 'modified');          offset = pi/2
	DH_Param_Init(&g_robot.DH_Param_joint[3], 0, 0,  0,  pi/2.0);
	g_robot.DH_Param_joint[3].offset = pi/2.0;

	// L(5) = Link([0, l3, l2, -pi/2], 'modified');       offset = 0
	// d=l3(轴向分量≈0.434m), a=l2(干涉偏移≈0.075m)，U形管分解建模
	DH_Param_Init(&g_robot.DH_Param_joint[4], 0, l3, l2,  -pi/2.0);
	
	// L(6) = Link([0, 0, 0, pi/2], 'modified');          offset = 0
	DH_Param_Init(&g_robot.DH_Param_joint[5], 0, 0,  0,  pi/2.0);
	
	// L(7) = Link([0, 0, 0, -pi/2], 'modified');         offset = pi
	DH_Param_Init(&g_robot.DH_Param_joint[6], 0, 0,  0,  -pi/2.0);
	g_robot.DH_Param_joint[6].offset = pi;
	
	// ===== 关节限位初始化
	g_robot.qlim[0].min = DEG2RAD(JOINT1_MIN_DEG);
	g_robot.qlim[0].max = DEG2RAD(JOINT1_MAX_DEG);
	
	g_robot.qlim[1].min = DEG2RAD(JOINT2_MIN_DEG);
	g_robot.qlim[1].max = DEG2RAD(JOINT2_MAX_DEG);

	g_robot.qlim[2].min = DEG2RAD(JOINT3_MIN_DEG);
	g_robot.qlim[2].max = DEG2RAD(JOINT3_MAX_DEG);

	g_robot.qlim[3].min = DEG2RAD(JOINT4_MIN_DEG);
	g_robot.qlim[3].max = DEG2RAD(JOINT4_MAX_DEG);
	
	g_robot.qlim[4].min = DEG2RAD(JOINT5_MIN_DEG);
	g_robot.qlim[4].max = DEG2RAD(JOINT5_MAX_DEG);

	g_robot.qlim[5].min = DEG2RAD(JOINT6_MIN_DEG);
	g_robot.qlim[5].max = DEG2RAD(JOINT6_MAX_DEG);
	
	g_robot.qlim[6].min = DEG2RAD(JOINT7_MIN_DEG);
	g_robot.qlim[6].max = DEG2RAD(JOINT7_MAX_DEG);
}

/* ==================== 基座变换设置函数 ==================== */

/**
 * @brief 设置世界系到基座系的变换矩阵（通过位置和ZYX欧拉角，角度单位为度）
 * @param x, y, z      基座原点在世界系中的位置 (单位: m)
 * @param yaw_deg      绕Z轴旋转角度 (单位: deg) - 第1步
 * @param pitch_deg    绕Y轴旋转角度 (单位: deg) - 第2步
 * @param roll_deg     绕X轴旋转角度 (单位: deg) - 第3步
 * @details 变换顺序: T = transl(x,y,z) * RotZ(yaw) * RotY(pitch) * RotX(roll)
 */
void Robot_SetWorldToBase_Deg(float64_t x, float64_t y, float64_t z, float64_t yaw_deg, float64_t pitch_deg, float64_t roll_deg)
{
    TfMatrix_Compose_ZYX_Deg(&g_robot.T_world_to_base, x, y, z, yaw_deg, pitch_deg, roll_deg);
}

