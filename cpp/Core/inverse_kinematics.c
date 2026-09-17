/**
 * @file inverse_kinematics.c
 * @brief 7DOF 机械臂逆运动学计算模块实现
 */

#include "inverse_kinematics.h"
#include "forward_kinematics.h"
#include "kinematics_types.h"
#include <string.h>

/* ==================== 私有内联辅助函数 ==================== */

static inline float64_t clip(float64_t x, float64_t max_val, float64_t min_val) {
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

static inline float64_t normalize_angle(float64_t angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

/* ==================== 全局变量 ==================== */

static DTCM_RAM Inverse_Kinematics_7DOF_t g_ik;
static DTCM_RAM IK_Continuity_7DOF_t g_ik_continuity = {0};

/* ==================== 单例访问函数 ==================== */

Inverse_Kinematics_7DOF_t* IK_GetInstance(void)
{
    return &g_ik;
}

IK_Continuity_7DOF_t* IK_GetContinuity(void)
{
    return &g_ik_continuity;
}

/* ==================== 初始化函数 ==================== */

void IK_Init(void)
{
    Robotic_Arm_FK_Init();  // 一定要先加载机器人模型
    
    Robot_t* robot = Robot_GetInstance();
    
    // ===== 初始化所有矩阵 =====
    TfMatrix_Init(&g_ik.T_end_to_tool_inv, g_ik.data_T_end_to_tool_inv);
    TfMatrix_Init(&g_ik.T_base_to_wrist, g_ik.data_T_base_to_wrist);
    TfMatrix_Init(&g_ik.T_world_to_base_inv, g_ik.data_T_world_to_base_inv);
    TfMatrix_Init(&g_ik.T_base_to_target, g_ik.data_T_base_to_target);
    TfMatrix_Init(&g_ik.R0_4, g_ik.data_R0_4);
    TfMatrix_Init(&g_ik.R0_4_inv, g_ik.data_R0_4_inv);
    TfMatrix_Init(&g_ik.R4_7, g_ik.data_R4_7);
    
    // 计算工具变换矩阵的逆
    TfMatrix_Inverse_SE3(&robot->T_end_to_tool, &g_ik.T_end_to_tool_inv);
}

/* ==================== 主逆解函数 ==================== */

/**
 * @brief 7DOF 机械臂逆解计算 (MDH 改良版专属, 双分支搜索)
 * 
 * 特性：
 * 1. 固定 J3
 * 2. J1/J2 双分支代数法求解，选择限位内最优解
 * 3. J5-J7 双解提取 (sin(q6)>0 / sin(q6)<0)，选择限位内最优解
 */
CheckResult_e IK_Solve_7DOF(const arm_matrix_instance_f64* T_world_to_target,
                            float64_t joint3_angle_deg,
                            const float64_t* last_q_sol,
                            float64_t q_out[NUM_JOINTS])
{
    if (T_world_to_target == NULL || q_out == NULL) return CHECK_FAIL;
    
    Robot_t* robot = Robot_GetInstance();
    
    // --- 0. 参数准备 ---
    // 获取 offset 数组
    float64_t offsets[NUM_JOINTS];
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        offsets[i] = robot->DH_Param_joint[i].offset;
    }
    
    // 固定 J3: q3_dh = deg2rad(joint3_angle_deg) + offset(3)
    float64_t q3_dh = DEG2RAD(joint3_angle_deg) + offsets[2];
    
    // --- 1. 计算目标腕心 ---
    // T_base_to_target = inv(T_world_to_base) * T_world_to_target
    TfMatrix_Inverse_SE3(&robot->T_world_to_base, &g_ik.T_world_to_base_inv);
    my_mat_mult_f64(&g_ik.T_world_to_base_inv, (arm_matrix_instance_f64*)T_world_to_target, &g_ik.T_base_to_target);
    
    // T_base_to_wrist = T_base_to_target / T_tool
    my_mat_mult_f64(&g_ik.T_base_to_target, &g_ik.T_end_to_tool_inv, &g_ik.T_base_to_wrist);
    
    // 提取腕心位置
    float64_t gx = g_ik.T_base_to_wrist.pData[3];
    float64_t gy = g_ik.T_base_to_wrist.pData[7];
    float64_t gz = g_ik.T_base_to_wrist.pData[11];
    float64_t Dist_sq = gx*gx + gy*gy + gz*gz;
    
    g_ik.gx = gx;
    g_ik.gy = gy;
    g_ik.gz = gz;
    
    // --- 2. 求解 J4---
    // Link5 在 MDH 下的几何参数: a=l2_phys, d=l3_phys
    // 虚拟小臂长度 L_v = sqrt(a^2 + d^2), 夹角 phi = atan2(a, d)
    float64_t l2_phys = robot->DH_Param_joint[4].a;
    float64_t l3_phys = robot->DH_Param_joint[4].d;
    float64_t L_v = sqrt(l2_phys * l2_phys + l3_phys * l3_phys);
    float64_t phi = atan2(l2_phys, l3_phys);

    // q4_virtual 由虚拟直杆余弦定理得到，q4_dh = q4_virtual + phi
    float64_t cos_q4v = (Dist_sq - l1*l1 - L_v*L_v) / (2.0 * l1 * L_v);
    cos_q4v = clip(cos_q4v, 1.0, -1.0);
    float64_t q4_virtual = acos(cos_q4v);
    float64_t q4_dh = q4_virtual + phi;
    
    // --- 3. 求解 J1, J2---
    // 以 Joint2 轴心为原点，前x左y上z建立坐标系
    float64_t c3 = cos(DEG2RAD(joint3_angle_deg));
    float64_t s3 = sin(DEG2RAD(joint3_angle_deg));
    float64_t c4v = cos(q4_virtual);
    float64_t s4v = sin(q4_virtual);
    
    float64_t X2 = L_v * s4v * c3;
    float64_t Y2 = L_v * s4v * s3;
    float64_t Z2 = l1 + L_v * c4v;
    
    // R 是 gz 在这一构型下的最大可达值
    float64_t R_arm = sqrt(X2 * X2 + Z2 * Z2);
    
    // 增加 1e-4 的容差，防止浮点误差导致误判
    if (fabs(gz) > R_arm + 1e-4) {
        return CHECK_FAIL;
    }
    
    // gamma: Joint2=0 时 Joint3+Joint4 带来的固有偏角
    float64_t gamma = atan2(X2, Z2);
    
    // 必须对比例进行强制裁切（clip），防止超范围引发 NaN 崩溃
    float64_t ratio = clip(gz / R_arm, 1.0, -1.0);
    float64_t acos_val = acos(ratio);
    
    // J2 的两个可能的严格数学解 (对应手臂前屈/后翻)
    float64_t q2_dh_choices[2] = {acos_val - gamma, -acos_val - gamma};
    
    uint8_t valid_q1q2 = 0;
    float64_t best_diff = 1e9;
    float64_t q1_dh = 0, q2_dh = 0;
    
    // 遍历两个解，找到在限位内且最优的分支
    for (uint8_t j = 0; j < 2; j++) {
        float64_t q2_test = q2_dh_choices[j];
        
        // 基于 q2 计算偏置项 U，进而求出确定的 q1
        float64_t U = cos(q2_test) * X2 + sin(q2_test) * Z2;
        float64_t q1_test = atan2(gy, gx) - atan2(Y2, U);
        
        float64_t q1_phy_test = normalize_angle(q1_test - offsets[0]);
        float64_t q2_phy_test = normalize_angle(q2_test - offsets[1]);
        float64_t q4_phy_test = normalize_angle(q4_dh - offsets[3]);
        
        // 限位检查与距离优选
        if (q1_phy_test >= robot->qlim[0].min - 1e-3 && q1_phy_test <= robot->qlim[0].max + 1e-3 &&
            q2_phy_test >= robot->qlim[1].min - 1e-3 && q2_phy_test <= robot->qlim[1].max + 1e-3 &&
            q4_phy_test >= robot->qlim[3].min - 1e-3 && q4_phy_test <= robot->qlim[3].max + 1e-3) {
            
            float64_t diff;
            if (last_q_sol != NULL) {
                diff = fabs(q1_phy_test - last_q_sol[0]) + fabs(q2_phy_test - last_q_sol[1]);
            } else {
                diff = 0;
            }
            
            if (diff < best_diff) {
                best_diff = diff;
                q1_dh = q1_test;
                q2_dh = q2_test;
                valid_q1q2 = 1;
            }
        }
    }
    
    if (!valid_q1q2) {
        return CHECK_FAIL;
    }
    
    // --- 4. 求解 J5, J6, J7 (姿态) ---
    float64_t q_final_dh[NUM_JOINTS] = {0}; // 养成好习惯，全零初始化
    q_final_dh[0] = q1_dh;
    q_final_dh[1] = q2_dh;
    q_final_dh[2] = q3_dh;
    q_final_dh[3] = q4_dh;
    
    // 【修正3】C 底层的 Update 函数会自动加上 offset！
    // 因此传给正解前向递推的，必须是【减去 offset 后的纯物理角度】
    // 并且必须将 7 个元素全部初始化，防止后三个关节的垃圾内存导致 sin/cos 异常
    float64_t q_final_phy[NUM_JOINTS] = {0};
    q_final_phy[0] = q1_dh - offsets[0];
    q_final_phy[1] = q2_dh - offsets[1];
    q_final_phy[2] = q3_dh - offsets[2];
    q_final_phy[3] = q4_dh - offsets[3];
    
    // 计算 T0_4: 内部更新关节矩阵，结果存入 robot->T_0_to_i[3]
    Robotic_Arm_T04_TFMatrix_Calcu(q_final_phy);
    arm_matrix_instance_f64* T0_4 = &robot->T_0_to_i[3];
    
    // R_link4_to_wrist = T0_4(1:3, 1:3)' * T_base_to_wrist(1:3, 1:3)
    // 先提取旋转部分
    float64_t R04[9], R_wrist[9], R4_7[9];
    
    // T0_4 旋转部分 (3x3)
    R04[0] = T0_4->pData[0];  R04[1] = T0_4->pData[1];  R04[2] = T0_4->pData[2];
    R04[3] = T0_4->pData[4];  R04[4] = T0_4->pData[5];  R04[5] = T0_4->pData[6];
    R04[6] = T0_4->pData[8];  R04[7] = T0_4->pData[9];  R04[8] = T0_4->pData[10];
    
    // T_base_to_wrist 旋转部分 (3x3)
    R_wrist[0] = g_ik.T_base_to_wrist.pData[0];  R_wrist[1] = g_ik.T_base_to_wrist.pData[1];  R_wrist[2] = g_ik.T_base_to_wrist.pData[2];
    R_wrist[3] = g_ik.T_base_to_wrist.pData[4];  R_wrist[4] = g_ik.T_base_to_wrist.pData[5];  R_wrist[5] = g_ik.T_base_to_wrist.pData[6];
    R_wrist[6] = g_ik.T_base_to_wrist.pData[8];  R_wrist[7] = g_ik.T_base_to_wrist.pData[9];  R_wrist[8] = g_ik.T_base_to_wrist.pData[10];
    
    // R4_7 = R04' * R_wrist (3x3 矩阵乘法，R04 转置)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R4_7[i*3+j] = 0;
            for (int k = 0; k < 3; k++) {
                R4_7[i*3+j] += R04[k*3+i] * R_wrist[k*3+j];  // 转置: R04[k][i]
            }
        }
    }
    
    // 提取 R4_7 矩阵元素
    float64_t r11 = R4_7[0], r12 = R4_7[1], r13 = R4_7[2];
    float64_t r21 = R4_7[3], r22 = R4_7[4], r23 = R4_7[5];
    float64_t r31 = R4_7[6], r32 = R4_7[7], r33 = R4_7[8];
    
    // 腕部姿态双解提取
    // c6 = R4_7(2,3) = r23
    float64_t c6 = clip(r23, 1.0, -1.0);
    float64_t s6_abs = sqrt(1.0 - c6 * c6);
    
    // 腕部两个解 (手腕上弯/下弯)
    float64_t s6_choices[2] = {s6_abs, -s6_abs};
    uint8_t valid_wrist = 0;
    float64_t best_diff_wrist = 1e9;
    
    for (uint8_t k = 0; k < 2; k++) {
        float64_t s6_test = s6_choices[k];
        float64_t q6_test = atan2(s6_test, c6);
        float64_t q5_test, q7_test;
        
        if (fabs(s6_test) > 1e-4) {
            // 正常提取
            // q5 = atan2(R4_7(3,3)/s6, -R4_7(1,3)/s6)
            q5_test = atan2(r33 / s6_test, -r13 / s6_test);
            // q7 = atan2(-R4_7(2,2)/s6, R4_7(2,1)/s6)
            q7_test = atan2(-r22 / s6_test, r21 / s6_test);
        } else {
            // [奇异点处理]
            if (c6 > 0) {
                float64_t sum_57 = atan2(-r12, r11);
                if (last_q_sol == NULL) {
                    q5_test = sum_57;
                    q7_test = 0;
                } else {
                    q5_test = last_q_sol[4] + offsets[4];
                    q7_test = sum_57 - q5_test;
                }
            } else {
                float64_t diff_57 = atan2(r12, -r11);
                if (last_q_sol == NULL) {
                    q5_test = diff_57;
                    q7_test = 0;
                } else {
                    q5_test = last_q_sol[4] + offsets[4];
                    q7_test = q5_test - diff_57;
                }
            }
        }
        
        float64_t q5_phy = normalize_angle(q5_test - offsets[4]);
        float64_t q6_phy = normalize_angle(q6_test - offsets[5]);
        float64_t q7_phy = normalize_angle(q7_test - offsets[6]);
        
        if (q5_phy >= robot->qlim[4].min - 1e-3 && q5_phy <= robot->qlim[4].max + 1e-3 &&
            q6_phy >= robot->qlim[5].min - 1e-3 && q6_phy <= robot->qlim[5].max + 1e-3 &&
            q7_phy >= robot->qlim[6].min - 1e-3 && q7_phy <= robot->qlim[6].max + 1e-3) {
            
            float64_t diff;
            if (last_q_sol != NULL) {
                diff = fabs(q5_phy - last_q_sol[4]) + fabs(q6_phy - last_q_sol[5]) + fabs(q7_phy - last_q_sol[6]);
            } else {
                diff = 0;
            }
            
            if (diff < best_diff_wrist) {
                best_diff_wrist = diff;
                q_final_dh[4] = q5_test;
                q_final_dh[5] = q6_test;
                q_final_dh[6] = q7_test;
                valid_wrist = 1;
            }
        }
    }
    
    if (!valid_wrist) {
        return CHECK_FAIL;
    }
    
    // --- 5. 输出 ---
    // 减去 offset 并归一化
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        q_out[i] = normalize_angle(q_final_dh[i] - offsets[i]);
    }
    
    // 保存到 g_ik 供调试
    memcpy(g_ik.q_dh, q_final_dh, sizeof(q_final_dh));
    memcpy(g_ik.q, q_out, sizeof(g_ik.q));
    
    return CHECK_PASS;
}

/* ==================== 限位检查函数 ==================== */

#define IK_JOINT_LIMIT_TOLERANCE   (1e-3)

CheckResult_e IK_CheckJointLimits(const float64_t q[NUM_JOINTS])
{
    if (q == NULL) return CHECK_FAIL;
    
    Robot_t* robot = Robot_GetInstance();
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        // 数值有效性检查
        if (!isfinite(q[i])) {
            return CHECK_FAIL;
        }
        // 限位检查
        if (q[i] < robot->qlim[i].min - IK_JOINT_LIMIT_TOLERANCE ||
            q[i] > robot->qlim[i].max + IK_JOINT_LIMIT_TOLERANCE) {
            return CHECK_FAIL;
        }
    }
    return CHECK_PASS;
}

CheckResult_e IK_SolveAndCheck(const arm_matrix_instance_f64* T_world_to_target,
                               float64_t joint3_angle_deg,
                               float64_t q_out[NUM_JOINTS])
{
    if (T_world_to_target == NULL || q_out == NULL) return CHECK_FAIL;
    
    // 执行逆解
    CheckResult_e result = IK_Solve_7DOF(T_world_to_target, joint3_angle_deg, NULL, q_out);
    if (result != CHECK_PASS) return CHECK_FAIL;
    
    // 限位检查
    return IK_CheckJointLimits(q_out);
}

/* ==================== 连续性优化函数 ==================== */

void IK_UpdateContinuity(IK_Continuity_7DOF_t* continuity, const float64_t q[NUM_JOINTS])
{
    if (continuity == NULL || q == NULL) return;
    memcpy(continuity->last_q, q, NUM_JOINTS * sizeof(float64_t));
    continuity->has_last = 1;
}

void IK_ResetContinuity(IK_Continuity_7DOF_t* continuity)
{
    if (continuity == NULL) return;
    continuity->has_last = 0;
    memset(continuity->last_q, 0, sizeof(continuity->last_q));
}
