#include "TFMtrix_mathlib.h"
#include "string.h"

/* ==================================================================================
 * 配置与全局变量
 * ================================================================================== */

const DH_Param_t DH_Param_I = {0};

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.017453292519943295)  // PI / 180
#endif

/* ==================================================================================
 * 核心矩阵运算
 * ================================================================================== */

/** * @brief 优化的矩阵乘法 (修复版：防止内存越界)
 * @note  根据实际矩阵尺寸分配缓存，支持任意维度的矩阵相乘
 */
void my_mat_mult_f64(
  const arm_matrix_instance_f64 * pSrcA,
  const arm_matrix_instance_f64 * pSrcB,
        arm_matrix_instance_f64 * pDst)
{
    // 1. 尺寸检查
    if ((pSrcA->numCols != pSrcB->numRows) ||
        (pSrcA->numRows != pDst->numRows)  ||
        (pSrcB->numCols != pDst->numCols))
    {
        return; // 错误：维度不匹配
    }

    // 2. 栈上分配缓存 (最大支持 4x4)
    float64_t bufA[16];
    float64_t bufB[16];
    
    arm_matrix_instance_f64 Srca;
    arm_matrix_instance_f64 Srcb;
    
    // 【关键修改】使用源矩阵的实际维度进行初始化
    arm_mat_init_f64(&Srca, pSrcA->numRows, pSrcA->numCols, bufA);
    arm_mat_init_f64(&Srcb, pSrcB->numRows, pSrcB->numCols, bufB);
    
    // 3. 复制数据 (只复制实际大小)
    memcpy(bufA, pSrcA->pData, pSrcA->numRows * pSrcA->numCols * sizeof(float64_t));
    memcpy(bufB, pSrcB->pData, pSrcB->numRows * pSrcB->numCols * sizeof(float64_t));
    
    // 4. 计算
    arm_mat_mult_f64(&Srca, &Srcb, pDst);
}

/**
 * @brief SE(3) 齐次变换矩阵求逆
 * @details 利用 T = [R t; 0 1] 的性质，避免通用高斯消元
 */
void TfMatrix_Inverse_SE3(const arm_matrix_instance_f64* T, arm_matrix_instance_f64* T_inv)
{
    if (T == NULL || T_inv == NULL || T->pData == NULL || T_inv->pData == NULL) return;
    
    float64_t* M = T->pData;
    float64_t* I = T_inv->pData;
    
    // R^T (转置)
    I[0] = M[0];  I[1] = M[4];  I[2] = M[8];
    I[4] = M[1];  I[5] = M[5];  I[6] = M[9];
    I[8] = M[2];  I[9] = M[6];  I[10] = M[10];
    
    // -R^T * t
    float64_t tx = M[3], ty = M[7], tz = M[11];
    I[3]  = -(I[0] * tx + I[1] * ty + I[2]  * tz);
    I[7]  = -(I[4] * tx + I[5] * ty + I[6]  * tz);
    I[11] = -(I[8] * tx + I[9] * ty + I[10] * tz);
    
    // 最后一行 [0, 0, 0, 1]
    I[12] = 0.0; I[13] = 0.0; I[14] = 0.0; I[15] = 1.0;
}

/**
 * @brief 矩阵深度复制
 */
void TfMatrix_Copy(const arm_matrix_instance_f64* src, arm_matrix_instance_f64* dst)
{
    if (dst == NULL || src == NULL || dst->pData == NULL || src->pData == NULL) return;
    memcpy(dst->pData, src->pData, 16 * sizeof(float64_t));
}

/* ==================================================================================
 * 初始化操作
 * ================================================================================== */

/**
 * @brief 初始化为单位矩阵 (eye(4))
 * @note 仅操作数据，不初始化结构体
 */
void TfMatrix_Eye(arm_matrix_instance_f64* tfmat)
{
    if (tfmat == NULL || tfmat->pData == NULL) return;
    
    float64_t* T = tfmat->pData;
    memset(T, 0, 16 * sizeof(float64_t));
    T[0] = 1.0;
    T[5] = 1.0;
    T[10] = 1.0;
    T[15] = 1.0;
}

/**
 * @brief 完整初始化 (init_f64 + eye)
 */
void TfMatrix_Init(arm_matrix_instance_f64* tfmat, float64_t* data)
{
    if (tfmat == NULL || data == NULL) return;
    arm_mat_init_f64(tfmat, 4, 4, data);
    TfMatrix_Eye(tfmat);
}

/* ==================================================================================
 * 基本几何变换 (Atomic)
 * ================================================================================== */

void TfMatrix_For_Rotation_Around_X(arm_matrix_instance_f64* tfmatB, float64_t angle)
{
    float64_t tfMatrixA_array[4][4] = {
        {1, 0,         0,          0},
        {0, cos(angle), -sin(angle), 0},
        {0, sin(angle),  cos(angle), 0},
        {0, 0,         0,          1},
    };
    arm_matrix_instance_f64 tfmatA;
    arm_mat_init_f64(&tfmatA, 4, 4, &tfMatrixA_array[0][0]);
    my_mat_mult_f64(tfmatB, &tfmatA, tfmatB);                           
}

void TfMatrix_For_Rotation_Around_Y(arm_matrix_instance_f64* tfmatB, float64_t angle)
{
    float64_t tfMatrixA_array[4][4] = {  
        { cos(angle),   0, sin(angle),  0},
        { 0,            1, 0,           0},
        {-sin(angle),   0, cos(angle),  0},
        { 0,            0, 0,           1},
    };
    arm_matrix_instance_f64 tfmatA;
    arm_mat_init_f64(&tfmatA, 4, 4, &tfMatrixA_array[0][0]);
    my_mat_mult_f64(tfmatB, &tfmatA, tfmatB);                           
}

void TfMatrix_For_Rotation_Around_Z(arm_matrix_instance_f64* tfmatB, float64_t angle)
{
    float64_t tfMatrixA_array[4][4] = {  
        {cos(angle), -sin(angle),   0, 0},
        {sin(angle),  cos(angle),   0, 0},
        {0,           0,            1, 0},
        {0,           0,            0, 1},
    };
    arm_matrix_instance_f64 tfmatA;
    arm_mat_init_f64(&tfmatA, 4, 4, &tfMatrixA_array[0][0]);
    my_mat_mult_f64(tfmatB, &tfmatA, tfmatB);                           
}

void TfMatrix_For_Translation_Along_X(arm_matrix_instance_f64* tfmatB, float64_t dis)
{
    float64_t tfMatrixA_array[4][4] ={  
        {1, 0, 0, dis},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };
    arm_matrix_instance_f64 tfmatA;
    arm_mat_init_f64(&tfmatA, 4, 4, &tfMatrixA_array[0][0]);
    my_mat_mult_f64(tfmatB, &tfmatA, tfmatB);                           
}

void TfMatrix_For_Translation_Along_Y(arm_matrix_instance_f64* tfmatB, float64_t dis)
{
    float64_t tfMatrixA_array[4][4] ={  
        {1, 0, 0, 0},
        {0, 1, 0, dis},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };
    arm_matrix_instance_f64 tfmatA;
    arm_mat_init_f64(&tfmatA, 4, 4, &tfMatrixA_array[0][0]);
    my_mat_mult_f64(tfmatB, &tfmatA, tfmatB);                           
}

void TfMatrix_For_Translation_Along_Z(arm_matrix_instance_f64* tfmatB, float64_t dis)
{
    float64_t tfMatrixA_array[4][4] ={  
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, dis},
        {0, 0, 0, 1},
    };
    arm_matrix_instance_f64 tfmatA;
    arm_mat_init_f64(&tfmatA, 4, 4, &tfMatrixA_array[0][0]);
    my_mat_mult_f64(tfmatB, &tfmatA, tfmatB);
}

/* ==================================================================================
 * 组合变换工具
 * ================================================================================== */

void TfMatrix_Transl(arm_matrix_instance_f64* start_tfmat, arm_matrix_instance_f64* end_tfmat, float64_t x, float64_t y, float64_t z)
{
    if (start_tfmat == NULL || start_tfmat->pData == NULL || end_tfmat == NULL || end_tfmat->pData == NULL) return;

    memcpy(end_tfmat->pData, start_tfmat->pData, 16 * sizeof(float64_t));

    TfMatrix_For_Translation_Along_X(end_tfmat, x);
    TfMatrix_For_Translation_Along_Y(end_tfmat, y);
    TfMatrix_For_Translation_Along_Z(end_tfmat, z);
}

void TfMatrix_Rotate_zyx(arm_matrix_instance_f64* start_tfmat, arm_matrix_instance_f64* end_tfmat, float64_t roll, float64_t pitch, float64_t yaw)
{
    if (start_tfmat == NULL || start_tfmat->pData == NULL || end_tfmat == NULL || end_tfmat->pData == NULL) return;

    memcpy(end_tfmat->pData, start_tfmat->pData, 16 * sizeof(float64_t));

    // Z -> Y -> X
    TfMatrix_For_Rotation_Around_Z(end_tfmat, yaw);
    TfMatrix_For_Rotation_Around_Y(end_tfmat, pitch);
    TfMatrix_For_Rotation_Around_X(end_tfmat, roll);
}

void TfMatrix_Rotate_zyz(arm_matrix_instance_f64* start_tfmat, arm_matrix_instance_f64* end_tfmat, float64_t yaw, float64_t pitch, float64_t roll)
{
    if (start_tfmat == NULL || start_tfmat->pData == NULL || end_tfmat == NULL || end_tfmat->pData == NULL) return;

    memcpy(end_tfmat->pData, start_tfmat->pData, 16 * sizeof(float64_t));

    // Z -> Y -> Z
    TfMatrix_For_Rotation_Around_Z(end_tfmat, yaw);
    TfMatrix_For_Rotation_Around_Y(end_tfmat, pitch);
    TfMatrix_For_Rotation_Around_Z(end_tfmat, roll);
}

/* ==================================================================================
 * 机器人学 DH 建模
 * ================================================================================== */

void DH_Param_Init(DH_Param_t*DH, float64_t thita, float64_t d, float64_t a, float64_t alpha)
{
    DH->thita = thita;
    DH->d = d;
    DH->a = a;
    DH->alpha = alpha;
}

/**
 * @brief 创建标准 DH 矩阵 (优化版)
 * @note 预计算三角函数，避免重复调用sin/cos
 */
void DH_Standard_TfMatrix_Create(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH)
{
    // 1. 预计算三角函数 (性能优化关键！)
    float64_t ct = cos(DH.thita);   // cos(theta)
    float64_t st = sin(DH.thita);   // sin(theta)
    float64_t ca = cos(DH.alpha);   // cos(alpha)
    float64_t sa = sin(DH.alpha);   // sin(alpha)
    float64_t a  = DH.a;
    float64_t d  = DH.d;

    // 2. 直接填充一维数组 (避免二维数组转一维的隐式开销)
    // 标准DH矩阵：
    // [ ct    -st*ca    st*sa    a*ct ]
    // [ st     ct*ca   -ct*sa    a*st ]
    // [ 0      sa       ca       d    ]
    // [ 0      0        0        1    ]
    float64_t tfMatrix_array[16] = {
        ct,    -st * ca,   st * sa,   a * ct,
        st,     ct * ca,  -ct * sa,   a * st,
        0.0,    sa,        ca,        d,
        0.0,    0.0,       0.0,       1.0
    };
    
    memcpy(joint_tfmat->pData, tfMatrix_array, 16 * sizeof(float64_t));
}

/**
 * @brief 更新标准 DH 矩阵 (优化版，不分配内存)
 * @note 预计算三角函数，避免重复调用sin/cos
 */
void DH_Standard_TfMatrix_Update(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH)
{
    if (joint_tfmat == NULL || joint_tfmat->pData == NULL) return;
    
    // 1. 预计算三角函数
    float64_t ct = cos(DH.thita);
    float64_t st = sin(DH.thita);
    float64_t ca = cos(DH.alpha);
    float64_t sa = sin(DH.alpha);
    float64_t a  = DH.a;
    float64_t d  = DH.d;

    // 2. 直接填充一维数组
    float64_t tfMatrix_array[16] = {
        ct,    -st * ca,   st * sa,   a * ct,
        st,     ct * ca,  -ct * sa,   a * st,
        0.0,    sa,        ca,        d,
        0.0,    0.0,       0.0,       1.0
    };
    
    memcpy(joint_tfmat->pData, tfMatrix_array, 16 * sizeof(float64_t));
}

/**
 * @brief 创建单位阵 (Standard DH 特例)
 */
void DH_Standard_Eye_TfMatrix_Create(arm_matrix_instance_f64* joint_tfmat)
{
    TfMatrix_Eye(joint_tfmat); // 优化：直接使用单位阵填充
}

/**
 * @brief 创建改进型 (Modified/Craig) DH 矩阵 (优化版)
 * @note 预计算三角函数，避免重复调用sin/cos
 */
void DH_Modified_TfMatrix_Create(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH)
{
    // 1. 预计算三角函数 (性能优化关键！)
    float64_t ct = cos(DH.thita);   // cos(theta)
    float64_t st = sin(DH.thita);   // sin(theta)
    float64_t ca = cos(DH.alpha);   // cos(alpha)
    float64_t sa = sin(DH.alpha);   // sin(alpha)
    float64_t a  = DH.a;
    float64_t d  = DH.d;

    // 2. 直接填充一维数组
    // Modified DH 矩阵：
    // [ ct       -st      0     a     ]
    // [ st*ca    ct*ca   -sa   -sa*d  ]
    // [ st*sa    ct*sa    ca    ca*d  ]
    // [ 0        0        0     1     ]
    float64_t tfMatrix_array[16] = {
        ct,        -st,       0.0,    a,
        st * ca,    ct * ca, -sa,    -sa * d,
        st * sa,    ct * sa,  ca,     ca * d,
        0.0,        0.0,      0.0,    1.0
    };
    
    memcpy(joint_tfmat->pData, tfMatrix_array, 16 * sizeof(float64_t));
}

/**
 * @brief 更新改进型 (Modified/Craig) DH 矩阵 (优化版，不分配内存)
 * @note 预计算三角函数，避免重复调用sin/cos
 */
void DH_Modified_TfMatrix_Update(arm_matrix_instance_f64* joint_tfmat, DH_Param_t DH)
{
    if (joint_tfmat == NULL || joint_tfmat->pData == NULL) return;
    
    // 1. 预计算三角函数
    float64_t ct = cos(DH.thita);
    float64_t st = sin(DH.thita);
    float64_t ca = cos(DH.alpha);
    float64_t sa = sin(DH.alpha);
    float64_t a  = DH.a;
    float64_t d  = DH.d;

    // 2. 直接填充一维数组
    // Modified DH 矩阵：
    // [ ct       -st      0     a     ]
    // [ st*ca    ct*ca   -sa   -sa*d  ]
    // [ st*sa    ct*sa    ca    ca*d  ]
    // [ 0        0        0     1     ]
    float64_t tfMatrix_array[16] = {
        ct,        -st,       0.0,    a,
        st * ca,    ct * ca, -sa,    -sa * d,
        st * sa,    ct * sa,  ca,     ca * d,
        0.0,        0.0,      0.0,    1.0
    };
    
    memcpy(joint_tfmat->pData, tfMatrix_array, 16 * sizeof(float64_t));
}

/* ==================================================================================
 * 空间点与向量操作
 * ================================================================================== */

void TfMatrix_TransformPoint(const arm_matrix_instance_f64* T,
                             const Coordinates_t* p_local,
                             Coordinates_t* p_world)
{
    if (T == NULL || p_local == NULL || p_world == NULL) return;
    
    float64_t* M = T->pData;
    p_world->x = M[0] * p_local->x + M[1] * p_local->y + M[2]  * p_local->z + M[3];
    p_world->y = M[4] * p_local->x + M[5] * p_local->y + M[6]  * p_local->z + M[7];
    p_world->z = M[8] * p_local->x + M[9] * p_local->y + M[10] * p_local->z + M[11];
}

void TfMatrix_TransformVector(const arm_matrix_instance_f64* T,
                              const Vector3D_t* v_local,
                              Vector3D_t* v_world)
{
    if (T == NULL || v_local == NULL || v_world == NULL) return;
    
    float64_t* M = T->pData;
    v_world->x = M[0] * v_local->x + M[1] * v_local->y + M[2]  * v_local->z;
    v_world->y = M[4] * v_local->x + M[5] * v_local->y + M[6]  * v_local->z;
    v_world->z = M[8] * v_local->x + M[9] * v_local->y + M[10] * v_local->z;
}

/**
 * @brief 将向量从世界系变换到局部系 ( v_local = R^T * v_world )
 * @details 利用旋转矩阵 R^T = R^-1 的性质，不需计算逆矩阵
 */
void TfMatrix_TransformVector_Inverse(const arm_matrix_instance_f64* T_local_to_world,
                                      const Vector3D_t* v_world, 
                                      Vector3D_t* v_local)
{
    if (T_local_to_world == NULL || v_world == NULL || v_local == NULL) return;
    
    float64_t* M = T_local_to_world->pData;
    
    // 原函数是按行乘 (Row * vec)，这里改为按列乘 (Col * vec)
    // 相当于乘以转置矩阵 R^T
    // M[0], M[4], M[8] 是矩阵的第一列 (对应局部系的 X 轴在世界系的投影)
    v_local->x = M[0] * v_world->x + M[4] * v_world->y + M[8]  * v_world->z;
    
    // M[1], M[5], M[9] 是矩阵的第二列
    v_local->y = M[1] * v_world->x + M[5] * v_world->y + M[9]  * v_world->z;
    
    // M[2], M[6], M[10] 是矩阵的第三列
    v_local->z = M[2] * v_world->x + M[6] * v_world->y + M[10] * v_world->z;
}

/**
 * @brief 罗德里格斯变换：绕任意轴旋转
 */
void TfMatrix_RotateAroundAxis(const arm_matrix_instance_f64* T_current,
                               const Vector3D_t* axis_vec,
                               const Coordinates_t* pivot_point,
                               float64_t angle,
                               arm_matrix_instance_f64* T_result)
{
    if (T_current == NULL || axis_vec == NULL || pivot_point == NULL || T_result == NULL) return;
    
    // 归一化旋转轴
    float64_t len_sq = axis_vec->x * axis_vec->x + axis_vec->y * axis_vec->y + axis_vec->z * axis_vec->z;
    float64_t len = sqrt(len_sq);
    if (len < 1e-6) return;
    
    float64_t kx = axis_vec->x / len;
    float64_t ky = axis_vec->y / len;
    float64_t kz = axis_vec->z / len;
    
    float64_t s = sin(angle);
    float64_t c = cos(angle);
    float64_t t = 1.0 - c;
    
    float64_t R[9];
    R[0] = c + kx * kx * t;           R[1] = kx * ky * t - kz * s;      R[2] = kx * kz * t + ky * s;
    R[3] = ky * kx * t + kz * s;      R[4] = c + ky * ky * t;           R[5] = ky * kz * t - kx * s;
    R[6] = kz * kx * t - ky * s;      R[7] = kz * ky * t + kx * s;      R[8] = c + kz * kz * t;
    
    float64_t px = pivot_point->x;
    float64_t py = pivot_point->y;
    float64_t pz = pivot_point->z;
    
    float64_t tx = (1.0 - R[0]) * px - R[1] * py - R[2] * pz;
    float64_t ty = -R[3] * px + (1.0 - R[4]) * py - R[5] * pz;
    float64_t tz = -R[6] * px - R[7] * py + (1.0 - R[8]) * pz;
    
    float64_t T_action_data[16];
    T_action_data[0]  = R[0];  T_action_data[1]  = R[1];  T_action_data[2]  = R[2];  T_action_data[3]  = tx;
    T_action_data[4]  = R[3];  T_action_data[5]  = R[4];  T_action_data[6]  = R[5];  T_action_data[7]  = ty;
    T_action_data[8]  = R[6];  T_action_data[9]  = R[7];  T_action_data[10] = R[8];  T_action_data[11] = tz;
    T_action_data[12] = 0.0;  T_action_data[13] = 0.0;  T_action_data[14] = 0.0;  T_action_data[15] = 1.0;
    
    arm_matrix_instance_f64 T_action;
    arm_mat_init_f64(&T_action, 4, 4, T_action_data);
    
    my_mat_mult_f64(&T_action, T_current, T_result);
}

void TfMatrix_TranslateAlongVector(arm_matrix_instance_f64* T_result,
                                   const Vector3D_t* direction,
                                   float64_t distance)
{
    if (T_result == NULL || direction == NULL) return;
    
    float64_t len_sq = direction->x * direction->x + direction->y * direction->y + direction->z * direction->z;
    float64_t len = sqrt(len_sq);
    if (len < 1e-6) {
        TfMatrix_Eye(T_result);
        return;
    }
    
    float64_t dx = (direction->x / len) * distance;
    float64_t dy = (direction->y / len) * distance;
    float64_t dz = (direction->z / len) * distance;
    
    TfMatrix_Eye(T_result);
    T_result->pData[3]  = dx;
    T_result->pData[7]  = dy;
    T_result->pData[11] = dz;
}

/* ==================================================================================
 * ZXZ 欧拉角 -> 变换矩阵
 * ================================================================================== */

void TfMatrix_SetPose_ZXZ(arm_matrix_instance_f64* tfmat, const Pose6D_ZXZ_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;

    TfMatrix_Eye(tfmat);
    TfMatrix_For_Translation_Along_X(tfmat, pose->x);
    TfMatrix_For_Translation_Along_Y(tfmat, pose->y);
    TfMatrix_For_Translation_Along_Z(tfmat, pose->z);
    
    // yaw1(Z) -> roll(X) -> yaw2(Z)
    TfMatrix_For_Rotation_Around_Z(tfmat, pose->yaw1);
    TfMatrix_For_Rotation_Around_X(tfmat, pose->roll);
    TfMatrix_For_Rotation_Around_Z(tfmat, pose->yaw2);
}

void TfMatrix_SetPose_ZXZ_Deg(arm_matrix_instance_f64* tfmat, const Pose6D_ZXZ_Deg_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;

    Pose6D_ZXZ_t pose_rad = {
        .x = pose->x, .y = pose->y, .z = pose->z,
        .yaw1 = DEG2RAD(pose->yaw1_deg),
        .roll = DEG2RAD(pose->roll_deg),
        .yaw2 = DEG2RAD(pose->yaw2_deg)
    };
    TfMatrix_SetPose_ZXZ(tfmat, &pose_rad);
}

void TfMatrix_Compose_ZXZ(arm_matrix_instance_f64* tfmat, float64_t x, float64_t y, float64_t z, float64_t yaw1, float64_t roll, float64_t yaw2)
{
    Pose6D_ZXZ_t pose = { .x = x, .y = y, .z = z, .yaw1 = yaw1, .roll = roll, .yaw2 = yaw2 };
    TfMatrix_SetPose_ZXZ(tfmat, &pose);
}

void TfMatrix_Compose_ZXZ_Deg(arm_matrix_instance_f64* tfmat, float64_t x, float64_t y, float64_t z, float64_t yaw1_deg, float64_t roll_deg, float64_t yaw2_deg)
{
    Pose6D_ZXZ_Deg_t pose = { .x = x, .y = y, .z = z, .yaw1_deg = yaw1_deg, .roll_deg = roll_deg, .yaw2_deg = yaw2_deg };
    TfMatrix_SetPose_ZXZ_Deg(tfmat, &pose);
}

/* ==================================================================================
 * 变换矩阵 -> ZXZ 欧拉角
 * ================================================================================== */

void TfMatrix_GetPose_ZXZ(const arm_matrix_instance_f64* tfmat, Pose6D_ZXZ_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;
    
    const float64_t* M = tfmat->pData;
    
    pose->x = M[3];
    pose->y = M[7];
    pose->z = M[11];
    
    // 修复：直接从旋转部分提取角度，避免累积误差
    pose->yaw1 = atan2(M[1], M[0]);
    pose->roll = atan2(M[9], M[10]);
    pose->yaw2 = atan2(-M[8], sqrt(M[0]*M[0] + M[1]*M[1]));
}

/* ==================================================================================
 * ZYX 欧拉角 -> 变换矩阵
 * ================================================================================== */

void TfMatrix_SetPose_ZYX(arm_matrix_instance_f64* tfmat, const Pose6D_ZYX_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;
    
    TfMatrix_Eye(tfmat);
    TfMatrix_For_Translation_Along_X(tfmat, pose->x);
    TfMatrix_For_Translation_Along_Y(tfmat, pose->y);
    TfMatrix_For_Translation_Along_Z(tfmat, pose->z);
    
    // yaw(Z) -> pitch(Y) -> roll(X)
    TfMatrix_For_Rotation_Around_Z(tfmat, pose->yaw);
    TfMatrix_For_Rotation_Around_Y(tfmat, pose->pitch);
    TfMatrix_For_Rotation_Around_X(tfmat, pose->roll);
}

void TfMatrix_SetPose_ZYX_Deg(arm_matrix_instance_f64* tfmat, const Pose6D_ZYX_Deg_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;
    
    Pose6D_ZYX_t pose_rad = {
        .x = pose->x, .y = pose->y, .z = pose->z,
        .yaw   = DEG2RAD(pose->yaw_deg),
        .pitch = DEG2RAD(pose->pitch_deg),
        .roll  = DEG2RAD(pose->roll_deg)
    };
    TfMatrix_SetPose_ZYX(tfmat, &pose_rad);
}

void TfMatrix_Compose_ZYX(arm_matrix_instance_f64* tfmat, float64_t x, float64_t y, float64_t z, float64_t yaw, float64_t pitch, float64_t roll)
{
    Pose6D_ZYX_t pose = { .x = x, .y = y, .z = z, .yaw = yaw, .pitch = pitch, .roll = roll };
    TfMatrix_SetPose_ZYX(tfmat, &pose);
}

void TfMatrix_Compose_ZYX_Deg(arm_matrix_instance_f64* tfmat, float64_t x, float64_t y, float64_t z, float64_t yaw_deg, float64_t pitch_deg, float64_t roll_deg)
{
    Pose6D_ZYX_Deg_t pose = { .x = x, .y = y, .z = z, .yaw_deg = yaw_deg, .pitch_deg = pitch_deg, .roll_deg = roll_deg };
    TfMatrix_SetPose_ZYX_Deg(tfmat, &pose);
}

/* ==================================================================================
 * ZYX 欧拉角 <- 变换矩阵 (提取)
 * ================================================================================== */

void TfMatrix_GetPose_ZYX_Deg(const arm_matrix_instance_f64* tfmat, Pose6D_ZYX_Deg_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;
    const float64_t* M = tfmat->pData;
    
    // 平移
    pose->x = M[3];
    pose->y = M[7];
    pose->z = M[11];

    // 旋转提取 (ZYX)
    float64_t r11 = M[0], r12 = M[1];
    float64_t r21 = M[4], r22 = M[5];
    float64_t r31 = M[8], r32 = M[9], r33 = M[10];

    // pitch = asin(-r31), clamp to [-1, 1] for numerical safety
    float64_t pitch = asin(fmax(fmin(-r31, 1.0), -1.0));
    float64_t cos_pitch = cos(pitch);
    float64_t yaw, roll;
    if (fabs(cos_pitch) > 1e-6) {
        yaw  = atan2(r21, r11);
        roll = atan2(r32, r33);
    } else {
        // Gimbal lock: pitch ~= +-pi/2
        // set yaw = 0 and compute roll from r12/r22
        yaw = 0.0;
        roll = atan2(-r12, r22);
    }

    pose->yaw_deg   = RAD2DEG(yaw);
    pose->pitch_deg = RAD2DEG(pitch);
    pose->roll_deg  = RAD2DEG(roll);
}

/**
 * @brief 从变换矩阵提取 ZYX 欧拉角位姿到分立参数 (角度制)
 */
void TfMatrix_Decompose_ZYX_Deg(const arm_matrix_instance_f64* tfmat,
                                 float64_t* x, float64_t* y, float64_t* z,
                                 float64_t* yaw_deg, float64_t* pitch_deg, float64_t* roll_deg)
{
    if (tfmat == NULL || tfmat->pData == NULL) return;
    const float64_t* M = tfmat->pData;
    
    // 平移
    if (x) *x = M[3];
    if (y) *y = M[7];
    if (z) *z = M[11];

    // 旋转提取 (ZYX)
    float64_t r11 = M[0], r12 = M[1];
    float64_t r21 = M[4], r22 = M[5];
    float64_t r31 = M[8], r32 = M[9], r33 = M[10];

    // pitch = asin(-r31), clamp to [-1, 1] for numerical safety
    float64_t pitch = asin(fmax(fmin(-r31, 1.0), -1.0));
    float64_t cos_pitch = cos(pitch);
    float64_t yaw, roll;
    if (fabs(cos_pitch) > 1e-6) {
        yaw  = atan2(r21, r11);
        roll = atan2(r32, r33);
    } else {
        // Gimbal lock: pitch ~= +-pi/2
        yaw = 0.0;
        roll = atan2(-r12, r22);
    }

    if (yaw_deg)   *yaw_deg   = RAD2DEG(yaw);
    if (pitch_deg) *pitch_deg = RAD2DEG(pitch);
    if (roll_deg)  *roll_deg  = RAD2DEG(roll);
}

/* ==================================================================================
 * ZYZ 欧拉角 -> 变换矩阵
 * ================================================================================== */

void TfMatrix_SetPose_ZYZ(arm_matrix_instance_f64* tfmat, const Pose6D_ZYZ_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;
    
    TfMatrix_Eye(tfmat);
    TfMatrix_For_Translation_Along_X(tfmat, pose->x);
    TfMatrix_For_Translation_Along_Y(tfmat, pose->y);
    TfMatrix_For_Translation_Along_Z(tfmat, pose->z);
    
    // yaw1(Z) -> pitch(Y) -> yaw2(Z)
    TfMatrix_For_Rotation_Around_Z(tfmat, pose->yaw1);
    TfMatrix_For_Rotation_Around_Y(tfmat, pose->pitch);
    TfMatrix_For_Rotation_Around_Z(tfmat, pose->yaw2);
}

void TfMatrix_SetPose_ZYZ_Deg(arm_matrix_instance_f64* tfmat, const Pose6D_ZYZ_Deg_t* pose)
{
    if (tfmat == NULL || tfmat->pData == NULL || pose == NULL) return;
    
    Pose6D_ZYZ_t pose_rad = {
        .x = pose->x, .y = pose->y, .z = pose->z,
        .yaw1  = DEG2RAD(pose->yaw1_deg),
        .pitch = DEG2RAD(pose->pitch_deg),
        .yaw2  = DEG2RAD(pose->yaw2_deg)
    };
    TfMatrix_SetPose_ZYZ(tfmat, &pose_rad);
}

void TfMatrix_Compose_ZYZ(arm_matrix_instance_f64* tfmat, float64_t x, float64_t y, float64_t z, float64_t yaw1, float64_t pitch, float64_t yaw2)
{
    Pose6D_ZYZ_t pose = { .x = x, .y = y, .z = z, .yaw1 = yaw1, .pitch = pitch, .yaw2 = yaw2 };
    TfMatrix_SetPose_ZYZ(tfmat, &pose);
}

void TfMatrix_Compose_ZYZ_Deg(arm_matrix_instance_f64* tfmat, float64_t x, float64_t y, float64_t z, float64_t yaw1_deg, float64_t pitch_deg, float64_t yaw2_deg)
{
    Pose6D_ZYZ_Deg_t pose = { .x = x, .y = y, .z = z, .yaw1_deg = yaw1_deg, .pitch_deg = pitch_deg, .yaw2_deg = yaw2_deg };
    TfMatrix_SetPose_ZYZ_Deg(tfmat, &pose);
}

/* ==================================================================================
 * 误差计算
 * ================================================================================== */

float64_t TfMatrix_PositionError(const arm_matrix_instance_f64* T1, const arm_matrix_instance_f64* T2)
{
    if (T1 == NULL || T2 == NULL || T1->pData == NULL || T2->pData == NULL) return -1.0;
    
    float64_t dx = T1->pData[3]  - T2->pData[3];
    float64_t dy = T1->pData[7]  - T2->pData[7];
    float64_t dz = T1->pData[11] - T2->pData[11];
    
    return sqrt(dx*dx + dy*dy + dz*dz);
}

float64_t TfMatrix_RotationError(const arm_matrix_instance_f64* T1, const arm_matrix_instance_f64* T2)
{
    if (T1 == NULL || T2 == NULL || T1->pData == NULL || T2->pData == NULL) return -1.0;
    
    float64_t* pData1 = T1->pData;
    float64_t* pData2 = T2->pData;
    float64_t sum = 0.0;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int idx = i * 4 + j;
            float64_t d = pData1[idx] - pData2[idx];
            sum += d * d;
        }
    }
    
    return sqrt(sum);
}

void TfMatrix_PoseError(const arm_matrix_instance_f64* T1, const arm_matrix_instance_f64* T2, float64_t* pos_error, float64_t* rot_error)
{
    if (T1 == NULL || T2 == NULL || T1->pData == NULL || T2->pData == NULL) {
        if (pos_error) *pos_error = -1.0;
        if (rot_error) *rot_error = -1.0;
        return;
    }
    
    float64_t* pData1 = T1->pData;
    float64_t* pData2 = T2->pData;

    if (pos_error) {
        float64_t dx = pData1[3]  - pData2[3];
        float64_t dy = pData1[7]  - pData2[7];
        float64_t dz = pData1[11] - pData2[11];
        *pos_error = sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    if (rot_error) {
        float64_t sum = 0.0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                int idx = i * 4 + j;
                float64_t d = pData1[idx] - pData2[idx];
                sum += d * d;
            }
        }
        *rot_error = sqrt(sum);
    }
}


/**
 * @brief [核心修复] 旋转矩阵归一化 (Gram-Schmidt 正交化)
 * @details 
 *   解决浮点误差导致的"散点"和"不丝滑"问题。
 *   
 *   【问题原因】
 *   float64_t 双精度浮点数有 15-16 位有效数字，但经过多次矩阵乘法后，
 *   旋转矩阵的列向量会失去正交性（夹角不再是90度），
 *   或者模长不再是1（变成0.9999999999999或1.0000000000001）。
 *   
 *   这会导致：
 *   1. acos(1.00000000000001) 返回 NaN → 散点
 *   2. 基于歪斜坐标系的逆解不连续 → 抖动
 *   
 *   【修复原理】Gram-Schmidt 正交化
 *   1. 锁定 n 轴 (X轴)，归一化使其模长严格为 1
 *   2. 用叉乘 a_new = n × o，得到垂直于 n 的新 Z 轴
 *   3. 用叉乘 o_new = a_new × n，得到垂直于 n 和 a 的新 Y 轴
 *   
 *   这样，三个轴两两正交，模长都为 1，旋转矩阵重新合法。
 * 
 * @param T 要归一化的 4x4 变换矩阵（只修改旋转部分，平移部分不变）
 */
void TFMatrix_NormalizeRotation(arm_matrix_instance_f64* T)
{
    if (T == NULL || T->pData == NULL) return;
    
    float64_t* pData = T->pData;
    
    // 提取旋转矩阵的列向量 (n, o, a)
    // 4x4 矩阵布局 (行主序):
    // [0]  [1]  [2]  [3]     n.x  o.x  a.x  tx
    // [4]  [5]  [6]  [7]  =  n.y  o.y  a.y  ty
    // [8]  [9]  [10] [11]    n.z  o.z  a.z  tz
    // [12] [13] [14] [15]    0    0    0    1
    
    float64_t nx = pData[0], ny = pData[4], nz = pData[8];
    float64_t ox = pData[1], oy = pData[5], oz = pData[9];
    // a 向量会通过叉乘重新计算
    
    // 1. 归一化 n 向量 (X轴)
    float64_t n_norm_sq = nx*nx + ny*ny + nz*nz;
    float64_t n_norm = sqrt(n_norm_sq);
    if (n_norm < 1e-6) return; // 防止除零
    nx /= n_norm; ny /= n_norm; nz /= n_norm;
    
    // 2. 重新计算 a 向量 (Z轴) = n × o
    //    叉乘结果天生垂直于 n
    float64_t ax_new = ny * oz - nz * oy;
    float64_t ay_new = nz * ox - nx * oz;
    float64_t az_new = nx * oy - ny * ox;
    
    // 归一化新的 a 向量
    float64_t a_norm_sq = ax_new*ax_new + ay_new*ay_new + az_new*az_new;
    float64_t a_norm = sqrt(a_norm_sq);
    if (a_norm < 1e-6) return;
    ax_new /= a_norm; ay_new /= a_norm; az_new /= a_norm;
    
    // 3. 重新计算 o 向量 (Y轴) = a_new × n
    //    保证 o 同时垂直于 n 和 a，且模长为 1
    float64_t ox_new = ay_new * nz - az_new * ny;
    float64_t oy_new = az_new * nx - ax_new * nz;
    float64_t oz_new = ax_new * ny - ay_new * nx;
    
    // 4. 写回矩阵 (只修改旋转部分，平移部分保持不变)
    pData[0] = nx;     pData[4] = ny;     pData[8]  = nz;
    pData[1] = ox_new; pData[5] = oy_new; pData[9]  = oz_new;
    pData[2] = ax_new; pData[6] = ay_new; pData[10] = az_new;
}

/**
 * @brief 旋转矩阵归一化 (修复版：保Z轴优先，用于机械臂末端姿态控制)
 * @details 当机械臂旋转矩阵因累积误差偏离正交时，优先保持Z轴(末端朝向)精度
 * 
 * 算法策略:
 *   1. 保持并归一化 a 向量 (Z轴, 末端朝向)
 *   2. 用 n × a 重新计算 o 向量 (Y轴)  
 *   3. 用叉乘 n_new = o × a，得到垂直于 o 和 a 的新 X 轴
 *   
 *   这样可以确保末端的指向精度，适用于抓取、焊接等对方向敏感的应用
 * 
 * @param T 要归一化的 4x4 变换矩阵
 */
void TFMatrix_NormalizeRotation_PreserveZ(arm_matrix_instance_f64* T)
{
    if (T == NULL || T->pData == NULL) return;
    
    float64_t* pData = T->pData;
    
    // 提取旋转矩阵的列向量 (n, o, a)
    float64_t nx = pData[0], ny = pData[4], nz = pData[8];
    float64_t ox = pData[1], oy = pData[5], oz = pData[9];
    float64_t ax = pData[2], ay = pData[6], az = pData[10];
    
    // 1. 归一化 a 向量 (Z轴) - 优先保持末端朝向
    float64_t a_norm_sq = ax*ax + ay*ay + az*az;
    float64_t a_norm = sqrt(a_norm_sq);
    if (a_norm < 1e-6) return; // 防止除零
    ax /= a_norm; ay /= a_norm; az /= a_norm;
    
    // 2. 重新计算 o 向量 (Y轴) = a × n
    //    叉乘结果天生垂直于 a
    float64_t ox_new = ay * nz - az * ny;
    float64_t oy_new = az * nx - ax * nz;
    float64_t oz_new = ax * ny - ay * nx;
    
    // 归一化新的 o 向量
    float64_t o_norm_sq = ox_new*ox_new + oy_new*oy_new + oz_new*oz_new;
    float64_t o_norm = sqrt(o_norm_sq);
    if (o_norm < 1e-6) return;
    ox_new /= o_norm; oy_new /= o_norm; oz_new /= o_norm;
    
    // 3. 重新计算 n 向量 (X轴) = o_new × a
    //    保证 n 同时垂直于 o 和 a，且模长为 1
    float64_t nx_new = oy_new * az - oz_new * ay;
    float64_t ny_new = oz_new * ax - ox_new * az;
    float64_t nz_new = ox_new * ay - oy_new * ax;
    
    // 4. 写回矩阵 (只修改旋转部分，平移部分保持不变)
    pData[0] = nx_new; pData[4] = ny_new; pData[8]  = nz_new;
    pData[1] = ox_new; pData[5] = oy_new; pData[9]  = oz_new;
    pData[2] = ax;     pData[6] = ay;     pData[10] = az;
}
