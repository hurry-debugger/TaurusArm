#include "Vector_mathlib.h"
#include "string.h"
#include <math.h>

/**
 * @brief Initialize a 3D Vector
 */
void Vector3D_Init(Vector3D_t* vec, float64_t x, float64_t y, float64_t z)
{
    vec->x = x;
    vec->y = y;
    vec->z = z;
}

/**
 * @brief Initialize a 3D Axis
 *        Automatically normalizes the direction vector
 */
void Axis3D_Init(Axis3D_t* axis, float64_t px, float64_t py, float64_t pz, float64_t dx, float64_t dy, float64_t dz)
{
    axis->point.x = px;
    axis->point.y = py;
    axis->point.z = pz;
    Vector3D_Init(&axis->direction, dx, dy, dz);
    Vector3D_Normalize(&axis->direction);
}

/**
 * @brief 计算向量的模长
 */
float64_t Vector3D_Magnitude(const Vector3D_t* vec)
{
    float64_t mag_sq = vec->x * vec->x + vec->y * vec->y + vec->z * vec->z;
    return sqrt(mag_sq);
}

/**
 * @brief 向量归一化（转换为单位向量）
 */
void Vector3D_Normalize(Vector3D_t* vec)
{
    float64_t mag = Vector3D_Magnitude(vec);
    if(mag > 1e-6)  // 避免除零
    {
        vec->x /= mag;
        vec->y /= mag;
        vec->z /= mag;
    }
}

/**
 * @brief 向量点积
 */
float64_t Vector3D_Dot(const Vector3D_t* v1, const Vector3D_t* v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

/**
 * @brief 向量叉积
 *        result = v1 × v2
 */
void Vector3D_Cross(const Vector3D_t* v1, const Vector3D_t* v2, Vector3D_t* result)
{
    Vector3D_t temp;
    temp.x = v1->y * v2->z - v1->z * v2->y;
    temp.y = v1->z * v2->x - v1->x * v2->z;
    temp.z = v1->x * v2->y - v1->y * v2->x;
    
    result->x = temp.x;
    result->y = temp.y;
    result->z = temp.z;
}

/**
 * @brief 向量标量乘法
 */
void Vector3D_Scale(const Vector3D_t* vec, float64_t scalar, Vector3D_t* result)
{
    result->x = vec->x * scalar;
    result->y = vec->y * scalar;
    result->z = vec->z * scalar;
}

/**
 * @brief 向量加法
 */
void Vector3D_Add(const Vector3D_t* v1, const Vector3D_t* v2, Vector3D_t* result)
{
    result->x = v1->x + v2->x;
    result->y = v1->y + v2->y;
    result->z = v1->z + v2->z;
}

/**
 * @brief 向量减法
 */
void Vector3D_Sub(const Vector3D_t* v1, const Vector3D_t* v2, Vector3D_t* result)
{
    result->x = v1->x - v2->x;
    result->y = v1->y - v2->y;
    result->z = v1->z - v2->z;
}

/**
 * @brief 罗德里格斯旋转公式 - 向量绕任意轴旋转
 *        公式: v_rot = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
 *        其中 k 是旋转轴的单位向量，θ 是旋转角度，v 是被旋转的向量
 */
void Rodrigues_Rotate(const Vector3D_t* vec, const Vector3D_t* axis, float64_t angle, Vector3D_t* result)
{
    // 确保轴是单位向量
    Vector3D_t k;
    k.x = axis->x;
    k.y = axis->y;
    k.z = axis->z;
    Vector3D_Normalize(&k);
    
    // 计算三角函数值
    float64_t cos_theta = cos(angle);
    float64_t sin_theta = sin(angle);
    
    // 计算 k × v (叉积)
    Vector3D_t k_cross_v;
    Vector3D_Cross(&k, vec, &k_cross_v);
    
    // 计算 k · v (点积)
    float64_t k_dot_v = Vector3D_Dot(&k, vec);
    
    // 罗德里格斯公式: v_rot = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
    // 第一项: v * cos(θ)
    Vector3D_t term1;
    Vector3D_Scale(vec, cos_theta, &term1);
    
    // 第二项: (k × v) * sin(θ)
    Vector3D_t term2;
    Vector3D_Scale(&k_cross_v, sin_theta, &term2);
    
    // 第三项: k * (k · v) * (1 - cos(θ))
    Vector3D_t term3;
    Vector3D_Scale(&k, k_dot_v * (1.0 - cos_theta), &term3);
    
    // 结果 = term1 + term2 + term3
    Vector3D_t temp;
    Vector3D_Add(&term1, &term2, &temp);
    Vector3D_Add(&temp, &term3, result);
}

/**
 * @brief 点绕空间中任意轴旋转
 *        先将点平移到轴过原点，进行罗德里格斯旋转，再平移回去
 */
void Rotate_Point_Around_Axis(const Coordinates_t* point, const Axis3D_t* axis, float64_t angle, Coordinates_t* result)
{
    // 1. 将点平移，使轴过原点
    //    translated = point - axis.point
    Vector3D_t translated;
    translated.x = point->x - axis->point.x;
    translated.y = point->y - axis->point.y;
    translated.z = point->z - axis->point.z;

    // 2. 使用罗德里格斯公式进行旋转
    Vector3D_t rotated;
    Rodrigues_Rotate(&translated, &axis->direction, angle, &rotated);
    
    // 3. 平移回去
    //    result = rotated + axis.point
    result->x = rotated.x + axis->point.x;
    result->y = rotated.y + axis->point.y;
    result->z = rotated.z + axis->point.z;
}

/**
 * @brief 基于罗德里格斯公式创建旋转矩阵
 *        R = I + sin(θ)*K + (1-cos(θ))*K?
 *        其中 K 是轴向量的反对称矩阵:
 *        K = |  0  -kz   ky |
 *            |  kz  0   -kx |
 *            | -ky  kx   0  |
 */
void Rodrigues_Rotation_Matrix(arm_matrix_instance_f64* tfmat, const Vector3D_t* axis, float64_t angle)
{
    // 确保轴是单位向量
    Vector3D_t k;
    k.x = axis->x;
    k.y = axis->y;
    k.z = axis->z;
    Vector3D_Normalize(&k);
    
    float64_t kx = k.x;
    float64_t ky = k.y;
    float64_t kz = k.z;
    
    float64_t cos_theta = cos(angle);
    float64_t sin_theta = sin(angle);
    float64_t one_minus_cos = 1.0 - cos_theta;
    
    // 罗德里格斯旋转矩阵:
    // R = I + sin(θ)*K + (1-cos(θ))*K?
    // 展开后:
    // R[0][0] = cos(θ) + kx?(1-cos(θ))
    // R[0][1] = kx*ky*(1-cos(θ)) - kz*sin(θ)
    // R[0][2] = kx*kz*(1-cos(θ)) + ky*sin(θ)
    // R[1][0] = ky*kx*(1-cos(θ)) + kz*sin(θ)
    // R[1][1] = cos(θ) + ky?(1-cos(θ))
    // R[1][2] = ky*kz*(1-cos(θ)) - kx*sin(θ)
    // R[2][0] = kz*kx*(1-cos(θ)) - ky*sin(θ)
    // R[2][1] = kz*ky*(1-cos(θ)) + kx*sin(θ)
    // R[2][2] = cos(θ) + kz?(1-cos(θ))
    
    float64_t tfMatrix_array[4][4] = {
        {cos_theta + kx*kx*one_minus_cos,       kx*ky*one_minus_cos - kz*sin_theta,  kx*kz*one_minus_cos + ky*sin_theta, 0},
        {ky*kx*one_minus_cos + kz*sin_theta,    cos_theta + ky*ky*one_minus_cos,     ky*kz*one_minus_cos - kx*sin_theta, 0},
        {kz*kx*one_minus_cos - ky*sin_theta,    kz*ky*one_minus_cos + kx*sin_theta,  cos_theta + kz*kz*one_minus_cos,    0},
        {0,                                     0,                                   0,                                  1}
    };
    
    memcpy(tfmat->pData, tfMatrix_array, 16*sizeof(float64_t));
}

/**
 * @brief 初始化空间线段
 */
void LineSegment3D_Init(LineSegment3D_t* seg, float64_t sx, float64_t sy, float64_t sz, float64_t ex, float64_t ey, float64_t ez)
{
    seg->start.x = sx;
    seg->start.y = sy;
    seg->start.z = sz;
    seg->end.x = ex;
    seg->end.y = ey;
    seg->end.z = ez;
}

/**
 * @brief 通过起点和方向向量初始化线段
 */
void LineSegment3D_InitFromDirection(LineSegment3D_t* seg, const Coordinates_t* start, const Vector3D_t* direction, float64_t length)
{
    seg->start.x = start->x;
    seg->start.y = start->y;
    seg->start.z = start->z;
    
    // 归一化方向向量
    Vector3D_t dir_normalized;
    dir_normalized.x = direction->x;
    dir_normalized.y = direction->y;
    dir_normalized.z = direction->z;
    Vector3D_Normalize(&dir_normalized);
    
    // 终点 = 起点 + 方向 * 长度
    seg->end.x = start->x + dir_normalized.x * length;
    seg->end.y = start->y + dir_normalized.y * length;
    seg->end.z = start->z + dir_normalized.z * length;
}

/**
 * @brief 获取线段的方向向量（单位向量）
 */
void LineSegment3D_GetDirection(const LineSegment3D_t* seg, Vector3D_t* direction)
{
    direction->x = seg->end.x - seg->start.x;
    direction->y = seg->end.y - seg->start.y;
    direction->z = seg->end.z - seg->start.z;
    Vector3D_Normalize(direction);
}

/**
 * @brief 获取线段的长度
 */
float64_t LineSegment3D_GetLength(const LineSegment3D_t* seg)
{
    Vector3D_t diff;
    diff.x = seg->end.x - seg->start.x;
    diff.y = seg->end.y - seg->start.y;
    diff.z = seg->end.z - seg->start.z;
    return Vector3D_Magnitude(&diff);
}

/**
 * @brief 空间线段（向量）绕任意轴旋转
 *        将线段的起点和终点分别绕轴旋转
 */
void Rotate_LineSegment_Around_Axis(const LineSegment3D_t* seg, const Axis3D_t* axis, float64_t angle, LineSegment3D_t* result)
{
    // 分别旋转起点和终点
    Rotate_Point_Around_Axis(&seg->start, axis, angle, &result->start);
    Rotate_Point_Around_Axis(&seg->end, axis, angle, &result->end);
}

/**
 * @brief 空间向量绕任意轴旋转（通过起点和终点描述）
 *        将起点和终点分别绕轴旋转
 */
void Rotate_Vector_Around_Axis(const Coordinates_t* start, const Coordinates_t* end, 
                                const Axis3D_t* axis, float64_t angle,
                                Coordinates_t* result_start, Coordinates_t* result_end)
{
    // 分别旋转起点和终点
    Rotate_Point_Around_Axis(start, axis, angle, result_start);
    Rotate_Point_Around_Axis(end, axis, angle, result_end);
}

/**
 * @brief 从方向向量计算欧拉角 (ZYX顺序)
 *        假设初始方向为X轴正方向(1,0,0)
 */
void Direction_To_Euler(const Vector3D_t* direction, EulerAngle_t* euler)
{
    Vector3D_t dir = *direction;
    Vector3D_Normalize(&dir);
    
    // pitch = -asin(dz)
    // 限制输入范围在 [-1, 1]
    float64_t dz = dir.z;
    if (dz > 1.0) dz = 1.0;
    if (dz < -1.0) dz = -1.0; 
    
    euler->pitch = -asin(dz);
    
    // yaw = atan2(dy, dx)
    euler->yaw = atan2(dir.y, dir.x);
    
    // roll = 0 (方向向量无法确定roll)
    euler->roll = 0.0;
}

/**
 * @brief 从方向向量计算旋转矩阵 (修复版：加入Up Vector约束)
 *        使用LookAt算法生成稳定的旋转矩阵，防止Roll角失控
 */
void Direction_To_RotationMatrix(const Vector3D_t* direction, arm_matrix_instance_f64* tfmat)
{
    // 1. 目标 X 轴 (Forward)
    Vector3D_t xaxis = *direction;
    Vector3D_Normalize(&xaxis);

    // 2. 默认的世界 Z 轴 (Up)
    Vector3D_t world_up = {0.0, 0.0, 1.0};

    // 3. 计算 Y 轴 (Left) = Up x Forward
    Vector3D_t yaxis;
    Vector3D_Cross(&world_up, &xaxis, &yaxis);
    
    // 处理死锁：如果目标方向直指天/地，Y轴会变成0
    if (Vector3D_Magnitude(&yaxis) < 1e-3) {
        // 此时 X 轴接近 (0,0,1)，我们改用 Y 轴 (0,1,0) 做参考
        Vector3D_t fallback_up = {0.0, 1.0, 0.0};
        Vector3D_Cross(&fallback_up, &xaxis, &yaxis);
    }
    Vector3D_Normalize(&yaxis);

    // 4. 计算 Z 轴 (Real Up) = Forward x Left (右手系)
    Vector3D_t zaxis;
    Vector3D_Cross(&xaxis, &yaxis, &zaxis);
    Vector3D_Normalize(&zaxis);

    // 5. 填入矩阵 (列向量)
    float64_t* m = tfmat->pData;
    m[0] = xaxis.x; m[1] = yaxis.x; m[2] = zaxis.x; m[3] = 0;
    m[4] = xaxis.y; m[5] = yaxis.y; m[6] = zaxis.y; m[7] = 0;
    m[8] = xaxis.z; m[9] = yaxis.z; m[10]= zaxis.z; m[11]= 0;
    m[12]= 0;       m[13]= 0;       m[14]= 0;       m[15]= 1;
}
