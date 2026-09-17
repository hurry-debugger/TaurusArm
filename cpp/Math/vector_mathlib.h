#ifndef __Vector_mathlib_H__
#define __Vector_mathlib_H__

#include "kinematics_types.h"
#include "arm_math.h"

// Vector3D_t is now defined in kinematics_types.h to unify with Coordinates_t.
// typedef Vector3D_t Coordinates_t; is also in kinematics_types.h

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
 * @brief 空间轴结构体，包含一个点和一个方向向量来表示空间中的轴
 *        轴的表示方式：过点 point，方向为 direction 的直线
 */
#pragma pack(1)
typedef struct Axis3D_t{
    Coordinates_t point;    // 轴上的一个点
    Vector3D_t direction;   // 轴的方向向量（单位向量）
}Axis3D_t;
#pragma pack()

/**
 * @brief 空间线段/向量结构体，用起点和终点表示
 *        可用于表示空间中的有向线段或位置向量
 */
#pragma pack(1)
typedef struct LineSegment3D_t{
    Coordinates_t start;    // 起点
    Coordinates_t end;      // 终点
}LineSegment3D_t;
#pragma pack()

/**
 * @brief 欧拉角结构体 (ZYX顺序: yaw, pitch, roll)
 */
#pragma pack(1)
typedef struct EulerAngle_t{
    float64_t yaw;      // 绕Z轴旋转 (rad)
    float64_t pitch;    // 绕Y轴旋转 (rad)
    float64_t roll;     // 绕X轴旋转 (rad)
}EulerAngle_t;
#pragma pack()

/**
 * @brief 初始化三维向量
 * @param vec 向量指针
 * @param x X分量
 * @param y Y分量
 * @param z Z分量
 */
void Vector3D_Init(Vector3D_t* vec, float64_t x, float64_t y, float64_t z);

/**
 * @brief 初始化空间轴
 * @param axis 轴指针
 * @param px 轴上点的X坐标
 * @param py 轴上点的Y坐标
 * @param pz 轴上点的Z坐标
 * @param dx 方向向量X分量
 * @param dy 方向向量Y分量
 * @param dz 方向向量Z分量
 */
void Axis3D_Init(Axis3D_t* axis, float64_t px, float64_t py, float64_t pz, float64_t dx, float64_t dy, float64_t dz);

/**
 * @brief 初始化空间线段
 * @param seg 线段指针
 * @param sx 起点X坐标
 * @param sy 起点Y坐标
 * @param sz 起点Z坐标
 * @param ex 终点X坐标
 * @param ey 终点Y坐标
 * @param ez 终点Z坐标
 */
void LineSegment3D_Init(LineSegment3D_t* seg, float64_t sx, float64_t sy, float64_t sz, float64_t ex, float64_t ey, float64_t ez);

/**
 * @brief 通过起点和方向向量初始化线段
 * @param seg 线段指针
 * @param start 起点
 * @param direction 方向向量
 * @param length 线段长度
 */
void LineSegment3D_InitFromDirection(LineSegment3D_t* seg, const Coordinates_t* start, const Vector3D_t* direction, float64_t length);

/**
 * @brief 向量归一化（转换为单位向量）
 * @param vec 向量指针
 */
void Vector3D_Normalize(Vector3D_t* vec);

/**
 * @brief 计算向量的模长
 * @param vec 向量指针
 * @return 向量的模长
 */
float64_t Vector3D_Magnitude(const Vector3D_t* vec);

/**
 * @brief 向量点积
 * @param v1 向量1
 * @param v2 向量2
 * @return 点积结果
 */
float64_t Vector3D_Dot(const Vector3D_t* v1, const Vector3D_t* v2);

/**
 * @brief 向量叉积
 * @param v1 向量1
 * @param v2 向量2
 * @param result 结果向量
 */
void Vector3D_Cross(const Vector3D_t* v1, const Vector3D_t* v2, Vector3D_t* result);

/**
 * @brief 向量标量乘法
 * @param vec 向量
 * @param scalar 标量
 * @param result 结果向量
 */
void Vector3D_Scale(const Vector3D_t* vec, float64_t scalar, Vector3D_t* result);

/**
 * @brief 向量加法
 * @param v1 向量1
 * @param v2 向量2
 * @param result 结果向量
 */
void Vector3D_Add(const Vector3D_t* v1, const Vector3D_t* v2, Vector3D_t* result);

/**
 * @brief 向量减法
 * @param v1 向量1
 * @param v2 向量2
 * @param result 结果向量 = v1 - v2
 */
void Vector3D_Sub(const Vector3D_t* v1, const Vector3D_t* v2, Vector3D_t* result);

/**
 * @brief 罗德里格斯旋转公式 - 向量绕任意轴旋转
 *        公式: v_rot = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
 *        其中 k 是旋转轴的单位向量，θ 是旋转角度，v 是被旋转的向量
 * @param vec 被旋转的向量
 * @param axis 旋转轴（方向向量需要是单位向量，函数内部会自动归一化）
 * @param angle 旋转角度（弧度）
 * @param result 旋转后的向量
 */
void Rodrigues_Rotate(const Vector3D_t* vec, const Vector3D_t* axis, float64_t angle, Vector3D_t* result);

/**
 * @brief 点绕空间中任意轴旋转
 *        先将点平移到轴过原点，进行罗德里格斯旋转，再平移回去
 * @param point 被旋转的点
 * @param axis 旋转轴（包含点和方向）
 * @param angle 旋转角度（弧度）
 * @param result 旋转后的点
 */
void Rotate_Point_Around_Axis(const Coordinates_t* point, const Axis3D_t* axis, float64_t angle, Coordinates_t* result);

/**
 * @brief 基于罗德里格斯公式创建旋转矩阵
 *        R = I + sin(θ)*K + (1-cos(θ))*K?
 *        其中 K 是轴向量的反对称矩阵
 * @param tfmat 输出的4x4齐次变换矩阵
 * @param axis 旋转轴方向向量
 * @param angle 旋转角度（弧度）
 */
void Rodrigues_Rotation_Matrix(arm_matrix_instance_f64* tfmat, const Vector3D_t* axis, float64_t angle);

/**
 * @brief 获取线段的方向向量（单位向量）
 * @param seg 线段指针
 * @param direction 输出的方向向量
 */
void LineSegment3D_GetDirection(const LineSegment3D_t* seg, Vector3D_t* direction);

/**
 * @brief 获取线段的长度
 * @param seg 线段指针
 * @return 线段长度
 */
float64_t LineSegment3D_GetLength(const LineSegment3D_t* seg);

/**
 * @brief 空间线段（向量）绕任意轴旋转
 *        将线段的起点和终点分别绕轴旋转
 * @param seg 被旋转的线段
 * @param axis 旋转轴（包含点和方向）
 * @param angle 旋转角度（弧度）
 * @param result 旋转后的线段
 */
void Rotate_LineSegment_Around_Axis(const LineSegment3D_t* seg, const Axis3D_t* axis, float64_t angle, LineSegment3D_t* result);

/**
 * @brief 空间向量绕任意轴旋转（通过起点和终点描述）
 *        将起点和终点分别绕轴旋转
 * @param start 向量起点
 * @param end 向量终点
 * @param axis 旋转轴（包含点和方向）
 * @param angle 旋转角度（弧度）
 * @param result_start 旋转后的起点
 * @param result_end 旋转后的终点
 */
void Rotate_Vector_Around_Axis(const Coordinates_t* start, const Coordinates_t* end, 
                                const Axis3D_t* axis, float64_t angle,
                                Coordinates_t* result_start, Coordinates_t* result_end);

/**
 * @brief 从方向向量计算欧拉角 (ZYX顺序)
 *        假设初始方向为X轴正方向(1,0,0)
 * @param direction 目标方向向量
 * @param euler 输出欧拉角
 */
void Direction_To_Euler(const Vector3D_t* direction, EulerAngle_t* euler);

/**
 * @brief 从方向向量计算旋转矩阵
 *        计算将X轴(1,0,0)旋转到目标方向的旋转矩阵
 * @param direction 目标方向向量
 * @param tfmat 输出旋转矩阵
 */
void Direction_To_RotationMatrix(const Vector3D_t* direction, arm_matrix_instance_f64* tfmat);

#endif
