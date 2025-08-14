#ifndef FLIGHT_CONTROL_ATTITUDE_CONTROL_H_
#define FLIGHT_CONTROL_ATTITUDE_CONTROL_H_

#include "position_control.h"
#include "position_estimator.h"
#include "attitude_estimator.h"

// 定义一个结构体类型 motor_out_t，用于存储四个电机的输出值
// 结构体中包含一个长度为 4 的浮点型数组 m，每个元素对应一个电机的输出
typedef struct { float m[4]; } motor_out_t;

// // 定义三维向量结构体
// typedef struct { float v[3]; } vec3f; // 3 轴向量

// 定义 3x3 矩阵结构体
typedef struct { float v[9]; } mat3f; // 3×3 矩阵



void update_rotation_matrix(void);//更新当前姿态旋转矩阵
void update_sprotation_matrix(void); // 更新期望姿态旋转矩阵

void attitude_error_to_rates(void);

void update_W_rate(void);//更新当前角速度和速度环增益
void rate_error_to_torque(float dt);
void torque_to_motor(const vec3f *torque, float thrust, motor_out_t *out);

#endif /* FLIGHT_CONTROL_ATTITUDE_CONTROL_H_ */



