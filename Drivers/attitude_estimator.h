#ifndef FLIGHT_CONTROL_ATTITUDE_ESTIMATOR_H_
#define FLIGHT_CONTROL_ATTITUDE_ESTIMATOR_H_

#include "position_estimator.h"
#include "imu_calib.h"

// // 三维向量
// typedef struct {
//     union {
//         struct {
//             float x, y, z;
//         };
//         float v[3];
//     };
// } vec3f;

// typedef struct { float w, x, y, z; } quatf; // 四元数
#define ZUPT_WINDOW_LEN  100

/* PX4 式双阈值 —— 零偏冻结条件 */
#define GYRO_BIAS_GYR_THRESH       0.08f   // rad/s ≈ 4.6 °/s
#define GYRO_BIAS_ACC_ERR_THRESH   0.15f   // 加速度误差相对阈值

#define GYRO_BIAS_LIMIT  0.04f  // ± X rad，可按需调整, 陀螺仪积分限幅

// 定义一个结构体 euler_t，用于存储欧拉角，包括滚转角（roll）、俯仰角（pitch）和偏航角（yaw）
typedef struct { float roll, pitch, yaw; } euler_t;

/* 实时传感器值（外部提供） */
extern vec3f _accel;   // 机体加速度 (m/s²)
extern vec3f _gyro;    // 机体角速度 (rad/s)
// extern vec3f _mag;     // 机体磁力计 (Gauss)
void attitude_update(float dt,  // 时间间隔，单位为秒，用于积分计算
                     const vec3f *accel,  // 指向加速度计数据的指针，单位为 m/s²
                     const vec3f *gyro );  // 指向陀螺仪数据的指针，单位为 rad/s
quatf attitude_get_quat(void);  // 获取当前姿态四元数

euler_t attitude_get_euler(void);

void static_zero_update(const vec3f *accel, const vec3f *gyro);
#endif /* FLIGHT_CONTROL_ATTITUDE_ESTIMATOR_H_ */
