//#ifndef FLIGHT_CONTROL_ATTITUDE_EKF_H_
//#define FLIGHT_CONTROL_ATTITUDE_EKF_H_
//
//#include <stdint.h>
//#include "position_estimator.h"
////typedef struct {
////    union {
////        struct { float x, y, z; };
////        float v[3];
////    };
////} vec3f;
//
////typedef struct {
////    union {
////        struct { float w, x, y, z; };
////        float q[4];
////    };
////} quatf;
//
//typedef struct { float roll, pitch, yaw; } euler_t;
//
///* 对外接口 */
//void attitude_update_ekf(float dt, const vec3f *accel, const vec3f *gyro);
//quatf   attitude_get_quat(void);
//euler_t attitude_get_euler(void);
//const float (*attitude_get_R(void))[3];
//
//
//
//#endif /* FLIGHT_CONTROL_ATTITUDE_EKF_H_ */
