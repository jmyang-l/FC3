#ifndef FLIGHT_CONTROL_POSITION_CONTROL_H_
#define FLIGHT_CONTROL_POSITION_CONTROL_H_
#include <stdint.h>
#include"main.h"
#include"position_estimator.h"

#define M_PI_F      3.14159265358979323846f
#define FLT_EPSILON 1.19209290e-07F  // 1e-5f 也可

extern pos_est_t _pos_est;//在position_estimator.h中定义的估计的姿态

// 位置环控制参数
typedef struct {
    float pos_p[3];      // [x, y, z] 位置环 P 增益
    float vel_hor_max;   // 最大水平速度 (m/s)
    float vel_up_max;    // 最大上升速度 (m/s)
    float vel_down_max;  // 最大下降速度 (m/s)
} pos_control_params_t;


//////////////////////////////////////////////////////////////////////////////////

/*速度环控制1（ PID 参数 ）*/
typedef struct {
    float vel_p[3];   // 速度环 P 增益
    float vel_i[3];   // 速度环 I 增益
    float vel_d[3];   // 速度环 D 增益
} vel_pid_params_t;

//////////////////////////////////////////////////////////////////////////////////
/* 输入：推力矢量（NED 坐标系，单位：N 或归一化） */
//  float _thrust_sp[3];   // [x, y, z]

/* 输出：期望姿态 */
typedef struct {
    float roll;
    float pitch;
    float yaw;      // rad，NED
    float thrust;   // 0~1（归一化总油门）
} att_sp_t;


void assign_estimated_values(void);//更新当前文件中的位置、速度

void set_position_target(float x_ned, float y_ned, float z_ned);
void calculate_velocity_setpoint_pure(void);
void calculate_thrust_from_velocity_error(float dt);
void thrust_to_attitude(float current_yaw_rad);



#endif /* FLIGHT_CONTROL_POSITION_CONTROL_H_ */
