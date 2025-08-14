#include "process.h"
#include <stdio.h>  // 为了使用 printf 进行调试，实际使用时可根据需要调整
#include <stdbool.h> // 包含 bool 类型定义

// float m_y;
// float m_x;
	extern vec3f acc_bias  ;
	extern vec3f gyro_bias ;

extern quatf _qd ;//期望四元数

extern float R_sp[3][3];
extern att_sp_t att_sp;

float x_ned = 0.0f;
float y_ned = 0.0f;

float z_ned = 0.215f;

extern float _vel_sp[3];//期望速度
extern float _thrust_sp[3];//推力矢量
extern vec3f _rates_sp;//期望角速度
extern vec3f torque;//期望扭矩
// 假设的控制周期
const float dt = 0.001f;

// 假设的当前偏航角
float current_yaw_rad = 0.0f;

extern vec3f torque;//力矩向量，由 rate_error_to_torque 函数计算得到
motor_out_t motor_out; // 电机输出值，由 torque_to_motor 函数计算得到

//const float DEG2RAD = 1.0f;


void process_main(void)
{

	if(!imu_gyrobias())//测量水平静止时陀螺仪的零偏
	{
//		printf("1\r\n");
		return;
	}


  // 0. 更新数据到position_estimator模块
   get_accel(BMI088.acc.m_s_2[xx] ,
	          BMI088.acc.m_s_2[yy] ,
	          BMI088.acc.m_s_2[zz] );
   get_gyro((BMI088.gyro.dps[xx] - gyro_bias.x) ,
           (BMI088.gyro.dps[yy] - gyro_bias.y) ,
           (BMI088.gyro.dps[zz] - gyro_bias.z) );



   vec3f accel = {{{BMI088.acc.m_s_2[xx] ,
	                BMI088.acc.m_s_2[yy] ,
	                BMI088.acc.m_s_2[zz] }}};
   vec3f gyro = {{{
		   (BMI088.gyro.dps[xx] - gyro_bias.x)   ,
			 (BMI088.gyro.dps[yy] - gyro_bias.y) ,
			 (BMI088.gyro.dps[zz] - gyro_bias.z)
   }}};

//   printf("%f,%f,%f \r\n", gyro.x, gyro.y, gyro.z);

//   printf("acc=%.3f,%.3f,%.3f\n",
//          accel.x, accel.y, accel.z);
//   printf("RAW: gyro=%.3f,%.3f,%.3f\n",
//          gyro.x, gyro.y, gyro.z);

    // 1. 姿态估计
    attitude_update(dt, &accel, &gyro);       // 旧
//   attitude_update_ekf(dt, &accel, &gyro);     // 新

    quatf current_quat = attitude_get_quat();//获取四元数
    euler_t current_euler = attitude_get_euler();//获取欧拉角
//
//    printf("  %f,  %f,  %f,  %f \r\n", current_quat.w, current_quat.x, current_quat.y, current_quat.z);
//    printf("euler: roll = %f, pitch = %f, yaw = %f\r\n", current_euler.roll, current_euler.pitch, current_euler.yaw);
    printf("%2f,%2f,%2f\r\n", current_euler.roll, current_euler.pitch, current_euler.yaw);

//    // 2. 位置估计
//    accel.z = -accel.z ;//再更新z方向
//
//    pos_est_predict(&current_quat, dt);
//
////    get_baro_alt(BARO_Data_Now.Actual_altitude);
////    pos_est_fuse_baro(dt);//气压计高度融合
//
//    //更新光流和高度数据
//    get_sonar_alt((float)payload.distance*0.001);
////    get_flow_vel(-((float)((int64_t)payload.flow_vel_x * payload.distance)*0.001),
////    		      (float)((int64_t)payload.flow_vel_y * payload.distance)*0.001,
////				  0.0f, &current_quat);
////    get_flow_vel(m_x,
////    		      m_y,
////				  0.0f, &current_quat);
//
//
//    pos_est_fuse_sonar(dt);//激光离地高度融合
//    pos_est_fuse_flow(dt);//光流融合
//
////    得到pos_est_t _pos_est;//估计的姿态（速度和位置）
////    printf("%f,%f,%f\r\n", _pos_est.pos.v[0], _pos_est.pos.v[1], _pos_est.pos.v[2]);
////    printf("vel_sp:  Vx = %f, Vy = %f, Vz = %f\r\n", _pos_est.vel.v[0], _pos_est.vel.v[1], _pos_est.vel.v[2]);
////    printf("%2f,%2f\n",_pos_est.pos.v[2],_pos_est.vel.v[2]);
////      printf("%2f,%2f\n",_pos_est.pos.v[0],_pos_est.vel.v[0]);
////      printf("%2f,%2f\n",_pos_est.pos.v[1],_pos_est.vel.v[1]);
//
//    // 3. 位置控制         //只要前面速度和位置估计的对这边就基本没问题
//    assign_estimated_values(); //更新位置和速度数据
//    set_position_target( x_ned,  y_ned,  z_ned);//设定目标位置
//    calculate_velocity_setpoint_pure();//计算期望速度
//    calculate_thrust_from_velocity_error(dt);//计算推力矢量
//    thrust_to_attitude(current_euler.yaw);//计算期望姿态角和期望总油门
//////    printf("V_SP: x = %f, y = %f, z = %f \r\n", _vel_sp[0], _vel_sp[1], _vel_sp[2]);
//////    printf("N_SP: x = %f, y = %f, z = %f \r\n", _thrust_sp[0], _thrust_sp[1], _thrust_sp[2]);
//////    printf("Desired attitude: roll = %f, pitch = %f, yaw = %f, thrust = %f\r\n", att_sp.roll, att_sp.pitch, att_sp.yaw, att_sp.thrust);
////
////    printf("%f,%f,%f\n\r\n", att_sp.roll, att_sp.pitch, att_sp.yaw);
//

    //调试姿态环 手动设定姿态（关闭位置环）/////////////////////////////////////////////////
        att_sp.roll = 0.0f;
        att_sp.pitch = 0.0f;
        att_sp.yaw = current_euler.yaw;
        att_sp.thrust = 9.80665f;

        float cr = cosf(att_sp.roll  * 0.5f);    //直接计算期望四元数
        float sr = sinf(att_sp.roll  * 0.5f);
        float cp = cosf(att_sp.pitch * 0.5f);
        float sp = sinf(att_sp.pitch * 0.5f);
        float cy = cosf(att_sp.yaw   * 0.5f);
        float sy = sinf(att_sp.yaw   * 0.5f);

        _qd.w = cr*cp*cy + sr*sp*sy;
        _qd.x = sr*cp*cy - cr*sp*sy;
        _qd.y = cr*sp*cy + sr*cp*sy;
        _qd.z = cr*cp*sy - sr*sp*cy;




    // 4. 姿态控制
     attitude_error_to_rates();//计算期望角速度
     rate_error_to_torque(dt);//计算期望扭矩
//     printf(" %f, %f, %f\r\n", _rates_sp.v[0], _rates_sp.v[1], _rates_sp.v[2] );
//     printf("%f,%f,%f\n", _rates_sp.v[0], _rates_sp.v[1], _rates_sp.v[2]);
//     printf("torque: %f, %f, %f\r\n", torque.v[0], torque.v[1], torque.v[2] );
     torque_to_motor(&torque, att_sp.thrust, &motor_out);//计算电机输出
//

//     printf(" %f, %f, %f, %f\r\n", motor_out.m[0], motor_out.m[1], motor_out.m[2], motor_out.m[3]);

}

//void process_main(void)
//{
//    /* 1. 取传感器数据 */
//    vec3f accel = {{{BMI088.acc.m_s_2[xx], BMI088.acc.m_s_2[yy], BMI088.acc.m_s_2[zz]}}};
//    vec3f  gyro = {{{
//        BMI088.gyro.dps[xx] * DEG2RAD,
//        BMI088.gyro.dps[yy] * DEG2RAD,
//        BMI088.gyro.dps[zz] * DEG2RAD
//    }}};
//
//    /* 2. 姿态估计 */
//    attitude_update(dt, &accel, &gyro);
//    euler_t current_euler = attitude_get_euler();
//
//    /* 3. 固定目标姿态（仅自稳） */
//    att_sp.roll  = 0.0f;
//    att_sp.pitch = 0.0f;
//    att_sp.yaw   = current_euler.yaw;  // 保持当前航向
//    att_sp.thrust = 9.8f;             // 试飞时慢慢推
//
//    /* 4. 姿态控制 → 电机输出 */
//    update_rotation_matrix();
//    update_sprotation_matrix();
//    attitude_error_to_rates();
//    rate_error_to_torque(dt);
//    torque_to_motor(&torque, att_sp.thrust, &motor_out);
//}

