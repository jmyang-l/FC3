#include "position_control.h"
#include <math.h>

  pos_control_params_t _params = {
    {1.0f, 1.0f, 1.0f},  // pos_p[3]，分别为 x, y, z 位置环 P 增益
    2.0f,                // vel_hor_max，最大水平速度 (m/s)
    2.0f,                // vel_up_max，最大上升速度 (m/s)
    1.0f                 // vel_down_max，最大下降速度 (m/s)
};

  vel_pid_params_t pid_params = {
    {0.5f, 0.5f, 0.5f},  // vel_p[3]，分别为 x, y, z 速度环 P 增益
    {0.2f, 0.2f, 0.005f},  // vel_i[3]，分别为 x, y, z 速度环 I 增益
    {0.0000f, 0.0000f, 0.0045f} // vel_d[3]，分别为 x, y, z 速度环 D 增益
};

// 当前位置、期望位置、期望速度
 float _pos[3];        // [x, y, z] 当前位置 (NED)
 float _pos_sp[3];     // [x, y, z] 期望位置 (NED)自己设定

 float _vel_sp[3];     // [x, y, z] 期望速度 (NED)
 float _vel[3];          // 当前速度 (NED)
// 控制模式标志
 uint8_t _run_pos_control; // 是否运行位置控制
 uint8_t _run_alt_control; // 是否运行高度控制

/* 推力输出 */
float _thrust_sp[3] = {0};     // 最终推力矢量（NED）
float _thrust_int[3] = {0};    // 速度环积分项

/* 速度误差微分项（可简单置 0 或实现滤波） */
float _vel_err_d[3] = {0};

    att_sp_t att_sp;       // 最终输出

extern pos_est_t _pos_est;//估计的姿态

// 此函数用于将 _pos_est 中的位置和速度数据赋值给 _pos 和 _vel
void assign_estimated_values(void) {
    //更新数据
//    _pos[0] = 0;
//    _pos[1] = 0;
    _pos[0] = _pos_est.pos.x;
    _pos[1] = _pos_est.pos.y;
    _pos[2] = _pos_est.pos.z;

//    _vel[0] = 0;
//    _vel[1] = 0;
    _vel[0] = _pos_est.vel.x;
    _vel[1] = _pos_est.vel.y;
    _vel[2] = _pos_est.vel.z;
}


/* ----------  工具函数 ---------- */
static inline float constrain(float val, float min, float max)
{
    return val < min ? min : (val > max ? max : val);
}

/* ----------  设定期望位置目标 ---------- */
void set_position_target(float x_ned, float y_ned, float z_ned)
{
    _pos_sp[0] = x_ned;
    _pos_sp[1] = y_ned;
    _pos_sp[2] = z_ned;

    _run_pos_control = 1;   /* 启用位置闭环 */
    _run_alt_control = 1;   /* 启用高度闭环 */
}

/* ----------  计算期望速度 ---------- */
void calculate_velocity_setpoint_pure(void)
{
    /* 1. 位置误差 → 速度期望（P 控制） */
    float vel_sp_pos[3];
    vel_sp_pos[0] = (_pos_sp[0] - _pos[0]) * _params.pos_p[0];
    vel_sp_pos[1] = (_pos_sp[1] - _pos[1]) * _params.pos_p[1];
    vel_sp_pos[2] = (_pos_sp[2] - _pos[2]) * _params.pos_p[2];
//    vel_sp_pos[2] = 1.0 * _params.pos_p[2];

    /* 2. 根据控制模式赋值 */
    if (_run_pos_control) {
        _vel_sp[0] = vel_sp_pos[0];
        _vel_sp[1] = vel_sp_pos[1];
    }
    if (_run_alt_control) {
        _vel_sp[2] = vel_sp_pos[2];
    }

    /* 3. 速度限幅 */
    float vel_norm_xy = sqrtf(_vel_sp[0]*_vel_sp[0] + _vel_sp[1]*_vel_sp[1]);
    if (vel_norm_xy > _params.vel_hor_max) {
        _vel_sp[0] *= _params.vel_hor_max / vel_norm_xy;
        _vel_sp[1] *= _params.vel_hor_max / vel_norm_xy;
    }
    _vel_sp[2] = constrain(_vel_sp[2],
                           -_params.vel_up_max,
                            _params.vel_down_max);
}



/* ----------  计算推力矢量 ----------
 * 功能：由速度误差经 PID 计算得到期望三轴推力矢量（速度环）
 * 输入：
 *   dt        : 控制周期，单位秒 (s)
 * 输出（写入全局变量）：
 *   _thrust_sp[3]   : 期望推力矢量，NED 坐标系 (N)
 *   _thrust_int[3]  : 速度环积分项（在函数内维护并限幅）
 *   _vel_err_d[3]   : 速度误差微分项（若未用 D，可置 0）
 */
void calculate_thrust_from_velocity_error(float dt)
{
    /* 1. 计算三轴速度误差：期望 - 实际 */
    // 定义一个临时数组 vel_err 用于存储三个方向（x、y、z）的速度误差
    float vel_err[3];
    // 通过循环遍历三个坐标轴，计算每个轴上的期望速度与实际速度的差值，得到速度误差
    for (int i = 0; i < 3; ++i) {
        // _vel_sp[i] 为第 i 个坐标轴的期望速度，_vel[i] 为第 i 个坐标轴的实际速度
        vel_err[i] = _vel_sp[i] - _vel[i];
    }

    /* 2. 更新积分项（速度环 I 部分） */
    // 积分项用于消除系统的稳态误差，通过对速度误差在时间上进行累积得到
    for (int i = 0; i < 3; ++i) {
        // _thrust_int[i] 为第 i 个坐标轴上的积分项，在原有值的基础上加上当前误差乘以积分增益再乘以控制周期
        // _params.vel_i[i] 为第 i 个坐标轴的积分增益
        _thrust_int[i] += vel_err[i] * pid_params.vel_i[i] * dt;

        /* ---- 积分限幅：防止饱和（±1 g 对应质量归一化推力） ---- */
        // 定义积分项的限幅范围，1g 对应的重力加速度约为 9.81 N/kg
        const float lim = 9.81f * 1.5f;        //加大积分限幅
        // 如果积分项的值超过上限，则将其限制为上限值
        if (_thrust_int[i] >  lim) _thrust_int[i] =  lim;
        // 如果积分项的值低于下限，则将其限制为下限值
        if (_thrust_int[i] < -lim) _thrust_int[i] = -lim;
    }

    /* 3. PID 合成推力矢量：P + I + D */
    // 将比例项、积分项和微分项相加，得到最终的期望推力矢量
    for (int i = 0; i < 3; ++i) {
        // 比例项：速度误差乘以比例增益
        // 积分项：之前更新并限幅后的积分项
        // 微分项：速度误差微分项乘以微分增益
        // _params.vel_p[i] 为第 i 个坐标轴的比例增益，_params.vel_d[i] 为第 i 个坐标轴的微分增益
        _thrust_sp[i] = vel_err[i] * pid_params.vel_p[i] +
                        _thrust_int[i] +
                        _vel_err_d[i] * pid_params.vel_d[i];
    }
    _thrust_sp[2] += 9.8;
    /* ---------- 4. 分轴限幅 ---------- */
    _thrust_sp[0] = constrain(_thrust_sp[0], -2.0f,  2.0f);   // x 方向 ±2 N/kg
    _thrust_sp[1] = constrain(_thrust_sp[1], -2.0f,  2.0f);   // y 方向 ±2 N/kg
    _thrust_sp[2] = constrain(_thrust_sp[2], 9.0f,  10.0f);   // z 方向 ±2 N/kg
}



/* 工具：把角度约束在 [-π, π] */
// 该函数用于将输入的角度限制在 [-π, π] 的范围内，避免角度值超出该范围
// 输入参数 ang 为需要进行约束的角度值，单位为弧度
// 返回值为约束在 [-π, π] 范围内的角度值
static inline float wrap_pi(float ang)
{
    // 当角度大于 π 时，不断减去 2π，直到角度落在 [-π, π] 范围内
    while (ang >  M_PI) ang -= 2.0f * M_PI;
    // 当角度小于 -π 时，不断加上 2π，直到角度落在 [-π, π] 范围内
    while (ang < -M_PI) ang += 2.0f * M_PI;
    return ang;
}

    // 最终期望旋转矩阵 R = Rz * Ry * Rx
    float R_sp[3][3];
/* ----------  计算期望姿态角和期望总油门 ---------- */
/* 推力矢量 → 期望姿态角 + 期望总油门
 * 输入：_thrust_sp[3]（NED）
 * 输出：att_sp.{roll,pitch,yaw,thrust}
 * 注意：yaw 保持当前值，如需外部给定可再传参
 */
// 该函数根据输入的推力矢量计算期望的姿态角（roll, pitch, yaw）和期望总油门
// 输入参数 current_yaw_rad 为当前的偏航角，单位为弧度
void thrust_to_attitude(float current_yaw_rad)
{
    /* 1. 计算推力模长（总油门） */
    // 推力模长代表了总油门的大小，通过对推力矢量的各个分量的平方和开平方得到
    float thrust_norm = sqrtf(_thrust_sp[0]*_thrust_sp[0] +
                              _thrust_sp[1]*_thrust_sp[1] +
                              _thrust_sp[2]*_thrust_sp[2]);

    /* 2. 归一化推力方向（防止除零） */
    // 初始化机体 Z 轴方向向量，默认朝上
    float body_z[3] = {0, 0, 1};
    // 当推力模长大于一个极小值（1e-4f）时，进行归一化操作
    if (thrust_norm > 1e-4f) {
        // 计算归一化后的推力方向向量，取负号是因为机体 Z 轴方向与推力方向相反
        body_z[0] = -_thrust_sp[0] / thrust_norm;
        body_z[1] = -_thrust_sp[1] / thrust_norm;
        body_z[2] = _thrust_sp[2] / thrust_norm;
    }

    /* 3. 构造机体 X 轴（前向） */
    // 根据当前偏航角构造一个辅助向量 y_C
    float y_C[3] = { -sinf(current_yaw_rad),
                      cosf(current_yaw_rad),
                      0.0f };

    // 定义机体 X 轴方向向量
    float body_x[3];
    // 通过向量叉乘计算机体 X 轴方向向量，body_x = y_C × body_z
    body_x[0] = y_C[1]*body_z[2] - y_C[2]*body_z[1];
    body_x[1] = y_C[2]*body_z[0] - y_C[0]*body_z[2];
    body_x[2] = y_C[0]*body_z[1] - y_C[1]*body_z[0];

    // 计算机体 X 轴方向向量的模长
    float len = sqrtf(body_x[0]*body_x[0] + body_x[1]*body_x[1] + body_x[2]*body_x[2]);
    // 若模长小于一个极小值（1e-4f），说明推力竖直向上或向下，此时将机体 X 轴设为默认前向
    if (len < 1e-4f) {
        body_x[0] = 1.0f; body_x[1] = 0.0f; body_x[2] = 0.0f;
    } else {
        // 若模长正常，则对机体 X 轴方向向量进行归一化
        body_x[0] /= len; body_x[1] /= len; body_x[2] /= len;
    }



    /* 4. 先限制 pitch，避免 ±90° 附近万向节锁 */
    float pitch_raw = atan2f(-body_z[0],
                             sqrtf(body_z[1]*body_z[1] + body_z[2]*body_z[2]));
    const float pitch_limit = M_PI_2 - 0.15f;  // 约 ±75°
    att_sp.pitch = constrain(pitch_raw, -pitch_limit, pitch_limit);



    /* 3.5. 计算期望 roll, pitch */
    // 根据归一化后的机体 Z 轴方向向量计算期望的横滚角（roll）
    att_sp.roll  = atan2f(-body_z[1], body_z[2]);      // roll  =  atan2(z_y, z_z)
    // 根据归一化后的机体 Z 轴方向向量计算期望的俯仰角（pitch）
    att_sp.pitch = atan2f(-body_z[0],                // pitch = -atan2(z_x, sqrt(z_y²+z_z²))
                          sqrtf(body_z[1]*body_z[1] + body_z[2]*body_z[2]));

//    /* 绕 FLU 的 X 轴（右）旋转 → roll */
//    att_sp.roll  = atan2f( body_z[1], body_z[2]);   // atan2(z_y, z_z)
//
//    /* 绕 FLU 的 Y 轴（前）旋转 → pitch */
//    att_sp.pitch = atan2f(-body_z[0], body_z[2]);   // atan2(-z_x, z_z)
//
//    /* 5. yaw 保持不变或外部给定 */
    // 将当前偏航角约束在 [-π, π] 范围内，并赋值给期望偏航角
    att_sp.yaw   = wrap_pi(current_yaw_rad);

    /* 6. 总油门 = 推力模长（归一化到 0~1） */
    // 将推力模长赋值给期望总油门，如需将其归一化到 0~1 范围，可根据最大推力再进行除法操作
    att_sp.thrust = thrust_norm  ;   // 如需 0~1，可根据最大推力再除

    /* 7. 根据欧拉角计算期望姿态旋转矩阵 */
    float cos_roll = cosf(att_sp.roll);
    float sin_roll = sinf(att_sp.roll);
    float cos_pitch = cosf(att_sp.pitch);
    float sin_pitch = sinf(att_sp.pitch);
    float cos_yaw = cosf(att_sp.yaw);
    float sin_yaw = sinf(att_sp.yaw);

    // 绕Z轴旋转的矩阵
    float Rz[3][3] = {
        {cos_yaw, -sin_yaw, 0},
        {sin_yaw, cos_yaw, 0},
        {0, 0, 1}
    };

    // 绕Y轴旋转的矩阵
    float Ry[3][3] = {
        {cos_pitch, 0, sin_pitch},
        {0, 1, 0},
        {-sin_pitch, 0, cos_pitch}
    };

    // 绕X轴旋转的矩阵
    float Rx[3][3] = {
        {1, 0, 0},
        {0, cos_roll, -sin_roll},
        {0, sin_roll, cos_roll}
    };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_sp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R_sp[i][j] += Rz[i][k] * Ry[k][j];
            }
        }
    }

    float temp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = R_sp[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_sp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R_sp[i][j] += temp[i][k] * Rx[k][j];
            }
        }
    }

//    // 添加 printf 语句来输出旋转矩阵 R_sp
//    printf("Rotation Matrix R_sp:\n");
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 3; ++j) {
//            printf("%.6f ", R_sp[i][j]);
//        }
//        printf("\n");
//    }

}


