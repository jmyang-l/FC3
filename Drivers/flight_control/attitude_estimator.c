#include "attitude_estimator.h"
#include <math.h>
#include <stdbool.h> // 包含 bool 类型定义


 quatf _q     = {{ {1, 0, 0, 0} }};   // 当前姿态四元数
 vec3f _gbias = {{ {0, 0, 0} }};      // 陀螺零偏

 /* ---------- 加速度计六面体 ---------- */
 /* ========== 你在这里填 6 面数据 ========== */
 static float acc_raw1[6][3] = {
     /*  +X,      -X,       +Y,       -Y,       +Z,       -Z  */
     { 9.735f,    -0.068f,    -0.221f },  // +X
     {-9.717f,    0.287f,    -0.380f },  // -X
     { 0.066f,    9.796f,    -0.113f },  // +Y
     { -0.291f,   -9.680f,    0.236f },  // -Y
     { 0.066f,    0.197f,    9.673f },  // +Z
     { -0.103f,  -0.092f,   -9.775f }   // -Z
 };

 /* ---------- 加速度计六面体 ---------- */
 /* ========== 你在这里填 6 面数据 ========== */
 static float acc_raw2[6][3] = {
     /*  +X,      -X,       +Y,       -Y,       +Z,       -Z  */
     { 9.745f,    -0.096f,    0.133f },  // +X
     {-9.698f,    0.152f,    -0.129f },  // -X
     { -0.048f,    9.699f,    0.210f },  // +Y
     { -0.231f,   -9.760f,    0.562f },  // -Y
     { 0.011f,    0.122f,    10.005f },  // +Z
     { 0.006f,    -0.031f,   -9.456f }   // -Z
 };

 /* ---------- 在线零偏估计（EKF 简化版） ---------- */
 static vec3f _gyro_bias_P = {{{1e-3f, 1e-3f, 1e-3f}}};   // 协方差初值   想让初始收敛更快	增大 P
 static vec3f _gyro_bias_Q = {{{5e-6f, 5e-6f, 5e-6f}}};   // 过程噪声      想让零偏“一直慢慢跟”	增大 Q（允许零偏随时间变化）
 static vec3f _gyro_bias_R = {{{5e-3f, 5e-3f, 5e-3f}}};   // 观测噪声     想让观测更激进	减小 R（更相信加速度计）
 static uint16_t _still_cnt = 0;                        // 静止计数器

quatf attitude_get_quat(void) {
    return _q;
}

float _R[3][3];  // 当前姿态旋转矩阵
#define GRAVITY 9.80665f
static inline float vec3f_norm(const vec3f *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/* ----------  工具函数 ---------- */
static inline float constrain(float val, float min, float max)
{
    return val < min ? min : (val > max ? max : val);
}

/* ---------------------------------------------------------
   2. 温度零偏刷新接口
--------------------------------------------------------- */
void update_gyro_bias_from_temp(void)
{
    /* 假设 BMI088_Read_Tmp_Data(false) 会把温度存到 BMI088.Temperature */
    BMI088_Read_Tmp_Data(false);
    float current_temp = BMI088.Temperature;          // ℃
    temp_to_gyro_bias(current_temp, _gbias.v);        // 直接刷新 _gbias
}

static inline quatf quat_from_euler(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

    quatf q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

/* ---------- 陀螺仪变化量静止检测 ---------- */
static float gyro_prev[3] = {0};
static float delta2_buf[ZUPT_WINDOW_LEN] = {0};   // 滑动窗口
static uint8_t buf_idx = 0;

bool is_gyro_stationary(const vec3f *gyro)
{
    /* 1. 计算差分能量 */
    float dx = gyro->x - gyro_prev[0];
    float dy = gyro->y - gyro_prev[1];
    float dz = gyro->z - gyro_prev[2];
    float delta2 = dx*dx + dy*dy + dz*dz;

    /* 2. 存入滑动窗口 */
    delta2_buf[buf_idx] = delta2;
    buf_idx = (buf_idx + 1) % ZUPT_WINDOW_LEN;

    /* 3. 更新上一帧 */
    gyro_prev[0] = gyro->x;
    gyro_prev[1] = gyro->y;
    gyro_prev[2] = gyro->z;

    /* 4. 计算窗口方差（简单平均） */
    float sum = 0;
    for (uint8_t i = 0; i < 100; ++i) sum += delta2_buf[i];
    float sigma2 = sum / (float)ZUPT_WINDOW_LEN;

    /* 5. 返回判断结果 */
    return sigma2 < 0.001f;  // 阈值可调
}

/* 把任意向量 v 就地归一化到单位长度，返回归一化后的长度（0 表示失败） */
static inline float vec3f_normalize(vec3f *v)
{
    float len = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
    if (len > 1e-6f) {
        float inv = 1.0f / len;
        v->x *= inv;
        v->y *= inv;
        v->z *= inv;
    } else {
        /* 长度过小，保持原值或给一个默认方向，防止除零 */
        v->x = 0.0f;
        v->y = 0.0f;
        v->z = 1.0f;   // 默认指地
        len = 0.0f;
    }
    return len;
}

extern bool use_imu;
/* 单步姿态更新：IMU → 四元数
 * 此函数用于根据IMU（惯性测量单元）的传感器数据（加速度计、陀螺仪和磁力计）
 * 进行单步的姿态更新，首先将陀螺仪数据积分得到四元数表示的姿态，
 * 然后对四元数进行归一化处理，最后使用加速度计和磁力计的数据进行修正。
 */
void attitude_update(float dt,  // 时间间隔，单位为秒，用于积分计算
                     const vec3f *accel,  // 指向加速度计数据的指针，单位为 m/s²
                     const vec3f *gyro )  // 指向陀螺仪数据的指针，单位为 rad/s

{
	static calib_result_t accel_calib1 = {0};
	static calib_result_t accel_calib2 = {0};
	calib_result_t *accel_calib;
	if(!accel_calib_compute(&accel_calib1 , acc_raw1)&&!accel_calib_compute(&accel_calib2 , acc_raw2))//计算加速度计校准后的零偏和比例因子
	{
		printf("error acc");
		return;
	}

    if(use_imu){
    	accel_calib = &accel_calib2;
    }
    else{
    	accel_calib = &accel_calib1;
    }

    /* 1. 陀螺积分（一阶龙格库塔）    //“陀螺仪 x 轴读数→gcorr.x，y 轴读数→gcorr.y，z 轴读数→gcorr.z”必须与你选用的机体坐标系定义完全一致
     *                               //只要对应关系对得上，龙格库塔公式本身不区分 FRD/FLU
     * 使用一阶龙格 - 库塔方法对陀螺仪数据进行积分，以更新姿态四元数。
     * 首先需要对陀螺仪数据进行零偏校正。
     */
    // 对陀螺仪数据进行零偏校正，得到校正后的陀螺仪数据
	vec3f gcorr = {{{
	    gyro->x - _gbias.x,
	    gyro->y - _gbias.y,
	    gyro->z - _gbias.z
	}}};

    // 保存上一时刻的姿态四元数
    quatf q_prev = _q;
    // 根据四元数运动学方程，使用一阶龙格 - 库塔方法更新四元数的 w 分量
    _q.w += 0.5f * (-gcorr.x*q_prev.x - gcorr.y*q_prev.y - gcorr.z*q_prev.z) * dt;
    // 更新四元数的 x 分量
    _q.x += 0.5f * ( gcorr.x*q_prev.w + gcorr.z*q_prev.y - gcorr.y*q_prev.z) * dt;
    // 更新四元数的 y 分量
    _q.y += 0.5f * ( gcorr.y*q_prev.w - gcorr.z*q_prev.x + gcorr.x*q_prev.z) * dt;
    // 更新四元数的 z 分量
    _q.z += 0.5f * ( gcorr.z*q_prev.w + gcorr.y*q_prev.x - gcorr.x*q_prev.y) * dt;

    /* 2. 归一化
     * 由于四元数表示姿态时需要满足模长为 1 的条件，
     * 但在积分过程中可能会引入误差导致模长不为 1，因此需要进行归一化处理。
     */
    // 计算四元数的模长
    float len = sqrtf(_q.w*_q.w + _q.x*_q.x + _q.y*_q.y + _q.z*_q.z);
    // 如果模长大于 0，则进行归一化处理
    if (len > 0.0f) {
        _q.w /= len;  // 归一化 w 分量
        _q.x /= len;  // 归一化 x 分量
        _q.y /= len;  // 归一化 y 分量
        _q.z /= len;  // 归一化 z 分量
    }

    /* 3. 加速度 + 磁力计修正（Mahony 简版）
     * 使用加速度计和磁力计的数据对姿态进行修正，以提高姿态估计的准确性。
     * 首先估计重力方向和磁力计方向，然后计算加速度误差和磁力计误差，
     * 最后将误差融合并用于修正陀螺仪的零偏和校正后的数据。
     */
    /* 重力方向估计
     * 根据当前的姿态四元数估计重力在机体坐标系下的方向。
     */
    vec3f g_est = {{{
    		2.0f * (_q.x*_q.z - _q.w*_q.y),
    		2.0f * (_q.y*_q.z + _q.w*_q.x),
    		-(_q.w*_q.w - _q.x*_q.x - _q.y*_q.y + _q.z*_q.z)
    }}};
    /* 归一化理论重力向量 */
    vec3f_normalize(&g_est);

    //六面校准后的加速度计数据处理
    /* 1. 先校正：offset + scale */
    vec3f acc_cal;//校准后的加速度计数据

    acc_cal.x = (accel->x - accel_calib->offset[0]) * accel_calib->scale[0];
    acc_cal.y = (accel->y - accel_calib->offset[1]) * accel_calib->scale[1];
    acc_cal.z = (accel->z - accel_calib->offset[2]) * accel_calib->scale[2];
//    printf("%f,%f,%f \r\n", acc_cal.x, acc_cal.y, acc_cal.z);
    /* 2. 再归一化 */
    vec3f acc_norm = acc_cal;
    vec3f_normalize(&acc_norm);

    /* 然后再用 acc_norm × g_est 计算误差 */
    vec3f acc_err = {
        acc_norm.y * g_est.z - acc_norm.z * g_est.y,
        acc_norm.z * g_est.x - acc_norm.x * g_est.z,
        acc_norm.x * g_est.y - acc_norm.y * g_est.x
    };



    /* ---- 4. PX4 式双阈值：是否允许零偏更新 ---- */
    const float acc_sq_norm = acc_cal.x*acc_cal.x + acc_cal.y*acc_cal.y + acc_cal.z*acc_cal.z;
    const float acc_err_norm = fabsf(acc_sq_norm - GRAVITY*GRAVITY) / (GRAVITY*GRAVITY);

    const float gyr_mag   = vec3f_norm(gyro);
    const bool  gyr_static = gyr_mag < GYRO_BIAS_GYR_THRESH;
    const bool  acc_static = acc_err_norm < GYRO_BIAS_ACC_ERR_THRESH;
    const bool  allow_bias_update = gyr_static && acc_static;

    /* ---- 5. 动态 Kp / Ki（保留你现有公式） ---- */
    const float acc_conf   = 1.0f - fminf(acc_err_norm * 4.0f, 1.0f);
    const float gyr_energy = fminf(gyr_mag * 2.0f, 1.0f);
    const float motion_conf = (1.0f - gyr_energy) * acc_conf;

    const float kp_max = 0.50f;
    const float kp_min = 0.1f * kp_max;
    const float ki_max = 0.0002f;
    const float ki_min = 0.000f;

//    /* 4-1 新可信度：err>0.20 直接关死 *///////////////改进一：把“剧烈机动”期间的加速度观测 完全关掉，避免非重力分量污染姿态
//float w_acc = (acc_err_norm < 0.20f)
//            ?(1.0f - powf(acc_err_norm * 5.0f, 3.0f))
//            : 0.0f;
//
///* 4-2 直接乘到 Kp 上 */
//float use_kp = kp_min + (kp_max - kp_min)  * w_acc;

    float use_kp = kp_min + (kp_max - kp_min)  * motion_conf;
    const float use_ki = ki_min + (ki_max - ki_min) * motion_conf;

    /* ---- 6. 融合修正 + 零偏更新（带冻结） ---- */
    for (int i = 0; i < 3; ++i) {
        /* 仅在双阈值满足时更新零偏 */
        if (allow_bias_update) {
//            printf("1\r\n");
            _gbias.v[i] += use_ki * acc_err.v[i] * dt;
            /* 积分限幅 */
            if (_gbias.v[i] >  GYRO_BIAS_LIMIT) _gbias.v[i] =  GYRO_BIAS_LIMIT;
            if (_gbias.v[i] < -GYRO_BIAS_LIMIT) _gbias.v[i] = -GYRO_BIAS_LIMIT;
        }

        /* 校正角速度 */
        gcorr.v[i] += use_kp * acc_err.v[i] + _gbias.v[i];
    }

//    /* ---------- 在线零偏估计 ---------- */
//    bool stationary =
//        (vec3f_norm(gyro) < 0.15f) &&                       // 陀螺静止
//        (fabsf(vec3f_norm(accel) - GRAVITY) < 0.20f);        // 加计可信
//
//    /* 静止计数器：连续 20 帧（400 Hz→50 ms）才更新 */
//    if (stationary) {
//        _still_cnt++;
//    } else {
//        _still_cnt = 0;
//    }
//
//    if (_still_cnt >= 30) {
////    	        printf("\r\n1\r\n");
//
//        /* 构造观测残差：加计重力叉乘 */
//        vec3f g_world = {{0.0f, 0.0f, GRAVITY}};
//        vec3f g_body;
//        /* 用当前四元数把世界重力转到机体 */
//        g_body.x = 2.0f * (_q.x * _q.z - _q.w * _q.y);
//        g_body.y = 2.0f * (_q.y * _q.z + _q.w * _q.x);
//        g_body.z = -(_q.w * _q.w - _q.x * _q.x - _q.y * _q.y + _q.z * _q.z);
//        vec3f_normalize(&g_body);
//
//        vec3f acc_cal;
//        acc_cal.x = (accel->x - accel_calib->offset[0]) * accel_calib->scale[0];
//        acc_cal.y = (accel->y - accel_calib->offset[1]) * accel_calib->scale[1];
//        acc_cal.z = (accel->z - accel_calib->offset[2]) * accel_calib->scale[2];
//        vec3f_normalize(&acc_cal);
//
//        vec3f innov;
//        innov.x = acc_cal.y * g_body.z - acc_cal.z * g_body.y;
//        innov.y = acc_cal.z * g_body.x - acc_cal.x * g_body.z;
//        innov.z = acc_cal.x * g_body.y - acc_cal.y * g_body.x;
//
//        /* 卡尔曼增益 & 更新零偏 *////////////////////////////////////////////////////////////加大p
//        for (int i = 0; i < 3; ++i) {
//            float K = _gyro_bias_P.v[i] / (_gyro_bias_P.v[i] + _gyro_bias_R.v[i]);
//            _gbias.v[i] += K * innov.v[i] * dt;               // 单位：rad/s
//            _gbias.v[i] = constrain(_gbias.v[i], -GYRO_BIAS_LIMIT, GYRO_BIAS_LIMIT);
//            _gyro_bias_P.v[i] = (1.0f - K) * _gyro_bias_P.v[i] + _gyro_bias_Q.v[i];
//
//            float delta = K * innov.v[i] * dt;
////            printf("delta[%d]=%.8f\n", i, delta);
//        }
//        _still_cnt = 0;   // 清零，避免连续触发
//    }

}



/**
 * @brief 从四元数转换得到欧拉角
 *
 * 此函数将当前存储的四元数 _q 转换为欧拉角表示（滚转角、俯仰角和偏航角）。
 * 欧拉角提供了一种直观的方式来描述物体在三维空间中的姿态。
 *
 * @return euler_t 包含滚转角、俯仰角和偏航角的结构体
 */
extern float _vel[3];
extern float dt;

#define PI 3.1415926
euler_t attitude_get_euler(void){
	// 声明一个 euler_t 类型的变量 e，用于存储转换后的欧拉角
    euler_t e;


    float w = _q.w ;
    float x = _q.x ;
    float y = _q.y ;
    float z = _q.z ;



        /* 2. 用 PX4 的 Z-Y-X（yaw-pitch-roll）公式（FRD 坐标系） */
            e.roll  = atan2f( 2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y) );   // 绕 x 轴
            e.pitch = asinf ( 2.0f*(w*y - z*x) );                            // 绕 y 轴
            e.yaw   = atan2f( 2.0f*(w*z + x*y), 1.0f - 2.0f*(x*x + z*z) );   // 绕 z 轴

            /* 3. 万向节锁：PX4 不强制 yaw=0，而是沿用上面的 atan2，保持连续性
               因此直接删掉你原来的 if (test>0.99) / if (test<-0.99) 分支 */


        /* 4. 把角度归一到 (-π, π] */
            if (e.roll  >  PI) e.roll  -= 2.0f*PI;
            if (e.roll  < -PI) e.roll  += 2.0f*PI;
            if (e.pitch >  PI) e.pitch -= 2.0f*PI;
            if (e.pitch < -PI) e.pitch += 2.0f*PI;
            if (e.yaw   >  PI) e.yaw   -= 2.0f*PI;
            if (e.yaw   < -PI) e.yaw   += 2.0f*PI;

    return e;
}





















