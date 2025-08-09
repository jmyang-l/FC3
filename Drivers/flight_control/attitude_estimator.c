#include "attitude_estimator.h"
#include <math.h>
#include <stdbool.h> // 包含 bool 类型定义
#include "BMI088.h"

 quatf _q     = {{ {1, 0, 0, 0} }};   // 当前姿态四元数
 vec3f _gbias = {{ {0, 0, 0} }};      // 陀螺零偏

quatf attitude_get_quat(void) {
    return _q;
}

float _R[3][3];  // 当前姿态旋转矩阵
#define GRAVITY 9.80665f
static inline float vec3f_norm(const vec3f *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}


///* ---------------------------------------------------------
//   1. 温度-零偏模型（全局常量）
//--------------------------------------------------------- */
//static const float T0_1 = 29.38f;
//static const float T0_2 = 40.00f;
//
////static const float gyro_bias_base1[3] = { 0.0001f*1.0f, -0.0001f,  0.0000f };
////static const float gyro_k1[3]         = { -0.00753f*1.5f ,  0.00041f, 0.00044f };
////
////static const float gyro_bias_base2[3] = { -0.0794f*3.5f,  0.0042f*1.1f,  0.0047f };
////static const float gyro_k2[3]         = {  0.0468f*1.7f,   1.1 * 0.0037f, -0.0041f };
//
//static const float gyro_bias_base1[3] = { 0.0001f , -0.0001f,  0.0000f };
//static const float gyro_k1[3]         = { -0.00753f ,  0.00041f, 0.00044f };
//
//static const float gyro_bias_base2[3] = { -0.0794f,  0.0042f,  0.0047f };
//static const float gyro_k2[3]         = {  0.0468f,  0.0037f, -0.0041f };
//
///* 温度转零偏，结果写入 bias_out[3] */
//static void temp_to_gyro_bias(float temp, float *bias_out)
//{
//    if (temp <= T0_2) {
//        for (int i = 0; i < 3; ++i)
//            bias_out[i] = gyro_bias_base1[i] + gyro_k1[i] * (temp - T0_1);
//    } else {
//        for (int i = 0; i < 3; ++i)
//            bias_out[i] = gyro_bias_base2[i] + gyro_k2[i] * (temp - T0_2);
//    }
//}

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
    return sigma2 < 0.5f;  // 阈值可调
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

/* 单步姿态更新：IMU → 四元数
 * 此函数用于根据IMU（惯性测量单元）的传感器数据（加速度计、陀螺仪和磁力计）
 * 进行单步的姿态更新，首先将陀螺仪数据积分得到四元数表示的姿态，
 * 然后对四元数进行归一化处理，最后使用加速度计和磁力计的数据进行修正。
 */
void attitude_update(float dt,  // 时间间隔，单位为秒，用于积分计算
                     const vec3f *accel,  // 指向加速度计数据的指针，单位为 m/s²
                     const vec3f *gyro )  // 指向陀螺仪数据的指针，单位为 rad/s

{
	/* 0. 每拍用温度校正一次零偏 */
//	    update_gyro_bias_from_temp();

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

    /* 归一化加速度计读数 */
    vec3f acc_norm = *accel;
    vec3f_normalize(&acc_norm);

    /* 然后再用 acc_norm × g_est 计算误差 */
    vec3f acc_err = {
        acc_norm.y * g_est.z - acc_norm.z * g_est.y,
        acc_norm.z * g_est.x - acc_norm.x * g_est.z,
        acc_norm.x * g_est.y - acc_norm.y * g_est.x
    };

    /* 1. 计算静态判别因子 */
    float acc_mag = vec3f_norm(accel);      // √(ax²+ay²+az²)  ////////////////和PX4一一对应
    float gyr_mag = vec3f_norm(gyro);       // √(gx²+gy²+gz²)

    /* 2. 动态/静态阈值 */
    bool is_static = (fabsf(acc_mag ) < 0.2f) && is_gyro_stationary(gyro);

    /* 3. 分段 kp/ki */
    float use_kp = is_static ? 2.5f  : 1.5f;   // 静止快，机动稳
    float use_ki = is_static ? 0.002f : 0.00f;   // 静止收敛，机动冻结
//    if(is_static)
//    {
//        printf("静止\r\n");
//    }
//    float use_kp = 0.5f;   // PX4默认值
//    float use_ki = 0.02f;   //

//    /* 动态零偏补偿 */
//        static vec3f bias_lpf = {0};  // 低通滤波器状态  ////////////////和PX4一一对应
//        float alpha = 0.9995f;        // 时间常数 ≈ 2 s，=T/(1-alpha)
//        if (is_static) {
//            for (int i = 0; i < 3; ++i) {
//                bias_lpf.v[i] = alpha * bias_lpf.v[i] + (1.0f - alpha) * gyro->v[i];
//                _gbias.v[i] = bias_lpf.v[i];
//            }
//        }

    /* 4. 融合修正 */
    for (int i = 0; i < 3; ++i) {
    	if(i==0)//x轴，roll
    	{
    		_gbias.v[i] +=  use_ki * acc_err.v[i] * dt;
            gcorr.v[i]  +=  use_kp * acc_err.v[i] + _gbias.v[i];
    	}
    	if(i==2)//z轴，yaw
    	{
            _gbias.v[i] +=  use_ki * acc_err.v[i] * dt;
//            gcorr.v[i]  += 0.8 * use_kp * acc_err.v[i];
            gcorr.v[i]  +=  use_kp * acc_err.v[i] + _gbias.v[i];
    	}
    	if(i==1)//y轴零偏比较严重
    	{
            _gbias.v[i] +=  use_ki * acc_err.v[i] * dt;
//            gcorr.v[i]  += 0.6 * use_kp * acc_err.v[i];
            gcorr.v[i]  +=  use_kp * acc_err.v[i] + _gbias.v[i];
    	}

    	// 🔒 积分限幅
    	    if (_gbias.v[i] > GYRO_BIAS_LIMIT) {
    	        _gbias.v[i] = GYRO_BIAS_LIMIT;
    	    } else if (_gbias.v[i] < -GYRO_BIAS_LIMIT) {
    	        _gbias.v[i] = -GYRO_BIAS_LIMIT;
    	    }

    }

//    printf("gyro_bias = %.4f, %.4f, %.4f\r\n",
//           _gbias.x, _gbias.y, _gbias.z);
//

//    /* ---------- 静止零偏矫正（ZUPT） ---------- */////////////////和PX4一一对应
//    static bool zupt_armed = false;
//    static float zupt_cnt  = 0.0f;
//
//    /* 1. 判断是否真正静止 */
//    bool is_zupt =
//        (fabsf(acc_mag ) < 0.2f) &&is_gyro_stationary(gyro);
//
//    /* 2. 计时器：连续静止 0.5 s 触发一次 */
//    if (is_zupt) {
//        zupt_cnt += dt;
//        if ( zupt_cnt >= 0.1f) {
//            /* 3. 生成“期望姿态”：roll=0, pitch=0, yaw保留 */
//            euler_t e_now = attitude_get_euler();
//            quatf q_zero  = quat_from_euler(0.0f, 0.0f, e_now.yaw);
//            _q = q_zero;
//
//            /* 4. 可选：把陀螺零偏也清零，防止后续漂移 */
//            _gbias = (vec3f){{ {0, 0, 0} }};
//
////            printf("大大大静止大大大静止大大大静止大大大静止大大大静止\r\n");
//            zupt_cnt = 0.0f;
//        }
//        zupt_armed = true;
//    } else {
//        zupt_cnt = 0.0f;
//        zupt_armed = false;
//    }


//    /* 4. 计算姿态旋转矩阵                      //直接用四元数误差来计算期望角速度
//     * 根据更新后的四元数计算全局姿态旋转矩阵。
//     */
//    float qw = _q.w;
//    float qx = _q.x;
//    float qy = _q.y;
//    float qz = _q.z;
//
//    _R[0][0] = 1 - 2*qy*qy - 2*qz*qz;
//    _R[0][1] = 2*qx*qy - 2*qz*qw;
//    _R[0][2] = 2*qx*qz + 2*qy*qw;
//    _R[1][0] = 2*qx*qy + 2*qz*qw;
//    _R[1][1] = 1 - 2*qx*qx - 2*qz*qz;
//    _R[1][2] = 2*qy*qz - 2*qx*qw;
//    _R[2][0] = 2*qx*qz - 2*qy*qw;
//    _R[2][1] = 2*qy*qz + 2*qx*qw;
//    _R[2][2] = 1 - 2*qx*qx - 2*qy*qy;

//    // 添加 printf 语句来输出旋转矩阵 _R
//    printf("Rotation Matrix _R:\n");
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 3; ++j) {
//            printf("%.6f ", _R[i][j]);
//        }
//        printf("\n");
//    }

    // /* 磁力计误差（水平面投影）
    //  * 首先根据当前的姿态四元数估计磁力计在机体坐标系下的方向，
    //  * 然后计算估计的磁力计方向与实际磁力计方向之间的误差。
    //  */
    // vec3f m_est = {
    //     _q.w*_q.w + _q.x*_q.x - _q.y*_q.y - _q.z*_q.z,  // 估计的磁力计方向在 x 轴的分量
    //     2.0f * (_q.x*_q.y + _q.w*_q.z),  // 估计的磁力计方向在 y 轴的分量
    //     2.0f * (_q.x*_q.z - _q.w*_q.y)   // 估计的磁力计方向在 z 轴的分量
    // };
    // // 计算磁力计误差，磁偏角可在此加入，当前磁偏角设为 0
    // float m_err = atan2f(m_est.y, m_est.x) - 0.0f;
    // // 根据磁力计误差计算磁力计修正量
    // vec3f mag_corr = {
    //     -m_err * 2.0f * (_q.x*_q.z - _q.w*_q.y),  // x 轴磁力计修正量
    //     -m_err * 2.0f * (_q.y*_q.z + _q.w*_q.x),  // y 轴磁力计修正量
    //     -m_err * (_q.w*_q.w - _q.x*_q.x - _q.y*_q.y + _q.z*_q.z)  // z 轴磁力计修正量
    // };

    // /* 融合修正量
    //  * 将加速度误差和磁力计误差融合，使用比例积分控制器对陀螺仪的零偏和校正后的数据进行修正。
    //  */
    // const float kp = 0.2f, ki = 0.1f;  // 比例系数和积分系数
    // for (int i = 0; i < 3; ++i) {
    //     // 积分部分：更新陀螺仪零偏
    //     _gbias.v[i] += ki * (acc_err.v[i] + mag_corr.v[i]) * dt;
    //     // 比例部分：更新校正后的陀螺仪数据
    //     gcorr.v[i]  += kp * (acc_err.v[i] + mag_corr.v[i]);
    // }
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
            e.yaw   = -atan2f( 2.0f*(w*z + x*y), 1.0f - 2.0f*(x*x + z*z) );   // 绕 z 轴

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





















