#include "attitude_control.h"
#include <stdbool.h> // 包含 bool 类型定义
#include <math.h>

// 声明姿态环比例（P）增益数组，对应 roll、pitch、yaw 轴
//float _att_p[3] = {5.0f, 5.0f, 4.0f};   // 姿态环 P 增益
extern float dt ;


extern vec3f _gbias;
extern vec3f _gyro;      // IMU 角速度（rad/s）
// 外部声明飞行器当前角速度，由传感器测量
vec3f _rates;
void update_W_rate(void)//更新当前角速度
{
	_rates.v[0] = _gyro.v[0] - _gbias.v[0];
	_rates.v[1] = _gyro.v[1] - _gbias.v[1];
	_rates.v[2] = _gyro.v[2] - _gbias.v[2];

}

/* 可外部调节的 P 增益和速率限幅 */
static vec3f _att_p = {{ 6.0f, 6.0f, 2.5f }};   // roll, pitch, yaw
static vec3f _rate_limit        = {{ 3.8f, 3.8f, 3.0f }};   // rad/s

/* 输入：当前四元数 _q、期望四元数 _qd
 * 输出：期望角速度 _rate_sp（机体系，rad/s）
 */
extern quatf _q;   // 当前姿态
quatf _qd;  // 期望姿态
vec3f _rates_sp;

/* ---------------- 新版 —— 不使用旋转矩阵误差，使用四元数误差 ---------------- */
void attitude_error_to_rates(void)
{
	    /* 1. 误差四元数 qe = q⁻¹ · qd */
	    quatf qe;
	    qe.w =  _q.w*_qd.w + _q.x*_qd.x + _q.y*_qd.y + _q.z*_qd.z;
	    qe.x = -_q.x*_qd.w + _q.w*_qd.x + _q.y*_qd.z - _q.z*_qd.y;
	    qe.y = -_q.y*_qd.w - _q.x*_qd.z + _q.w*_qd.y + _q.z*_qd.x;
	    qe.z = -_q.z*_qd.w + _q.x*_qd.y - _q.y*_qd.x + _q.w*_qd.z;

//	    float qe_norm = sqrtf(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
//	    vec3f eq = {{{0}}};
//	    if (qe_norm > 1e-6f) {
//	            float theta = 2.0f * atan2f(qe_norm, fabsf(qe.w));
//	            eq.x = theta * qe.x / qe_norm;
//	            eq.y = theta * qe.y / qe_norm;
//	            eq.z = theta * qe.z / qe_norm;
//	        }
	    /* 2. 轴角误差向量 eq = 2·sign(qe.w)·qe.imag()   （小角度近似） */
	    	    float sign = (qe.w >= 0.0f) ? 1.0f : -1.0f;
	    	    vec3f eq = {{{ 2.0f * sign * qe.x,
	    	                  2.0f * sign * qe.y,
	    	                  2.0f * sign * qe.z }}};

	    /* 3. P 增益放大 → 期望角速度 */
	    _rates_sp.x = _att_p.x * eq.x;
	    _rates_sp.y = _att_p.y * eq.y;
	    _rates_sp.z = _att_p.z * eq.z;

	    /* 4. 限幅 */
	    for (int i = 0; i < 3; ++i) {
	        float *v = (float *)&_rates_sp;
	        if      (v[i] >  _rate_limit.v[i]) v[i] =  _rate_limit.v[i];
	        else if (v[i] < -_rate_limit.v[i]) v[i] = -_rate_limit.v[i];
	    }

}



/* ----------  工具函数 ---------- */
static inline float constrain(float val, float min, float max)
{
    return val < min ? min : (val > max ? max : val);
}



float _rate_p[3] = {
		0.15f,0.15f,0.25f
};// 角速度环比例（P）增益数组，对应 roll、pitch、yaw 轴
float _rate_i[3] = {
		0.05f,0.05f,0.20f
};;// 角速度环积分（I）增益数组，对应 roll、pitch、yaw 轴
float _rate_d[3] = {
		0.005f,0.005f,0.0f
};;// 角速度环微分（D）增益数组，对应 roll、pitch、yaw 轴

float max_int = 0.5f; //积分限幅

/* ----------  D项滤波器状态 ---------- */
static vec3f _d_term_lpf = {{0.0f, 0.0f, 0.0f}};  // 初始化为零

// 定义并初始化角速度环前馈（feed-forward）增益数组，对应 roll、pitch、yaw 轴
float _rate_ff[3] = {0.0f, 0.0f, 0.0f}; // 具体值可根据实际情况调整

// 角速度误差积分状态数组，对应 roll、pitch、yaw 轴
float _rate_int[3] = {0.0f, 0.0f, 0.0f}; // 积分状态,初始值为 0，调用函数会直接更新
// 上一周期飞行器角速度数组，用于计算变化率
float _rate_prev[3] = {0.0f, 0.0f, 0.0f};// 上周期角速度

vec3f torque; // 力矩向量，由 rate_error_to_torque 函数计算得到

// 该函数用于根据期望角速度和当前角速度的误差，计算出需要施加的力矩
// 参数 dt 表示时间间隔，用于积分和微分计算
void rate_error_to_torque(float dt)
{
    // 计算期望角速度 _rates_sp 和当前角速度 _rates 之间的误差
    // 误差向量 err 的每个分量分别对应 roll、pitch 和 yaw 轴的误差
    vec3f err = {{{
        _rates_sp.v[0] - _rates.v[0],  // roll 轴的误差
        _rates_sp.v[1] - _rates.v[1],  // pitch 轴的误差
        _rates_sp.v[2] - _rates.v[2]   // yaw 轴的误差
    }}};

    /* 积分累加（需外部清零） */
    // 对误差进行积分操作，积分项用于消除稳态误差
    // 积分结果存储在 _rate_int 数组中，每个元素对应一个轴
    // 积分公式为：积分项 = 积分增益 * 误差 * 时间间隔
    for (int i = 0; i < 3; ++i)
        {_rate_int[i] += _rate_i[i] * err.v[i] * dt;
        _rate_int[i] = constrain(_rate_int[i], -max_int, max_int);
        }

    /* 微分项 */
    // 计算当前角速度相对于上一周期角速度的变化率，即微分项
    // 微分项用于预测系统的变化趋势，提高系统的响应速度
    // 变化率向量 deriv 的每个分量分别对应 roll、pitch 和 yaw 轴的变化率
    vec3f deriv = {
        (_rates.v[0] - _rate_prev[0]) / dt,
        (_rates.v[1] - _rate_prev[1]) / dt,
        (_rates.v[2] - _rate_prev[2]) / dt
    };

                                /* 给微分项（D 项）再套一个 80 Hz 一阶低通，防止高频噪声被放大成抖振 */
    const float dterm_lpf_tau = 1.0f / (2.0f * M_PI * 80.0f);   // τ = 1/(2πf)
    const float alpha_d = dt / (dterm_lpf_tau + dt);
    for (int i = 0; i < 3; ++i) {
        _d_term_lpf.v[i] += alpha_d * (deriv.v[i] - _d_term_lpf.v[i]);
    }

    // 更新上一周期的角速度，为下一次微分计算做准备
    for (int i = 0; i < 3; ++i)
        _rate_prev[i] = _rates.v[i];

    /* 合成力矩（标量形式，后续乘以惯量矩阵即可） */
    // 根据 PID 控制算法，将比例项、积分项、微分项和前馈项合成所需的力矩
    // 比例项：误差乘以比例增益
    // 积分项：前面计算得到的积分结果
    // 微分项：变化率乘以微分增益
    // 前馈项：期望角速度乘以馈增益
    // 最终的力矩向量 torque 的每个分量分别对应 roll、pitch 和 yaw 轴的力矩

    /* ---------- 合成力矩（使用滤波后的 D 项） ---------- */
    torque.v[0] = _rate_p[0] * err.v[0] + _rate_int[0] + _rate_d[0] * _d_term_lpf.v[0] + _rate_ff[0] * _rates_sp.v[0];

    torque.v[1] = _rate_p[1] * err.v[1] + _rate_int[1] + _rate_d[1] * _d_term_lpf.v[1] + _rate_ff[1] * _rates_sp.v[1];

    torque.v[2] = _rate_p[2] * err.v[2] + _rate_int[2] + _rate_d[2] * _d_term_lpf.v[2] + _rate_ff[2] * _rates_sp.v[2];

    /* 保存结果供混控使用 */
    /* torque.v[0/1/2] 即为 roll/pitch/yaw 轴的力矩指令（Nm） */
}



// 该函数的作用是将输入的力矩和推力转换为四个电机的输出值
// 参数 torque 是一个指向 vec3f 结构体的指针，包含 roll、pitch 和 yaw 轴的力矩
// 参数 thrust 是一个浮点数，表示总的推力，extern att_sp_t att_sp.thrust;
// 参数 out 是一个指向 motor_out_t 结构体的指针，用于存储计算得到的四个电机的输出值
void torque_to_motor(const vec3f *torque, float thrust, motor_out_t *out)
{

	const float mix[4][3] = {
	    {  1.0f, -1.0f, -1.0f },   // 左上 3（顺时针）
	    {  1.0f,  1.0f,  1.0f },   // 右上 4（逆时针）
	    { -1.0f, -1.0f,  1.0f },   // 右下 1（逆时针）
	    { -1.0f,  1.0f, -1.0f }    // 左下 2（顺时针）
	};

    /* 计算四个电机输出（顺序与 mix 行号一致） */
    for (int i = 0; i < 4; ++i)
    {
        out->m[i] = thrust
                  + mix[i][0] * torque->v[0]  // roll
                  + mix[i][1] * torque->v[1]  // pitch
                  + mix[i][2] * torque->v[2]; // yaw

        /* 如需限幅，在此完成 */
        out->m[i] = constrain(out->m[i], 8.6f, 11.0f);  // 例子：0-100% 油门
    }
}

