/*
 * imu_selector.c
 *
 * 实时评估两个 IMU（BMI088 与 BMI088_2）的数据质量，
 * 给出 0.0~1.0 的“健康分”，并决定选用哪一个 IMU。
 * 算法对标 PX4 的 IMU 打分机制，但做了最简实现。
 *
 * 使用流程：
 * 1. 业务线程循环调用 imu_selector_get_active()。
 * 2. 根据返回的 use_imu1 标志去读取对应 IMU 数据。
 * 3. 若返回 false，说明两个 IMU 均不可信，应触发故障保护或停飞。
 */

#include "imu_selector.h"
#include "BMI088.h"
#include <math.h>

/*---------------- 内部数据结构 ----------------*/
typedef struct {
    float acc_err_norm;   // 加速度模值与标准重力的相对误差
    float gyr_energy;     // 陀螺仪三轴能量（模值）
    float temp_drift;     // 预留：温度漂移（暂未使用）
    float score;          // 综合得分 0.0~1.0，越大越可靠
} imu_health_t;

/* 两个 IMU 的实时健康状态 */
static imu_health_t imu1_health = {0};
static imu_health_t imu2_health = {0};

/*---------------- 内部函数：给单个 IMU 打分 ----------------*/
/*
 * 根据加速度计与陀螺仪数据计算健康分数
 *  1. 加速度误差：与标准重力 9.80665 m/s² 的偏差
 *  2. 陀螺能量：机体角速度模值，静止时越小越可信
 *  3. 温度漂移：预留位，可扩展
 *
 *  @param accel 加速度计数据（单位 m/s²）
 *  @param gyro  陀螺仪数据（单位 °/s）
 *  @param temp  当前温度（单位 ℃，暂未参与计算）
 *  @return      健康分 0.0~1.0
 */
static float imu_compute_score(const vec3f* accel, const vec3f* gyro, float temp)
{
    /* 加速度模值误差 */
    float acc_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float acc_err  = fabsf(acc_norm - 9.80665f) / 9.80665f;

    /* 陀螺能量 */
    float gyr_mag = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);

    /* 加权得分：误差越小、能量越低 → 分越高 */
    float w_acc = 1.0f - fminf(acc_err * 5.0f, 1.0f);   // 放大 5 倍后饱和
    float w_gyr = 1.0f - fminf(gyr_mag * 2.0f, 1.0f);   // 放大 2 倍后饱和

    return w_acc * w_gyr;   // 综合得分
}


static float score1_lpf = 1.0f;   // 低通滤波后的分数
static float score2_lpf = 1.0f;
/*---------------- 内部函数：更新两个 IMU 的得分 ----------------*/
/*
 * 从全局结构体 BMI088 / BMI088_2 中读取最新数据，
 * 计算并保存各自的 health.score。
 */
static void imu_selector_update(void)
{
    /* IMU1 */
    IMU_Read(false);
    vec3f accel1 = {{ BMI088.acc.m_s_2[0], BMI088.acc.m_s_2[1], BMI088.acc.m_s_2[2] }};
    vec3f gyro1  = {{ BMI088.gyro.dps[0],  BMI088.gyro.dps[1],  BMI088.gyro.dps[2] }};
    float s1 = imu_compute_score(&accel1, &gyro1, BMI088.Temperature);
    score1_lpf = score1_lpf * 0.9f + s1 * 0.1f;   // 一阶低通
    imu1_health.score = score1_lpf;               // 用滤波值

    /* IMU2 */
    IMU_Read(true);
    vec3f accel2 = {{ BMI088_2.acc.m_s_2[0], BMI088_2.acc.m_s_2[1], BMI088_2.acc.m_s_2[2] }};
    vec3f gyro2  = {{ BMI088_2.gyro.dps[0],  BMI088_2.gyro.dps[1],  BMI088_2.gyro.dps[2] }};
    float s2 = imu_compute_score(&accel2, &gyro2, BMI088_2.Temperature);
    score2_lpf = score2_lpf * 0.9f + s2 * 0.1f;
    imu2_health.score = score2_lpf;
}

/*---------------- 对外接口：获取当前应使用的 IMU ----------------*/
/*
 *  调用者需传入一个 bool 指针，函数把“是否选用 IMU1”写回该地址。
 *
 *  @param[out] use_imu1  true  → 建议用 IMU1
 *                        false → 建议用 IMU2
 *  @return       bool    true  → 至少有一个 IMU 可信
 *                        false → 两个 IMU 均不可信（需触发保护）
 */
bool imu_selector_get_active(bool* use_imu)
{
    imu_selector_update();   // 先算最新分数

    /* 阈值 0.7，可在线调整 */
    const float SCORE_THRESH = 0.3f;

//printf("%f \n", imu1_health.score);
//printf("%f \n", imu2_health.score);

    if (imu1_health.score > SCORE_THRESH &&
        imu1_health.score > imu2_health.score) {
        *use_imu = true;
        return true;
    }

    if (imu2_health.score > SCORE_THRESH) {
        *use_imu = false;
        return true;
    }

    /* 两个 IMU 都不可靠 */
    return false;
}
