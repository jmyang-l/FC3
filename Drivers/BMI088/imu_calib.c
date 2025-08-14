#include "imu_calib.h"
#include <string.h>
#include <math.h>

  vec3f acc_bias  = {0};
  vec3f gyro_bias = {0};

  /* ---------- 加速度计六面体 ---------- */
  /* ========== 你在这里填 6 面数据 ========== */
  static float acc_raw[6][3] = {
      /*  +X,      -X,       +Y,       -Y,       +Z,       -Z  */
      { 9.741f,    -0.085f,    -0.368f },  // +X
      {-9.950f,    0.055f,    -0.132f },  // -X
      { -0.006f,    9.766f,    0.482f },  // +Y
      { -0.326f,   -9.881f,    0.597f },  // -Y
      { -0.062f,    0.009f,    9.713f },  // +Z
      { -0.115f,    -0.088f,   -9.949f }   // -Z
  };

	static bool calib_done = false;
	static int  calib_cnt  = 0;

	static vec3f acc_sum   = {0};
	static vec3f gyro_sum  = {0};

bool imu_gyrobias(void)
{
	/*============================================================
	 * 1. 上电静止 3 s 零偏校准（ 陀螺仪）
	 *----------------------------------------------------------*/


	if (!calib_done) {
	    /* 累积原始数据 */
	    acc_sum.x  += BMI088.acc.m_s_2[xx];
	    acc_sum.y  += BMI088.acc.m_s_2[yy];
	    acc_sum.z  += BMI088.acc.m_s_2[zz];

	    gyro_sum.x += BMI088.gyro.dps[xx];
	    gyro_sum.y += BMI088.gyro.dps[yy];
	    gyro_sum.z += BMI088.gyro.dps[zz];

	    if (++calib_cnt >= 5000) {              // 3 s @1 kHz
	        /* 计算平均值作为零偏 */
	        acc_bias.x  = acc_sum.x  / calib_cnt;
	        acc_bias.y  = acc_sum.y  / calib_cnt;
	        acc_bias.z  = acc_sum.z  / calib_cnt;

	        gyro_bias.x = gyro_sum.x / calib_cnt;
	        gyro_bias.y = gyro_sum.y / calib_cnt;
	        gyro_bias.z = gyro_sum.z / calib_cnt;

	        calib_done = true;

	        acc_raw[4][0]=acc_bias.x;
			acc_raw[4][1]=acc_bias.y;
			acc_raw[4][2]=acc_bias.z;

//	        printf("IMUGYRO零偏校准完成\r\n");
//	        printf("  acc bias:  x=%.3f y=%.3f z=%.3f\r\n",
//	               acc_bias.x, acc_bias.y, acc_bias.z);
//	        printf("  gyro bias: x=%.3f y=%.3f z=%.3f\r\n",
//	               gyro_bias.x, gyro_bias.y, gyro_bias.z);
	    }
	    return calib_done;          // 校准未完，直接退出主流程
	}
	else
	{
//		calib_done=!calib_done;
//		calib_cnt=0;
//	    acc_sum.x  = 0;
//	    acc_sum.y  = 0;
//	    acc_sum.z  = 0;
		return calib_done;
	}

}




/* ========== 重力常量 ========== */
#define GRAVITY 9.80665f

/* ========== 计算 3×3 矩阵逆 ========== */
static bool mat3_inv(float A[3][3], float inv[3][3]) {
    float det = 0;
    for (int i = 0; i < 3; ++i)
        det += A[0][i] * (A[1][(i + 1) % 3] * A[2][(i + 2) % 3] -
                          A[1][(i + 2) % 3] * A[2][(i + 1) % 3]);
    if (fabsf(det) < 1e-6f) return false;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float c = A[(j + 1) % 3][(i + 1) % 3] * A[(j + 2) % 3][(i + 2) % 3] -
                      A[(j + 1) % 3][(i + 2) % 3] * A[(j + 2) % 3][(i + 1) % 3];
            inv[i][j] = c / det;
        }
    return true;
}

/* ========== 主入口 ========== */
bool accel_calib_compute(calib_result_t *out) {
	static bool accel_calib_ok = true;

	if(accel_calib_ok)
	{
		float off[3];
		    /* 1. 零偏 = 正反平均值 */
		    for (int i = 0; i < 3; ++i)
		        off[i] = (acc_raw[i * 2][i] + acc_raw[i * 2 + 1][i]) * 0.5f;

		    /* 2. 构建矩阵 A = [a+ - b] */
		    float A[3][3], inv[3][3];
		    for (int i = 0; i < 3; ++i)
		        for (int j = 0; j < 3; ++j)
		            A[i][j] = acc_raw[i * 2][j] - off[j];

		    if (!mat3_inv(A, inv)) return false;

		    /* 3. 输出结果 */
		    for (int i = 0; i < 3; ++i) {
		        out->offset[i] = off[i];                  // 零偏
		        out->scale[i]  = inv[i][i] * GRAVITY;     // 比例因子
		    }
		    accel_calib_ok = false;
		    return !accel_calib_ok;
	}
	else
	{
		return !accel_calib_ok;
	}

}
