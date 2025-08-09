#include "position_estimator.h"
#include <math.h>
#include <stdbool.h> // 包含 bool 类型定义
#include "light_flow.h"

// 传感器输入
vec3f _accel ;      // IMU 加速度（m/s²）
vec3f _gyro ;      // IMU 角速度（rad/s）
float _baro_alt ;  // 气压计高度（m）（海拔）

float _sonar_alt ; // 超声波高度（m，地面向上）
vec3f _flow_vel ;  // 光流 NE 速度（m/s）

pos_est_t _pos_est;//估计的姿态

/* ---------- 位置、速度推算（IMU + 可选传感器融合） ---------- */

// 加速度计噪声 & 过程噪声
static const float acc_noise = 0.039f;      // m/s²
static const float proc_noise_vel = 0.1f;   // (m/s)/√Hz
static const float proc_noise_pos = 0.1f;   // m/√Hz

// 重力
static const float GRAVITY = 9.80665f;

// 6 维状态协方差矩阵： [posx posy posz vx vy vz]
static float P[6][6] = {0};

//把两个 6×6 矩阵逐元素相加 → 得到第三个矩阵。
static inline void matrix6_add(float A[6][6], float B[6][6], float C[6][6])
{
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            C[i][j] = A[i][j] + B[i][j];
}

//把 6×6 矩阵所有元素乘一个标量 → 就地缩放。
static inline void matrix6_scale(float A[6][6], float s)
{
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            A[i][j] *= s;
}

// 矩阵乘法函数，用于计算 A * B 并将结果存储在 C 中
// A: 第一个 6x6 矩阵
// B: 第二个 6x6 矩阵
// C: 存储结果的 6x6 矩阵
static inline void matrix6_multiply(float A[6][6], float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 6; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 矩阵转置函数，用于计算 A 的转置并将结果存储在 B 中
// A: 原始 6x6 矩阵
// B: 存储转置结果的 6x6 矩阵
static inline void matrix6_transpose(float A[6][6], float B[6][6]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            B[i][j] = A[j][i];
        }
    }
}

// 把机体坐标系的向量旋转到 NED(北东地) 坐标系 → 实现“机体 → 世界”坐标变换。
static inline void rotate_body_to_ned(const quatf *q, const vec3f *in, vec3f *out)
{
    float w = q->q[0], x = q->q[1], y = q->q[2], z = q->q[3];
    float vx = in->x, vy = in->y, vz = in->z;

    out->x = (1 - 2 * (y*y + z*z)) * vx + 2 * (x*y - w*z) * vy + 2 * (x*z + w*y) * vz;
    out->y = 2 * (x*y + w*z) * vx + (1 - 2 * (x*x + z*z)) * vy + 2 * (y*z - w*x) * vz;
    out->z = 2 * (x*z - w*y) * vx + 2 * (y*z + w*x) * vy + (1 - 2 * (x*x + y*y)) * vz;
}


// 卡尔曼一步预测（IMU 积分）
// 该函数使用IMU（惯性测量单元）的数据进行卡尔曼滤波器的一步预测，通过对加速度计数据进行积分来更新位置和速度估计
// 参数 att: 指向姿态四元数的指针，用于将机体坐标系的加速度转换到NED（北东地）坐标系
// 参数 dt: 时间间隔，单位为秒，用于积分计算
void pos_est_predict(const quatf *att, float dt)
{
    // 定义一个三维向量 acc_ned，用于存储转换到NED坐标系后的加速度
    vec3f acc_ned;
    // 调用 rotate_body_to_ned 函数，将机体坐标系的加速度 _accel 转换到NED坐标系，结果存储在 acc_ned 中
    rotate_body_to_ned(att, &_accel, &acc_ned);

    // 去除重力
    // 在NED坐标系中，重力方向为负z轴方向，因此需要在加速度的z分量上加上重力加速度，以去除重力的影响
//    acc_ned.z -= GRAVITY;

    // 状态向量 [pos, vel] 6×1
    // 根据运动学公式 v = v0 + a * dt，更新速度估计
    // _pos_est.vel.x 是当前估计的x方向速度，acc_ned.x 是x方向的加速度，dt 是时间间隔
    _pos_est.vel.x += acc_ned.x * dt;
    // 更新y方向速度估计
    _pos_est.vel.y += acc_ned.y * dt;
    // 更新z方向速度估计
    _pos_est.vel.z += acc_ned.z * dt;

    // 根据运动学公式 s = s0 + v * dt，更新位置估计
    // _pos_est.pos.x 是当前估计的x方向位置，_pos_est.vel.x 是x方向的速度，dt 是时间间隔
    _pos_est.pos.x += _pos_est.vel.x * dt;
    // 更新y方向位置估计
    _pos_est.pos.y += _pos_est.vel.y * dt;
    // 更新z方向位置估计
    _pos_est.pos.z += _pos_est.vel.z * dt;

    /* 预测协方差：F = I + dt·A */
    // 定义状态转移矩阵 F，用于描述状态向量在一个时间步长内的变化
    // F 是一个 6x6 的矩阵，其形式基于运动学模型，考虑了位置和速度的关系
    float F[6][6] = {
        {1, 0, 0, dt, 0, 0},
        {0, 1, 0, 0, dt, 0},
        {0, 0, 1, 0, 0, dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };
    // 定义过程噪声协方差矩阵 Q，用于描述系统模型的不确定性
    // Q 是一个 6x6 的矩阵，初始化为零矩阵
    float Q[6][6] = {0};
    // 为 Q 矩阵的对角元素赋值
    // 加速度计噪声对速度的影响，乘以时间间隔 dt
    Q[3][3] = Q[4][4] = Q[5][5] = acc_noise * acc_noise * dt * 5.0f;// 增大1倍，增强对Vz误差的容忍
    // 过程噪声对位置的影响，乘以时间间隔 dt
    Q[0][0] = Q[1][1] = Q[2][2] = proc_noise_pos * proc_noise_pos * dt;
    // 过程噪声对速度和位置关系的影响，乘以时间间隔 dt
    Q[3][0] = Q[4][1] = Q[5][2] = proc_noise_vel * proc_noise_vel * dt;

    /* P = F·P·Fᵀ + Q */
    // 定义 F 的转置矩阵 Ft
    float Ft[6][6];
    // 定义临时矩阵 tmp，用于存储中间计算结果
    float tmp[6][6];
    // 定义矩阵 FPFt，用于存储 F * P * Fᵀ 的结果
    float FPFt[6][6];

    // 计算 F 的转置
    // 调用 matrix6_transpose 函数，将矩阵 F 转置，结果存储在 Ft 中
    matrix6_transpose(F, Ft);
    // 计算 F * P
    // 调用 matrix6_multiply 函数，计算矩阵 F 和 P 的乘积，结果存储在 tmp 中
    matrix6_multiply(F, P, tmp);
    // 计算 (F * P) * Fᵀ
    // 调用 matrix6_multiply 函数，计算矩阵 tmp 和 Ft 的乘积，结果存储在 FPFt 中
    matrix6_multiply(tmp, Ft, FPFt);
    // 计算 F * P * Fᵀ + Q
    // 调用 matrix6_add 函数，将矩阵 FPFt 和 Q 相加，结果存储在 P 中，更新估计误差协方差矩阵
    matrix6_add(FPFt, Q, P);
}

// 融合气压计高度
void pos_est_fuse_baro(float dt)
{
    // 定义气压计高度测量的方差，通常为噪声的估计值
    const float R_alt = 2.0f * 2.0f; // 假设噪声的标准差为2.0，这里使用的是方差，即2.0的平方
    
    // 计算卡尔曼增益K，用于衡量融合气压计高度信息的权重
    float K = P[2][2] / (P[2][2] + R_alt);
    
    // 使用卡尔曼增益K融合气压计高度信息到估计的位置_z坐标
    // _baro_alt是气压计测量的高度值
    // _pos_est.pos.z是当前估计的高度位置
    _pos_est.pos.z += K * (_baro_alt - _pos_est.pos.z);
    
    // 更新估计误差协方差矩阵P的第2行第2列元素（即高度方向的方差）
    // 这一步反映了融合新测量值后，高度估计的不确定性降低
    P[2][2] *= (1 - K);
}


// 融合超声波高度
void pos_est_fuse_sonar(float dt)
{


	const float R_alt = 0.005f * 0.001f;  // 测量方差（需确保与实际噪声匹配）
	// 计算卡尔曼增益（位置z和速度z的增益）
	float S = P[2][2] + R_alt;  // innovation方差（避免重复计算分母）
	float K_pos = P[2][2] / S;  // pos.z的增益
	float K_vel = P[5][2] / S *1.0f;  // vel.z的增益（利用位置-速度相关性）

	// 状态修正（位置和速度）
	float residual = _sonar_alt - _pos_est.pos.z;  // 残差（只算一次）
	_pos_est.pos.z += K_pos * residual;  // 修正位置
	_pos_est.vel.z += K_vel * residual ;  // 修正速度
//	_pos_est.vel.z*=0.1f;
	// 协方差矩阵更新（完整应用 (I - K*H) * P）
	// 注：K是向量[0,0,K_pos,0,0,K_vel]^T（因H只关注pos.z，对应索引2）
	// 定义完整的卡尔曼增益向量（6维）
	    float K[6] = {0.0f, 0.0f, K_pos, 0.0f, 0.0f, K_vel};
	    // 对应：[px增益, py增益, pz增益, vx增益, vy增益, vz增益]
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			// K*H的非零项只有第2列，即 K[0]*H[0][j] + ... + K[5]*H[5][j] = K[i] * (j==2 ? 1 : 0)
			// 因此 (I - K*H)[i][j] = I[i][j] - K[i] * (j==2 ? 1 : 0)
			P[i][j] = P[i][j] - K[i] * P[2][j];  // 核心更新公式
		}
	}

}

//float a_x,a_y,b_x,b_y;
//float m_x;
//float m_y;
//float c_x = 0;
//float c_y = 0;
//float d_x, d_y;
//extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
//// 融合光流 NE 速度
//// 该函数使用卡尔曼滤波器的思想来融合光流传感器提供的 NE 方向速度信息
//void pos_est_fuse_flow(float dt) // dt 表示时间间隔，单位为秒
//{
////	const float R_vel = 0.001f * 0.001f;  // 保持较小值，增强光流可信度
////
////	float flow_mag = sqrtf(_flow_vel.x*_flow_vel.x + _flow_vel.y*_flow_vel.y);
////
////	bool is_static = flow_mag < 0.02f;   // 2 cm/s 以下视为静止
////
////	float k_pos_corr = is_static ? 1.0f : 2.0f;
//
//
//	  //处理光流数据
//	  a_x= -payload.flow_vel_x;
//	  d_x=payload.distance*0.001;//距离m
//	  b_x=a_x*d_x;//cm/s
//	  b_x=flow_lpf(b_x, 0.5f);
//	  m_x=b_x*0.01;//m/s
//	  c_x+=m_x*dt*10.0f;//m
//
//	  a_y=payload.flow_vel_y;
//	  d_y=payload.distance*0.001;//距离m
//	  b_y=a_y*d_y;//cm/s
//	  b_y=flow_lpf(b_y, 0.2f);
//	  m_y=b_y*0.01;//m/s
//	  c_y+=m_y*dt*10.0f;//m
//
////	  	  printf("  %f  \r\n",a_x);
////	  	  printf("高度:  %f m \r\n",d_x);
////	  	  printf("  %f cm/s \r\n",b_x);
////	  	  printf("  %f m/s \r\n",m_x);
////	  	  printf("  %f m \r\n\r\n",c_x);
//
////	  	  printf("  %f  \r\n",a_y);
////	  	  printf("高度:  %f m \r\n",d_y);
////	  	  printf("  %f cm/s \r\n",b_y);
////	  	  printf("  %f m/s \r\n",m_y);
////	  	  printf("  %f m \r\n\r\n",c_y);
//
//	  _pos_est.vel.v[0] = m_x;
//	  _pos_est.vel.v[1] = m_y;
//	  _pos_est.pos.v[0] = c_x;
//	  _pos_est.pos.v[1] = c_y;
//
////
////
////	    for (int i = 0; i < 2; ++i) {  // i=0:北向(vx), i=1:东向(vy)
////	        // 修正：恢复标准卡尔曼增益公式
////	        double K =( P[i+3][i+3] / (P[i+3][i+3] + R_vel))*2.0f;
////
////	        // 计算速度残差并修正速度
////	        _pos_est.vel.v[i] = _flow_vel.v[i];
////
////
//////	        // 计算速度残差并修正速度
//////	        float residual = _flow_vel.v[i] - _pos_est.vel.v[i];
//////	        _pos_est.vel.v[i] += K * residual;
////
////	        // 新增：直接修正位置（关键！解决位置不更新问题）
////	        // 系数k_pos_corr控制光流对位置的直接影响程度，需调整
//////	        const float k_pos_corr = 50.0f;
//////	        _pos_est.pos.v[i] += _pos_est.vel.v[i] * dt;
////
////	        _pos_est.pos.v[i] += k_pos_corr * _pos_est.vel.v[i] * dt;
//////	        _pos_est.pos.v[i] -= (is_static ? 1.0f : 1.0f) * k_pos_corr * residual * dt;
////
////
////	        // 修正：完整且对称地更新协方差矩阵
////	        for (int j = 0; j < 6; ++j) {
////	            // 更新位置-所有状态的协方差
////	            float old_P_ij = P[i][j];
////	            float old_P_i3j = P[i+3][j];
////	            P[i][j] = old_P_ij - K * old_P_i3j;
////	            P[j][i] = P[i][j];  // 保持对称性
////
////	            // 更新速度-所有状态的协方差
////	            P[i+3][j] = old_P_i3j - K * old_P_i3j;
////	            P[j][i+3] = P[i+3][j];  // 保持对称性
////
//////	            /* 在 fuse_flow 末尾额外补一段微小积分 */
//////	            _pos_est.pos.v[i] += _pos_est.vel.v[i] * dt;
////	        }
////	    }
//}

/* ---------- 卡尔曼滤波器专用 ---------- */
typedef struct {
    float X[4];            // [Px, Py, Vx, Vy]
    float P[4][4];         // 4×4 协方差
    float Q[4][4];         // 过程噪声
    float R[2][2];         // 观测噪声
} flow_kf_t;

static flow_kf_t fk = {
    .X = {0},              // 初值 0
    .P = { {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1} },              // 初值 0
    /* 过程噪声 Q：经验值，试飞后可调 */
    .Q = {
        { 10.0f, 0,   0,   0   },
        { 0,   10.0f, 0,   0   },
        { 0,   0,   0.2f, 0   },
        { 0,   0,   0,   0.2f }
    },
    /* 观测噪声 R：光流噪声方差，按实际调 */
    .R = {
        { 0.01f, 0       },
        { 0,       0.01f }
    }
};
void pos_est_fuse_flow(float dt)
{
    /* ---------- 1. 拿到光流速度（NED m/s） ---------- */
    float of_vx = (-payload.flow_vel_x) * (payload.distance * 0.001f) * 0.01f;
    float of_vy = ( payload.flow_vel_y) * (payload.distance * 0.001f) * 0.01f;


    /* ---------- 2. 预测（匀速模型） ---------- */
    float F[4][4] = {
        {1,0, dt,0 },
        {0,1, 0, dt},
        {0,0, 1, 0 },
        {0,0, 0, 1 }
    };

    /* 2.1 状态预测 X = F*X */
    float x1 = fk.X[0] + fk.X[2]*dt;
    float x2 = fk.X[1] + fk.X[3]*dt;
    /* 速度保持不变 */

    /* 2.2 把预测结果写回状态向量 */
    fk.X[0] = x1;
    fk.X[1] = x2;

    /* 2.2 协方差预测 P = F*P*Fᵀ + Q */
    /* 这里用简化写法，直接对角近似即可 */
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            fk.P[i][j] = fk.P[i][j] + fk.Q[i][j]*dt;

    /* ---------- 3. 观测更新（光流速度） ---------- */
    /* 3.1 卡尔曼增益计算（2×4） */
    float S[2][2] = {
        { fk.P[2][2] + fk.R[0][0], fk.P[2][3]              },
        { fk.P[3][2],              fk.P[3][3] + fk.R[1][1] }
    };
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    float K[4][2];
    K[0][0] = ( fk.P[0][2]*S[1][1] - fk.P[0][3]*S[1][0] ) / det;
    K[0][1] = (-fk.P[0][2]*S[0][1] + fk.P[0][3]*S[0][0] ) / det;
    K[1][0] = ( fk.P[1][2]*S[1][1] - fk.P[1][3]*S[1][0] ) / det;
    K[1][1] = (-fk.P[1][2]*S[0][1] + fk.P[1][3]*S[0][0] ) / det;
    K[2][0] = ( fk.P[2][2]*S[1][1] - fk.P[2][3]*S[1][0] ) / det;
    K[2][1] = (-fk.P[2][2]*S[0][1] + fk.P[2][3]*S[0][0] ) / det;
    K[3][0] = ( fk.P[3][2]*S[1][1] - fk.P[3][3]*S[1][0] ) / det;
    K[3][1] = (-fk.P[3][2]*S[0][1] + fk.P[3][3]*S[0][0] ) / det;

    /* 3.2 残差 */
    float res[2] = { of_vx - fk.X[2], of_vy - fk.X[3] };

    /* 3.3 更新状态 */
    for(int i=0;i<4;i++)
        fk.X[i] += K[i][0]*res[0] + K[i][1]*res[1];

    /* 3.4 协方差更新：仅对角线，避免写崩 */
    float KH[4][4] = {0};
    for(int i=0;i<4;i++){
        KH[i][2] = K[i][0];   // H = [0 0 1 0; 0 0 0 1]
        KH[i][3] = K[i][1];
    }
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            float iKH = (i==j) - KH[i][j];   // (I-KH)
            fk.P[i][j] = iKH * fk.P[i][j] + fk.Q[i][j]*dt; // 再加回Q
        }
    }

    /* ---------- 4. 把结果给出去 ---------- */
    _pos_est.pos.v[0] = fk.X[0]*10;
    _pos_est.pos.v[1] = fk.X[1]*10;
    _pos_est.vel.v[0] = fk.X[2];
    _pos_est.vel.v[1] = fk.X[3];
}

//
// 这些函数用于获取传感器数据
void get_accel(float a, float b, float c) {
    _accel.x = a; // 加速度数据
    _accel.y = b;
    _accel.z = c;
}

void get_gyro(float a, float b, float c) {
    _gyro.x = a; // 陀螺仪数据
    _gyro.y = b;
    _gyro.z = c;
}

void get_baro_alt(float a) {
    _baro_alt = a; // 气压计数据
}

void get_sonar_alt(float a) {
    _sonar_alt = a; // 声纳数据
}

void get_flow_vel(float a, float b, float c, const quatf *att) {

	vec3f body = { a ,
	               b ,
	               c };
	vec3f ned;
    rotate_body_to_ned(att, &body, &ned);

    /* 结果直接给到融合接口（只取 x/y） */
        _flow_vel.x = body.x*1;   // 北向
        _flow_vel.y = body.y*1;   // 东向
        _flow_vel.z = 0.0f;    // 不用
//        printf("估计姿态速度:  Vx = %f, Vy = %f, Vz = %f\r\n\r\n", _flow_vel.x, _flow_vel.y, _flow_vel.z);

}







