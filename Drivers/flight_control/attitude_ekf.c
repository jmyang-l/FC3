//#include "attitude_ekf.h"
//#include <math.h>
//#include <stdbool.h> // 包含 bool 类型定义
//
///* ========== 1. 内部状态 ========== */
//#define N 7          // 状态维：四元数4 + 零偏3
//#define M 3          // 观测维：加速度3
//
//typedef struct {
//    float q[4];      // [w,x,y,z]
//    float bias[3];   // gyro bias
//} state_t;
//
///* 全局状态与协方差 */
//static state_t X = { {1,0,0,0}, {0,0,0} };
//static float P[N][N] = {0};
//static float Q[N][N] = {0};
//static float R[M][M] = {0};
//
///* 对外包装 */
//static quatf   _ekf_quat;
//static float   _ekf_R[3][3];
//
///* ========== 2. 初始化 ========== */
//static void ekf_init_once(void)
//{
//    static bool inited = false;
//    if (inited) return;
//    inited = true;
//
//    /* 过程噪声 Q */
//    float q_q = 1e-4f, q_b = 1e-6f;
//    for (int i = 0; i < 4; ++i) Q[i][i] = q_q;
//    for (int i = 4; i < 7; ++i) Q[i][i] = q_b;
//
//    /* 观测噪声 R */
//    float r_acc = 1e-2f;
//    for (int i = 0; i < 3; ++i) R[i][i] = r_acc;
//
//    /* 初始协方差 P */
//    for (int i = 0; i < 4; ++i) P[i][i] = 1.0f;
//    for (int i = 4; i < 7; ++i) P[i][i] = 0.1f;
//}
//
///* ========== 3. 工具函数 ========== */
//static inline void normalize(float v[4])
//{
//    float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
//    if (n > 0) { v[0]/=n; v[1]/=n; v[2]/=n; v[3]/=n; }
//        else { v[0] = 1; v[1] = 0; v[2] = 0; v[3] = 0; } // 防止除以零
//}
//
///* 四元数乘法 */
//static inline void quat_mul(const float a[4], const float b[4], float out[4])
//{
//    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
//    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
//    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
//    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
//}
//
///* 四元数转重力向量 */
//static inline void quat2gravity(const float q[4], float g[3])
//{
//    g[0] = 2*(q[1]*q[3] - q[0]*q[2]);
//    g[1] = 2*(q[0]*q[1] + q[2]*q[3]);
//    g[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
//}
//
///* ========== 4. 预测步骤 ========== */
//static void ekf_predict(float gx, float gy, float gz, float dt)
//{
//    /* 去零偏 */
//    float wx = gx - X.bias[0];
//    float wy = gy - X.bias[1];
//    float wz = gz - X.bias[2];
//
//    /* 状态转移矩阵 F = I + dt*A */
//    float F[N][N] = {0};
//    for (int i = 0; i < N; ++i) F[i][i] = 1.0f;
//    F[0][1] = -0.5f*dt*wx;  F[0][2] = -0.5f*dt*wy;  F[0][3] = -0.5f*dt*wz;
//    F[1][0] =  0.5f*dt*wx;  F[1][2] =  0.5f*dt*wz;  F[1][3] = -0.5f*dt*wy;
//    F[2][0] =  0.5f*dt*wy;  F[2][3] =  0.5f*dt*wx;  F[2][1] = -0.5f*dt*wz;
//    F[3][0] =  0.5f*dt*wz;  F[3][1] =  0.5f*dt*wy;  F[3][2] = -0.5f*dt*wx;
//
//    /* 四元数积分（一阶） */
//    float dq[4] = {0};
//    dq[0] = 0.5f*(-X.q[1]*wx - X.q[2]*wy - X.q[3]*wz);
//    dq[1] = 0.5f*( X.q[0]*wx + X.q[2]*wz - X.q[3]*wy);
//    dq[2] = 0.5f*( X.q[0]*wy - X.q[1]*wz + X.q[3]*wx);
//    dq[3] = 0.5f*( X.q[0]*wz + X.q[1]*wy - X.q[2]*wx);
//    for (int i = 0; i < 4; ++i) X.q[i] += dq[i]*dt;
//    normalize(X.q);
//
//    /* P = F*P*Fᵀ + Q*dt */
//    float tmp[N][N] = {0}, Ft[N][N] = {0};
//    for (int i = 0; i < N; ++i)
//        for (int j = 0; j < N; ++j) {
//            for (int k = 0; k < N; ++k) tmp[i][j] += F[i][k]*P[k][j];
//            Ft[j][i] = F[i][j];
//        }
//    for (int i = 0; i < N; ++i)
//        for (int j = 0; j < N; ++j) {
//            P[i][j] = 0;
//            for (int k = 0; k < N; ++k) P[i][j] += tmp[i][k]*Ft[k][j];
//            P[i][j] += Q[i][j]*dt;
//        }
//}
//
///* ========== 5. 更新步骤 ========== */
//static void ekf_update(float ax, float ay, float az)
//{
//	float n = sqrtf(ax*ax + ay*ay + az*az);
//	if (n < 0.5f || n > 15.0f) return;   // 异常数据直接丢弃
//
//    float z[3] = {ax, ay, az};
//    normalize(z);
//
//    float h[3];
//    quat2gravity(X.q, h);
//    normalize(h);
//
//    /* 观测雅可比 H (3×7) */
//    float H[3][7] = {0};
//    float q0 = X.q[0], q1 = X.q[1], q2 = X.q[2], q3 = X.q[3];
//    H[0][0] = -2*q2; H[0][1] =  2*q3; H[0][2] = -2*q0; H[0][3] =  2*q1;
//    H[1][0] =  2*q1; H[1][1] =  2*q0; H[1][2] =  2*q3; H[1][3] =  2*q2;
//    H[2][0] =  2*q0; H[2][1] = -2*q1; H[2][2] = -2*q2; H[2][3] =  2*q3;
//
//    /* H*P*Hᵀ + R */
//    float HP[3][7] = {0}, HPH[3][3] = {0};
//    for (int i = 0; i < 3; ++i)
//        for (int j = 0; j < 7; ++j)
//            for (int k = 0; k < 7; ++k) HP[i][j] += H[i][k]*P[k][j];
//    for (int i = 0; i < 3; ++i)
//        for (int j = 0; j < 3; ++j)
//            for (int k = 0; k < 7; ++k) HPH[i][j] += HP[i][k]*H[j][k];
//    for (int i = 0; i < 3; ++i) HPH[i][i] += R[i][i];
//
//    /* 3×3 逆矩阵（伴随矩阵法） */
//    float det = HPH[0][0]*(HPH[1][1]*HPH[2][2]-HPH[1][2]*HPH[2][1]) -
//                HPH[0][1]*(HPH[1][0]*HPH[2][2]-HPH[1][2]*HPH[2][0]) +
//                HPH[0][2]*(HPH[1][0]*HPH[2][1]-HPH[1][1]*HPH[2][0]);
//    if (fabsf(det) < 1e-3f) return;
//    float inv[3][3];
//    inv[0][0] = (HPH[1][1]*HPH[2][2]-HPH[1][2]*HPH[2][1])/det;
//    inv[1][1] = (HPH[0][0]*HPH[2][2]-HPH[0][2]*HPH[2][0])/det;
//    inv[2][2] = (HPH[0][0]*HPH[1][1]-HPH[0][1]*HPH[1][0])/det;
//    inv[0][1] = -(HPH[0][1]*HPH[2][2]-HPH[0][2]*HPH[2][1])/det;
//    inv[0][2] =  (HPH[0][1]*HPH[1][2]-HPH[0][2]*HPH[1][1])/det;
//    inv[1][0] = -(HPH[1][0]*HPH[2][2]-HPH[1][2]*HPH[2][0])/det;
//    inv[1][2] = -(HPH[0][0]*HPH[1][2]-HPH[0][2]*HPH[1][0])/det;
//    inv[2][0] =  (HPH[1][0]*HPH[2][1]-HPH[1][1]*HPH[2][0])/det;
//    inv[2][1] = -(HPH[0][0]*HPH[2][1]-HPH[0][1]*HPH[2][0])/det;
//
//    /* K = P*Hᵀ*inv */
//    float K[7][3] = {0};
//    for (int i = 0; i < 7; ++i)
//        for (int j = 0; j < 3; ++j)
//            for (int k = 0; k < 3; ++k)
//                K[i][j] += HP[i][k]*inv[j][k];
//
//    /* 状态修正 */
//    float y[3] = {z[0]-h[0], z[1]-h[1], z[2]-h[2]};
//    for (int i = 0; i < 7; ++i)
//        for (int j = 0; j < 3; ++j) {
//            float delta = K[i][j] * y[j];
//            if (i < 4) X.q[i] += delta;
//            else       X.bias[i-4] += delta;
//        }
//
//    /* 协方差更新 P = (I-K*H)*P */
//    float IKH[7][7] = {0};
//    for (int i = 0; i < 7; ++i)
//        for (int j = 0; j < 7; ++j) {
//            IKH[i][j] = (i==j);
//            for (int k = 0; k < 3; ++k)
//                IKH[i][j] -= K[i][k]*H[k][j];
//        }
//    float newP[7][7] = {0};
//    for (int i = 0; i < 7; ++i)
//        for (int j = 0; j < 7; ++j)
//            for (int k = 0; k < 7; ++k)
//                newP[i][j] += IKH[i][k]*P[k][j];
//    for (int i = 0; i < 7; ++i)
//        for (int j = 0; j < 7; ++j)
//            P[i][j] = newP[i][j];
//
//    normalize(X.q);
//}
//
///* ========== 6. 对外包装 ========== */
//void attitude_update_ekf(float dt, const vec3f *accel, const vec3f *gyro)
//{
//    ekf_init_once();
//    ekf_predict(gyro->x, gyro->y, gyro->z, dt);
//    ekf_update (accel->x, accel->y, accel->z);
//
//    /* 对外四元数 */
//    _ekf_quat.w = X.q[0];
//    _ekf_quat.x = X.q[1];
//    _ekf_quat.y = X.q[2];
//    _ekf_quat.z = X.q[3];
//
//    /* 对外矩阵 */
//    float w=X.q[0],x=X.q[1],y=X.q[2],z=X.q[3];
//    _ekf_R[0][0]=1-2*(y*y+z*z);
//    _ekf_R[0][1]=2*(x*y-z*w);
//    _ekf_R[0][2]=2*(x*z+y*w);
//    _ekf_R[1][0]=2*(x*y+z*w);
//    _ekf_R[1][1]=1-2*(x*x+z*z);
//    _ekf_R[1][2]=2*(y*z-x*w);
//    _ekf_R[2][0]=2*(x*z-y*w);
//    _ekf_R[2][1]=2*(y*z+x*w);
//    _ekf_R[2][2]=1-2*(x*x+y*y);
//}
//
//quatf attitude_get_quat(void) { return _ekf_quat; printf("q=%.3f %.3f %.3f %.3f\n", _ekf_quat.w, _ekf_quat.x, _ekf_quat.y, _ekf_quat.z);}
//
//const float (*attitude_get_R(void))[3] { return _ekf_R; }
//
///* 欧拉角（沿用旧算法） */
//euler_t attitude_get_euler(void)
//{
//    euler_t e;
//    float w = _ekf_quat.w, x = _ekf_quat.x, y = _ekf_quat.y, z = _ekf_quat.z;
//    float test = w*x - z*y;
//    if (test > 0.499f) {
//        e.roll = 2*atan2f(x,w);
//        e.pitch = -M_PI_2;
//        e.yaw = 0;
//        return e;
//    }
//    if (test < -0.499f) {
//        e.roll = -2*atan2f(x,w);
//        e.pitch =  M_PI_2;
//        e.yaw = 0;
//        return e;
//    }
//    e.roll  = atan2f( 2*(w*y + x*z), 1-2*(y*y+z*z) );
//    e.pitch = asinf(  -2*(w*x - z*y) );
//    e.yaw   = atan2f( 2*(w*z + x*y), 1-2*(x*x+y*y) );
//    return e;
//}
