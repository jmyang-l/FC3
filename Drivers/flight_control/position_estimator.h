#ifndef FLIGHT_CONTROL_POSITION_ESTIMATOR_H_
#define FLIGHT_CONTROL_POSITION_ESTIMATOR_H_


//////////////////////////////////////////////////////////////////////////////////
// 三维向量
typedef struct {
    union {
        struct {
            float x, y, z;
        };
        float v[3];
    };
} vec3f;

// 姿态四元数（w,x,y,z）
typedef struct {
 union {
		struct {
			float w, x, y, z;
		};
		float q[4];
      };
}quatf;

// 当前估计结果
typedef struct {
    vec3f pos;   // NED（m）位置坐标
    vec3f vel;   // NED（m/s）速度矢量坐标
} pos_est_t;


void pos_est_predict(const quatf *att, float dt);

void pos_est_fuse_baro(float dt);
void pos_est_fuse_sonar(float dt);
void pos_est_fuse_flow(float dt);


void get_accel(float a, float b, float c); 
void get_gyro(float a, float b, float c); 
void get_baro_alt(float a); 
void get_sonar_alt(float a); 
void get_flow_vel(float a, float b, float c, const quatf *att);


#endif /* FLIGHT_CONTROL_POSITION_ESTIMATOR_H_ */




