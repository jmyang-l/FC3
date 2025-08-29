#ifndef BMI088_400HZ_OFFSET_H_
#define BMI088_400HZ_OFFSET_H_



typedef struct {
    float dθ[3];   // 角增量 (rad)
    float dv[3];   // 速度增量 (m/s)
    uint64_t ts;   // 产生时刻 (µs)
} ImuSubSample_t;

void on_gyro_raw(const float dθ[3], uint64_t ts);
void on_acc_raw(const float dv[3], uint64_t ts);
void imu_400hz_task(void);






#endif /* BMI088_400HZ_OFFSET_H_ */
