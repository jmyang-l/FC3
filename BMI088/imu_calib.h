#ifndef BMI088_IMU_CALIB_H_
#define BMI088_IMU_CALIB_H_

#include <stdint.h>
#include <stdbool.h>
#include "position_estimator.h"
#include "BMI088.h"

typedef struct {
    float offset[3];
    float scale[3];
} calib_result_t;

/* ---------- 陀螺仪零偏校准 ---------- */

bool imu_gyrobias(void);


/* ---------- 加速度计六面体校准 ---------- */

bool accel_calib_compute(calib_result_t *out);

#endif /* BMI088_IMU_CALIB_H_ */
