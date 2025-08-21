#ifndef BMI088_IMU_SELECTOR_H_
#define BMI088_IMU_SELECTOR_H_

#include <stdbool.h>
#include "position_estimator.h"

bool imu_selector_get_active(bool* use_imu1);


#endif /* BMI088_IMU_SELECTOR_H_ */
