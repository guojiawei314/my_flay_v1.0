#ifndef __IMU_H
#define __IMU_H

#include "stm32f1xx_hal.h"
#include "data_common.h"

void IMUupdate(float mpu_acc[3],float mpu_gyro[3], float q[4], float dcm[3][3], float dt);
void IMUupdate_2(float dt);

#endif
