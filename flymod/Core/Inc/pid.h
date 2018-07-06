#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"
#include "data_common.h"



void PID_Common_Update(PID_Typedef *pid, float target, float measure, float dt);



#endif

