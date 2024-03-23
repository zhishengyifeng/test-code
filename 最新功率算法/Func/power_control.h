#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include "stm32f4xx.h"
#include "chassis_task.h"

#define Debug_Power				90//没有裁判系统时的功率限制
#define	Deta_Power				70

float Chassis_Power_Control(chassis_t *chassis_power_control);


#endif 



