#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include "stm32f4xx.h"
#include "chassis_task.h"

#define Debug_Power				40//û�в���ϵͳʱ�Ĺ�������
#define Deta_Power				30//�����������Ĺ��ʲ�

extern float Charge_factor;


float Chassis_Power_Control(chassis_t *chassis_power_control);


#endif 
