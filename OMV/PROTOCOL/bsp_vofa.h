#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f4xx.h"


typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;

extern float vofa_debug[10];
extern float vofa_gimbal[4];
extern float vofa_chassis;
extern float vofa_comm;
extern float vofa_detect;
extern float vofa_info;
extern float vofa_judge;
extern float vofa_mode;
extern float vofa_pcrx;
extern float vofa_shoot;
extern float vofa_imu;

void JustFloat_Send(float * fdata,uint16_t fdata_num,USART_TypeDef *Usart_choose);


#endif
