#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f4xx.h"

#define Vofa_Num		4

typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;

extern float vofa_debug[Vofa_Num];


void JustFloat_Send(float * fdata,uint16_t fdata_num,USART_TypeDef *Usart_choose);
void Vofa_Get_Info(void);


#endif
