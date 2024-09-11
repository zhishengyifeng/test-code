#ifndef __FILTER_H___
#define __FILTER_H___

#include "stm32f4xx.h"

#define ORDER 0

#if (ORDER == 20)
#define WINDOWS Fir_hanning_20_30Hz
extern float Fir_hanning_20_30Hz[ORDER+1];
#elif (ORDER == 10)
#define WINDOWS Fir_hanning_10_30Hz
extern float Fir_hanning_10_30Hz[ORDER+1];
#elif (ORDER == 8)//Æ¯ÒÆÁ¿×îÐ¡
#define WINDOWS Fir_hanning_8_30Hz
extern float Fir_hanning_8_30Hz[ORDER+1];
#elif (ORDER == 6)
#define WINDOWS Fir_hanning_6_30Hz
extern float Fir_hanning_6_30Hz[ORDER+1];
#elif (ORDER == 4)
#define WINDOWS Fir_hanning_4_30Hz
extern float Fir_hanning_4_30Hz[ORDER+1];
#elif (ORDER == 2)
#define WINDOWS Fir_hanning_2_30Hz
extern float Fir_hanning_2_30Hz[ORDER+1];
#else
#endif


float Filter(float data , float* data_input , float* windows);


#endif

