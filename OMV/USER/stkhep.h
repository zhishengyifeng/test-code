#ifndef __STKHEP_H
#define __STKHEP_H

#include"stm32f4xx.h"

int stack_set_guard(void);
int stack_detect_guard(void);
int heap_set_guard(void);
int heap_detect_guard(void);

#endif // __STKHEP_H