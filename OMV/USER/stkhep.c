#include "stkhep.h"
 /******************************************************************************/

extern int __initial_sp;
extern int __stack_base;
extern int __heap_base;
extern int __heap_limit;

const int s_magic = 0x43218765;
const int h_magic = 0x56781234;
 
int stack_set_guard(void)
{
		__disable_irq();
	
		int* msp = (int *)__get_MSP();
		int* base = &__stack_base;
 
		if(msp < base) {
				__enable_irq();
				return -1;
		}
				
	
		for( ; base != msp; base++)
				*base = s_magic;
		
		__enable_irq();
		
		return (uint32_t)msp - (uint32_t)&__stack_base;
}
int stack_detect_guard(void)
{
		__disable_irq();
	
		int* msp = (int *)__get_MSP();
		int* base = &__stack_base;
 
		if(msp < base || *base != s_magic) {
				__enable_irq();
				return -1;
		}
	
		for( ; base != msp; base++) {
				if(*base != s_magic)
						break;
		}
		
		__enable_irq();
		
		return (uint32_t)base - (uint32_t)&__stack_base;
}
int heap_set_guard(void)
{
		__disable_irq();
	
		int* msp = (int *)__get_MSP();
		int* base = &__heap_base;
 
		if(msp > (base + 0x4000)) {
				__enable_irq();
				return -1;
		}
				
	
		for( ; base != msp; base++)
				*base = h_magic;
		
		__enable_irq();
		
		return (uint32_t)msp - (uint32_t)&__heap_base;
}
int heap_detect_guard(void)
{
		__disable_irq();
	
		int* msp = (int *)__get_MSP();
		int* base = &__heap_base;
 
		if(msp < base || *base != h_magic) {
				__enable_irq();
				return -1;
		}
	
		for( ; base != msp; base++) {
				if(*base != h_magic)
						break;
		}
		
		__enable_irq();
		
		return (uint32_t)base - (uint32_t)&__heap_base;
}

 /******************************************************************************/
