#include "STM32_TIM_BASE.h"
#include "system_stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

__IO uint32_t uwTick;
uint8_t timer_flag;


/*10-1,8400-1*/
void TIM_BASE_Init(u16 per, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 使能TIM4时钟

	TIM_TimeBaseInitStructure.TIM_Period = per;	   // 自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc; // 分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 设置向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // 开启定时器中断
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			  // 定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE); // 使能定时器
}

// 每过1ms，uwTick这个变量+1
__weak void HAL_IncTick(void)
{
	uwTick++;
	if (uwTick == 0xffffffff)
		uwTick = 0;
}

// 返回当前系统计数值
__weak uint32_t HAL_GetTick(void)
{
	return uwTick;
}

/*确定我们是处于线程模式还是处理程序模式。*/
static int inHandlerMode(void)
{
	return __get_IPSR() != 0;
}

/******************************************
获取内核SysTick计时器的值(操作系统是用滴答定时器作为时间源)
osKernelSysTick应该在每个CMSIS-RTOS中保持一致
*******************************************/
uint32_t osKernelSysTick(void)
{
	if (inHandlerMode())
	{
		return xTaskGetTickCountFromISR();
	}
	else
	{
		return xTaskGetTickCount();
	}
}

uint8_t KEY_GetFlag(void)
{
	uint8_t temp = timer_flag;
	timer_flag = 0;
	return temp;
}


/*用定时器4记录代码运行时间*/
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update))
	{
		HAL_IncTick();
		timer_flag = 1;

	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}
