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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // ʹ��TIM4ʱ��

	TIM_TimeBaseInitStructure.TIM_Period = per;	   // �Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc; // ��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // �������ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // ������ʱ���ж�
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			  // ��ʱ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE); // ʹ�ܶ�ʱ��
}

// ÿ��1ms��uwTick�������+1
__weak void HAL_IncTick(void)
{
	uwTick++;
	if (uwTick == 0xffffffff)
		uwTick = 0;
}

// ���ص�ǰϵͳ����ֵ
__weak uint32_t HAL_GetTick(void)
{
	return uwTick;
}

/*ȷ�������Ǵ����߳�ģʽ���Ǵ�������ģʽ��*/
static int inHandlerMode(void)
{
	return __get_IPSR() != 0;
}

/******************************************
��ȡ�ں�SysTick��ʱ����ֵ(����ϵͳ���õδ�ʱ����Ϊʱ��Դ)
osKernelSysTickӦ����ÿ��CMSIS-RTOS�б���һ��
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


/*�ö�ʱ��4��¼��������ʱ��*/
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update))
	{
		HAL_IncTick();
		timer_flag = 1;

	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}
