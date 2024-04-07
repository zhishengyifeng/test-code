#include "gpio.h"
#include "delay.h"
#include "bsp_flash.h"
#include "bsp_can.h"
#include "BMI088Middleware.h"
#include "stm32f4xx.h"

static void KEY_EXTI(void);
static void BMI088_INT_GPIO_Init(void);

/**************************************************
GPIOH:PIN10-�� PIN11-�� PIN12-��
GPIOA:PIN0  key���������ڼ�¼����λ��
//GPIOF:PIN6  imuƬѡ�ţ����ͱ�ʾ��������ѡ�и�����
***************************************************/
void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

	/*LED��*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	// ��ԭ��ͼ��֪���������ͣ�LED��������һ��ʼȫ�����ߵ�ƽ
	GPIO_SetBits(GPIOH, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);

	/*KEY����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*������*/
	/*Configure GPIO pin : PtPin */
	GPIO_InitStructure.GPIO_Pin = CS1_ACCEL_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(CS1_ACCEL_GPIO_Port, &GPIO_InitStructure);
	GPIO_SetBits(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);

	/*Configure GPIO pin : PtPin */
	GPIO_InitStructure.GPIO_Pin = CS1_GYRO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(CS1_GYRO_GPIO_Port, &GPIO_InitStructure);
	GPIO_SetBits(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);

	//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 	5;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitStructure);
	BMI088_INT_GPIO_Init();

	KEY_EXTI();
}

static void BMI088_INT_GPIO_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/*****************************************************************
PB2:KEY�����ⲿ�ⲿ�ж�����,��ԭ��ͼ���Կ�����Ҫ���ó������ж�
******************************************************************/
static void KEY_EXTI(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);		  // ʹ���ⲿ�ж�Դʱ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0); // ӳ�䵽IO��

	// EXTI0 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;		  // EXTI0�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

uint8_t key_state; // ����״̬��־λ
// ���ж����ڼ�¼��̨�Ĺ���λ�ò�������д�����flash�У�ʹ�öϵ����Ȼ�ܹ������������λ�õ�����
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) == 1)
	{
		delay_ms(10); // ������������
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)
		{
			cali_param.yaw_offset = moto_yaw.ecd;						 // ��¼yaw��ƫ��
			cali_param.pitch_offset = moto_pit.ecd;						 // ��¼pit��ƫ��
			cali_param.cali_state = CALI_DONE;							 // ��У׼״̬����Ϊ CALI_DONE����ʾУ׼��ɡ�
			BSP_FLASH_WRITE((uint8_t *)&cali_param, sizeof(cali_sys_t)); // ���� BSP_FLASH_WRITE ������ cali_param �ṹ�������д��FLASH�洢����
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0); // ����жϱ�־λ
}
