/**
  ******************************************************************************
  * ������  �� ����STM32F4��׼�� 
  * оƬ�ͺţ� STM32F407IGHx
  * ����汾�� ��һ�����
  * ������ڣ� 2022.6.19
  ******************************************************************************
  *                          RM . ���֮��
  *
  *                  һ�걸����ãã��д���򣬵�������
  *                      ���д��룬Bug�δ��ء�
  *                  ��еÿ�����뷨������ģ�����æ��
  *
  *                  �Ӿ�����������������ģ�Ϧ�ϳ���
  *                      ������ԣ�Ω����ǧ�С�
  *                  ÿ���ƻ���ɺ����ҹ���£������Ρ�
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/*freertos*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
/*config*/
#include "delay.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "STM32_TIM_BASE.h"
/*bsp*/
#include "bsp_flash.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"
/*task*/
#include "start_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "imu_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "stdlib.h"
#include "kalman_filter.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_core.h"
#include "Exp_Calculate_grade.h"
void flash_cali(void);
void Config_SystemClock(uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

int main(void)
{
	//Config_SystemClock(6,168, 2,7);
	SysTick_Init(168);//ϵͳ�δ�ʱ����ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//�жϷ�������
	
#ifdef USE_USB_OTG_FS 
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb, &USR_cb);	 
#endif            
	
	TIM_BASE_Init(10-1,8400-1);//���ö�ʱ��4Ϊ1ms�ж�һ�Σ�����ϵͳ���м���������֮��ʵ��һЩ��ʱ����
	GPIO_INIT();//����LED�����а�����24V��Դ���������������SPI�����ŵĳ�ʼ�������忴A���ԭ��ͼ
  TIM8_DEVICE(20000-1,168-1);//���PWM������Ҫ���ó�20ms�����0-180���ӦΪ�ߵ�ƽ����ʱ��0.5ms-2.5ms
	TIM1_DEVICE(2500-1,168-1);//Ħ����
//	TIM12_DEVICE(400-1,90-1);//��������PWMƵ����2700HZʱ���������ռ�ձ���Ҫ���ó�5���ĸߵ�ƽ���˴�����Ϊ2500HZ
  TIM10_DEVICE(5000-1,0);//�ṩһ·PWMʹ�ü��ȵ������£����ں��¼���IMU
	CAN1_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_10tq, CAN_BS2_3tq, 3);//CAN1����
	CAN2_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_10tq, CAN_BS2_3tq, 3);
	USART3_DEVICE();//����ң��ͨ�ţ����õ���DMA�ӿ����жϵķ�ʽ��������
	USART1_DEVICE();//����PCͨ�����ݣ����õ���DMA�ӿ����жϵķ�ʽ��������
  USART6_DEVICE();//���ڲ���ϵͳͨ�ţ����õ���DMA�ӿ����жϵķ�ʽ��������

//	USART8_DEVICE();
	SPI_DEVICE();//���ڶ�ȡ����������
	//��̨�����̣���������ģ���һЩ��ʼ��
  gimbal_param_init();
  chassis_param_init();
	shoot_param_init();
	//���߼������ĳ�ʼ��
	detect_param_init();
	//����ϵͳ��С����ͨ������ĳ�ʼ��
  judgement_rx_param_init();
  judgement_tx_param_init();
  pc_rx_param_init();
  pc_tx_param_init();

  Exp_Calculate_grade_Init();//经验值计算等级
	//Kalman
	//Kalman_Init();
	DWT_Init(168);
	/*�Ӱ���FLASH������̨����λ������*/
	flash_cali();
  while(BMI088_init()){};
	TASK_START();//�������������趨���ȼ��Ͷ�ջ��С
	vTaskStartScheduler();//�����������
	while(1){}
}

void Config_SystemClock(uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ)
{
	RCC_DeInit();  
	RCC_HSEConfig(RCC_HSE_ON);	
	RCC_HSICmd(DISABLE);	
	if(RCC_WaitForHSEStartUp() == SUCCESS) 
	{
		RCC_ClockSecuritySystemCmd(ENABLE);	
		
		RCC_PLLConfig(RCC_PLLSource_HSE,PLLM,PLLN,PLLP,PLLQ); 
		RCC_PLLCmd(ENABLE);	
		
		RCC_HCLKConfig(RCC_SYSCLK_Div1);	
		RCC_PCLK1Config(RCC_HCLK_Div4);		
		RCC_PCLK2Config(RCC_HCLK_Div2);		
		
		
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
	}
}


void flash_cali(void)
{
  BSP_FLASH_READ();
  if(cali_param.cali_state != CALI_DONE)
  {
    for( ; ; );
  }
  else
  {
    gimbal.pit_center_offset = cali_param.pitch_offset;
    gimbal.yaw_center_offset = cali_param.yaw_offset;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
