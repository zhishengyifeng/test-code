/**
 ******************************************************************************
 * º¯Êı¿â  £º »ùÓÚSTM32F4±ê×¼¿â
 * Ğ¾Æ¬ĞÍºÅ£º STM32F407IGHx
 * ´úÂë°æ±¾£º µÚÒ»´ú¿ò¼Ü
 * Íê³ÉÈÕÆÚ£º 2022.6.19
 ******************************************************************************
							RM . µç¿ØÖ®¸è
 *
 *                  Ò»Äê±¸ÈüÁ½Ã£Ã££¬Ğ´³ÌĞò£¬µ½ÌìÁÁ¡£
 *                      ÍòĞĞ´úÂë£¬BugºÎ´¦²Ø¡£
 *                  »úĞµÃ¿ÌìĞÂÏë·¨£¬ÌìÌì¸Ä£¬ÈÕÈÕÃ¦¡£
 *
 *                  ÊÓ¾õµ÷ÊÔÓÖÔõÑù£¬³¯Áî¸Ä£¬Ï¦¶Ï³¦¡£
 *                      Ïà¹ËÎŞÑÔ£¬Î©ÓĞÀáÇ§ĞĞ¡£
 *                  Ã¿ÍíµÆ»ğÀ»Éº´¦£¬Ò¹ÄÑÃÂ£¬¼ÌĞø¸Î¡£
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
#include "iwdg.h"  
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
#include "IST8310.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_core.h"

void flash_cali(void);
void Config_SystemClock(uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

int main(void)
{
	// Config_SystemClock(6,168, 2,7);
	SysTick_Init(168);								// ÏµÍ³µÎ´ğ¶¨Ê±Æ÷³õÊ¼»¯
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // ÖĞ¶Ï·Ö×éÉèÖÃ

#ifdef USE_USB_OTG_FS
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
#endif

	TIM_BASE_Init(10 - 1, 8400 - 1); // ÅäÖÃ¶¨Ê±Æ÷4Îª1msÖĞ¶ÏÒ»´Î
	GPIO_INIT();					 // °åÔØLED¡¢¹éÖĞ°´¼ü¡¢24VµçÔ´Êä³ö¡¢°åÔØÍÓÂİÒÇSPIµÈÒı½ÅµÄ³õÊ¼»¯£¬¾ßÌå¿´A°åµÄÔ­ÀíÍ¼¼
//	TIM12_DEVICE(400-1,90-1);//·äÃùÆ÷£¬PWMÆµÂÊÔÚ2700HZÊ±ÉùÒô×î´ó£¬ÇÒÕ¼¿Õ±ÈĞèÒªÉèÖÃ³É5£¥µÄ¸ßµçÆ½£¬´Ë´¦ÉèÖÃÎª2500HZ
//	TIM10_DEVICE(5000 - 1, 0);												  // Ìá¹©Ò»Â·PWMÊ¹µÃ¼ÓÈÈµç×èÉıÎÂ£¬ÓÃÓÚºãÎÂ¼ÓÈÈIMU
	CAN1_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_9tq, CAN_BS2_4tq, 3); // CAN1ÅäÖÃ
	CAN2_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_9tq, CAN_BS2_4tq, 3);
	USART3_DEVICE(); // ÓÃÓÚÒ£¿ØÍ¨ĞÅ£¬²ÉÓÃµÄÊÇDMA¼Ó¿ÕÏĞÖĞ¶ÏµÄ·½Ê½½ÓÊÕÊı¾İ
	Debug_USART_DMA_Config(); // ÓÃÓÚDebug
	Debug_USART_Config();
	USART6_DEVICE(); // ÓÃÓÚ²ÃÅĞÏµÍ³Í¨ĞÅ£¬²ÉÓÃµÄÊÇDMA¼Ó¿ÕÏĞÖĞ¶ÏµÄ·½Ê½½ÓÊÕÊı¾İ
//	USART8_DEVICE();
	SPI_DEVICE(); // ÓÃÓÚ¶ÁÈ¡°åÔØÍÓÂİÒÇ
	/* ÔÆÌ¨£¬µ×ÅÌ£¬·¢ÉäÈı¸öÄ£¿éµÄÒ»Ğ©³õÊ¼»¯ */
	gimbal_param_init();
	chassis_param_init();
	shoot_param_init();
	/* ÀëÏß¼ì²âÈÎÎñµÄ³õÊ¼»¯ */
	detect_param_init();
	/* ²ÃÅĞÏµÍ³ºÍĞ¡µçÄÔÍ¨ĞÅÈÎÎñµÄ³õÊ¼»¯ */
	judgement_rx_param_init();
	judgement_tx_param_init();
	pc_rx_param_init();
	pc_tx_param_init();
//	Kalman
//	Kalman_Init();
	DWT_Init(168);

	/*Ëã·¨²¹³¥½Ç³õÊ¼»¯*/
//  pc_send_mesg.pc_need_information.pit_set = -0.6f;
//	pc_send_mesg.pc_need_information.yaw_set = -0.5f;
	/*´Ó°åÔØFLASH¶Á³öÔÆÌ¨¹éÖĞÎ»ÖÃÊı¾İ*/

	flash_cali();
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
	}
	else	
	{
      while(BMI088_init())
	  {};

	}
	GPIO_ResetBits(GPIOH,GPIO_Pin_12);
	GPIO_SetBits(GPIOH,GPIO_Pin_11);
	RCC_ClearFlag();
	
	
	TIM8_DEVICE(20000 - 1, 168 - 1); // ¶æ»úPWMÖÜÆÚĞèÒªÅäÖÃ³É20ms£¬¶æ»ú0-180¡ã¶ÔÓ¦Îª¸ßµçÆ½³ÖĞøÊ±¼ä0.5ms-2.5ms
	IWDG_Config(IWDG_Prescaler_64 ,1250);//1s²»Î¹¹·×Ô¶¯¸´Î»,ÔÚmodeswitchÀï±ßÎ¹¹
	TASK_START();		  // ´´½¨¸÷¸öÈÎÎñ£¬Éè¶¨ÓÅÏÈ¼¶ºÍ¶ÑÕ»´óĞ¡

	vTaskStartScheduler();  // ¿ªÆôÈÎÎñµ÷¶È½
	while (1)
	{
	}
}

void Config_SystemClock(uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	RCC_HSICmd(DISABLE);
	if (RCC_WaitForHSEStartUp() == SUCCESS)
	{
		RCC_ClockSecuritySystemCmd(ENABLE);

		RCC_PLLConfig(RCC_PLLSource_HSE, PLLM, PLLN, PLLP, PLLQ);
		RCC_PLLCmd(ENABLE);

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div4);
		RCC_PCLK2Config(RCC_HCLK_Div2);

		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			;
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	}
}

void flash_cali(void)
{
	BSP_FLASH_READ();
	if (cali_param.cali_state != CALI_DONE)
	{
		for (;;)
			;
	}
	else
	{
		gimbal.pit_center_offset = cali_param.pitch_offset;
		gimbal.yaw_center_offset = cali_param.yaw_offset;
	}
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
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
