/**
 ******************************************************************************
 * @file   usbd_usr.c
 * @author Liao Wenhao
 * @version 1.0
 * @date 2023-05
 * @brief   USB Device �û���ص�����
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 ZCShou</center></h2>
 *
 * USB Device�û�����ص�������ʵ�֣�
 * �û����Ը�����Ҫ���ڶ�Ӧ�ĺ���������Ӧ�Ĵ���
 *
 * Ŀǰ���ڲ���ʱ����disconnected������롣ԭ��δ֪
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_usr.h"
#include <stdio.h>

/** @addtogroup USBD_USER
 * @{
 */

/** @addtogroup USBD_MSC_DEMO_USER_CALLBACKS
 * @{
 */

/** @defgroup USBD_USR
 * @brief    This file includes the user application layer
 * @{
 */

/** @defgroup USBD_USR_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Variables
 * @{
 */
/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
USBD_Usr_cb_TypeDef USR_cb =
	{
		USBD_USR_Init,
		USBD_USR_DeviceReset,
		USBD_USR_DeviceConfigured,
		USBD_USR_DeviceSuspended,
		USBD_USR_DeviceResumed,
		USBD_USR_DeviceConnected,
		USBD_USR_DeviceDisconnected,
};

/**
 * @}
 */

/** @defgroup USBD_USR_Private_Constants
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_USR_Private_FunctionPrototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Functions
 * @{
 */

/**
 * @brief  �豸����������ɳ�ʼ����ʱ���øûص�
 * @param  None
 * @retval None
 */
void USBD_USR_Init(void)
{
//	char *str = "USBD_USR_Init\r\n";

	// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
}

/**
 * @brief  ���豸��⵽���������������¼�ʱ������ô˻ص�
 * @param  speed : device speed
 * @retval None
 */
void USBD_USR_DeviceReset(uint8_t speed)
{
//	char *str = "USB_OTG_SPEED_HIGH\r\n";
//	char *str1 = "USB_OTG_SPEED_FULL\r\n";
//	char *str2 = "USBD_USR_DeviceReset Error\r\n";

	switch (speed)
	{
	case USB_OTG_SPEED_HIGH:
	{
		// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
		break;
	}
	case USB_OTG_SPEED_FULL:
	{
		// udwComSendData(COM_CONSOLE, (unsigned char*)str1, strlen(str1));
		break;
	}
	default:
	{
		// udwComSendData(COM_CONSOLE, (unsigned char*)str2, strlen(str2));
		break;
	}
	}
}

/**
 * @brief  ���豸�յ����õ���������ʱ������ô˻ص�
 * @param  None
 * @retval Status
 */
void USBD_USR_DeviceConfigured(void)
{
//	char *str = "USBD_USR_DeviceConfigured\r\n";

	// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
}
/**
 * @brief  ���豸��⵽������������ͣ�¼�ʱ������ô˻ص�
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceSuspended(void)
{
//	char *str = "USBD_USR_DeviceSuspended\r\n";

	// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
}

/**
 * @brief  ���豸��⵽���������Ļָ��¼�ʱ������ô˻ص�
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceResumed(void)
{
//	char *str = "USBD_USR_DeviceResumed\r\n";

	// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
}

/**
 * @brief  ���豸���ӵ�����ʱ���ô˻ص�
 * @param  None
 * @retval Status
 */
void USBD_USR_DeviceConnected(void)
{
//	char *str = "USBD_USR_DeviceConnected\r\n";

	// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
}

/**
 * @brief  ���豸�������Ͽ�����ʱ���ô˻ص�
 * @param  None
 * @retval Status
 */
void USBD_USR_DeviceDisconnected(void)
{
//	char *str = "USBD_USR_DeviceDisconnected\r\n";

	// udwComSendData(COM_CONSOLE, (unsigned char*)str, strlen(str));
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
