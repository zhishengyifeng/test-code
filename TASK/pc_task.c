#include "pc_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"
#include "kalman_filter.h"

extern TaskHandle_t pc_rx_Task_Handle;
float nowfps;
UBaseType_t pc_tx_stack_surplus;
UBaseType_t pc_rx_stack_surplus;

float my_time;
void pc_tx_task(void *parm)
{
	uint32_t pc_wake_time = osKernelSysTick();
	while (1)
	{
		pc_send_data_packet_pack();
		send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
		pc_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&pc_wake_time, 20);
	}
}

TaskHandle_t myxHandle;
TaskStatus_t myxTaskDetails;
void vAFunction(void)
{

	/* Obtain the handle of a task from its name. */
	myxHandle = xTaskGetHandle("judge_rx_task");

	/* Check the handle is not NULL. */
	configASSERT(myxHandle);

	/* Use the handle to obtain further information about the task. */
	vTaskGetInfo(/* The handle of the task being queried. */
				 myxHandle,
				 /* The TaskStatus_t structure to complete with information
				 on xTask. */
				 &myxTaskDetails,
				 /* Include the stack high water mark value in the
				 TaskStatus_t structure. */
				 pdTRUE,
				 /* Include the task state in the TaskStatus_t structure. */
				 eInvalid);
}

WorldTime_RxTypedef PC_WorldTime;
WorldTime_RxTypedef PC_KF_Time;
uint32_t PC_FPS;
uint32_t pc_last_times, pc_times;
#ifdef USE_USB_OTG_FS
extern fifo_s_t pc_rxdata_fifo;
extern uint8_t USB_USART_RX_BUF[64];
extern uint32_t USB_USART_RX_STA;
#endif
void pc_rx_task(void *parm)
{
	while (1)
	{
#ifdef USE_USB_OTG_FS
		pc_times = HAL_GetTick() - pc_last_times;
		pc_last_times = HAL_GetTick();
		Get_FPS(&PC_WorldTime, &PC_FPS);
		nowfps = (float)PC_FPS;
		PC_KF_Time.WorldTime = xTaskGetTickCount();
		fifo_s_puts(&pc_rxdata_fifo, USB_USART_RX_BUF, USB_USART_RX_STA);
		unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
#else
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)PC_UART_IDLE_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			if (Signal & PC_UART_IDLE_SIGNAL)
			{
				pc_times = HAL_GetTick() - pc_last_times;
				pc_last_times = HAL_GetTick();
				Get_FPS(&PC_WorldTime, &PC_FPS);
				nowfps = (float)PC_FPS;
				PC_KF_Time.WorldTime = xTaskGetTickCount();
				dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT);
				unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
			}
		}
#endif
		pc_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

/*USART1 �жϺ���*/
void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET && USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_IDLE); // ��������жϱ�־λ

		err_detector_hook(PC_SYS_OFFLINE);

		if (pc_rx_Task_Handle != NULL) // ��������û���ü������ͷ����ź��������¿��ڶ��Ի�����
		{
			xTaskNotifyFromISR((TaskHandle_t)pc_rx_Task_Handle,
							   (uint32_t)PC_UART_IDLE_SIGNAL,
							   (eNotifyAction)eSetBits,
							   (BaseType_t *)&xHigherPriorityTaskWoken);

			/*�����������л�*/
			if (xHigherPriorityTaskWoken != pdFALSE)
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}
