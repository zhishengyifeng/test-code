#include "judge_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"

extern TaskHandle_t judge_rx_Task_Handle;

UBaseType_t judge_tx_stack_surplus;
UBaseType_t judge_rx_stack_surplus;
uint8_t DATA[113] = {"AwakeLion!!!"}; // ������������������˽��������ݣ����û������޸�
void judge_tx_task(void *parm)
{
	uint32_t judge_wake_time = osKernelSysTick();
	static uint8_t i;
	while (1)
	{
		i++;
		//			judgement_client_packet_pack(DATA);
		send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		if (i < 5)
		{
			judgement_client_graphics_draw_pack(1);
			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		}
		else if (i >= 5 && i < 10)
		{
			judgement_client_graphics_draw_pack(2);
			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		}
		else if (i >= 10 && i < 15)
		{
			judgement_client_graphics_draw_pack(3);
			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		}
		else if (i >= 15)
		{
			judgement_client_graphics_draw_pack(4);
			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			i = 0;
		}

		judge_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&judge_wake_time, 100);
	}
}

void judge_rx_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)JUDGE_UART_IDLE_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			if (Signal & JUDGE_UART_IDLE_SIGNAL)
			{
				USART_ClearFlag(USART1, USART_FLAG_IDLE);				  // ��������жϱ�־λ
				dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT); // ͨ��ָ��ȡַ�ķ����Ѵ��ڽ��յ����ݷŽ�FIFO
				unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);			  // ͬ����ͨ��ָ��ȡַ�ķ�����FIFO��������ó����Ž�һ��������
			}
		}
		judge_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

/*USART6 �жϺ���*/
void USART6_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetFlagStatus(USART6, USART_FLAG_IDLE) != RESET // �ж��Ƿ��������              USART_FLAG_TC
		&& USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) // �ж��Ƿ���������ж�
	{
		USART_ReceiveData(USART6);
		USART_ClearFlag(USART6, USART_FLAG_IDLE); // ��������жϱ�־λ
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		err_detector_hook(JUDGE_SYS_OFFLINE);

		if (judge_rx_Task_Handle != NULL) // ��������û���ü������ͷ����ź��������¿��ڶ��Ի�����
		{
			xTaskNotifyFromISR((TaskHandle_t)judge_rx_Task_Handle,
							   (uint32_t)JUDGE_UART_IDLE_SIGNAL,
							   (eNotifyAction)eSetBits,
							   (BaseType_t *)&xHigherPriorityTaskWoken);
			/*�����������л�*/
			if (xHigherPriorityTaskWoken != pdFALSE)
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}
