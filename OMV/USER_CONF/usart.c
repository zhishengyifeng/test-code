#include "usart.h"
#include "dma.h"
#include "stdio.h"
#include "gimbal_task.h"

USART_InitTypeDef USART_InitStructure3;
void USART3_DEVICE(void) /*DBUS*/
{
	/**USART3 GPIO Configuration
	PC11     ------> USART3_RX
	PC10     ------> USART3_TX
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_DeInit(USART3);
	USART_InitStructure3.USART_BaudRate = 100000;
	USART_InitStructure3.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure3.USART_StopBits = USART_StopBits_1;
	USART_InitStructure3.USART_Parity = USART_Parity_Even;
	USART_InitStructure3.USART_Mode = USART_Mode_Rx;
	USART_InitStructure3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure3);

	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_IDLE);
	USART_ClearITPendingBit(USART3, USART_FLAG_IDLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART3_DMA();
}

void USART6_DEVICE(void) /*JUDGE*/
{
	/**USART6 GPIO Configuration
	PG14     ------> USART6_TX
	PG9     ------> USART6_RX
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	//	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	//	GPIO_Init(GPIOG,&GPIO_InitStructure);
	//
	//	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
	//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	//	GPIO_Init(GPIOG,&GPIO_InitStructure);
	//
	USART_DeInit(USART6);
	//	USART6->CR1 |= USART_CR1_IDLEIE;
	//	USART6->CR1 |= USART_CR1_RXNEIE;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);

	USART_ClearFlag(USART6, USART_FLAG_IDLE);
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
	USART_Cmd(USART6, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART6_DMA();
}

/************************************************************************************
*****************************					���Դ���			***********************************
************************************************************************************/
u8 Tx_done = 1;
u8 Rx_Flag;
uint8_t Tx_Buffer[Tx_Buffer_Num];
uint8_t Rx_Buffer[Rx_Buffer_Num];

static void Debug_Usart_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  /* �������ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

void Debug_USART_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	//���ڷ���DMA
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_Cmd(DMA2_Stream7,DISABLE);
	DMA_DeInit(DMA2_Stream7);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (DEBUG_USART->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = Tx_Buffer_Num;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	
	//���ڽ���DMA
	
	DMA_Cmd(DMA2_Stream5,DISABLE);
	DMA_DeInit(DMA2_Stream5);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (DEBUG_USART->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Rx_Buffer_Num;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream5, ENABLE);				//����DMA����
}


void Debug_USART_Config(void)
{
	/**USART1 GPIO Configuration
	PB7     ------> USART1_RX
	PA9     ------> USART1_TX
	*/
	
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);

  /* ʹ�� USART ʱ�� */
  RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
  
  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;  
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE,DEBUG_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
  
  /* ���ô�DEBUG_USART ģʽ */
  /* ���������ã�DEBUG_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
  /* �ֳ�(����λ+У��λ)��8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* ֹͣλ��1��ֹͣλ */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* У��λѡ�񣺲�ʹ��У�� */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* Ӳ�������ƣ���ʹ��Ӳ���� */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* ���USART��ʼ������ */
  USART_Init(DEBUG_USART, &USART_InitStructure); 
	USART_ClearFlag(DEBUG_USART,USART_FLAG_TC);
	USART_ClearFlag(DEBUG_USART,USART_FLAG_RXNE);
	//�������ڿ����ж�&DMA
	USART_ITConfig(DEBUG_USART,USART_IT_IDLE,ENABLE);
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);
	Debug_Usart_NVIC_Configuration();
  /* ʹ�ܴ��� */
  USART_Cmd(DEBUG_USART, ENABLE);
}

void DMA_Debug_USART_Tx_Data(uint32_t size)
{
	if(Tx_done == 0)
		return;
	else
	{
		Tx_done = 0;
		DMA_SetCurrDataCounter(DMA2_Stream7,size);
		DMA_Cmd(DMA2_Stream7, ENABLE);				//��ʼDMA����
	}
}

void Info_Proc(void)
{
	if((Rx_Buffer[0] == 'P')&&(Rx_Buffer[1] == 'O')&&(Rx_Buffer[2] == 'S'))
	{
		memcpy((void*)&gimbal.pid.yaw_angle_ref,Rx_Buffer+4,4);
	}
}


/*USART1 �жϺ���*/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET)
	{
		Rx_Flag = 1;		
		DMA_Cmd(DMA2_Stream5, DISABLE);
		DMA_SetCurrDataCounter(DMA2_Stream5,Rx_Buffer_Num);
		USART_ReceiveData(USART1);
		DMA_Cmd(DMA2_Stream5, ENABLE);
		USART_ClearITPendingBit(USART1,USART_IT_IDLE);
	}
}

//���ڷ���DMA�ж�
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) == SET)
	{
		Tx_done = 1;
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);
	}
}
///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
