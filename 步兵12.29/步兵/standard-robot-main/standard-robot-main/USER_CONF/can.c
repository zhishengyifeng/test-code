#include "can.h"

/*CAN1����*//*CAN1_rx D0*//*CAN1_TX D1*/
void CAN1_DEVICE(uint8_t mode,uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//ʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);//canʱ��ʹ��
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);//can���踴λ
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//GPIO���ù���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;//����ģʽ
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);//��ӳ��
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
	
	CAN_InitStructure.CAN_ABOM = ENABLE;//����Զ����߹���
	CAN_InitStructure.CAN_AWUM = DISABLE;//˯��ģʽͨ���������
	CAN_InitStructure.CAN_TTCM = DISABLE;//��ʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_TXFP = DISABLE;//���ȼ��ɱ��ı�ʶ������
	CAN_InitStructure.CAN_NART = DISABLE;//��ֹ�����Զ�����
	CAN_InitStructure.CAN_RFLM = DISABLE;//���Ĳ��������µĸ��Ǿɵ�
	CAN_InitStructure.CAN_Mode = mode;//ģʽ����
	CAN_InitStructure.CAN_Prescaler = psc;//Ԥ��Ƶϵ��
	CAN_InitStructure.CAN_BS1 = bs1;//����ͬ����Ծ���
	CAN_InitStructure.CAN_BS2 = bs2;//
	CAN_InitStructure.CAN_SJW = sjw;//
	CAN_Init(CAN1,&CAN_InitStructure);//��ʼ��can1
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;//����λ��ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;//32λ��
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;//32ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;//
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λmask
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;//
	CAN_FilterInitStructure.CAN_FilterNumber = 0;//������0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//��ʼ���˲���
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;// �����ȼ�Ϊ5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


/*CAN2����*//*CAN2_RX B5*//*CAN2_TX B6*/
void CAN2_DEVICE(uint8_t mode,uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
	
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_Mode = mode;
	CAN_InitStructure.CAN_Prescaler = psc;
	CAN_InitStructure.CAN_BS1 = bs1;
	CAN_InitStructure.CAN_BS2 = bs2;
	CAN_InitStructure.CAN_SJW = sjw;
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 	5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

