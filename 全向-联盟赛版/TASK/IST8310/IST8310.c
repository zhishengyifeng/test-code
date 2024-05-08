#include "IST8310.h"
#include "stm32f4xx.h"
#include "delay.h"
#include "IST8310reg.h"
#include "I2C.h"
#include "struct_typedef.h"

#include <string.h>

ist8310_data_t IST8310;

uint8_t IST8310_INIT(void)
{
	I2C_INIT();
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = IST8310_GPIOp;
	GPIO_Init(IST8310_GPIOx, &GPIO_InitStructure);

    // 把磁力计重启
    GPIO_ResetBits(IST8310_GPIOx, IST8310_GPIOp);
    delay_ms(50);
    GPIO_SetBits(IST8310_GPIOx, IST8310_GPIOp);
    delay_ms(50);

    // 基础配置
    // 不使能中断，直接读取
    IST8310_WriteReg(IST8310_CNTL2_ADDR, IST8310_STAT2_NONE_ALL);
    // 平均采样四次
    IST8310_WriteReg(IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH);
    // 200Hz的输出频率
    IST8310_WriteReg(IST8310_CNTL1_ADDR, IST8310_CNTL1_CONTINUE);

    IST8310.meg_error |= VerifyMegId(&IST8310.chip_id);
	
	return IST8310.meg_error;
}

uint32_t I2C_EVENT_1;
void IST8310_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	I2C_EVENT_1 = I2C_EVENT;
	uint32_t Timeout;
	Timeout = 10000;
	while(I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout--;
		if(Timeout == 0)
			break;
	}
}

void IST8310_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    // 等待I2C3处于空闲状态
    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C3, ENABLE);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C3, IST8310_Address, I2C_Direction_Transmitter);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C3, RegAddress);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C3, Data);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C3, ENABLE);
}

uint8_t IST8310_ReadReg(uint8_t RegAddress)
{	
    // 等待I2C3处于空闲状态
    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));
	
	uint8_t Data;
	
	I2C_GenerateSTART(I2C3, ENABLE);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C3, IST8310_Address, I2C_Direction_Transmitter);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C3, RegAddress);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C3, ENABLE);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C3, IST8310_Address, I2C_Direction_Receiver);
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C3, DISABLE);
	I2C_GenerateSTOP(I2C3, ENABLE);
	
	IST8310_WaitEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C3);

	I2C_AcknowledgeConfig(I2C3, ENABLE);
	
	return Data;
}

void ReadIST8310Data(fp32 Mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
	
    buf[0] = IST8310_ReadReg(IST8310_DATA_XL_ADDR);
    buf[1] = IST8310_ReadReg(IST8310_DATA_XH_ADDR);
    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    Mag[0] = MAG_SEN * temp_ist8310_data;
	
    buf[2] = IST8310_ReadReg(IST8310_DATA_YL_ADDR);
    buf[3] = IST8310_ReadReg(IST8310_DATA_YH_ADDR);
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    Mag[1] = MAG_SEN * temp_ist8310_data;
	
    buf[4] = IST8310_ReadReg(IST8310_DATA_ZL_ADDR);
    buf[5] = IST8310_ReadReg(IST8310_DATA_ZH_ADDR);
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    Mag[2] = MAG_SEN * temp_ist8310_data;
}

ist8310_error_e VerifyMegId(uint8_t* id)
{
    *id = IST8310_ReadReg(IST8310_CHIP_ID_ADDR);
    if (*id != IST8310_CHIP_ID_VAL)
	{
        return MEG_ID_ERROR;
    }
	else
	{
        return IST8310_NO_ERROR;
    }
}

