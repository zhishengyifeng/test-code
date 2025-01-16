#include "bsp_vofa.h"
#include "string.h"
#include "usart.h"
#include "gimbal_task.h"

float vofa_debug[Vofa_Num] = {0};

const uint8_t tail [4] = {0x00, 0x00, 0x80, 0x7f};	//ึกฮฒ
uint8_t c_data[4];//สýพÝึก
/*JustFloat*/

void float_turn_u8(float f,uint8_t * c){
	uint8_t x;
	FloatLongType data;
	data.fdata=f;
	
	for(x=0;x<4;x++){
		c[x]=(uint8_t)(data.ldata>>(x*8));
	}

}

void JustFloat_Send(float * fdata,uint16_t fdata_num,USART_TypeDef *Usart_choose){
	uint16_t x;
	uint8_t y;
		for(x=0;x<fdata_num;x++){
			float_turn_u8(fdata[x],c_data);
			for(y=0;y<4;y++){
				Usart_choose->DR=c_data[y];
				while((Usart_choose->SR&0X40)==0);
			}
		}
		for(y=0;y<4;y++){
				Usart_choose->DR=tail[y];
			while((Usart_choose->SR&0X40)==0);
		}

}

void Data_Packed(float * input,uint8_t * output)
{
	u8 y =0;
	memcpy(output,input,Vofa_Num*4);
	for(y=0;y<4;y++)
	{
		output[Vofa_Num*4+y]=tail[y];
	}
}

void Vofa_Get_Info(void)
{
	vofa_debug[0] = gimbal.pid.yaw_angle_ref;
	vofa_debug[1] = gimbal.pid.yaw_angle_fdb;
	vofa_debug[2] = gimbal.pid.pit_angle_ref;
	vofa_debug[3] = gimbal.pid.pit_angle_fdb;
	Data_Packed(vofa_debug,Tx_Buffer);
}

