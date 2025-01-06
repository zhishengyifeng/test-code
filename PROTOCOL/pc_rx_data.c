#include "pc_rx_data.h"
#include "dma.h"
#include "string.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

infantry_structure_t glb_struct;

//----------С����

/* pc system dma receive data object */
uart_dma_rxdata_t pc_rx_obj;
//----------�������
unpack_data_t pc_unpack_obj;

/* pc receive data fifo and buffer */
static SemaphoreHandle_t pc_rxdata_mutex;
fifo_s_t  pc_rxdata_fifo;
static uint8_t   pc_rxdata_buf[PC_RX_FIFO_BUFLEN];

void pc_rx_param_init(void)
{
    /* create the judge_rxdata_mutex mutex  */  
  pc_rxdata_mutex = xSemaphoreCreateMutex();
    
  /* judge data fifo init */
  fifo_s_init(&pc_rxdata_fifo, pc_rxdata_buf, PC_RX_FIFO_BUFLEN, pc_rxdata_mutex); //���judge_rxdata_fifo�Ļ�����
	
  
  /* initial judge data dma receiver object */
  pc_rx_obj.dma_stream = DMA2_Stream5;
  pc_rx_obj.data_fifo = &pc_rxdata_fifo;
  pc_rx_obj.buff_size = PC_RX_FIFO_BUFLEN;
  pc_rx_obj.buff[0] = pc_rxbuf[0];
  pc_rx_obj.buff[1] = pc_rxbuf[1];

  /* initial judge data unpack object */
  pc_unpack_obj.data_fifo = &pc_rxdata_fifo;
  pc_unpack_obj.p_header = (frame_header_t *)pc_unpack_obj.protocol_packet;
  pc_unpack_obj.index = 0;
  pc_unpack_obj.data_len = 0;
  pc_unpack_obj.unpack_step = STEP_HEADER_SOF;
}


/* data receive */
receive_pc_t pc_recv_mesg;	//������pc���͵�Ŀ��λ����Ϣ

/*****************************�����쵶��*************************************/

//for debug
int pc_seq            = 0;
int once_lost_num     = 0;
int lost_pack_percent = 0;

int pack_num_cnt   = 0;
int lost_num_sum_t = 0;
int pack_lost      = 0;


int gim_mode_js;
int pitch_angle_js;
int yaw_angle_js;
int dis_mm_js;

uint32_t pc_yaw_time;

int8_t recv_pc_glb  = 0;
int8_t glb_err_exit = 0;


int pc_state;

float pit_rec_now=0;
float pit_rec_last=0;
float pit_rec_next=0;
float pit_rec_real=0;

float yaw_rec_now=0;
float yaw_rec_last=0;
float yaw_rec_next=0;
float yaw_rec_real=0;

void pc_data_handler(uint8_t *p_frame)
{	
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
	
  //lost pack monitor
{
  pack_num_cnt++;    //��������1
  
  if (pack_num_cnt <= 100)
  {
    once_lost_num = p_header->seq - pc_seq - 1;   //��ⶪ������
    
    if (once_lost_num < 0)
    {
      once_lost_num += 256;
    }
   
    lost_num_sum_t += once_lost_num;
  }
  else
  {
    lost_pack_percent = lost_num_sum_t;
    lost_num_sum_t    = 0;
    pack_num_cnt      = 0;
  }
  
  
  if (once_lost_num != 0)
  {
    pack_lost = 1;    //��ʾ��һ��֡�������ж���
  }
  else
  {
    pack_lost = 0;
  }
  
  pc_seq = p_header->seq;
  //end lost pack monitor
}

  taskENTER_CRITICAL();
  
  switch (cmd_id)
  {
//    case CHASSIS_CTRL_ID:
//    {
//      memcpy(&pc_recv_mesg.chassis_control_data, data_addr, data_length);     
//      
//    }
//    break;

    case GIMBAL_CTRL_ID:
    {
			if(pit_rec_now==0)pit_rec_last=pc_recv_mesg.gimbal_control_data.pit_ref;
			else
			pit_rec_last=pit_rec_now;
			
			if(yaw_rec_now==0)yaw_rec_last=pc_recv_mesg.gimbal_control_data.yaw_ref;
			else
			yaw_rec_last=yaw_rec_now;
			
			
			
			pit_rec_now=pc_recv_mesg.gimbal_control_data.pit_ref;
			yaw_rec_now=pc_recv_mesg.gimbal_control_data.yaw_ref;
			
      memcpy(&pc_recv_mesg.gimbal_control_data, data_addr, data_length);
			
			pit_rec_next=pc_recv_mesg.gimbal_control_data.pit_ref;
			yaw_rec_next=pc_recv_mesg.gimbal_control_data.yaw_ref;
			
			if(pit_rec_next-pit_rec_last<1&&pit_rec_next-pit_rec_last>-1&&(pit_rec_next-pit_rec_now>10)||(pit_rec_next-pit_rec_now<-10))
				pit_rec_real=pit_rec_last;
			else
				pit_rec_real=pit_rec_now;
			
			if(yaw_rec_next-yaw_rec_last<1&&yaw_rec_next-yaw_rec_last>-1&&(yaw_rec_next-yaw_rec_now>10)||(yaw_rec_next-yaw_rec_now<-10))
				yaw_rec_real=yaw_rec_last;
			else
				yaw_rec_real=yaw_rec_now;
			

		}
    
    break;

    case SHOOT_CTRL_ID:
    {
      memcpy(&pc_recv_mesg.shoot_control_data, data_addr, data_length);
    }
    break;
    
    case ERROR_LEVEL_ID:
    {
      memcpy(&pc_recv_mesg.global_error_level, data_addr, data_length);
      
      pc_state = pc_recv_mesg.global_error_level.err_level;
      
    }
    break;
    
//    case INFANTRY_STRUCT_ID:
//    {
//      memcpy(&pc_recv_mesg.structure_data, data_addr, data_length);
//    }
//    break;
//          
//    case CALI_GIMBAL_ID:
//    {
//      memcpy(&pc_recv_mesg.cali_cmd_data, data_addr, data_length);
//    }
//    break; 

  }
  
  taskEXIT_CRITICAL();

}
