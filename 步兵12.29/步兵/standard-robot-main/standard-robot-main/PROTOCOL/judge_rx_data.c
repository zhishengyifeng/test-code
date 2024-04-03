#include "judge_rx_data.h"
#include "dma.h"
#include "string.h"
#include "judge_task.h"
#include "chassis_task.h"
#include "modeswitch_task.h"


/*����ϵͳ�����ݽ��� --����ϵͳ�������ݣ�stm32���գ�
  ���Ͷ�Ӧ��������cmd_id,Ȼ���ȡ��Ӧ��Ϣ*/
	
judge_rxdata_t judge_recv_mesg;
judge_data_limit_t judge_data_limit;

/* judge system dma receive data object */
uart_dma_rxdata_t judge_rx_obj;
//----------�������
/* unpack object */
unpack_data_t judge_unpack_obj;

static SemaphoreHandle_t judge_rxdata_mutex;
fifo_s_t  judge_rxdata_fifo;
static uint8_t   judge_rxdata_buf[JUDGE_RX_FIFO_BUFLEN];

void judgement_rx_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */  
  judge_rxdata_mutex = xSemaphoreCreateMutex();
    
  /* judge data fifo init fifo�洢��--�Ƚ��ȳ�*/
  fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_RX_FIFO_BUFLEN, judge_rxdata_mutex); //���judge_rxdata_fifo�Ļ�����

  
  /* initial judge data dma receiver object ��ʼ��������dma���ն��� */
  judge_rx_obj.dma_stream = DMA2_Stream1;
  judge_rx_obj.data_fifo = &judge_rxdata_fifo;
  judge_rx_obj.buff_size = JUDGE_RX_FIFO_BUFLEN;
  judge_rx_obj.buff[0] = judge_rxbuf[0];
  judge_rx_obj.buff[1] = judge_rxbuf[1];

  /* initial judge data unpack object ��ʼ�������ݽ������ */
  judge_unpack_obj.data_fifo = &judge_rxdata_fifo;
  judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;//Э�����֡ͷ����
  judge_unpack_obj.index = 0;
  judge_unpack_obj.data_len = 0;
  judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
  
}



uint8_t interover_data_overflow;
/*һ֡���ݰ�����֡ͷHEADER + ����CMD + ����data + У��λCRC*/
/*@ p_frame�������׵�ַ*/
uint16_t CMD_ID;
void judgement_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;//֡ͷ--����֡��ʼ�ֽڣ��̶�ֵ0xA5��+����֡�ĳ���+�����+CRC8У��
  memcpy(p_header, p_frame, HEADER_LEN);              //��֡ͷ��Ϣ���Ƶ�p_header����
  
  uint16_t data_length = p_header->data_length;              //���ݳ���
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);// CMD�Ŀ�ʼ��ַ = �����׵�ַ + ֡ͷ���ȣ���cmd_id��ֵ�ض���ַ�ڵ�ֵ��
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;     //���ݵĿ�ʼ��ַ = �����׵�ַ + ֡ͷ���� + CMD���� 
  CMD_ID =  cmd_id; 
  switch (cmd_id)
  {
    case GAME_STATE_ID:                                               //001,����״̬����,1HZ���ڷ���
    {
      memcpy(&judge_recv_mesg.game_state, data_addr, data_length);//��ȡ����״̬���ݱ����ڽṹ��judge_recv_mesg��
		
		/**judge_recv_mesg.game_state.game_progress����ǰ�����׶���Ϣ**/
		 if (judge_recv_mesg.game_state.game_progress == 3)  //5 seconds count down 5�뵹��ʱ
		 {
			 /*�û�����*/
		 }
      
      if (judge_recv_mesg.game_state.game_progress == 1)   //prepare stage ׼���׶�
      {
        if (judge_recv_mesg.game_state.stage_remain_time < 240)//�׶α���ʱ��
        {
          /*�û�����*/
        }
      }
      else
      {
          /*�û�����*/
      }
			
			if(judge_recv_mesg.game_state.game_progress == 4 && judge_recv_mesg.game_state.stage_remain_time >= 0 )
			{
				chassis_mode = CHASSIS_DODGE_MODE;
			}
    }
    break;
		
	  case GAME_RESULT_ID:                                              //002,�����������
      memcpy(&judge_recv_mesg.game_result, data_addr, data_length);   
    break;
		
    case GAME_ROBOT_HP_ID:                                            //003������������Ѫ������,1HZ���ڷ���
      memcpy(&judge_recv_mesg.game_robot_HP, data_addr, data_length); 
    break;
			
    case EVENT_DATA_ID:                                               //004,�����¼����ݣ�1HZ���ڷ���
      memcpy(&judge_recv_mesg.event_data, data_addr, data_length);    
    break; 
		
    case SUPPLY_PROJECTILE_ACTION_ID:                                 //102�����ز���վ������ʶ����������������
      memcpy(&judge_recv_mesg.supply_projectile_action, data_addr, data_length);
    break;
		
		case REFEREE_WARNING_ID:                                          //104�����о������ݣ��������
      memcpy(&judge_recv_mesg.referee_warning, data_addr, data_length);
		break;
		
		case DART_REMAINING_TINME_ID:                                     //105�����ڷ���ڵ���ʱ,1HZ���ڷ���
			memcpy(&judge_recv_mesg.dart_remaining_time, data_addr, data_length);
		break;
		
		case GAME_ROBOT_STATE_ID:                                         //201��������״̬10HZ���ڷ���
			memcpy(&judge_recv_mesg.game_robot_state, data_addr, data_length);
		break;
		
		case POWER_HEAT_DATA_ID:                                          //202��ʵʱ�����������ݣ�50HZ���ڷ���
			memcpy(&judge_recv_mesg.power_heat_data, data_addr, data_length);
		break;
		
		case GAME_ROBOT_POS_ID:                                           //203��������λ�����ݣ�10HZ���ڷ���
			memcpy(&judge_recv_mesg.game_robot_pos, data_addr, data_length);
		break;
		
    case BUFF_ID:                                                     //204���������������ݣ�1HZ���ڷ���
			memcpy(&judge_recv_mesg.buff, data_addr, data_length);
		break;
		
		case AERIAL_ROBOT_ENERGY_ID:                                      //205�����л���������״̬���ݣ�10HZ���ڷ��ͣ�ֻ�п������ط���
			memcpy(&judge_recv_mesg.aerial_robot_energy, data_addr, data_length);
		break;
		
		case ROBOT_HURT_ID:                                               //206 �˺�״̬���ݣ��˺���������
			memcpy(&judge_recv_mesg.robot_hurt, data_addr, data_length);
		break;
		
		case SHOOT_DATA_ID:                                               //207��ʵʱ������ݣ��ӵ��������
			memcpy(&judge_recv_mesg.shoot_data, data_addr, data_length);
			judge_data_limit.shooter_id1_17mm_speed_limit = 30;  //m/s
			judge_data_limit.shooter_id2_17mm_speed_limit = 30;  //m/s
		break;
		
		case BULLET_REMAINING_ID:                                         //208������ʣ���������ڱ�������ICRA�����ˣ�1HZ����
			memcpy(&judge_recv_mesg.bullet_remaining, data_addr, data_length);
		break;
		
		case RFID_STATE_ID:                                               //209��������RFID״̬��1HZ����
			memcpy(&judge_recv_mesg.rfid_state, data_addr, data_length);
		break;
		
		case DAT_CLIENT_CMD_ID:                                           //20A�����ڻ����˿ͻ���ָ�����ݣ�10HZ���ڷ���
			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case GROUND_ROBOT_POSITION_ID:                                    //20B�����������λ������
			memcpy(&judge_recv_mesg.ground_robot_position, data_addr, data_length);
		break;

		case RADAR_MARK_DATA_ID:                                          //20C���״��ǽ�������
			memcpy(&judge_recv_mesg.radar_mark_data, data_addr, data_length);
		break;

		case SENTRY_INFO_ID:                                              //020D���ڱ��������������Ϣͬ��
			memcpy(&judge_recv_mesg.sentry_info, data_addr, data_length);
		break;
		
		case RADAR_INFO_ID:                                               //020E���״�����������Ϣͬ��
			memcpy(&judge_recv_mesg.radar_info, data_addr, data_length);
		break;
//	  case STUDENT_INTERACTIVE_HEADER_DATA_ID:                         //301 �����˼佻�����ݣ����ͷ���������
//	  {
//		  if(data_length <= 119)                                         //���ݶ�ͷ�ṹ����+�������ݳ���
//		  {
//			  memcpy(&judge_recv_mesg.student_interactive_header_data, data_addr, data_length);
//			  interover_data_overflow = 0;
//		  }
//			else
//			{
//				interover_data_overflow = 1;
//			}
//		}break;	

		case CUSTOM_CONTROLLER_INTERACTION_DATA_ID:                                        //302���Զ���������������ݽӿڣ�ͨ���ͻ��˴������ͣ����� 30HZ
			memcpy(&judge_recv_mesg.robot_interactive_data, data_addr, data_length);
		break;
		
		case CLIENT_MINIMAP_INTERACTIVE_DATDA_ID:                                          //303���ͻ���С��ͼ�������ݣ���������
			memcpy(&judge_recv_mesg.minimap_interactive_data, data_addr, data_length);
		break;
		
		case KEYBOARD_AND_MOUSE_INFORMATION_ID:                                            //304�����̡������Ϣ��ͨ��ͼ�����ڷ���
			memcpy(&judge_recv_mesg.mouse_keyboard_informationt, data_addr, data_length);
    break;
		
		case MAP_ROBOT_DATA_ID:                                                            //305��ѡ�ֶ�С��ͼ�����״�����
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case CUSTOM_CLIENT_DATA_ID:                                               				 //306���Զ����������ѡ�ֶ˽�������
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case MAP_DATA_ID:                                               									 //307��ѡ�ֶ�С��ͼ�����ڱ�����
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case CUSTOM_INFO_ID:                                               								 //308��ѡ�ֶ�С��ͼ���ջ�������Ϣ
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		default:
    {
		}
    break;
  }
  
}
