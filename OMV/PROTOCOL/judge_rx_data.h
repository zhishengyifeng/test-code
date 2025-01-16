#ifndef _judge_rx_data_H
#define _judge_rx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 500

/* V1.7.0�汾�޶���¼
�޶������룺0x0003,0x0101,0x0202,0x0204,0x0209,0x020B
ɾ�������룺0x0102,0x0205
������0x0309
*/


typedef enum //���շ��͵�ID�Ŷ�������
{
	GAME_STATE_ID                      = 0x0001,  //����״̬����
	GAME_RESULT_ID 	                   = 0x0002,  //�����������
	GAME_ROBOT_HP_ID                   = 0x0003,	//����������Ѫ������            
	EVENT_DATA_ID 				             = 0x0101,	//�����¼�����
	//SUPPLY_PROJECTILE_ACTION_ID        = 0x0102,	//1.7.0�汾����ɾ��
	REFEREE_WARNING_ID                 = 0x0104,  // ����ϵͳ������Ϣ
	DART_REMAINING_TINME_ID            = 0x0105,  //���ڷ���ڵ���ʱ
	GAME_ROBOT_STATE_ID                = 0x0201,	//������״̬����                  
	POWER_HEAT_DATA_ID                 = 0x0202,	//ʵʱ������������
	GAME_ROBOT_POS_ID                  = 0x0203,	//������λ������
	BUFF_ID                            = 0x0204,	//��������������
	//AERIAL_ROBOT_ENERGY_ID             = 0x0205,	//1.7.0�汾����ɾ��
	ROBOT_HURT_ID                      = 0x0206,	//�˺�״̬���ݣ��˺���������
	SHOOT_DATA_ID                      = 0x0207,	//ʵʱ������ݣ��ӵ��������
	BULLET_REMAINING_ID                = 0x0208,  //�ӵ�ʣ�����������л����ˣ��ڱ��������Լ�ICRA������
  RFID_STATE_ID                      = 0x0209,  //������RFID״̬
	DAT_CLIENT_CMD_ID                  = 0x020A,  //���ڻ����˿ͻ���ָ������
/* NEW */
	GROUND_ROBOT_POSITION_ID           = 0x020B,  //���������λ������
	RADAR_MARK_DATA_ID                 = 0x020C,  //�״��ǽ�������
	SENTRY_INFO_ID                     = 0x020D,  //�ڱ��������������Ϣͬ��
	RADAR_INFO_ID                      = 0x020E,  //�״�����������Ϣͬ��
/* NEW */	
	STUDENT_INTERACTIVE_HEADER_DATA_ID    = 0x0301,	//�����˼佻������ �� �ͻ���ͨ�ţ����ͷ��������ͣ� ���շ��Ͷ���

  CUSTOM_CONTROLLER_INTERACTION_DATA_ID = 0x0302, //�Զ���������������ݽӿڣ�ͨ���ͻ��˴������ͣ����� 30HZ	 |ͼ����·
	CLIENT_MINIMAP_INTERACTIVE_DATDA_ID   = 0x0303, //�ͻ���С��ͼ��������
	KEYBOARD_AND_MOUSE_INFORMATION_ID     = 0x0304, //���̡������Ϣ��ͨ��ͼ�����ڷ���  |ͼ����·
/* NEW */
  MAP_ROBOT_DATA_ID                     = 0x0305, //ѡ�ֶ�С��ͼ�����״�����
	CUSTOM_CLIENT_DATA_ID                 = 0x0306, //�Զ����������ѡ�ֶ˽�������
	MAP_DATA_ID                           = 0x0307, //ѡ�ֶ�С��ͼ�����ڱ�����
	CUSTOM_INFO_ID                        = 0x0308, //ѡ�ֶ�С��ͼ���ջ�������Ϣ
/* NEW */	
   CUSTOM_CONTROLLER_RECEIVE_DATA_ID    = 0x0309, //�Զ�����������ջ��������ݣ�Ƶ��10Hz,ͼ����· 
} judge_data_id_e;

/*����ϵͳ���ݽṹ��*/
/*0001*/
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
}game_status_t;

/*0002*/
typedef __packed struct
{
 uint8_t winner;
}game_result_t;


/*0003*/  //1.7.0�޸�
typedef __packed struct 
{ 
  uint16_t red_1_robot_HP; 
  uint16_t red_2_robot_HP; 
  uint16_t red_3_robot_HP; 
  uint16_t red_4_robot_HP; 
  uint16_t reserved1; 
  uint16_t red_7_robot_HP; 
  uint16_t red_outpost_HP; 
  uint16_t red_base_HP; 
  uint16_t blue_1_robot_HP; 
  uint16_t blue_2_robot_HP; 
  uint16_t blue_3_robot_HP; 
  uint16_t blue_4_robot_HP; 
  uint16_t reserved2; 
  uint16_t blue_7_robot_HP; 
  uint16_t blue_outpost_HP; 
  uint16_t blue_base_HP; 
}game_robot_HP_t; 

/*0101*/   //1.7.0�޸�
typedef __packed struct
{ __packed union
  {
		uint32_t event_data;
	__packed struct 
	{
		//0��δռ�� 1�����ѷ�ռ�� 2�����Է�ռ��
        uint32_t self_supply_station : 3;  
		/* 	bit 0��������һ������ص��Ĳ�����ռ��״̬��1Ϊ��ռ��
				bit 1: ������һ����ص��Ĳ�����ռ��״̬��1Ϊ��ռ�� 
				bit 2: ������������ռ��״̬��1Ϊ��ռ�죨�� RMUL ���ã�
		*/
        uint32_t bit3_4 : 2;
		/*	bit 3: ����С�������صļ���״̬��1Ϊ�Ѽ��� 
				bit 4: �������������صļ���״̬��1Ϊ�Ѽ���*/
		
				uint32_t bit5_6 : 2; 
		//  bit 5-6����������ߵص�ռ��״̬��1Ϊ������ռ�죬2Ϊ���Է�ռ�� 
				
				uint32_t bit7_8 : 2; 
		//	bit 7-8���������θߵص�ռ��״̬��1Ϊ��ռ�� 
		
				uint32_t bit9_17 : 9;
		//	bit 9-17���Է��������һ�λ��м���ǰ��վ����ص�ʱ�䣨0-420������Ĭ��Ϊ0��
				
        uint32_t bit18_20 : 3;
		//	bit 18-20���Է��������һ�λ��м���ǰ��վ����صľ���Ŀ�꣬����Ĭ��Ϊ0��1Ϊ����ǰ��վ��2Ϊ���л��ع̶�Ŀ�꣬3Ϊ���л�������̶�Ŀ�꣬4Ϊ���л�������ƶ�Ŀ�� 
		
        uint32_t bit21_22 : 2;
		//	bit 21-22������������ռ��״̬��0Ϊδ��ռ�죬1Ϊ������ռ�죬2Ϊ���Է�ռ�죬3Ϊ��˫��ռ�졣����RMUL���ã�
		
        uint32_t bit23_31 : 9;
		//  bit 23-31������λ
        
    } fields;
	}event_data;
}event_data_t;

/*0102*/
typedef __packed struct
{
 uint8_t reserved;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/*0104*/
typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}referee_warning_t;

/*0105*/
typedef __packed struct
{
 uint8_t dart_remaining_time;
 uint8_t dart_aim_state;
}dart_info_t;

/*0201*/
typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t current_HP;
 uint16_t maximum_HP;
 uint16_t shooter_barrel_cooling_value;
 uint16_t shooter_barrel_heat_limit;
 uint16_t chassis_power_limit;
 uint8_t power_management_gimbal_output : 1;
 uint8_t power_management_chassis_output : 1;
 uint8_t power_management_shooter_output : 1;
}robot_status_t;

/*0202*/   //1.7.0�޸�
typedef __packed struct
{
 uint16_t reserved;
 uint16_t reserved2;
 float reserved3;
 uint16_t buffer_energy;
 uint16_t shooter_17mm_1_barrel_heat;
 uint16_t shooter_17mm_2_barrel_heat;
 uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

/*0203*/
typedef __packed struct
{
 float x;
 float y;
 float angle;
}robot_pos_t;

/*0204*/  //1.7.0�޸�
typedef __packed struct 
{ 
  uint8_t recovery_buff;  
  uint8_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
  uint8_t remaining_energy; 
}buff_t;

/*0205*/
typedef __packed struct
{
 uint8_t airforce_status;
 uint8_t time_remain;
}air_support_data_t;

/*0206*/
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

/*0207*/
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

/*0208*/
typedef __packed struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;

/*0209*/      //1.7.0�޸�
typedef __packed struct
{
	__packed union
  {
		uint32_t rfid_status;
	__packed struct 
	{
		//0:δ��� 1:�Ѽ��
		
		uint32_t self_base : 1;  //���������
		uint32_t self_central_highland : 1;   //��������ߵ������
		uint32_t enemy_central_highland : 1;  //�з�����ߵ������
		uint32_t self_highland : 1; 					//�������θߵ������
		uint32_t enemy_highland : 1;					//�з����θߵ������
		uint32_t self_fly_slope_front  : 1;		//�������������ǰ
		uint32_t self_fly_slope_back   : 1;		//��������������
		uint32_t ememy_fly_slope_front : 1;		//�з����������ǰ
		uint32_t ememy_fly_slope_back  : 1;		//�з�����������
		uint32_t self_highland_down : 1; 			//�������ο�Խ����㣨����ߵ��·���
		uint32_t self_highland_up : 1;				//�������ο�Խ����㣨����ߵ��Ϸ���
		uint32_t enemy_highland_down : 1; 		//�з����ο�Խ����㣨����ߵ��·���
		uint32_t enemy_highland_up : 1;				//�з����ο�Խ����㣨����ߵ��Ϸ���
		uint32_t self_highway_down : 1;				//�������ο�Խ����㣨��·�·���
		uint32_t self_highway_up : 1;					//�������ο�Խ����㣨��·�Ϸ���
		uint32_t enemy_highway_down : 1;				//�з����ο�Խ����㣨��·�·���
		uint32_t enemy_highway_up : 1;					//�з����ο�Խ����㣨��·�Ϸ���
		uint32_t fortress_buff : 1;							//������������� 
		uint32_t self_outpost : 1;							//����ǰ��վ�����
		uint32_t self_blood_point : 1;					//������һ������ص��Ĳ�����/RMUL������ 
		uint32_t self_blood_point2 : 1;					//������һ����ص��Ĳ����� 
	
		uint32_t self_Resource_Island  : 1;			//��������Դ��
		uint32_t ememy_Resource_Island : 1;			//�з�����Դ��
		uint32_t centry_buff : 1;								//���������(RMUL)
		uint32_t reserve : 8;									//����λ
    } fields;
	}rfid_status;
}rfid_status_t;

/*020A*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

/*020B*/	//1.7.0 �޸�
typedef __packed struct 
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float reserved;  
  float reserved2; 
}ground_robot_position_t; 
/*020C*/
typedef __packed struct
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}radar_mark_data_t;
/*020D*/
typedef __packed struct
{
 uint32_t sentry_info;
} sentry_info_t;
/*020E*/
typedef __packed struct
{
 uint8_t radar_info;
} radar_info_t;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

/*0302*/
typedef __packed struct
{
  uint8_t data[30];
}robot_interactive_data_t;

/*0303*/
typedef __packed struct
{
 float target_position_x;
 float target_position_y;
 uint8_t cmd_keyboard;
 uint8_t target_robot_id;
 uint8_t cmd_source;
}map_command1_t;

/*0304*/
typedef __packed struct
{
 int16_t mouse_x;
 int16_t mouse_y;
 int16_t mouse_z;
 int8_t left_button_down;
 int8_t right_button_down;
 uint16_t keyboard_value;
 uint16_t reserved;
}remote_control_t;

 

typedef struct
{
  game_status_t                      	        game_state;               	 	   //0x0001
  game_result_t                      	        game_result;               	     //0x0002
  game_robot_HP_t             			 	        game_robot_HP;     		           //0x0003
  event_data_t  										          event_data;                	     //0x0101
  ext_supply_projectile_action_t     	        supply_projectile_action;  	     //0x0102
 	referee_warning_t                           referee_warning;                 //0x0104
	dart_info_t                                 dart_remaining_time;             //0x0105
	
  robot_status_t        		                  game_robot_state;      		       //0x0201
  power_heat_data_t   				                power_heat_data;     		 	       //0x0202
  robot_pos_t   				                      game_robot_pos;				           //0x0203
  buff_t						                          buff;						                 //0x0204
  air_support_data_t					                aerial_robot_energy;			       //0x0205
  hurt_data_t						                      robot_hurt;						           //0x0206
  shoot_data_t						                    shoot_data;			           	     //0x0207
	projectile_allowance_t                      bullet_remaining;                //0x0208
	rfid_status_t                               rfid_state;                      //0x0209
	dart_client_cmd_t                           dart_client_cmd;                 //0x020A
	ground_robot_position_t                     ground_robot_position;           //0x020B
	radar_mark_data_t                           radar_mark_data;                 //0x020C
	sentry_info_t                               sentry_info;                     //0x020D
	radar_info_t                                radar_info;                      //0x020E
  //ext_student_interactive_header_data_t   	  student_interactive_header_data; //0x0301
 
	ext_student_interactive_header_data_t       student_interactive_header_data_t;			
  robot_interactive_data_t                    robot_interactive_data;            //0x0302
	map_command1_t                              minimap_interactive_data;          //0x0303
	remote_control_t                            mouse_keyboard_informationt;       //0x0304 //ͼ����·
	
} judge_rxdata_t;

typedef struct
{
	uint8_t robot_level;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t chassis_power_limit;
}judge_data_limit_t;
extern judge_data_limit_t judge_data_limit; 
extern judge_rxdata_t judge_recv_mesg;//��ȡ���������ڸñ���
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);

#endif

