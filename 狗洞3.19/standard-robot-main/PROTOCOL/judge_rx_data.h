#ifndef _judge_rx_data_H
#define _judge_rx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 500

/**���������21��������ϵͳͨ��ȱ��ģ������***

*0x0004 -- ���ڷ���״̬�����ڷ���ʱ����     �����
*0x0005 -- �˹�������ս���ӳ���ͷ���״̬   �����

*0x0103 -- ���󲹸�վ�������ݣ��ɲ����ӷ��� ����10HZ RM�Կ�����δ������δ���
*0x0104 -- ���о������ݣ����淢������     �����
*0x0105 -- ���ڷ���ڵ���ʱ                �����

*0x0208 -- ����ʣ�෢�����������л����ˡ��ڱ��������Լ�ICRA�����˷��� �����
*0x0209 -- ������RFID״̬                  �����
*0x020A -- ���ڻ����˿ͻ���ָ������         �����
**/
typedef enum
{
	GAME_STATE_ID                      = 0x0001,  //����״̬����
	GAME_RESULT_ID 	                   = 0x0002,  //�����������
	GAME_ROBOT_HP_ID                   = 0x0003,	//����������Ѫ������            
	EVENT_DATA_ID 				             = 0x0101,	//�����¼�����
	SUPPLY_PROJECTILE_ACTION_ID        = 0x0102,	//����վ������ʶ���ݣ�����վ�����ͷ�ʱ��������      
	REFEREE_WARNING_ID                 = 0x0104,  //���о�������										
	DART_REMAINING_TINME_ID            = 0x0105,  //���ڷ����������
	GAME_ROBOT_STATE_ID                = 0x0201,	//������������ϵ����                
	POWER_HEAT_DATA_ID                 = 0x0202,	//ʵʱ������������
	GAME_ROBOT_POS_ID                  = 0x0203,	//������λ������
	BUFF_ID                            = 0x0204,	//��������������
	AERIAL_ROBOT_SUPPORT_ID            = 0x0205,	//����֧Ԯʱ������
	ROBOT_HURT_ID                      = 0x0206,	//�˺�״̬���ݣ��˺���������
	SHOOT_DATA_ID                      = 0x0207,	//ʵʱ������ݣ��ӵ��������
	BULLET_ALLOWABLE_ID                = 0x0208,  //�������������л����ˣ��ڱ��������Լ�ICRA������
  RFID_STATE_ID                      = 0x0209,  //������RFID״̬
	DART_CLIENT_CMD_ID                 = 0x020A,  //����ѡ�ֶ�ָ������                     ���񲽱��ϲ���Ҫ��һ��//
	
	
	GAME_ROBOT_ALL_POS_ID              = 0x020B,  //���������λ������                     ����//�����ձ���
	GIME_ROBOT_RADAR_MARK_ID    		   = 0x020C,  //�״��ǽ�������                       ����//�����״������
	GAME_SENTRY_DECISION_SYNC_ID       = 0x020D,	//�ڱ��������������Ϣͬ��							  ����//�����ձ���
	GAME_RADAR_DECISION_SYNC_ID				 = 0x020E,	//�״�����������Ϣͬ��									  ����//�����״�����˵�
	
	
	STUDENT_INTERACTIVE_HEADER_DATA_ID 		= 0x0301,	//�����˼佻������ �� �ͻ���ͨ�ţ����ͷ��������ͣ�
  CUSTOM_CONTROLLER_INTERACTION_DATA_ID = 0x0302,//�Զ��������������˽������ݣ����ͷ��������ͣ�Ƶ������Ϊ 30Hz	
	CLIENT_MINIMAP_INTERACTIVE_DATDA_ID   = 0x0303,//�ͻ���С��ͼ�������ݣ�ѡ�ֶ˴�������
	KEYBOARD_AND_MOUSE_INFORMATION_ID     = 0x0304,//���̡������Ϣ��ͨ��ͼ�����ڷ���
	
	
	CLIENT_MINIMAP_INTERACTIVE_RADAR_DATE_ID = 0x0305,//ѡ�ֶ�С��ͼ�����״�����           ����//
	CUSTOM_CONTROLLER_INTERACTIVE_DATE_ID    = 0x0306,//�Զ����������ѡ�ֶ˽�������				����//
	CUSTOM_MINIMAP_SENTRY_DATE_ID						 = 0x0307,//ѡ�ֶ�С��ͼ�����ڱ�����           ����//���ڱ��Ͱ��Զ����ƻ����˵ĺ��񲢲���Ҫ
	CLIENT_MINIMAP_INTERACTIVE_ROBOT_DATE_ID = 0x0308,//ѡ�ֶ�С��ͼ���ջ�������Ϣ         ����//
	
	
	
	
	
} judge_data_id_e;

/*����ϵͳ���ݽṹ��*/		

//�µ���һ�ݹ�����Ľṹ�����ͷ����һ���»��ߣ�����ȫ�����ˣ���֪����Ӱ�죩


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
/*0003*/
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
}game_robot_HP_t;
/*0101*/
typedef __packed struct
{
 uint32_t event_type;
}event_data_t;
/*0102*/
typedef __packed struct
{
	uint8_t reserved;
  uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
}ext_supply_projectile_action_t;
/*0104*/
typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}referee_warning_t;
/*0105*///����ʣ�෢��ʱ��
typedef __packed struct
{
 uint8_t dart_remaining_time;
 uint8_t dart_aim_state;
}dart_info_t;
/*0201*/
typedef __packed struct
{
 uint8_t robot_id;													//��������id
 uint8_t robot_level;												//�����˵ȼ�
 uint16_t current_HP; 											//��ǰѪ��
 uint16_t maximum_HP;												//Ѫ������
 uint16_t shooter_barrel_cooling_value;			//ǹ������ÿ����ȴֵ
 uint16_t shooter_barrel_heat_limit;				//ǹ����������
 uint16_t chassis_power_limit; 							//���̹�������
 uint8_t power_management_gimbal_output : 1;
 uint8_t power_management_chassis_output : 1; 
 uint8_t power_management_shooter_output : 1;
}robot_status_t;
/*0202*/
typedef __packed struct
{
 uint16_t chassis_voltage;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t buffer_energy;        //ԭchassis_power_buffer���ǻ��幦��
 uint16_t shooter_17mm_1_barrel_heat;//ǹ�ڵ�ǰ����
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
/*0204*/
typedef __packed struct
{
 uint8_t recovery_buff;					//�����˻�Ѫ����
 uint8_t cooling_buff;					//ǹ����ȴ����
 uint8_t defence_buff;					//����buff
 uint8_t vulnerability_buff;		//�����˸�����
 uint16_t attack_buff;					//����buff
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
 uint8_t launching_frequency;//����/��Ƶ
 float initial_speed;  //����
}shoot_data_t;
/*0208*/
typedef __packed struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;
/*0209*/
typedef __packed struct
{
  uint32_t rfid_status;
}rfid_status_t;
/*020A*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;
/*020B*/
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
 float standard_5_x;
 float standard_5_y;
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
/*0301*/
typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[30];//����Ϊ�������ݶο���������Ϊ113����һ���ֵΪ30
}robot_interaction_data_t;
//�ڱ���������ָ��0120							Ӧ����tx
typedef __packed struct
{
	uint32_t sentry_cmd; 
} sentry_cmd_t;
//�״���������ָ��0121							Ӧ����tx
typedef __packed struct
{
	uint8_t radar_cmd;
} radar_cmd_t;

//�Զ����������������/*0302*/
typedef __packed struct
{
uint8_t data[30];//�Զ������ݵĴ�С
}custom_robot_data_t;
//С��ͼ��������0303
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
}map_command_t;

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
//�Զ��������ģ�����/*0306*/
typedef __packed struct
{
uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
}custom_client_data_t;


typedef struct
{
  game_status_t                      	    game_state;               	 	  		 //0x0001
  game_result_t                      	    game_result;               	    		 //0x0002
  game_robot_HP_t             			 	    game_robot_HP;     		          		 //0x0003
  event_data_t  										      event_data;                	     		 //0x0101
  ext_supply_projectile_action_t     	    supply_projectile_action;  	     		 //0x0102
	referee_warning_t                       referee_warning;              			 //0x0104
	dart_info_t                   					dart_remaining_time;								 //0x0105
  robot_status_t        		          		game_robot_state;      		      		 //0x0201ֱ�Ӹ��ȼ�
  power_heat_data_t   				            power_heat_data;     		 	 					 //0x0202����
  robot_pos_t   				             		  game_robot_pos;				           		 //0x0203������λ��
  buff_t						                      buff;						                	 	 //0x0204
	air_support_data_t											air_support_data;										 //0x0205���л�����
  hurt_data_t						                	robot_hurt;						           		 //0x0206�˺�
  shoot_data_t						                shoot_data;			           	     		 //0x0207����
	projectile_allowance_t                  bullet_allowable;										 //0x0208��������
	rfid_status_t                           rfid_state;													 //0x0209rfid
	dart_client_cmd_t                       dart_client_cmd;										 //0x020A����
	ground_robot_position_t									ground_robot_position;							 //0x020Bȫ��������λ��
	radar_mark_data_t												radar_mark_data;										 //0x020C�״�
	sentry_info_t														sentry_info;												 //0x020D�ڱ�
	radar_info_t														radar_info;													 //0x020E	
  robot_interaction_data_t                robot_interaction_data;		  				 //0x0301
	sentry_cmd_t														sentry_cmd;													 //0x0120�ڱ�����
	radar_cmd_t															radar_cmd;													 //0x0121�״�buff
	custom_robot_data_t											custom_robot_data;									 //0x0302�Զ��������
	map_command_t														map_command;												 //0x0303
	remote_control_t                        mouse_keyboard_informationt;				 //0x0304����
	custom_client_data_t										custom_client_data;									 //0x0306ģ�����		
} judge_rxdata_t;

extern judge_rxdata_t judge_recv_mesg;//��ȡ���������ڸñ���
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);
void Exp_Calculate_grade(void);

#endif

