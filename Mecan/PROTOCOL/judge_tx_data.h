#ifndef _judge_tx_data_H
#define _judge_tx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_TX_FIFO_BUFLEN 500//���ڸ���������ĳ�ʼ��
#define STUDENT_DATA_LENGTH   sizeof(ext_student_interactive_data_t_t)//�������ݵĳ��� 

//�����˼佻�����ݰ�
typedef __packed struct
{
	uint16_t data_cmd_id;//����ID��������ID��������ID�ͽ�������
	uint16_t sender_ID;
	uint16_t receiver_ID;
	uint8_t data[113];//���113
}ext_student_interactive_data_t_t;

//ѧ������id
typedef enum
{
	//ѧ������������id
	STUDENT_RED_HERO_ID       			= 1,
	STUDENT_RED_ENGINEER_ID	  			= 2,
	STUDENT_RED_INFANTRY3_ID  			= 3,
	STUDENT_RED_INFANTRY4_ID  			= 4,
	STUDENT_RED_INFANTRY5_ID  			= 5,
	STUDENT_RED_AERIAL_ID	    			= 6,
	STUDENT_RED_SENTRY_ID	    			= 7,
	STUDENT_RED_RADAR_ID            = 8,
	STUDENT_RED_RadarStation_ID	    = 9,
	STUDENT_RED_Outpost_ID	    		= 10,//ǰ��վ(ǰ�ںͻ��ص�ID����С��ͼ��������)
	STUDENT_RED_Base_ID	    				= 11,//����
	STUDENT_BLUE_HERO_ID	    			= 101,
	STUDENT_BLUE_ENGINEER_ID  			= 102,
	STUDENT_BLUE_INFANTRY3_ID 			= 103,
	STUDENT_BLUE_INFANTRY4_ID 			= 104,
	STUDENT_BLUE_INFANTRY5_ID 			= 105,
	STUDENT_BLUE_AERIAL_ID    			= 106,
	STUDENT_BLUE_SENTRY_ID	  			= 107,
	STUDENT_BLUE_RADAR_ID           = 108,
	STUDENT_BLUE_RadarStation_ID	  = 109,
	STUDENT_BLUE_Outpost_ID	    		= 110,
	STUDENT_BLUE_Base_ID	    			= 111,
	
	//�����˶�Ӧ�ͻ���id
	RED_HERO_CLIENT_ID	      = 0x0101,
	RED_ENGINEER_CLIENT_ID    = 0x0102,
	RED_INFANTRY3_CLIENT_ID	  = 0x0103,
	RED_INFANTRY4_CLIENT_ID	  = 0x0104,
	RED_INFANTRY5_CLIENT_ID   = 0x0105,
	RED_AERIAL_CLIENT_ID      = 0x0106,
	BLUE_HERO_CLIENT_ID	      = 0x0165,
	BLUE_ENGINEER_CLIENT_ID   = 0x0166,
	BLUE_INFANTRY3_CLIENT_ID  = 0x0167,
	BLUE_INFANTRY4_CLIENT_ID  = 0x0168,
	BLUE_INFANTRY5_CLIENT_ID  = 0x0169,
	BLUE_AERIAL_CLIENT_ID	    = 0x016A,	
	
	//����ϵͳ�������������ڱ����״���������ָ�
	JUDGE_SERVER_ID           = 0x8080,
}interactive_id_e;

//����ID������ID
typedef enum
{
	/*stm32��С��ģʽ������һ��32λ�޷�����0x12345678,
	�ӵ͵�ַ���ߵ�ַ���δ洢78��56��34��12
	
	32bit�����0x12345678��С��ģʽ�����ģʽCPU�д�ŵķ�ʽ�������ַ��0x4000��ʼ��ţ�Ϊ��
	�ڴ��ַ		С��ģʽ�������		���ģʽ�������
	0x4000			0x78								0x12
	0x4001			0x56								0x34
	0x4002			0x34								0x56
	0x4003			0x12								0x78							*/
	//����ID
	RobotCommunication														= 0x0200,//������0x0200~0x02FF���������һ��������Ϊ����ID
	Client_Delete_Graph														= 0x0100,
	Client_Draw_One_Graph													= 0x0101,
	Client_Draw_Two_Graph													= 0x0102,
	Client_Draw_Five_Graph												= 0x0103,
	Client_Draw_Seven_Graph												= 0x0104,
	Client_Draw_Character_Graph										= 0x0110,
	/* NEW */
	SENTRY_CMD_ID                                 = 0x0120, //�ڱ���������ָ��
	RADAR_CMD_ID                                  = 0x0121, //�״���������ָ��
  /* NEW */
	//����ID
	Robot_communicative_data											= 0x0301,
	Custom_control_interactive_port								= 0x0302,
	Client_map_intercactive_data									= 0x0303,
	Keyboard_mouse_data														= 0x0304,
	
}Content_ID;

//�ͻ���ɾ��ͼ��
typedef __packed struct
{
	uint16_t data_cmd_id;//����ID��������ID��������ID��ͼ������
	uint16_t sender_ID;
	uint16_t receiver_ID;
	uint8_t operate_tpye;
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

//ͼ���������ã�����鿴����ϵͳ����Э�鸽¼
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;// ����˵���� λ������λ�򳤶�,�����λ�򳤶ȱ�ʾ��λ��ռ�˶��ٸ�λ
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11; 
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;

//�ͻ��˻���һ��ͼ��
typedef __packed struct
{
	uint16_t data_cmd_id;//����ID��������ID��������ID��ͼ������
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

//�ͻ��˻��ƶ���ͼ��
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

//�ͻ��˻������ͼ��
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

//�ͻ��˻����߸�ͼ��
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

//�ͻ��˻����ַ�
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];//��Ҫ���͵��ַ�����
}ext_client_custom_character_t;

//ͼ������
typedef enum
{
	//ͼ�����������������
	Fric												  =1,
	DODGE												  =2,
	SENTRY_line									  =3,
	Pitch												  =4,
	line_1s												=5,
	line_2s												=6,
	line_3s												=7,		
	line_4s												=8,
	line_5s												=9,
	line_6s												=10,
	line_7s												=11,
	line_8s												=12,
	line_9s												=13,
	line_10s											=14,
	line_11s											=15,
	line_12s											=16,
	line_13s											=17,
	line_14s											=18,
	line_15s											=19,
	line_16l											=20,
	line_17l											=21,
	line_18l											=22,
	line_19l											=23,
	line_20l											=24,
	line_1v												=25,
	line_1c												=26,
	line_2c												=27,
	line_3c												=28,
	line_4c												=29,
	line_5c												=30,	
	Text													=31,
	Chassis												=32,
	Gimbal												=33,
	Shoot													=34,
	Cap														=35,
	View													=36,
	circle                        =37,
	Distance											=38,
	Compensates                   =39,
	Speed													=40,
	fps                           =41,
	blood                         =42,
	Bullet                        =43,
	//ͼ�β���
	Null_operate								=0,//�ղ���
	Delete_graph								=1,//ɾ��ͼ��
	Delete_all									=2,//ɾ������
	
	Add													=1,//����
	Change											=2,//�޸�
	Delete											=3,//ɾ��
	//ͼ������
	Straight_line								=0,//ֱ��
	Rectangle										=1,//����
	Circle											=2,//��Բ
	ellipse											=3,//��Բ
	Circular_arc								=4,//Բ��
	Floatnumber									=5,//������
	Intnumber										=6,//������
	Character										=7,//�ַ�
	//ͼ����
	layer0											=0,
	layer1											=1,
	layer2											=2,
	layer3											=3,
	layer4											=4,
	layer5											=5,
	layer6											=6,
	layer7											=7,
	layer8											=8,
	layer9											=9,
	//��ɫ
	Redblue											=0,//������ɫ
	Yellow											=1,//��ɫ
	Green												=2,//��ɫ
	Orange											=3,//��ɫ
	Amaranth										=4,//�Ϻ�ɫ
	Pink												=5,//��ɫ
	Cyan												=6,//��ɫ
	Black												=7,//��ɫ
	White												=8,//��ɫ
}client_graphic_draw_operate_data_e;

/* 0301�µ�0120 */
typedef __packed struct  
{ 
 __packed union
  {
     //uint32_t sentry_cmd;
     __packed struct 
    {
      uint32_t confirm_rebirth      :  1; // �Ƿ�ȷ�϶�������  0 ������/1 ����
      uint32_t exchange_rebirth     :  1; // �Ƿ�ȷ�϶һ�����  0 ���һ�/1 �һ������ѽ�ң�[ROUNDUP����420- ����ʣ��ʱ����/60����80+�����˵ȼ���20]���/1 ̨��
			uint32_t bullet_number        : 11; // ��Ҫ�һ��ķ�����  �ɽ����0�޸���X��������X��ҳɹ��һ�X�����������˺��ڱ��ɽ����X�޸���X+Y
			uint32_t remote_bullet_times  :  4; // Զ�̶һ�����������,��ʼΪ0������������ÿ�ν�������1
			uint32_t remote_blood_times   :  4; // Զ�̶һ�Ѫ���������������ʼΪ0������������ÿ�ν�������1 (���ѽ�ң�[50+ROUNDUP����420- ����ʣ��ʱ���� /60����20]���/1 ��)
      uint32_t reserved             : 11; // ����λ
		} info;
	} mode_Union;
} sentry_cmd_t;
/*
���ڱ����͸�������ʱ�������������մ���Ե�λ����Ը�λ
��ԭ�����δ�����Щָ�������ȫ���ɹ����ܴ���Ϊֹ��
����������������Ϊ 0����ʱ�ڱ�ս�������Ƿ�ȷ�ϸ��
��ֵΪ 1�����Ƿ���Ҫ�һ����������ֵΪ 1������Ҫ�һ�
�ķ�����ֵ��Ϊ 100�����ٶ�֮ǰ�ڱ�δ�һ���������������
��ʱ��������������ʹ�ڱ��һ��������������������
�Ӻ���ָ��ȴ��ڱ����͵���һ��ָ�
bit 21-31������
*/
/* 0301�µ�0121 */
typedef __packed struct
{
uint8_t radar_cmd;
} radar_cmd_t;

/*0307*/
typedef __packed struct
{
 uint8_t intention;
 uint16_t start_position_x;
 uint16_t start_position_y;
 int8_t delta_x[49];
 int8_t delta_y[49];
 uint16_t sender_id;
}map_data_t;

/*0308*/
typedef __packed struct
{ 
 uint16_t sender_id;
 uint16_t receiver_id;
 uint16_t user_data[30];
}custom_info_t;
/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
	//0x0200~0x02FF���͵�����
  ext_student_interactive_data_t_t	  					ext_student_interactive_data;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_long_line;
	ext_client_custom_character_t									ext_client_custom_character_text;
	//0x0100~0x0110���͵�����
	ext_client_custom_graphic_delete_t						ext_client_custom_graphic_delete;
	ext_client_custom_graphic_single_t						ext_client_custom_graphic_single;
	ext_client_custom_graphic_double_t						ext_client_custom_graphic_double;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_five;
	ext_client_custom_graphic_five_t 							ext_client_custom_graphic_short_line2;
	ext_client_custom_graphic_five_t  						ext_client_custom_graphic_short_line3;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_car_line;
	ext_client_custom_graphic_seven_t							ext_client_custom_graphic_seven;
	ext_client_custom_character_t									ext_client_custom_character;
	ext_client_custom_character_t									ext_client_custom_character_chassis;//����״̬
	ext_client_custom_character_t									ext_client_custom_character_shoot;  //���״̬
	ext_client_custom_character_t									ext_client_custom_character_gimbal; //��̨״̬
	//0x0120~0x0121���͵�����
	sentry_cmd_t                                  sentry_cmd;                         //0x0120 �ڱ���������ָ��
	radar_cmd_t                                   radar_cmd;                          //0x0121 �״���������ָ��
	//0x0307~0x0308���͵�����
	map_data_t                                    map_data;                           //0x0307 ����·����������    
	custom_info_t                                 custom_info;                        //0x0308 ���������˶˷����Զ�����Ϣ
} judge_txdata_t;

extern judge_txdata_t judge_send_mesg;
extern fifo_s_t  judge_txdata_fifo;

void judgement_tx_param_init(void);
void judgement_client_packet_pack(uint8_t *p_data);
void sentry_cmd_packet_pack(uint8_t *p_data);
void map_data_packet_pack(uint8_t *p_data);
void judgement_client_graphics_draw_pack(uint8_t text_twist);
static void client_graphic_DODGE_low_voltage (void);
static void client_graphic_draw_accelerate (void);
static void client_graphic_delete_accelerate (void);
static void client_graphic_draw_pitch (void);
static void client_graphic_draw_5m_line (void);
extern float pit_set,yaw_set;
extern float cap_store;
#endif 

