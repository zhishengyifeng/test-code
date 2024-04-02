#ifndef _judge_rx_data_H
#define _judge_rx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 500

/**代码相对于21赛季裁判系统通信缺少模块如下***

*0x0004 -- 飞镖发射状态，飞镖发射时发送     已添加
*0x0005 -- 人工智能挑战赛加成与惩罚区状态   已添加

*0x0103 -- 请求补给站补弹数据（由参赛队发送 上限10HZ RM对抗赛尚未开发）未添加
*0x0104 -- 裁判警告数据，警告发生后发送     已添加
*0x0105 -- 飞镖发射口倒计时                已添加

*0x0208 -- 弹丸剩余发射数，仅空中机器人、哨兵机器人以及ICRA机器人发送 已添加
*0x0209 -- 机器人RFID状态                  已添加
*0x020A -- 飞镖机器人客户端指令数据         已添加
**/
typedef enum
{
	GAME_STATE_ID                      = 0x0001,  //比赛状态数据
	GAME_RESULT_ID 	                   = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_ID                   = 0x0003,	//比赛机器人血量数据            
	EVENT_DATA_ID 				             = 0x0101,	//场地事件数据
	SUPPLY_PROJECTILE_ACTION_ID        = 0x0102,	//补给站动作标识数据，补给站弹丸释放时触发发送      
	REFEREE_WARNING_ID                 = 0x0104,  //裁判警告数据										
	DART_REMAINING_TINME_ID            = 0x0105,  //飞镖发射相关数据
	GAME_ROBOT_STATE_ID                = 0x0201,	//机器人性能体系数据                
	POWER_HEAT_DATA_ID                 = 0x0202,	//实时功率热量数据
	GAME_ROBOT_POS_ID                  = 0x0203,	//机器人位置数据
	BUFF_ID                            = 0x0204,	//机器人增益数据
	AERIAL_ROBOT_SUPPORT_ID            = 0x0205,	//空中支援时间数据
	ROBOT_HURT_ID                      = 0x0206,	//伤害状态数据，伤害发生后发送
	SHOOT_DATA_ID                      = 0x0207,	//实时射击数据，子弹发射后发送
	BULLET_ALLOWABLE_ID                = 0x0208,  //允许发弹量：空中机器人，哨兵机器人以及ICRA机器人
  RFID_STATE_ID                      = 0x0209,  //机器人RFID状态
	DART_CLIENT_CMD_ID                 = 0x020A,  //飞镖选手端指令数据                     好像步兵上不需要这一条//
	
	
	GAME_ROBOT_ALL_POS_ID              = 0x020B,  //地面机器人位置数据                     新增//但是烧饼的
	GIME_ROBOT_RADAR_MARK_ID    		   = 0x020C,  //雷达标记进度数据                       新增//但是雷达机器人
	GAME_SENTRY_DECISION_SYNC_ID       = 0x020D,	//哨兵自主决策相关信息同步							  新增//但是烧饼的
	GAME_RADAR_DECISION_SYNC_ID				 = 0x020E,	//雷达自主决策信息同步									  新增//但是雷达机器人的
	
	
	STUDENT_INTERACTIVE_HEADER_DATA_ID 		= 0x0301,	//机器人间交互数据 和 客户端通信（发送方触发发送）
  CUSTOM_CONTROLLER_INTERACTION_DATA_ID = 0x0302,//自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz	
	CLIENT_MINIMAP_INTERACTIVE_DATDA_ID   = 0x0303,//客户端小地图交互数据，选手端触发发送
	KEYBOARD_AND_MOUSE_INFORMATION_ID     = 0x0304,//键盘、鼠标信息，通过图传串口发送
	
	
	CLIENT_MINIMAP_INTERACTIVE_RADAR_DATE_ID = 0x0305,//选手端小地图接收雷达数据           新增//
	CUSTOM_CONTROLLER_INTERACTIVE_DATE_ID    = 0x0306,//自定义控制器与选手端交互数据				新增//
	CUSTOM_MINIMAP_SENTRY_DATE_ID						 = 0x0307,//选手端小地图接收哨兵数据           新增//是哨兵和半自动控制机器人的好像并不需要
	CLIENT_MINIMAP_INTERACTIVE_ROBOT_DATE_ID = 0x0308,//选手端小地图接收机器人消息         新增//
	
	
	
	
	
} judge_data_id_e;

/*裁判系统数据结构体*/		

//新的那一份规则里的结构体好像开头少了一个下划线，这里全补上了（不知有无影响）


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
/*0105*///飞镖剩余发射时间
typedef __packed struct
{
 uint8_t dart_remaining_time;
 uint8_t dart_aim_state;
}dart_info_t;
/*0201*/
typedef __packed struct
{
 uint8_t robot_id;													//本机器人id
 uint8_t robot_level;												//机器人等级
 uint16_t current_HP; 											//当前血量
 uint16_t maximum_HP;												//血量上限
 uint16_t shooter_barrel_cooling_value;			//枪口热量每秒冷却值
 uint16_t shooter_barrel_heat_limit;				//枪口热量上限
 uint16_t chassis_power_limit; 							//底盘功率限制
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
 uint16_t buffer_energy;        //原chassis_power_buffer就是缓冲功率
 uint16_t shooter_17mm_1_barrel_heat;//枪口当前热量
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
 uint8_t recovery_buff;					//机器人回血倍率
 uint8_t cooling_buff;					//枪口冷却倍率
 uint8_t defence_buff;					//防御buff
 uint8_t vulnerability_buff;		//机器人负防御
 uint16_t attack_buff;					//攻击buff
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
 uint8_t launching_frequency;//射速/射频
 float initial_speed;  //弹速
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
 uint8_t user_data[30];//这里为内容数据段可填的最大数为113，上一版的值为30
}robot_interaction_data_t;
//哨兵自主决策指令0120							应该是tx
typedef __packed struct
{
	uint32_t sentry_cmd; 
} sentry_cmd_t;
//雷达自主决策指令0121							应该是tx
typedef __packed struct
{
	uint8_t radar_cmd;
} radar_cmd_t;

//自定义控制器交互数据/*0302*/
typedef __packed struct
{
uint8_t data[30];//自定义数据的大小
}custom_robot_data_t;
//小地图交互数据0303
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
//自定义控制器模拟键鼠/*0306*/
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
  robot_status_t        		          		game_robot_state;      		      		 //0x0201直接给等级
  power_heat_data_t   				            power_heat_data;     		 	 					 //0x0202能量
  robot_pos_t   				             		  game_robot_pos;				           		 //0x0203机器人位置
  buff_t						                      buff;						                	 	 //0x0204
	air_support_data_t											air_support_data;										 //0x0205空中机器人
  hurt_data_t						                	robot_hurt;						           		 //0x0206伤害
  shoot_data_t						                shoot_data;			           	     		 //0x0207射速
	projectile_allowance_t                  bullet_allowable;										 //0x0208允许发弹量
	rfid_status_t                           rfid_state;													 //0x0209rfid
	dart_client_cmd_t                       dart_client_cmd;										 //0x020A飞镖
	ground_robot_position_t									ground_robot_position;							 //0x020B全部机器人位置
	radar_mark_data_t												radar_mark_data;										 //0x020C雷达
	sentry_info_t														sentry_info;												 //0x020D哨兵
	radar_info_t														radar_info;													 //0x020E	
  robot_interaction_data_t                robot_interaction_data;		  				 //0x0301
	sentry_cmd_t														sentry_cmd;													 //0x0120哨兵复活
	radar_cmd_t															radar_cmd;													 //0x0121雷达buff
	custom_robot_data_t											custom_robot_data;									 //0x0302自定义控制器
	map_command_t														map_command;												 //0x0303
	remote_control_t                        mouse_keyboard_informationt;				 //0x0304键鼠
	custom_client_data_t										custom_client_data;									 //0x0306模拟键鼠		
} judge_rxdata_t;

extern judge_rxdata_t judge_recv_mesg;//读取回来保存在该变量
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);
void Exp_Calculate_grade(void);

#endif

