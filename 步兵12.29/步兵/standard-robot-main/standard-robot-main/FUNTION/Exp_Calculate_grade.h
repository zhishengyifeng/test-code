#ifndef _Exp_Calculate_grade_H
#define _Exp_Calculate_grade_H


void Exp_Calculate_grade_Init(void);

typedef struct
{
int chassis_power_limit;//功率优先
int chassis_power_HPF;//血量优先 
int shooter_17mm_cooling_rate_EF;//17mm弹丸冷却速率爆发优先
int shooter_17mm_cooling_rate_CF;//17mm弹丸冷却速率冷却优先
int robot_level;//等级
int shooter_17mm_cooling_limit_EF;//爆发优先枪口热量上限
int shooter_17mm_cooling_limit_CF;//冷却优先枪口热量上限
}Calculate_grade_d;
extern Calculate_grade_d Calculate_grade;

//裁判系统更改中新增了
/*

	GAME_ROBOT_ALL_POS_ID              = 0x020B,  //地面机器人位置数据                     新增//但是烧饼的
	GIME_ROBOT_RADAR_MARK_ID    		   = 0x020C,  //雷达标记进度数据                       新增//但是雷达机器人
	GAME_SENTRY_DECISION_SYNC_ID       = 0x020D,	//哨兵自主决策相关信息同步							  新增//但是烧饼的
	GAME_RADAR_DECISION_SYNC_ID				 = 0x020E,	//雷达自主决策信息同步									  新增//但是雷达机器人的

	CLIENT_MINIMAP_INTERACTIVE_RADAR_DATE_ID = 0x0305,//选手端小地图接收雷达数据           新增//
	CUSTOM_CONTROLLER_INTERACTIVE_DATE_ID    = 0x0306,//自定义控制器与选手端交互数据				新增//
	CUSTOM_MINIMAP_SENTRY_DATE_ID						 = 0x0307,//选手端小地图接收哨兵数据           新增//是哨兵和半自动控制机器人的好像并不需要
	CLIENT_MINIMAP_INTERACTIVE_ROBOT_DATE_ID = 0x0308,//选手端小地图接收机器人消息         新增//

	其他的也有少量改动但文件名已经更换

	另外0x0301的文件不确定是rx的文件还是tx的文件

*/

#endif
