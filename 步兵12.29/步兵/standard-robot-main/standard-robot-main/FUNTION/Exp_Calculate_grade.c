#include "judge_rx_data.h"
#include "Exp_Calculate_grade.h"


Calculate_grade_d Calculate_grade;
void Exp_Calculate_grade_Init(void)
{
	
	  //经验等级十级,底盘功率提升			枪管热量的计算还没写
	/*																																					枪管
			 功率优先	PF				血量优先HpF								爆发优先EF 									冷却优先CF
	1				60									45											 200				10								50				40
	2				65									50											 250				15								85				45
	3				70									55											 300				20								120				50
	4				75									60											 350				25								155				55
	5				80									65											 400				30								190				60
	6				85									70											 450				35								225				65
	7				90									75											 500				40								260				70
	8				95									80											 550				45								295				75
	9				100									90											 600				50								330				80
	10			100									100											 650				60								400				80
	*/
  int level;
	int chassis_power;//功率优先
	int chassis_power_hpf;//血量优先
	int shooter_17mm_cooling_rate_ef ,shooter_17mm_cooling_limit_ef;//爆发
	int shooter_17mm_cooling_rate_cf ,shooter_17mm_cooling_limit_cf;//冷却
	while (1)
	{
			if (judge_recv_mesg.game_robot_state.current_exp < 5000)
			{
				level = 1+ judge_recv_mesg.game_robot_state.current_exp / 550;
			}
			else
			{
				level = 10;
			}
			switch (level)
			{
				case 1:
					chassis_power = 60;
					chassis_power_hpf =45;
					shooter_17mm_cooling_limit_ef =200;
					shooter_17mm_cooling_limit_cf =50;
					shooter_17mm_cooling_rate_ef =10;
					shooter_17mm_cooling_rate_cf =40;
				break;
				case 2:
					chassis_power = 65;
					chassis_power_hpf =50;
					shooter_17mm_cooling_limit_ef =250;
					shooter_17mm_cooling_limit_cf =85;
					shooter_17mm_cooling_rate_ef =15;
					shooter_17mm_cooling_rate_cf =45;
				break;
				case 3:
					chassis_power = 70;
					chassis_power_hpf =55;
					shooter_17mm_cooling_limit_ef =300;
					shooter_17mm_cooling_limit_cf =120;
					shooter_17mm_cooling_rate_ef =20;
					shooter_17mm_cooling_rate_cf =50;
				break;
				case 4:
					chassis_power = 75;
					chassis_power_hpf =60;
					shooter_17mm_cooling_limit_ef =350;
					shooter_17mm_cooling_limit_cf =155;
					shooter_17mm_cooling_rate_ef =25;
					shooter_17mm_cooling_rate_cf =55;
				break;
				case 5:
					chassis_power = 80;
					chassis_power_hpf =65;
					shooter_17mm_cooling_limit_ef =400;
					shooter_17mm_cooling_limit_cf =190;
					shooter_17mm_cooling_rate_ef =30;
					shooter_17mm_cooling_rate_cf =60;
				break;
				case 6:
					chassis_power = 85;
					chassis_power_hpf =70;
					shooter_17mm_cooling_limit_ef =450;
					shooter_17mm_cooling_limit_cf =225;
					shooter_17mm_cooling_rate_ef =35;
					shooter_17mm_cooling_rate_cf =65;
				break;
				case 7:
					chassis_power = 90;
					chassis_power_hpf =75;
					shooter_17mm_cooling_limit_ef =500;
					shooter_17mm_cooling_limit_cf =260;
					shooter_17mm_cooling_rate_ef =40;
					shooter_17mm_cooling_rate_cf =70;
				break;
				case 8:
					chassis_power = 95;
					chassis_power_hpf =80;
					shooter_17mm_cooling_limit_ef =550;
					shooter_17mm_cooling_limit_cf =295;
					shooter_17mm_cooling_rate_ef =45;
					shooter_17mm_cooling_rate_cf =75;
				break;
				case 9:
					chassis_power = 100;
					chassis_power_hpf =90;
					shooter_17mm_cooling_limit_ef =600;
					shooter_17mm_cooling_limit_cf =330;
					shooter_17mm_cooling_rate_ef =50;
					shooter_17mm_cooling_rate_cf =80;
				break;
				case 10:
					chassis_power = 100;
					chassis_power_hpf =100;
					shooter_17mm_cooling_limit_ef =650;
					shooter_17mm_cooling_limit_cf =400;
					shooter_17mm_cooling_rate_ef =60;
					shooter_17mm_cooling_rate_cf =80;
				break;
				default:
				{
				}
				break;
			}
			//开局判断底盘功率是否为功率优先
			if ((judge_recv_mesg.game_robot_state.current_HP == 150 && level ==1)||
					(judge_recv_mesg.game_robot_state.current_HP == 175 && level ==2)||
					(judge_recv_mesg.game_robot_state.current_HP == 200 && level ==3)||
					(judge_recv_mesg.game_robot_state.current_HP == 225 && level ==4)||
					(judge_recv_mesg.game_robot_state.current_HP == 250 && level ==5)||
					(judge_recv_mesg.game_robot_state.current_HP == 275 && level ==6)||
					(judge_recv_mesg.game_robot_state.current_HP == 300 && level ==7)||
					(judge_recv_mesg.game_robot_state.current_HP == 325 && level ==8)||
					(judge_recv_mesg.game_robot_state.current_HP == 355 && level ==9)||
					(judge_recv_mesg.game_robot_state.current_HP == 400 && level ==10))
			{
				Calculate_grade.chassis_power_limit =chassis_power;
			}
			else
			{
				Calculate_grade.chassis_power_limit =chassis_power_hpf;
			}
			
			Calculate_grade.shooter_17mm_cooling_rate_EF = shooter_17mm_cooling_rate_ef;
			Calculate_grade.shooter_17mm_cooling_rate_CF = shooter_17mm_cooling_rate_cf;
			Calculate_grade.shooter_17mm_cooling_limit_EF =shooter_17mm_cooling_limit_ef;
			Calculate_grade.shooter_17mm_cooling_limit_CF =shooter_17mm_cooling_limit_cf;
			Calculate_grade.robot_level = level ;
	}
}
