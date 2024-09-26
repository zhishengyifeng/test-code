#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "string.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h" 
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "pc_rx_data.h"
#include "rc.h"
#include "gimbal_task.h"
#include "math.h"
#include "remote_ctrl.h"

uint16_t shoot_speed = 40;//无裁判系统射频，1000/shoot_speed = 每秒发弹量

float debug1,debug2;

UBaseType_t shoot_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern gimbal_t gimbal;

shoot_t   shoot;
trigger_t trig;
uint8_t last_fric_wheel_run;
uint16_t normal_speed;
uint16_t Fric_Spd_Ajt;//3为初始未使用状态
int CHECK;

	/*
		RMUC2024规则
														枪管
		爆发优先EF 									冷却优先CF
			热量				冷却值							热量			冷却值
			 200				10								50				40
			 250				15								85				45
			 300				20								120				50
			 350				25								155				55
			 400				30								190				60
			 450				35								225				65
			 500				40								260				70
			 550				45								295				75
			 600				50								330				80
			 650				60								400				80
	*/		


#define SPEED_LIMIT//

#if (INFANTRY_CLASS == INFANTRY_MECANNUM)
/*摩擦轮pid*/
float fric_pid[3] = {24, 0, 0};

// 拨盘参数
float trig_pid[6] = {250, 0, 30, 10, 0, 10};
/* 摩擦轮转速 */
uint16_t speed = 6000;//8200
/*弹仓盖开关*/
float ccr_open  = 500;
float ccr_close = 2350;
/*拨叶叶树，每次转过角度*/
#define shifter_fork_number 8 // 拨叶数量

#elif (INFANTRY_CLASS == INFANTRY_OMV)
/*摩擦轮pid*/
float fric_pid[3] = {24, 0, 0};

// 拨盘参数
float trig_pid[6] = {250, 0, 30, 10, 0, 10};

/* 摩擦轮转速 */
int16_t speed = 6500;//8200

///*弹仓盖开关*/
float ccr_open  = 2500;//1550;
float ccr_close = 500;//430;

/*拨叶叶树，每次转过角度*/
#define shifter_fork_number 8 // 拨叶数量

//uint32_t Shit_Count; // 由于屎山不得不继续拉 记录转过圈数

#else
	#error "INFANTRY_CLASS define error!"
#endif

float Angle = 360.0f / (float)shifter_fork_number;//计算2006转过角度

	int close_down  = 1;     //弹仓盖关闭完成标志位
	int open_down   = 1;      //弹仓盖打开完成标志位
	int lspd;
	int rspd;	
	
/*顺序：普通-基地-吊射-buff*/
/*热量限制拨盘转速*/
float heat_limit_pid[3] = {30, 0, 10};
/*热量限制连发时间间隔*/
float heat_time_pid[3]  = {9, 0, 0};//{3, 0 ,3};
	
	
//动态射速pid
float speedlimit[3]={5,0,0};
float last_shoot_speed=0;//记录上一颗子弹的射速，作为PID输入值
	
int settime=5;//剩余热量的阈值
float	heat_time_p=1,radio_freq=40;
float single_shoot_angle;
uint16_t single_time = 100; //单发延时时间间隔初始值
uint16_t continue_time = 420;//连发延时时间间隔初始值
float surplus_heat;   //剩余热量
float extra_time; 
float shoot_delay;//发射延迟 
uint8_t switch_freq = 1;//射频转换标志
	
void shoot_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  
	while(1)
	{
		STAUS = xTaskNotifyWait((uint32_t) NULL, 
								(uint32_t) INFO_GET_SHOOT_SIGNAL, 
								(uint32_t *)&Signal, 
								(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_SHOOT_SIGNAL)
			{
				if(shoot_mode != SHOOT_DISABLE)
				{
					continue_time = 1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10);//根据枪口冷却速率算出连发时间间隔	
					surplus_heat = judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - judge_recv_mesg.power_heat_data.shooter_17mm_1_barrel_heat;//剩余热量
					radio_freq = (10000/judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit)-7;//根据热量上限算出允许的最小连发时间间隔，也就是最大射频

					/*热量控制*/
					PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, DONE );
					
					/*连发间隔时间控制*/
					PID_Struct_Init(&pid_heat_time, (1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10)-40)*0.01-heat_time_p, heat_time_pid[1], heat_time_pid[2], 
					1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10)-radio_freq, 0, DONE );//原数据heat_time_p1,radio_freq40
					pid_calc(&pid_heat_time,surplus_heat,settime);//0

					/*摩擦轮*/
					for(int i=0;i<2;i++)
					{
						PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, DONE ); 
					}
				  PID_Struct_Init(&pid_Spd_limit, speedlimit[0], speedlimit[1], speedlimit[2], 8000, 500, DONE );
					/* 拨盘 电机的PID参数 */
					PID_Struct_Init(&pid_trigger, trig_pid[0], trig_pid[1], trig_pid[2], 16000, 0, DONE);
					PID_Struct_Init(&pid_trigger_spd, trig_pid[3], trig_pid[4], trig_pid[5], 16000, 3000, DONE);

					if (shoot.fric_wheel_run)//若摩擦轮开启
					{
						
						pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, -speed);
						pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, speed);
						#ifndef SPEED_LIMIT
						
						#else 
						float newspeed = 0;
						if(last_fric_wheel_run)
					  newspeed = speed_limit();
						pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, -speed-newspeed);
						pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, speed+newspeed);
						#endif
						
						shoot_bullet_handler();  //拨盘控制   
						debug1 = moto_fric[0].speed_rpm;
						debug2 = moto_fric[1].speed_rpm;
						last_fric_wheel_run = 1;
					}
					else
					{
						single_shoot_angle = moto_trigger.total_angle;
						shoot.shoot_cmd   = 0;//单发标志位
						shoot.c_shoot_cmd = 0;//连发标志位
						shoot.fric_wheel_spd = 0;//给小值 eg：1000 摩擦轮持续加热
						pid_trigger_spd.out = 0;//拨盘
						trig.angle_ref = moto_trigger.total_angle; //记录当前拨盘电机编码位
						trig.one_sta = TRIG_INIT;//保证下次开启摩擦轮时，单发状态为初始化

						pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, 0);
						pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, 0);
						
					}
					glb_cur.fric_cur[0] = pid_fric[0].out;//摩擦轮电流值赋值
					glb_cur.fric_cur[1] = pid_fric[1].out;
					glb_cur.trigger_cur = pid_trigger_spd.out;//拨盘电流赋值
				}
				else
				{
					single_shoot_angle = moto_trigger.total_angle;
					glb_cur.trigger_cur = 0;
					glb_cur.fric_cur[0] = 0;
					glb_cur.fric_cur[1] = 0;
				}
				
								
				xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
									(uint32_t) SHOT_MOTOR_MSG_SIGNAL, 
									(eNotifyAction) eSetBits, 
									(uint32_t *)NULL );
			}
		}
		shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}

} 


/* 好像找不到有调用到地方 */
void get_last_shoot_mode(void)
{
	shoot.last_para_mode = shoot.para_mode;
}


/*射击模式选择
* @ SHOOTBUFF_MODE 神符模式
*	@ SHOOTTHROW_MODE 高射速吊射模式
*	@ SHOOTMAD_MODE 低射速近战模式
* @ SHOOTNOR_MODE 普通射速模式
**/

/*没搞懂多套一层干嘛*/
//static void shoot_para_ctrl(void)
//{
//	shoot.fric_wheel_spd = speed;	
//}

/*摩擦轮控制*/
//static void fric_wheel_ctrl(void)
//{
//	if (shoot.fric_wheel_run)
//		turn_on_friction_wheel(speed);
//	else
//		turn_off_friction_wheel();
//	
//	glb_cur.fric_cur[0] = pid_fric[0].out;
//	glb_cur.fric_cur[1] = pid_fric[1].out;
//	
//}


///*打开摩擦轮*/
//static void turn_on_friction_wheel(int16_t spd)
//{
//	pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, -spd);
//	pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, spd);
////	glb_cur.fric_cur[0] = pid_fric[0].out;
////	glb_cur.fric_cur[1] = pid_fric[1].out;
//}

///*关闭摩擦轮*/
//static void turn_off_friction_wheel(int16_t spd)
//{
//	pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, -spd);
//	pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, spd);
////	glb_cur.fric_cur[0] = pid_fric[0].out;
////	glb_cur.fric_cur[1] = pid_fric[1].out;
//}

/*弹仓盖控制*/
//static void ball_storage_ctrl(void)
//{
//  if (shoot.ball_storage_open)
//  {
//    TIM_SetCompare3(TIM8,ccr_open);
//  }
//  else
//  {
//    TIM_SetCompare3(TIM8,ccr_close);
//  }
//}
/**
 * @brief		拨盘角度变换，解决二连发问题，解决使用绝对值卡弹问题
 * @param[in]	state:该参数传入单发或者连发两种之一的状态
 * @param[in]	shoot_delay:该参数传入当前这一发子弹的间隔时间，单位ms
 * @retval		none
 */
static void shoot_delay_hanlder(trig_state_e *state, float shoot_delay)
{
	static int16_t stop_time;
	static uint8_t ngtv_flag;
	if (*state == TRIG_INIT)
	{
		trig.one_time = HAL_GetTick();
		/* 卡弹角度锁定不正常则不自增角度 */
		if (fabs(moto_trigger.total_angle - single_shoot_angle) <= 10)
		{
			stop_time = 0;
			if(ngtv_flag)
			{
				ngtv_flag = 0;
				single_shoot_angle += 15; // 发射（single_shoot_angle设为当前角度加上Angle）				
			}
			else
				single_shoot_angle += Angle; // 发射（single_shoot_angle设为当前角度加上Angle）
			*state = TRIG_PRESS_DOWN;	 // 发射按键已按下
		}
		else if(stop_time == 300)
		{
			single_shoot_angle -= (Angle + 15);
			ngtv_flag = 1;
			stop_time++;
			CHECK++;
		}
		else
		{
			if(KEY_GetFlag())
				stop_time++;
		}
	}
	/* 发射延时 */
	else if (*state == TRIG_PRESS_DOWN) // 若发射按键已按下便进行延时
	{
		if (HAL_GetTick() - trig.one_time >= shoot_delay)
		{
			*state = TRIG_ONE_DONE; // 延时完毕后，状态才为发射已完成
		}
	}
	if (*state == TRIG_ONE_DONE) // 若发射状态为已完成
	{
		if (*state == trig.one_sta) // 单发模式下重置单发状态
		{
			shoot.shoot_cmd = 0;	  // 单发标志位置零
			trig.one_sta = TRIG_INIT; // 单发状态设为初始化
		}
		// 连发和单发都必须重置连发状态
		trig.c_sta = TRIG_INIT; // 连发状态设为初始化
		shoot.shoot_bullets++;	// 发射子弹计数
	}
	trig.angle_ref = single_shoot_angle; // 拨盘目标角度设为single_shoot_angle
}

static void shoot_bullet_handler(void)
{

	if (shoot.shoot_cmd)//单发
	{
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//没连接裁判系统时
			single_time = shoot_speed;
		else
			single_time = 500/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10 + judge_recv_mesg.game_robot_state.robot_level);
  
		shoot_delay_hanlder(&trig.one_sta, single_time);
		
  }
	
	else if (shoot.c_shoot_cmd && !gimbal.big_buff_ctrl && !gimbal.small_buff_ctrl)//连发做多次单发，如果在打符模式不连发
	{
        //计算延时时间
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//没连接裁判系统时
		{
			shoot_delay_hanlder(&trig.c_sta, shoot_speed);
        }
        else if ((judge_recv_mesg.game_robot_state.robot_level== 1 &&judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value  ==40)|| 
			      		(judge_recv_mesg.game_robot_state.robot_level == 2 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==45)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 3 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==50)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 4 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==55)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 5 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==60)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 6 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==75)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 7 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==70)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 8 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==75)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 9 && judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==80)||
			      		(judge_recv_mesg.game_robot_state.robot_level == 10&& judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value ==80))//冷却优先
		{
			//这个是以相对固定的射频进行射击
			if((judge_recv_mesg.shoot_data.launching_frequency < (judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10 + judge_recv_mesg.game_robot_state.robot_level*2)) && surplus_heat>20)
				shoot_delay = 500/((judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10)+judge_recv_mesg.game_robot_state.robot_level)+25;
			else
				shoot_delay = 500/((judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10)-judge_recv_mesg.game_robot_state.robot_level);
			shoot_delay_hanlder(&trig.c_sta, shoot_delay);
		}
		else//爆发优先
		{
			extra_time = pid_heat_time.out;//小于0
			shoot_delay_hanlder(&trig.c_sta, continue_time + extra_time);
		}
	}
	else
	{
		trig.angle_ref = single_shoot_angle; // 拨盘目标角度设为single_shoot_angle
	}
	
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.power_heat_data.shooter_17mm_1_barrel_heat 
		>= (judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit-10)))
		trig.angle_ref = single_shoot_angle; // 拨盘目标角度设为single_shoot_angle
	
   //pid计算
  pid_calc(&pid_trigger,moto_trigger.total_angle,trig.angle_ref);
	
	    trig.spd_ref = pid_trigger.out;
	pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && judge_recv_mesg.game_robot_state.power_management_shooter_output == 0)//摩擦轮被裁判系统断电后让拨盘停转
		pid_trigger_spd.out = 0;
	//防止裁判系统异常出现超热量的情况
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value == 0 || judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit == 0))
		pid_trigger_spd.out = 0;
	glb_cur.trigger_cur = pid_trigger_spd.out;
	
}

float speed_limit()
{
	float expect_speed = 22.0f;
	float limit_speed = pid_calc(&pid_Spd_limit,judge_recv_mesg.shoot_data.initial_speed,expect_speed);
	return limit_speed;
}
//初始化
void shoot_param_init(void)
{
	memset(&shoot, 0, sizeof(shoot_t));

	shoot.ctrl_mode      = SHOT_DISABLE;
	shoot.para_mode			 = SHOOTNOR_MODE;
	memset(&trig, 0, sizeof(trigger_t));

	trig.shoot_spd			 = 0;
	trig.c_shoot_spd     = 0;
	trig.one_sta         = TRIG_INIT;
	shoot.shoot_bullets = 0;
  
	/*热量控制*/
	PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, INIT );
	/*热量控制时间间隔*/
	PID_Struct_Init(&pid_heat_time, heat_time_pid[0], heat_time_pid[1], heat_time_pid[2], 400, 0, INIT );
  
	/*摩擦轮*/
	for(int i=0;i<2;i++)
	{
	PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, INIT ); 
	}
	/* 拨盘 电机的PID参数 */
	PID_Struct_Init(&pid_trigger, trig_pid[0], trig_pid[1], trig_pid[2], 8000, 0, DONE);
	PID_Struct_Init(&pid_trigger_spd, trig_pid[3], trig_pid[4], trig_pid[5], 8000, 3000, DONE);

}


