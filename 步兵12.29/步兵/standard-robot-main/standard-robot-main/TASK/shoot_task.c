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

#include "Exp_Calculate_grade.h"

UBaseType_t shoot_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern gimbal_t gimbal;

static float shooter_17mm_speed_limit = 30 ;//默认射速固定为30        新增
shoot_t   shoot;
trigger_t trig;
uint16_t normal_speed;


  
  #if(INFANTRY_NUM == INFANTRY_3)
  /*摩擦轮pid*/
  float fric_pid[3] = {24, 0, 0};
	/* 摩擦轮转速 */
	uint16_t speed_15 =	4080;//4450，3800
	uint16_t speed_18 = 4650;//4430
	uint16_t speed_30 = 8300;//8200

	  /*弹仓盖开关*/
	float ccr_open  = 500;
  float ccr_close = 2350;
  #elif(INFANTRY_NUM == INFANTRY_4)
  /*摩擦轮pid*/
  float fric_pid[3] = {24, 0, 0};
	/* 摩擦轮转速 */
	uint16_t speed_15 =	4000;//4450，3800
	uint16_t speed_18 = 4500;//4430
	uint16_t speed_30 = 8000;//8200

	  /*弹仓盖开关*/
	float ccr_open  = 500;
  float ccr_close = 2350;
    #elif(INFANTRY_NUM == INFANTRY_5)
  /*摩擦轮pid*/
  float fric_pid[3] = {24, 0, 0};
	/* 摩擦轮转速 */
	uint16_t speed_15 =	8000;//4450，3800//新规则转速拉满了
	uint16_t speed_18 = 8000;//4430
	uint16_t speed_30 = 8000;//8200

	  /*弹仓盖开关*/
	float ccr_open  = 1550;
  float ccr_close = 430;
#else
		#error "INFANTRY_NUM define error!"
#endif

	int close_down  = 1;     //弹仓盖关闭完成标志位
	int open_down   = 1;      //弹仓盖打开完成标志位
	
	/* 拨盘转速 */
	int normal_cshoot				= 1000 ;
	int lspd;
	int rspd;	
	
	/*顺序：普通-基地-吊射-buff*/
	/*热量限制拨盘转速*/
  float heat_limit_pid[3] = {30, 0, 10};
	/*热量限制连发时间间隔*/
	float heat_time_pid[3]  = {9, 0, 0};//{3, 0 ,3};
	
	
	//动态射速pid
	float speedlimit[3]={10,0,1};
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
	
	
int shooter_17mm_cooling_rate;  //17毫米弹丸默认冷却速度按等级来划分		
int shooter_17mm_cooling_limit; //17毫米弹丸的热量限制	
void shooter_17mm_cooling_rate_choose(void)//新规则没有给出冷却值，要按等级来算
{				//17mm弹丸冷却速率爆发优先
	if((Calculate_grade.robot_level == 1&&Calculate_grade.shooter_17mm_cooling_rate_EF ==10) || 
		(Calculate_grade.robot_level == 2 && Calculate_grade.shooter_17mm_cooling_rate_EF ==15)||
		(Calculate_grade.robot_level == 3 && Calculate_grade.shooter_17mm_cooling_rate_EF ==20)||
		(Calculate_grade.robot_level == 4 && Calculate_grade.shooter_17mm_cooling_rate_EF ==25)||
		(Calculate_grade.robot_level == 5 && Calculate_grade.shooter_17mm_cooling_rate_EF ==30)||
		(Calculate_grade.robot_level == 6 && Calculate_grade.shooter_17mm_cooling_rate_EF ==35)||
		(Calculate_grade.robot_level == 7 && Calculate_grade.shooter_17mm_cooling_rate_EF ==40)||
		(Calculate_grade.robot_level == 8 && Calculate_grade.shooter_17mm_cooling_rate_EF ==45)||
		(Calculate_grade.robot_level == 9 && Calculate_grade.shooter_17mm_cooling_rate_EF ==50)||
		(Calculate_grade.robot_level == 10&& Calculate_grade.shooter_17mm_cooling_rate_EF ==60))
	{		
		shooter_17mm_cooling_rate = Calculate_grade.shooter_17mm_cooling_rate_EF;
		shooter_17mm_cooling_limit= Calculate_grade.shooter_17mm_cooling_limit_EF;
	}
	else
	{  //冷却优先
		shooter_17mm_cooling_rate = Calculate_grade.shooter_17mm_cooling_rate_CF;
		shooter_17mm_cooling_limit= Calculate_grade.shooter_17mm_cooling_limit_CF;
	}
	
}
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
                    
					continue_time = 1000/((shooter_17mm_cooling_rate*judge_recv_mesg.buff.cooling_buff/10));//根据枪口冷却速率算出连发时间间隔	
					surplus_heat = shooter_17mm_cooling_limit - judge_recv_mesg.power_heat_data.shooter_17mm_1_barrel_heat;//剩余热量
					radio_freq = (10000/(shooter_17mm_cooling_limit*judge_recv_mesg.buff.cooling_buff))-7;//根据热量上限算出允许的最小连发时间间隔，也就是最大射频
					
          /*热量控制*/
          PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, DONE );
					/*连发间隔时间控制*/
          PID_Struct_Init(&pid_heat_time, (1000/((shooter_17mm_cooling_rate*judge_recv_mesg.buff.cooling_buff)/10)-40)*0.01-heat_time_p, heat_time_pid[1], heat_time_pid[2], 
					1000/(shooter_17mm_cooling_rate*judge_recv_mesg.buff.cooling_buff/10)-radio_freq, 0, DONE );//原数据heat_time_p1,radio_freq40
					pid_calc(&pid_heat_time,surplus_heat,settime);//0

					if((float)shooter_17mm_speed_limit-1-judge_recv_mesg.shoot_data.initial_speed>5){
						speedlimit[0]=300;
						speedlimit[1]=0;
						speedlimit[2]=1;
					}else{
					  speedlimit[0]=10;
						speedlimit[1]=0;
						speedlimit[2]=1;
					}
				
					PID_Struct_Init(&pid_speedlimit, speedlimit[0], speedlimit[1], speedlimit[2], 7000, 0, DONE );
					
          /*摩擦轮*/
          for(int i=0;i<2;i++)
          {
            PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, DONE ); 
          }


					
					
          shoot_para_ctrl();						// 射击模式切换
          ball_storage_ctrl();					// 舵机控制弹仓盖
          fric_wheel_ctrl();						// 启动摩擦轮
          
          if (shoot.fric_wheel_run)//若摩擦轮开启
          {
            shoot_bullet_handler();      
          }
          else
          {
            shoot.shoot_cmd   = 0;//单发标志位
            shoot.c_shoot_cmd = 0;//连发标志位
            shoot.fric_wheel_spd = 1000;//摩擦轮转速
            pid_trigger_spd.out = 0;//拨盘
            trig.angle_ref = moto_trigger.total_angle; //记录当前拨盘电机编码位
						trig.one_sta = TRIG_INIT;//保证下次开启摩擦轮时，单发状态为初始化
          }
          
        }
        else
        {
          pid_trigger_spd.out = 0;
          shoot.fric_wheel_spd = 0;
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



void get_last_shoot_mode(void)
{
	shoot.last_para_mode = shoot.para_mode;
}


/*射击模式选择
* @ SHOOTBUFF_MODE 神符模式				 //好像要删掉没看到有什么用
*	@ SHOOTTHROW_MODE 高射速吊射模式  //没看到
*	@ SHOOTMAD_MODE 低射速近战模式		 //没看到
* @ SHOOTNOR_MODE 普通射速模式
**/

static void shoot_para_ctrl(void)
{

	normal_speed = speed_30;

if(shoot.para_mode == SHOOTBUFF_MODE) 
		shoot.fric_wheel_spd = speed_30;
  else	
		shoot.fric_wheel_spd = normal_speed;
	
	/* 拨盘转速 */
	trig.shoot_spd			 = 3000;							//拨弹盘转速没调不知道要不要调
	trig.c_shoot_spd		 = normal_cshoot;
}

/*摩擦轮控制*/
static void fric_wheel_ctrl(void)
{
	if (shoot.fric_wheel_run)
	{
		turn_on_friction_wheel(shoot.fric_wheel_spd, shoot.fric_wheel_spd);
	}
	else
	{
		turn_off_friction_wheel();
	}
}


/*打开摩擦轮*/
static void turn_on_friction_wheel(int16_t lspd,int16_t rspd)
{
		pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, -lspd);
		pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, rspd);
		glb_cur.fric_cur[0] = pid_fric[0].out;
		glb_cur.fric_cur[1] = pid_fric[1].out;
}

/*关闭摩擦轮*/
static void turn_off_friction_wheel(void)
{
	pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, 0);
	pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, 0);
	glb_cur.fric_cur[0] = pid_fric[0].out;
	glb_cur.fric_cur[1] = pid_fric[1].out;
}


/*弹仓盖控制*/
static void ball_storage_ctrl(void)
{
  if (shoot.ball_storage_open)
  {
    TIM_SetCompare3(TIM8,ccr_open);
  }
  else
  {
    TIM_SetCompare3(TIM8,ccr_close);
  }
}

int Angle=45;
static void shoot_bullet_handler(void)
{	
  if (shoot.shoot_cmd)//单发
  {
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//没连接裁判系统时
			single_time = 100;
		else
			single_time = 1000/(shooter_17mm_cooling_rate/10 + Calculate_grade.robot_level);
  
        shoot_delay = single_time;
		if (trig.one_sta == TRIG_INIT)
		{
			trig.one_time = HAL_GetTick();
			single_shoot_angle = moto_trigger.total_angle + Angle;//发射（single_shoot_angle设为当前角度加上Angle）
			trig.one_sta = TRIG_PRESS_DOWN;//发射按键已按下
    }
		/* 发射延时 */
    else if (trig.one_sta == TRIG_PRESS_DOWN)//若发射按键已按下便进行延时
    {			
        if(RC_VISION_SINGLE_SHOOT)
        {
        shoot_delay = 1000;
        }
      if (HAL_GetTick() - trig.one_time >= shoot_delay)
      {
        trig.one_sta = TRIG_ONE_DONE;//延时完毕后，状态才为发射已完成
      }
    }
     
    if (trig.one_sta == TRIG_ONE_DONE)//若发射状态为已完成
    {
		single_shoot_angle = moto_trigger.total_angle;
        shoot.shoot_cmd = 0;//单发标志位置零
		trig.one_sta = TRIG_INIT;//单发状态设为初始化
		trig.c_sta = TRIG_INIT;//连发状态设为初始化
      shoot.shoot_bullets++;//发射子弹计数
    }
		trig.angle_ref = single_shoot_angle;//拨盘目标角度设为single_shoot_angle
		
  }
	
	else if (shoot.c_shoot_cmd)//连发做多次单发
  {
        //计算延时时间
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//没连接裁判系统时
		{
            if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* 发射处理 */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= 200)  //延时
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* 发射完成 */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//发射子弹计数
			}
			trig.angle_ref = single_shoot_angle;
     }
        else if(shooter_17mm_speed_limit == 30)//射速优先									不知道这一段要不要删新规则已经固定射速了//
		{
			if((shooter_17mm_cooling_limit-judge_recv_mesg.power_heat_data.shooter_17mm_1_barrel_heat) > 25)
				shoot_delay = 500/(shooter_17mm_cooling_rate/10+Calculate_grade.robot_level);
			else
				shoot_delay = 500/(shooter_17mm_cooling_rate/10);
				if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* 发射处理 */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= shoot_delay)  //延时
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* 发射完成 */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//发射子弹计数
			}
			trig.angle_ref = single_shoot_angle;
     }
        
		 else if((Calculate_grade.robot_level == 1&& shooter_17mm_cooling_rate ==40)|| 
				(Calculate_grade.robot_level == 2 && shooter_17mm_cooling_rate ==45)||
				(Calculate_grade.robot_level == 3 && shooter_17mm_cooling_rate ==50)||
				(Calculate_grade.robot_level == 4 && shooter_17mm_cooling_rate ==55)||
				(Calculate_grade.robot_level == 5 && shooter_17mm_cooling_rate ==60)||
				(Calculate_grade.robot_level == 6 && shooter_17mm_cooling_rate ==75)||
				(Calculate_grade.robot_level == 7 && shooter_17mm_cooling_rate ==70)||
				(Calculate_grade.robot_level == 8 && shooter_17mm_cooling_rate ==75)||
				(Calculate_grade.robot_level == 9 && shooter_17mm_cooling_rate ==80)||
				(Calculate_grade.robot_level == 10&& shooter_17mm_cooling_rate ==80))//冷却优先
		{
			//这个是以相对固定的射频进行射击     		//2024没改
			if((judge_recv_mesg.shoot_data.launching_frequency < (shooter_17mm_cooling_rate/10 + Calculate_grade.robot_level*2)) && surplus_heat>20)
				shoot_delay = 500/((shooter_17mm_cooling_rate/10)+Calculate_grade.robot_level)+25;
			else
			{
				shoot_delay = 500/((shooter_17mm_cooling_rate/10)-Calculate_grade.robot_level);

			}

			if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* 发射处理 */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= shoot_delay)  //延时
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* 发射完成 */
			if (trig.c_sta == TRIG_ONE_DONE)//若发射状态为已完成
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//发射子弹计数
			}
			trig.angle_ref = single_shoot_angle;
				}
		else//爆发优先
    {
			extra_time = pid_heat_time.out;//小于0
			
			if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* 发射处理 */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= continue_time + extra_time)  //延时
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* 发射完成 */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
//				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//发射子弹计数

			}
			trig.angle_ref = single_shoot_angle;
		}
	}
	
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.power_heat_data.shooter_17mm_1_barrel_heat 
		>= (shooter_17mm_cooling_limit-10)))
		trig.angle_ref = moto_trigger.total_angle;
	
	if(trig.angle_ref%45!=0){//解决二连发问题
	int i=trig.angle_ref%45;
		if(i>=25)trig.angle_ref+=45-i;
		else trig.angle_ref-=i;
	}
   //pid计算
  pid_calc(&pid_trigger,moto_trigger.total_angle,trig.angle_ref);
  trig.spd_ref = pid_trigger.out;//
	pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
	
    
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && judge_recv_mesg.game_robot_state.power_management_shooter_output == 0)//摩擦轮被裁判系统断电后让拨盘停转
		pid_trigger_spd.out = 0;
	//防止裁判系统异常出现超热量的情况
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (shooter_17mm_cooling_rate == 0 || shooter_17mm_cooling_limit == 0))
		pid_trigger_spd.out = 0;
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

}


