#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
//#include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "data_packet.h"
#include "arm_math.h"

/*2022赛季规则

初始状态			100 40

功率优先
        1级		150	60
        2级		200 80
        3级		250	100
血量优先
        1级		200	45
        2级		300	50
        3级		400	55

*/

UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

extern pid_t pid_chassis_vw;
extern pid_t pid_chassis_power_buffer;

chassis_t chassis;

//前三个旋转pid 后三个速度pid
#if (INFANTRY_NUM == INFANTRY_1) 

//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_2))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_3))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_4))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_5))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.5, 0.0f, 0};
#else
		#error "INFANTRY_NUM define error!"
#endif

float cap_vol_pid[3] = {1.0f, 0, 0.0f}; //超级电容电压控制PID
float chassis_power_buffer_pid[3] = {1, 0, 0};//缓冲焦耳控制PID
float chassis_vw_pid[3] = {0.1, 0, 0.1};//小陀螺转速控制PID

float cap_store=24;//电容存储大小(给定初值为25)
float cx=0,cy=0,cw=0;//缓启动赋值变量
uint8_t fast_flag = 0; //加速状态标志位
int Speed_up = 1;//加速键标志位
float cap_ratio;//电流值比例参数，根据当前电流值来调整电流输出上限

void chassis_task(void *parm)
{
  uint32_t Signal;
  BaseType_t STAUS;

  while (1)
  {
    STAUS = xTaskNotifyWait((uint32_t)NULL,
                            (uint32_t)INFO_GET_CHASSIS_SIGNAL,
                            (uint32_t *)&Signal,
                            (TickType_t)portMAX_DELAY);
    if (STAUS == pdTRUE)
    {
//		if(INFANTRY_NUM != INFANTRY_2){//由于现在步兵二号装的是新版电容组，电压可以稳定在25V，因而无需实时更新电容上限值,等全部车都换上稳压后，电容实时更新容量的函数可以删除
//			if((judge_recv_mesg.game_robot_state.mains_power_chassis_output==1&&chassis.CapData[1]>20)||chassis.CapData[1]>cap_store)
//			Cap_refresh();
//		}
			
      if (Signal & INFO_GET_CHASSIS_SIGNAL)
      {
        /*底盘vw旋转的pid*/
        PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, DONE);
        PID_Struct_Init(&pid_chassis_power_buffer, chassis_power_buffer_pid[0], chassis_power_buffer_pid[1], chassis_power_buffer_pid[2], 50, 10, DONE);
        PID_Struct_Init(&pid_chassis_vw, chassis_vw_pid[0], chassis_vw_pid[1], chassis_vw_pid[2], 500, 50, DONE);
				
				if(Speed_up&&chassis.CapData[1]!=0)
				cap_ratio=1-((cap_store-chassis.CapData[1])/(cap_store-10));
				else
				cap_ratio=1;
				
        /*底盘vx,vy平移的pid*/
        for (int i = 0; i < 4; i++)
          PID_Struct_Init(&pid_spd[i], chassis_pid[3], chassis_pid[4], chassis_pid[5], 15000*cap_ratio, 1384*cap_ratio, DONE);

        if (chassis_mode != CHASSIS_RELEASE && gimbal.state != GIMBAL_INIT_NEVER) //云台归中之后底盘才能动
        {
          switch (chassis_mode)
          {
          /*云台底盘跟随模式*/
          case CHASSIS_NORMAL_MODE:
          {
            chassis_normal_handler();
          }
          break;
          /*云台底盘分离模式*/
          case CHASSIS_SEPARATE_MODE:
          {
            chassis_separate_handler();
          }
          break;
          /*小陀螺模式*/
          case CHASSIS_DODGE_MODE:
          {
            chassis_dodge_handler();
          }
          break;
          /*底盘停止模式*/
          case CHASSIS_STOP_MODE:
          {
            chassis_stop_handler();
          }
          break;

          default:{}
          break;
          }

					if(FAST_SPD && fast_flag){
					cx=chassis.vx;
					cy=chassis.vy;
					cw=chassis.vw;
					}else{
					buffer(&cx,chassis.vx,50,100);	//缓启动	
					buffer(&cy,chassis.vy,50,100);		
					buffer(&cw,chassis.vw, 1, 1);
					}
					
          mecanum_calc(cx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
		
          for (int i = 0; i < 4; i++)
            chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);

          if (!chassis_is_controllable())
            memset(chassis.current, 0, sizeof(chassis.current));

          memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
        }
        else
          memset(glb_cur.chassis_cur, 0, sizeof(glb_cur.chassis_cur));

        xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
                           (uint32_t)CHASSIS_MOTOR_MSG_SIGNAL,
                           (eNotifyAction)eSetBits,
                           (uint32_t *)NULL);
      }
    }

    chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

void Cap_refresh(){	//不断刷新并记录当前电池所能给电容充的最大电压
//电容的最大压值随着电池格数进行变化，因而需要不断刷新
	static int i=0;
	if(judge_recv_mesg.power_heat_data.chassis_power_buffer==60&&(chassis.CapData[0]-chassis.CapData[1]<2||chassis.CapData[1]>=20)){
	if(i!=10)i++;
	}else
		i=0;
	if(i==10)cap_store=chassis.CapData[1];
	if(chassis.CapData[1]>cap_store)cap_store=chassis.CapData[1];
	if(cap_store<20.f)cap_store=20;
};



extern int Speed_up;
//chassis_power_contorl(&pid_power,&chassis_vx,&chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
void chassis_power_contorl(pid_t *power_pid,float *power_vx,float *power_vy,float *power_yaw_speed,float real_time_Cap_remain,float real_time_Cap_can_store,float judge_power_limit)
{
    static  float max = 3000;
    static float min = 1150;//1000;
    static float parameter;            // 变速系数
    static float Cap_low = 14.0f;      // 何时停止加速
    float cap_flag = real_time_Cap_can_store - 18.0f; // 18~real_time_Cap_can_store这一段进行按比例赋值
    if (real_time_Cap_remain < 15.0f)
    {
        *power_yaw_speed = (5 - (15.0f - real_time_Cap_remain)) / 5 * 0.01f;
    } // 低电容按比例减少鼠标左右移动速度
    else
    {
        *power_yaw_speed = 0.01f;
    }

    if (real_time_Cap_remain > 18.0f && real_time_Cap_remain < real_time_Cap_can_store)
    {
          min = 1350 + (cap_flag - (real_time_Cap_can_store - real_time_Cap_remain)) / cap_flag * (judge_power_limit - 45) / 45 * 500;
		}
    else
    {
        min = 1150;//1000;
    }
        // 按加速键后Speed_up = 0,以3508的最快速度进行冲刺
        if (Speed_up == 1)
        {

            if (real_time_Cap_remain == real_time_Cap_can_store)
            { // 检测到电容充满则pid进行加速
                pid_calc(power_pid, real_time_Cap_remain + 0.5f, real_time_Cap_can_store);
            }
            else
            {
							if(real_time_Cap_remain!=0)
                pid_calc(power_pid, real_time_Cap_remain, real_time_Cap_can_store);
            }
            if (power_pid->out > 0)
            {
                parameter = 1;
            } // 更改这里可分别改变提速的速率或降速的速率
            else
            {
                parameter = 1;
            }
        }
        else if (real_time_Cap_remain < Cap_low)
            Speed_up = 1; // 一旦检测到电容低则自动关闭加速

        if (km.vy > 0 || rm.vy > 0)
        {
            if (Speed_up == 0)
            {
                *power_vy = CHASSIS_KB_MAX_SPEED_Y;
            }
            else
            {
                if (*power_vy < 0)
                {
                    *power_vy = -*power_vy;
                }
                *power_vy -= parameter * power_pid->out;
                VAL_LIMIT(*power_vy, min * 0.7f, max); // 低电容平移，速度下限更低
            }
        }
        else
        {
            if (km.vy < 0 || rm.vy < 0)
            {
                if (Speed_up == 0)
                    *power_vy = -CHASSIS_KB_MAX_SPEED_Y;
                else
                {
                    if (*power_vy > 0)
                    {
                        *power_vy = -*power_vy;
                    }
                    *power_vy += parameter * power_pid->out;
                    VAL_LIMIT(*power_vy, -max, -min * 0.7f);
                }
            }
            else
                *power_vy = 0;
        }

        if (km.vx > 0 || rm.vx > 0)
        {
            if (Speed_up == 0)
            {
                *power_vx = CHASSIS_KB_MAX_SPEED_X;
            }
            else
            {
                if (*power_vx < 0)
                {
                    *power_vx = -*power_vx;
                }
                *power_vx -= parameter * power_pid->out;
                VAL_LIMIT(*power_vx, min, max);
            }
        }
        else
        {
            if (km.vx < 0 || rm.vx < 0)
            {
                if (Speed_up == 0)
                    *power_vx = -CHASSIS_KB_MAX_SPEED_X;
                else
                {
                    if (*power_vx > 0)
                    {
                        *power_vx = -*power_vx;
                    }
                    *power_vx += parameter * power_pid->out;
                    VAL_LIMIT(*power_vx, -max, -min);
                }
            }
            else
            {
                *power_vx = 0;
            }
        }

    if (SLOW_SPD) // 慢速
    {
        if (*power_vx > 0)
            *power_vx = 1000;
        if (*power_vx < 0)
            *power_vx = -1000;
        if (*power_vy > 0)
            *power_vy = 1000;
        if (*power_vy < 0)
            *power_vy = -1000;
    }
    else if (FAST_SPD && fast_flag) // 快速
    {
        Speed_up = 0;
    }
    else // 正常速度
    {
        Speed_up = 1;
    }

    if (real_time_Cap_remain > Cap_low && !FAST_SPD)
    {
        fast_flag = 1;
    } // 防止加速过程中反复开关加速模式（抽搐）
    if (real_time_Cap_remain < Cap_low)
    {
        fast_flag = 0;
    }
}

static void chassis_normal_handler(void)
{
  float nor_chassis_vx,nor_chassis_vy;
  //vx,vy,yaw_speed power control
  chassis_power_contorl(&pid_power,&nor_chassis_vx,&nor_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
  
  int position_ref = 0;
  if (chassis_mode == CHASSIS_NORMAL_MODE)
  {
    if (input_flag == 1) 
		{chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));}
    else
		{chassis.vw = 0;}
  }
  chassis.vx=nor_chassis_vx;
  chassis.vy=nor_chassis_vy;
}


static void chassis_separate_handler(void)
{
  float cha_x, cha_y, separ_angle;
  separ_angle = gimbal.sensor.yaw_relative_angle;
  cha_x = (rm.vx + km.vx);
  cha_y = (rm.vy + km.vy);
  chassis.vy = (cha_y * arm_cos_f32(-PI / 180 * separ_angle) - cha_x * arm_sin_f32(-PI / 180 * separ_angle));
  chassis.vx = (cha_x * arm_cos_f32(-PI / 180 * separ_angle) + cha_y * arm_sin_f32(-PI / 180 * separ_angle));
  chassis.vw = -pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, 45);
}

static void chassis_dodge_handler(void)
{
	float chassis_vx_dodge,chassis_vy_dodge;
    float dodge_angle;
	float dodge_min=150;
	float dodge_max=500;
	float dodge_chassis_vx,dodge_chassis_vy;
	static int dodge_cap=0;
	
	if(last_chassis_mode!=CHASSIS_DODGE_MODE){
	chassis.vw=dodge_min+((float)judge_recv_mesg.game_robot_state.chassis_power_limit-45)/45*150;
	
	if(cap_store-chassis.CapData[1]<0.5)dodge_cap=chassis.CapData[1]-0.5;//防止充放平衡
		else 
			dodge_cap=chassis.CapData[1];
		
	}
	
  /*小陀螺*/
  dodge_angle = gimbal.sensor.yaw_relative_angle;

  if ((rm.vx != 0 || rm.vy != 0) &&(km.vx == 0 || km.vy == 0))//小陀螺时遥控方向降低速度
  {
    dodge_chassis_vx = rm.vx * 0.5f;
    dodge_chassis_vy = rm.vy * 0.5f;
  }
  else
		//键盘时则通过功率算法来控制速度，防止掉电容
		//vx,vy,yaw_speed power control
    	chassis_power_contorl(&pid_power,&dodge_chassis_vx,&dodge_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32( PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32( PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32( PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32( PI / 180 * dodge_angle));


  pid_calc(&pid_chassis_vw, chassis.CapData[1], dodge_cap);

  chassis.vw -= pid_chassis_vw.out;
  if (chassis.vw < dodge_min)
    chassis.vw = dodge_min;
  if (chassis.vw > dodge_max)
    chassis.vw = dodge_max;

}



static void chassis_stop_handler(void)
{
  for (int i = 0; i < 4; i++)
  {
    chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], 0);
  }

  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}

void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis));

  /*底盘vw旋转的pid*/
  PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, INIT);
  PID_Struct_Init(&pid_power, cap_vol_pid[0], cap_vol_pid[1], cap_vol_pid[2], 100, 20, INIT);
  /*底盘vx,vy平移的pid*/
  for (int i = 0; i < 4; i++)
    PID_Struct_Init(&pid_spd[i], chassis_pid[3], chassis_pid[4], chassis_pid[5], 10000, 500, INIT);
}

/**
 * @brief mecanum chassis velocity decomposition
 * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
 *        output: every wheel speed(rpm)
 * @trans 输入：		前后左右的量
 *				 输出：		每个轮子对应的速度
 * @note  1=FR 2=FL 3=BL 4=BR
 * @work	 分析演算公式计算的效率
 */
int rotation_center_gimbal = 0;
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;

  taskENTER_CRITICAL();
  if (chassis_mode == CHASSIS_DODGE_MODE || chassis.dodge_ctrl)
  {
    chassis.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis.rotate_y_offset = 0;
  }
  else
  {
    if (rotation_center_gimbal)
    {
      chassis.rotate_x_offset = glb_struct.gimbal_x_offset;
      chassis.rotate_y_offset = glb_struct.gimbal_y_offset;
    }
    else
    {
      chassis.rotate_x_offset = 0;
      chassis.rotate_y_offset = 0;
    }
  }
  //@works
  rotate_ratio_fr = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f - chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_fl = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f - chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_bl = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f + chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_br = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f + chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;

  wheel_rpm_ratio = 60.0f / (glb_struct.wheel_perimeter * CHASSIS_DECELE_RATIO);
  taskEXIT_CRITICAL();

  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
  /*小陀螺以外的模式，vw限制正常*/
  if (chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED); // deg/s
  /*小陀螺时，vw不受限制*/
  //	else
  //		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

  int16_t wheel_rpm[4];
  float max = 0;

  wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = (vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = (vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

  // find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  // equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)

      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}


void buffer(float *a, float b, float high_parameter, float low_parameter)
{

  if (((*a - b) < high_parameter) && ((*a - b) > -low_parameter))
  {
    *a = b;
  }
  else
  {
    if (*a < b)
      *a += high_parameter;
    if (*a > b)
      *a -= low_parameter;
  }
}
