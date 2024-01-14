#include "info_get_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "keyboard.h"
#include "remote_ctrl.h"
#include "bsp_can.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "imu_task.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "stdlib.h"
#include "math.h"
#include "bsp_flash.h"

UBaseType_t info_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t gimbal_Task_Handle;
extern TaskHandle_t chassis_Task_Handle;
extern TaskHandle_t shoot_Task_Handle;

extern cali_sys_t cali_param;
extern int direction;
extern int direction_change;
void info_get_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if(Signal & MODE_SWITCH_INFO_SIGNAL)
			{
        taskENTER_CRITICAL();//操作系统的临界保护函数，防止下面的操作被任务切换和中断所打断
        
        keyboard_global_hook();
        get_chassis_info();
        get_gimbal_info();
        get_shoot_info();
        get_global_last_info();
        
        taskEXIT_CRITICAL();//退出临界保护
        if(global_mode == RELEASE_CTRL)//当遥控右边打到最下时，电机全部断电
        {
				direction=1;
				gimbal.pit_center_offset = cali_param.pitch_offset;
				gimbal.yaw_center_offset = cali_param.yaw_offset;
          MEMSET(&glb_cur,motor_current_t);
          xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                              (uint32_t) MODE_SWITCH_MSG_SIGNAL, 
                              (eNotifyAction) eSetBits, 
                              (uint32_t *)NULL );
        } 
        else//正常执行云台底盘发射这3个任务
        {
          xTaskGenericNotify( (TaskHandle_t) gimbal_Task_Handle, 
                            (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
          xTaskGenericNotify( (TaskHandle_t) chassis_Task_Handle, 
                            (uint32_t) INFO_GET_CHASSIS_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
          xTaskGenericNotify( (TaskHandle_t) shoot_Task_Handle, 
                            (uint32_t) INFO_GET_SHOOT_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
        }
        
        info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      }
    }

  }
}

static void get_chassis_info(void)
{
	 /* 转换成相对角度 */
	static float wheel1_ecd_ratio = WHEEL1_MOTO_POSITIVE_DIR*WHEEL1_DECELE_RATIO/ENCODER_ANGLE_RATIO;
	static float wheel2_ecd_ratio = WHEEL2_MOTO_POSITIVE_DIR*WHEEL2_DECELE_RATIO/ENCODER_ANGLE_RATIO;
	
	chassis.wheel1_relative_angle = wheel1_ecd_ratio*get_relative_pos(moto_chassis[2].ecd, chassis.wheel1_center_offset);
	chassis.wheel2_relative_angle = wheel2_ecd_ratio*get_relative_pos(moto_chassis[3].ecd, chassis.wheel2_center_offset);
	
	if(chassis.state == CHASSIS_INIT_NEVER)
	{
		chassis.wheel_fdb[2] = chassis.wheel1_relative_angle;
		chassis.wheel_fdb[3] = chassis.wheel2_relative_angle;
	}
	else if(chassis.state == CHASSIS_INIT_DONE)
	{
		/* 得到底盘轮速 */
		for (uint8_t i = 0; i < 2; i++)
		{
			chassis.wheel_fdb[i] = moto_chassis[i].speed_rpm;
		}
		/* 得到舵轮角度 */
		chassis.wheel_fdb[2] = chassis.wheel1_relative_angle;
		chassis.wheel_fdb[3] = chassis.wheel2_relative_angle;
	}
  /* get remote and keyboard chassis control information */
  keyboard_chassis_hook();
  remote_ctrl_chassis_hook();
	
}

int debug_yaw_spd = 1;
int debug_pit_spd = -1;//左上为正是+号。左下为正为-号

static void get_gimbal_info(void)
{
  /* 转换成相对角度 */
  static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR*YAW_DECELE_RATIO/ENCODER_ANGLE_RATIO;
  static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR*PIT_DECELE_RATIO/ENCODER_ANGLE_RATIO;

  static int i = 0;
  if (direction_change == 1)
  { // 检测到换头指令
    i++;
    if (i == 1)
    {
      gimbal.pid.yaw_angle_ref += 180;
      if (gimbal.yaw_center_offset > 4096)
        gimbal.yaw_center_offset -= 4096;
      else
        gimbal.yaw_center_offset += 4096;
    }
    i = 1;
  }
  else
    i = 0;

  gimbal.sensor.yaw_relative_angle = debug_pit_spd*yaw_ecd_ratio*get_relative_pos(moto_yaw.ecd, gimbal.yaw_center_offset);
  gimbal.sensor.pit_relative_angle = debug_pit_spd*pit_ecd_ratio*get_relative_pos(moto_pit.ecd, gimbal.pit_center_offset);
  	
  /*获取遥控和键鼠的数据*/
  keyboard_gimbal_hook();
  remote_ctrl_gimbal_hook();
  
}


static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)						//分中轴线象限
  {
		if (raw_ecd - center_offset > - 4096)
      tmp = raw_ecd - center_offset;							//在右边 = 负数
    else
      tmp = raw_ecd + 8192 - center_offset;				//在左边 = 正数
  }
  else
  {
		if (raw_ecd - center_offset > 4096)
      tmp = raw_ecd - 8192 - center_offset;				//在右边 = 负数
    else
      tmp = raw_ecd - center_offset;							//在左边 = 正数
  }
  return tmp;
}

static void get_shoot_info(void)
{
  remote_ctrl_shoot_hook();
  keyboard_shoot_hook();
  
	shoot.para_mode					= SHOOTNOR_MODE; 

  
}

static void get_global_last_info(void)
{
  glb_sw.last_sw1 = rc.sw1;
  glb_sw.last_sw2 = rc.sw2; 
	glb_sw.last_iw = rc.iw;
}
