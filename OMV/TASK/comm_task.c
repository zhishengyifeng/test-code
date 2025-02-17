#include "comm_task.h"
#include "STM32_TIM_BASE.h"
#include "judge_rx_data.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "judge_task.h"
#include "bsp_can.h"
#include "modeswitch_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "sys_config.h"
#include "pid.h"
#include "keyboard.h"

//外部调试
#include "bsp_vofa.h"
#include "stdio.h"

UBaseType_t can_stack_surplus;
motor_current_t glb_cur;

extern pid_t pid_chassis_power_buffer;
extern uint8_t Speed_up;
void can_msg_send_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)GIMBAL_MOTOR_MSG_SIGNAL |CHASSIS_MOTOR_MSG_SIGNAL |SHOT_MOTOR_MSG_SIGNAL |MODE_SWITCH_MSG_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			#ifndef POWER_NEW
					if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)//根据裁判系统选择控制板功率输出
					{

						pid_calc(&pid_chassis_power_buffer,judge_recv_mesg.power_heat_data.buffer_energy,60-5);//60是正常情况下的缓冲焦耳，这个5可以根据实际情况调节
						// 其余情况则控制在55J（原因在于控制板无法精准控制功率，会有波动震荡，控制在55J可以最大程度的利用这部分被浪费的功率

						if(pid_chassis_power_buffer.out>0)
							pid_chassis_power_buffer.out=0;

						// 不用说控制在30W去防止超功率，除了功率控制板限不住的情况

						// 此处就是为了防止功率控制板限不住所作的保险
						if(judge_recv_mesg.power_heat_data.buffer_energy<=10)
							pid_chassis_power_buffer.out=judge_recv_mesg.game_robot_state.chassis_power_limit;
			
						send_cap_power_can((judge_recv_mesg.game_robot_state.chassis_power_limit-pid_chassis_power_buffer.out)*100);//发给功率控制板的值
					}
					else
						send_cap_power_can(5000);//没接入裁判系统或则检录模式的时候给定50w
			#endif

			if (Signal & GIMBAL_MOTOR_MSG_SIGNAL) // 发送云台电流
			{
				send_gimbal_cur(glb_cur.gimbal_cur[1], glb_cur.gimbal_cur[0]);
				#ifdef DM_MOTOR_PITCH
					send_dm_cur(glb_cur.gimbal_cur[1]);
				#endif
			}
			if (Signal & CHASSIS_MOTOR_MSG_SIGNAL) // 发送底盘电流
			send_chassis_cur(glb_cur.chassis_cur[0], glb_cur.chassis_cur[1], glb_cur.chassis_cur[2], glb_cur.chassis_cur[3]);

			if (Signal & SHOT_MOTOR_MSG_SIGNAL) // 发送发射机构电流
				send_fric_cur(glb_cur.fric_cur[0], glb_cur.fric_cur[1], glb_cur.trigger_cur);

			if (Signal & MODE_SWITCH_MSG_SIGNAL) // 发送电流全为0
			{
				send_gimbal_cur(glb_cur.gimbal_cur[1], glb_cur.gimbal_cur[0]);
				#ifdef DM_MOTOR_PITCH
					send_dm_cur(glb_cur.gimbal_cur[1]);
				#endif
				send_chassis_cur(glb_cur.chassis_cur[0], glb_cur.chassis_cur[1], glb_cur.chassis_cur[2], glb_cur.chassis_cur[3]);
				send_fric_cur(glb_cur.fric_cur[0], glb_cur.fric_cur[1], glb_cur.trigger_cur);
			}
		}
		can_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

	}
}
