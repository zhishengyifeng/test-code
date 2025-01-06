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
UBaseType_t can_stack_surplus;
motor_current_t glb_cur;

extern pid_t pid_chassis_power_buffer;
extern uint8_t Speed_up;
void can_msg_send_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL  | \
                                       CHASSIS_MOTOR_MSG_SIGNAL | \
                                       SHOT_MOTOR_MSG_SIGNAL  | \
                                       MODE_SWITCH_MSG_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)//���ݲ���ϵͳѡ����ư幦�����
		{

			pid_calc(&pid_chassis_power_buffer,judge_recv_mesg.power_heat_data.chassis_power_buffer,60-5);//60����������µĻ��役�������5���Ը���ʵ���������
			//��������������55J��ԭ�����ڿ��ư��޷���׼���ƹ��ʣ����в����𵴣�������55J�������̶ȵ������ⲿ�ֱ��˷ѵĹ���
		
			//if(pid_chassis_power_buffer.out>0)pid_chassis_power_buffer.out=0;

			//����˵������30Wȥ��ֹ�����ʣ����˹��ʿ��ư��޲�ס�����
			
			//�˴�����Ϊ�˷�ֹ���ʿ��ư��޲�ס�����ı���
			if(judge_recv_mesg.power_heat_data.chassis_power_buffer<=10)pid_chassis_power_buffer.out=judge_recv_mesg.game_robot_state.chassis_power_limit;
			
			send_cap_power_can((judge_recv_mesg.game_robot_state.chassis_power_limit-pid_chassis_power_buffer.out)*100);//�������ʿ��ư��ֵ
		}else
			send_cap_power_can(3900);//û�������ϵͳ�����¼ģʽ��ʱ�����39W

		
			if(Signal & GIMBAL_MOTOR_MSG_SIGNAL)//������̨����
      send_gimbal_cur(glb_cur.gimbal_cur[1],glb_cur.gimbal_cur[0], glb_cur.gimbal_cur[2]);

      if(Signal & CHASSIS_MOTOR_MSG_SIGNAL)//���͵��̵���
			{
//				send_chassis_cur(glb_cur.chassis_cur[0],glb_cur.chassis_cur[1],0,0);
//				send_chassis_turn_cur(glb_cur.chassis_cur[2],glb_cur.chassis_cur[3]);
			}

      if(Signal & SHOT_MOTOR_MSG_SIGNAL)//����Ħ���ֵ���
//				send_fric_cur(glb_cur.fric_cur[0],glb_cur.fric_cur[1]);
      
      if(Signal & MODE_SWITCH_MSG_SIGNAL) //���͵���ȫΪ0
      {
        send_gimbal_cur(glb_cur.gimbal_cur[1],glb_cur.gimbal_cur[0], glb_cur.gimbal_cur[2]);
        send_chassis_cur(glb_cur.chassis_cur[0],glb_cur.chassis_cur[1],glb_cur.chassis_cur[2],glb_cur.chassis_cur[3]);
        send_fric_cur(glb_cur.fric_cur[0],glb_cur.fric_cur[1]);
      }
		}
    can_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

