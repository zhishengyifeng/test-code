#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "detect_task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "judge_rx_data.h"

UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;
extern uint8_t twist_doge;
global_status global_mode;
global_status last_global_mode;

gimbal_status gimbal_mode;
gimbal_status last_gimbal_mode;

chassis_status chassis_mode;
chassis_status last_chassis_mode;

shoot_status shoot_mode;
shoot_status last_shoot_mode;

uint16_t time1;
rc_status rc_bit;
int keyboard_flag = 1;
int KB_FRIC = 0;
int KB_BALL = 0;
int FRIC_SPD_UP = 0;
int FRIC_SPD_DOWN = 0;
int SOFT_RESET = 0;
int SMALL_BUFF = 0;
int BIG_BUFF = 0;
int PC_DODGE = 0;
char Flag = 1;
char Flag_Done = 0;
float yaw_set = 0.0f;
float pit_set = 0.0f; // �����������

void get_keyboard(void)
{

	if (rc.kb.bit.Q && keyboard_flag)
		KB_FRIC = 1;
	if (rc.kb.bit.Q && rc.kb.bit.CTRL)
		KB_FRIC = 0;

	if (rc.kb.bit.R && keyboard_flag)
		KB_BALL = 1;
	if (rc.kb.bit.R && rc.kb.bit.SHIFT)
		KB_BALL = 0;

	if (rc.kb.bit.B && keyboard_flag)
		SMALL_BUFF = 1;
	if (rc.kb.bit.B && rc.kb.bit.SHIFT)
		SMALL_BUFF = 0;

	if (rc.kb.bit.G && keyboard_flag)
		BIG_BUFF = 1;
	if (rc.kb.bit.G && rc.kb.bit.SHIFT)
		BIG_BUFF = 0;

	if (rc.kb.bit.E && keyboard_flag)
		PC_DODGE = 1;
	if (rc.kb.bit.E && rc.kb.bit.SHIFT)
		PC_DODGE = 0;

	if (rc.kb.bit.Z && rc.kb.bit.CTRL)
		FRIC_SPD_UP = 1;
	if (rc.kb.bit.X && rc.kb.bit.CTRL)
		FRIC_SPD_DOWN = 1;

	if (rc.kb.bit.F && rc.kb.bit.CTRL)
		SOFT_RESET = 1;
	if (
		!rc.kb.bit.Q &&
		!rc.kb.bit.R &&
		!rc.kb.bit.B &&
		!rc.kb.bit.G &&
		!rc.kb.bit.E &&
		!rc.kb.bit.SHIFT &&
		!rc.kb.bit.CTRL &&
		!rc.kb.bit.Z &&
		!rc.kb.bit.X &&
		!rc.kb.bit.F)
		keyboard_flag = 1;
	else
		keyboard_flag = 0; // ���ֺ��ִ��
}

rc_status remote_get_bit(void)
{
	if (rc.kb.bit.W)
		return RC_W;
	if (rc.kb.bit.A)
		return RC_A;
	if (rc.kb.bit.S)
		return RC_S;
	if (rc.kb.bit.D)
		return RC_D;
	else
		return RC_NUN;
}

void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
	while (1)
	{
		get_last_mode();
		get_main_mode();
		get_gimbal_mode();
		get_chassis_mode();
		get_shoot_mode();
		get_keyboard();

		/*********��buff����*****************************/
		if (gimbal_mode == GIMBAL_SHOOT_BUFF)
		{
			rc_bit = remote_get_bit();
			if (Flag == 1)
			{
				if (rc_bit != RC_NUN)
				{
					switch (rc_bit)
					{
					case RC_W:
						pit_set = 0.2f;
						break;
					case RC_A:
						yaw_set = -0.2f;
						break;
					case RC_S:
						pit_set = -0.2f;
						break;
					case RC_D:
						yaw_set = 0.2f;
						break;
					default:
						break;
					}
					Flag_Done = 1;
					time1 = HAL_GetTick();
					Flag = 0;
				}
			}
			else if (Flag == 0 && HAL_GetTick() - time1 > 200) // �������ʱ�� 200ms
			{
				Flag = 1;
			}
		}
		/************************************************/

		xTaskGenericNotify((TaskHandle_t)info_get_Task_Handle,
						   (uint32_t)MODE_SWITCH_INFO_SIGNAL,
						   (eNotifyAction)eSetBits,
						   (uint32_t *)NULL);

		mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&mode_switch_wake_time, 2);
	}
}
void get_last_mode(void)
{
	last_global_mode = global_mode;	  // ��ȡ��һ��ȫ��״̬
	last_chassis_mode = chassis_mode; // ��ȡ��һ�ε���״̬
	last_shoot_mode = shoot_mode;	  // ��ȡ��һ�����״̬
}

void get_main_mode(void)
{
	if (global_err.list[REMOTE_CTRL_OFFLINE].err_exist == 1 ||
		global_err.list[GIMBAL_YAW_OFFLINE].err_exist == 1 ||
		global_err.list[GIMBAL_PIT_OFFLINE].err_exist == 1)
	{
		global_mode = RELEASE_CTRL;
		ramp_init(&pit_ramp, 1000);
		ramp_init(&yaw_ramp, 1000);
	}
	else
	{
		switch (rc.sw2)
		{
		case RC_UP:
		{
			if (TRACK_CTRL || RC_TRACK_ARMOR_MODE) // �����Ӿ�
				global_mode = SEMI_AUTOMATIC_CTRL;
			else
				global_mode = MANUAL_CTRL;
		}
		break;
		case RC_MI:
		{
			if (TRACK_CTRL) // �����Ӿ�
				global_mode = SEMI_AUTOMATIC_CTRL;
			else
				global_mode = MANUAL_CTRL;
		}
		break;
		case RC_DN:
		{
			global_mode = RELEASE_CTRL;
			ramp_init(&pit_ramp, 1000);
			ramp_init(&yaw_ramp, 1000);
		}
		break;
		default:
		{
		}
		break;
		}
	}
}

void get_gimbal_mode(void)
{
	//  if(KB_CHASSIS_STOP)
	//		gimbal.chassis_stop = 1;
	//	if(KB_CHASSIS_RECOVER)
	//		gimbal.chassis_stop = 0;
	//
	//	if(KB_SEPARATE_MODE)
	//		gimbal.separate_ctrl = 1;
	//	if(KB_SEPARATE_CLOSE)
	//		gimbal.separate_ctrl = 0;

	switch (global_mode)
	{
	case MANUAL_CTRL:
	{
		if (chassis_mode == CHASSIS_DODGE_MODE)
			gimbal_mode = GIMBAL_DODGE_MODE;
		else
			gimbal_mode = GIMBAL_NORMAL_MODE; // ��̨���̸���ģʽ

		/*��С�����ģʽѡ��*/
		if (SMALL_BUFF)
			gimbal.small_buff_ctrl = 1;
		else
			gimbal.small_buff_ctrl = 0;

		if (BIG_BUFF)
			gimbal.big_buff_ctrl = 1;
		else
			gimbal.big_buff_ctrl = 0;

		if (gimbal.small_buff_ctrl || gimbal.big_buff_ctrl)
			gimbal_mode = GIMBAL_SHOOT_BUFF; // ��buffģʽ
	}
	break;

	case SEMI_AUTOMATIC_CTRL:
	{
		gimbal_mode = GIMBAL_TRACK_ARMOR; // ��̨����ģʽ
										  // gimbal_mode = GIMBAL_SHOOT_BUFF;  //����ң�ص������������л���δ���
	}
	break;

	case FULL_AUTOMATIC_CTRL:
		gimbal_mode = GIMBAL_NORMAL_MODE;
		break;

	case RELEASE_CTRL:
	{
		gimbal_mode = GIMBAL_RELEASE;
		gimbal.state = GIMBAL_INIT_NEVER;
	}
	break;
	default:
	{
		gimbal_mode = GIMBAL_RELEASE;
		gimbal.state = GIMBAL_INIT_NEVER;
	}
	break;
	}
}

void get_chassis_mode(void)
{
	if (judge_recv_mesg.game_robot_state.mains_power_chassis_output == 0 || chassis_mode == CHASSIS_RELEASE) // ��ֹ����С����
		PC_DODGE = 0;

	if (chassis.CapData[1] < 14.5f) // ���ݵ�ѹ�͹ر�С����,�����ѹ��13-14.5V
		PC_DODGE = 0;

	if (PC_DODGE)
		chassis.dodge_ctrl = 1;
	else
	{
		chassis.dodge_ctrl = 0;
		gimbal.separate_ctrl = 0;
	}

	switch (global_mode)
	{
	case RELEASE_CTRL:
		chassis_mode = CHASSIS_RELEASE;

		break;
	/*******6.29�߼�����*******/
	case MANUAL_CTRL:
	{

		if (gimbal_mode == GIMBAL_SHOOT_BUFF)
			chassis_mode = CHASSIS_STOP_MODE;
		else if ((RC_DODGE_MODE || chassis.dodge_ctrl)) // ����������ģʽ�����̽���С����&&twist_doge
			chassis_mode = CHASSIS_DODGE_MODE;
		else // ������̨����ģʽ
			chassis_mode = CHASSIS_NORMAL_MODE;
	}
	break;

	case SEMI_AUTOMATIC_CTRL:
	{
		//      if(gimbal.chassis_stop)
		//				chassis_mode = CHASSIS_STOP_MODE;
		if (gimbal_mode == GIMBAL_SHOOT_BUFF) // ��������buff������ֹͣ�˶�
			chassis_mode = CHASSIS_STOP_MODE;
		else if (chassis.dodge_ctrl) // ����������飬�ҿ������ģʽ�����̽���С����
			chassis_mode = CHASSIS_DODGE_MODE;
		else // ������̨����ģʽ
			chassis_mode = CHASSIS_NORMAL_MODE;
	}
	break;

	case FULL_AUTOMATIC_CTRL:
	{
		chassis_mode = CHASSIS_NORMAL_MODE;
	}
	break;

	default:
	{
	}
	break;
	}
}

void get_shoot_mode(void)
{
	switch (global_mode)
	{
	case MANUAL_CTRL:
	case SEMI_AUTOMATIC_CTRL:
	case FULL_AUTOMATIC_CTRL:
	{
		shoot_mode = SHOOT_ENABLE;
	}
	break;

	case RELEASE_CTRL:
		shoot_mode = SHOOT_DISABLE;
		break;

	default:
	{
		shoot_mode = SHOOT_DISABLE;
	}
	break;
	}
}
