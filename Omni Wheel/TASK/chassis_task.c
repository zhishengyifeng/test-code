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
// #include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "data_packet.h"
#include "arm_math.h"
#include "power_control.h"
float a1, b1, a2, b2, a3, b3, a4, b4;
float ob_total_power;
/*2023��������

��ʼ״̬			100 40

��������
		1��		150 60
		2��		200 80
		3��		250 100
Ѫ������
		1��		200 45
		2��		300 50
		3��		400 55

*/

UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

extern pid_t pid_chassis_vw;
extern pid_t pid_chassis_power_buffer;

chassis_t chassis;

// ǰ������תpid �������ٶ�pid

////////////////////
#if (INFANTRY_NUM == INFANTRY_4)
float chassis_pid[3] = {10.0f, 0.0, 0.0f};
float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},
							   {10.5, 0.0f, 0},
							   {10.5, 0.0f, 0},
							   {10.5, 0.0f, 0}};

#elif (INFANTRY_NUM == INFANTRY_5)
float chassis_pid[3] = {10.0f, 0.0, 0.0f};
//float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},\
//								{10, 0.0f, 0},\
//								{10, 0.0f, 0},\
//								{10, 0.0f, 0}};
float chassis_spd_pid[4][3] = {{10, 0.0f, 0},
							   {10, 0.0f, 0},
							   {10, 0.0f, 0},
							   {10, 0.0f, 0}};
#elif (INFANTRY_NUM == INFANTRY_24)
float chassis_pid[3] = {10.0f, 0.0, 0.0f};
//float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},\
//								{10, 0.0f, 0},\
//								{10, 0.0f, 0},\
//								{10, 0.0f, 0}};
float chassis_spd_pid[4][3] = {{10, 0.0f, 0},
							   {10, 0.0f, 0},
							   {10, 0.0f, 0},
							   {10, 0.0f, 0}};
#else
#error "INFANTRY_NUM define error!"
#endif

#define POWER_NEW//�¹����㷨����Ҫ���Ϲ����㷨��ע�͵���һ��
								 

float	power_new_pid[3] = {0.0f, 2.0f, 0.0f}; //�¹����㷨PID�������±ߵ�cap_vol_pid��
float cap_vol_pid[3] = {1.0f, 0, 0.0f};		   // �������ݵ�ѹ����PID
float chassis_power_buffer_pid[3] = {1, 0, 0}; // ���役������PID
#ifndef POWER_NEW
float chassis_vw_pid[3] = {0.1, 0, 0.1};	   // С����ת�ٿ���PID
#else
float chassis_vw_pid[3] = {0.0f, 0.3f, 0.0f};	   // С����ת�ٿ���PID
#endif

float cap_store = 24;		  // ���ݴ洢��С(������ֵΪ24)
float cx = 0, cy = 0, cw = 0; // ��������ֵ����
uint8_t fast_flag = 0;		  // ����״̬��־λ
int Speed_up = 0;			  // ���ټ���־λ

float Vw = 0;						//ʵ��ת��

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

			if (Signal & INFO_GET_CHASSIS_SIGNAL)
			{
				/*����vw��ת��pid*/
				PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, DONE);
				PID_Struct_Init(&pid_chassis_power_buffer, chassis_power_buffer_pid[0], chassis_power_buffer_pid[1], chassis_power_buffer_pid[2], 50, 10, DONE);
				PID_Struct_Init(&pid_chassis_vw, chassis_vw_pid[0], chassis_vw_pid[1], chassis_vw_pid[2], 600, 600, DONE);
				
				/*����vx,vyƽ�Ƶ�pid*/
				for (int i = 0; i < 4; i++)
					PID_Struct_Init(&pid_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 16000, 1500, DONE);

				if (chassis_mode != CHASSIS_RELEASE && gimbal.state != GIMBAL_INIT_NEVER) // ��̨����֮����̲��ܶ�
				{
					Cap_refresh();
					switch (chassis_mode)
					{
					/*��̨���̸���ģʽ*/
					case CHASSIS_NORMAL_MODE:
					{
						chassis_normal_handler();
					}
					break;
					/*С����ģʽ*/
					case CHASSIS_DODGE_MODE:
					{
						chassis_dodge_handler();
					}
					break;
					/*����ֹͣģʽ*/
					case CHASSIS_STOP_MODE:
					{
						chassis_stop_handler();
					}
					break;

					default:
					{
					}
					break;
					}

					if (FAST_SPD && fast_flag)
					{
						cx = chassis.vx;
						cy = chassis.vy;
						cw = chassis.vw;
					}
					else
					{
						buffer(&cx, chassis.vx, 50, 100); // ������
						buffer(&cy, chassis.vy, 50, 100);
						buffer(&cw, chassis.vw, 1, 1);
					}

					// �������������ݴ�����㣨ǰ�������������Լ�С���ݲ����л�������
					mecanum_calc(cx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);

					for (int i = 0; i < 4; i++)
						chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);

					ob_total_power = Chassis_Power_Control(&chassis); // ���ʿ���

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

void Cap_refresh()
{
	// ����ˢ�²���¼��ǰ������ܸ����ݳ������ѹ
	// ���ݵ����ѹֵ���ŵ�ظ������б仯�������Ҫ����ˢ��
	static int i = 0;
	if (judge_recv_mesg.power_heat_data.buffer_energy == 60 && (chassis.CapData[0] - chassis.CapData[1] < 2 || chassis.CapData[1] >= 20))
	{
		if (i != 10)
			i++;
	}
	else
		i = 0;
	if (i == 10)
		cap_store = chassis.CapData[1];
	if (chassis.CapData[1] > cap_store)
		cap_store = chassis.CapData[1];
	if (cap_store < 20.f)
		cap_store = 20;
};

extern int Speed_up;
#ifndef POWER_NEW
//�Ϲ����㷨--����pid
// ���̹��ʿ��ƺ�����ͨ��������ֵ�����ݵ�ǰֵ�����̹�������ֵ�����������x��y���ϵ��ٶ�
void chassis_power_contorl(pid_t *power_pid, float *power_vx, float *power_vy, float *power_yaw_speed, float real_time_Cap_remain, float real_time_Cap_can_store, float judge_power_limit)
{
	static float max = 3000;
	static float min = 1150;						  // 1000;
	static float parameter;							  // ����ϵ��
	static float Cap_low = 14.0f;					  // ��ʱֹͣ����
	float cap_flag = real_time_Cap_can_store - 18.0f; // 18~real_time_Cap_can_store��һ�ν��а�������ֵ
	if (real_time_Cap_remain < 15.0f)
	{
		*power_yaw_speed = (5 - (15.0f - real_time_Cap_remain)) / 5 * 0.01f;
	} // �͵��ݰ�����������������ƶ��ٶ�
	else
	{
		*power_yaw_speed = 0.01f;
	}

	if (real_time_Cap_remain > 18.0f && real_time_Cap_remain < real_time_Cap_can_store) // ��ǰ������18��������֮��
	{																					// ����   ͨ������ó�����x���y�����С�ٶ�
		min = 1350 + (cap_flag - (real_time_Cap_can_store - real_time_Cap_remain)) / cap_flag * (judge_power_limit - 45) / 45 * 500;
	}
	else
	{				// ������С��18ʱֱ�������
		min = 1150; // 1000;
	}

	// �����ټ���Speed_up = 0,��3508������ٶȽ��г�̣�Speed_up = 1��δ�����ټ�
	if (Speed_up == 1)
	{
		if (real_time_Cap_remain == real_time_Cap_can_store) // ���ݳ���
		{
			pid_calc(power_pid, real_time_Cap_remain + 0.5f, real_time_Cap_can_store);
		}
		else
		{
			if (real_time_Cap_remain != 0)
				pid_calc(power_pid, real_time_Cap_remain, real_time_Cap_can_store);
		}
	}
	else if (real_time_Cap_remain < Cap_low)
		Speed_up = 1; // һ����⵽���ݵ����Զ��رռ���

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
			VAL_LIMIT(*power_vy, min * 0.7f, max); // �͵���ƽ�ƣ��ٶ����޸���
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

	if (SLOW_SPD) // ����
	{
		if (*power_vx > 0)
			*power_vx = 1000;
		else if (*power_vx < 0)
			*power_vx = -1000;
		
		if (*power_vy > 0)
			*power_vy = 1000;
		else if (*power_vy < 0)
			*power_vy = -1000;
	}
	else if (fast_flag && (rc.ch2 == 660 || FAST_SPD)) // ����
	{
		Speed_up = 0;
	}
	else // �����ٶ�
	{
		Speed_up = 1;
	}

	if (real_time_Cap_remain > Cap_low && !FAST_SPD)
	{
		fast_flag = 1;
	} // ��ֹ���ٹ����з������ؼ���ģʽ���鴤��
	if (real_time_Cap_remain < Cap_low)
	{
		fast_flag = 0;
	}
}

// С����ģʽ
static void chassis_dodge_handler(void)
{
	float chassis_vx_dodge, chassis_vy_dodge;
	float dodge_angle;
	float dodge_min = 150;
	float dodge_max = 500;
	float dodge_chassis_vx, dodge_chassis_vy;
	static int dodge_cap = 0;

	if (last_chassis_mode != CHASSIS_DODGE_MODE)
	{
		/* �ոս���С����ʱ����ת�ٶ�ͨ������õ� */
		chassis.vw = dodge_min + ((float)judge_recv_mesg.game_robot_state.chassis_power_limit - 45) / 45 * 150;

		if (cap_store - chassis.CapData[1] < 0.5)
			dodge_cap = chassis.CapData[1] - 0.5; // ��ֹ���ƽ��
		else
			dodge_cap = chassis.CapData[1];
	}

	/*С����*/
	dodge_angle = gimbal.sensor.yaw_relative_angle;
	//
	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // С����ʱң�ط��򽵵��ٶ�
	{
		dodge_chassis_vx = rm.vx * 0.5f;
		dodge_chassis_vy = rm.vy * 0.5f;
	}
	else
		// ����ʱ��ͨ�������㷨�������ٶȣ���ֹ������
		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

	pid_calc(&pid_chassis_vw, chassis.CapData[1], dodge_cap);

	chassis.vw -= pid_chassis_vw.out;
	if (chassis.vw < dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw > dodge_max)
		chassis.vw = dodge_max;
}

#else
//�¹����㷨
void chassis_power_contorl(pid_t *power_pid,float *power_vx,float *power_vy,float *power_yaw_speed,float real_time_Cap_remain,float real_time_Cap_can_store,float judge_power_limit)
{
    static  float max = 3300;
    static float min = 1150;//1000;
    static float Cap_low = 11.0f;      // ��ʱֹͣ����
		float Ref_temp = 0;//���ʻ�������ʱ����

		if (real_time_Cap_remain < 15.0f)
		{
			*power_yaw_speed = (5 - (15.0f - real_time_Cap_remain)) / 5 * 0.01f;
		} // �͵��ݰ�����������������ƶ��ٶ�
		else
		{
			*power_yaw_speed = 0.01f;
		}


		if (real_time_Cap_remain < Cap_low)
				Speed_up = 1; // һ����⵽���ݵ����Զ��رռ���
		
		if(!SLOW_SPD)
		{
			Charge_factor = 1.0f;
		}
		else//���ٸ����̳��
		{
			Charge_factor = 0.6f;// 1-0.6=40% �ĵ��������ݳ��
		}
		
		if(km.vy != 0 || rm.vy != 0 || km.vx != 0 || rm.vx != 0)//�������벻Ϊ0,�Ž���pid���㣨��ֹ���̾�ֹʱ��pid->out������
		{
				if (Speed_up == 1)//������				
				{
					if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
					{
						if((judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor - ob_total_power > Deta_Power)//����������
						{
							Ref_temp = ob_total_power + Deta_Power;							
							pid_calc(power_pid,ob_total_power,Ref_temp);
						}
						else
							pid_calc(power_pid,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
					}
					else//û�Ӳ���ϵͳ
						pid_calc(power_pid,ob_total_power,(Debug_Power+5)*Charge_factor);//û�����Ӳ���ϵͳ��С�������������ʴﵽ45w			
				}
				else if (Speed_up == 0)//���¼��ٹ��ʶ�100w(�������ȼ����ڼ���)
				{
					if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
					{
							pid_calc(power_pid,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit+105));
					}
					else//û�Ӳ���ϵͳ
					{
							pid_calc(power_pid,ob_total_power,105+Debug_Power);//û�����Ӳ���ϵͳ��С�������������ʴﵽ150w
					}
				}		
			
		}

		if (km.vy > 0 || rm.vy > 0)
		{
			 *power_vy = (min + power_pid->out);
			 VAL_LIMIT(*power_vy, min * 0.7f, max);
		}
		else if (km.vy < 0 || rm.vy < 0)
		{
			 *power_vy = -(min + power_pid->out);
			 VAL_LIMIT(*power_vy, -max, -min * 0.7f);
		}
		else
			*power_vy = 0;

		if (km.vx > 0 || rm.vx > 0)
		{
				*power_vx = (min + power_pid->out);
			 VAL_LIMIT(*power_vx, min, max);
		}
		else if (km.vx < 0 || rm.vx < 0)
		{
			*power_vx = -(min + power_pid->out);
			VAL_LIMIT(*power_vx, -max, -min);
		}
		else
			*power_vx = 0;
		
		
	if (fast_flag && (rc.ch2 == 660 || FAST_SPD)) // ����
	{
		Speed_up = 0;
	}
	else // �����ٶ�
	{
		Speed_up = 1;
	}

	if ((real_time_Cap_remain > Cap_low )&& (!FAST_SPD))
	{
		fast_flag = 1;
	} // ��ֹ���ٹ����з������ؼ���ģʽ���鴤��
	if (real_time_Cap_remain < Cap_low)
	{
		fast_flag = 0;
	}
}

// С����ģʽ
static void chassis_dodge_handler(void)
{
	float chassis_vx_dodge, chassis_vy_dodge;
	float dodge_angle;
	float dodge_min = 150;
	float dodge_max = 600;
	float dodge_chassis_vx, dodge_chassis_vy;
	static int dodge_cap = 0;

	if (last_chassis_mode != CHASSIS_DODGE_MODE)
	{
		PID_Clear(&pid_chassis_vw);
	}

	/*С����*/
	dodge_angle = gimbal.sensor.yaw_relative_angle;
	//
	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // С����ʱң�ط��򽵵��ٶ�
	{
		dodge_chassis_vx = rm.vx * 0.5f;
		dodge_chassis_vy = rm.vy * 0.5f;
	}
	else
		// ����ʱ��ͨ�������㷨�������ٶȣ���ֹ������
		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

	if (Speed_up == 1)//������				
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
		}
		else//û�Ӳ���ϵͳ
			pid_calc(&pid_chassis_vw,ob_total_power,(Debug_Power+5)*Charge_factor);//û�����Ӳ���ϵͳ��С�������������ʴﵽ45w
	}
	else if (Speed_up == 0)//���¼��ٹ��ʶ�100w
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit+105));
		}
		else//û�Ӳ���ϵͳ
		{
			pid_calc(&pid_chassis_vw,ob_total_power,105+Debug_Power);//û�����Ӳ���ϵͳ��С�������������ʴﵽ150w
		}
	}	

	chassis.vw = dodge_min+pid_chassis_vw.out;
	if (chassis.vw < dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw > dodge_max)
		chassis.vw = dodge_max;
}
#endif

// ��ͨģʽ
static void chassis_normal_handler(void)
{
	float nor_chassis_vx, nor_chassis_vy;
	// �������x��y���ϵ��ٶ�
	chassis_power_contorl(&pid_power, &nor_chassis_vx, &nor_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	int position_ref = 0;
	if (chassis_mode == CHASSIS_NORMAL_MODE)
	{
		if (input_flag == 1)
		// w�᷽�������̨��
		{
			chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));
		}
		else
		{
			chassis.vw = 0;
		}
	}
	chassis.vx = nor_chassis_vx;
	chassis.vy = nor_chassis_vy;
}


// ֹͣģʽ
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
// ��ʼ��
void chassis_param_init(void)
{
	memset(&chassis, 0, sizeof(chassis));

	/*����vw��ת��pid*/
	PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, INIT);
	#ifndef POWER_NEW
	PID_Struct_Init(&pid_power, cap_vol_pid[0], cap_vol_pid[1], cap_vol_pid[2], 100, 20, INIT);//�Ϲ����㷨
	#else
  PID_Struct_Init(&pid_power, power_new_pid[0], power_new_pid[1], power_new_pid[2], 3000, 2000, INIT);//�¹����㷨
	#endif
	/*����vx,vyƽ�Ƶ�pid*/
	/////////////////
	for (int i = 0; i < 4; i++)
		PID_Struct_Init(&pid_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 10000, 500, INIT);
	////////////////
}

/**
 * @brief mecanum chassis velocity decomposition
 * @param input : ��=+vx(mm/s)  ��=+vy(mm/s)  ccw=+vw(deg/s)
 *        output: every wheel speed(rpm)
 * @trans ���룺		ǰ�����ҵ���
 *				 �����		ÿ�����Ӷ�Ӧ���ٶ�
 * @note  1=FR 2=FL 3=BL 4=BR
 * @work	 �������㹫ʽ�����Ч��
 */
int rotation_center_gimbal = 0;
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]) // ���̽��㣬�õ����̻����Ӧ�ٶ���Ҫ���ĸ����ֵ
{
	static float rotate_ratio_fr;
	static float rotate_ratio_fl;
	static float rotate_ratio_bl;
	static float rotate_ratio_br;
	static float wheel_rpm_ratio;

	taskENTER_CRITICAL();
	//@work
	rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO);
	taskEXIT_CRITICAL();

	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
	VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
	/*С���������ģʽ��vw��������*/
	if (chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED); // deg/s
	/*С����ʱ��vw��������*/
	//	else
	//		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

	int16_t wheel_rpm[4];
	float max = 0;

	
	wheel_rpm[0] = (0.707*(-vx - vy) - (vw * rotate_ratio_fr)) * wheel_rpm_ratio;
	wheel_rpm[1] = (0.707*(vx - vy) - (vw * rotate_ratio_fl)) * wheel_rpm_ratio;
	wheel_rpm[2] = (0.707*(vx + vy) - (vw * rotate_ratio_bl)) * wheel_rpm_ratio;
	wheel_rpm[3] = (0.707*(-vx + vy) - (vw * rotate_ratio_br)) * wheel_rpm_ratio;
	
	//���˶�ѧ����
	float k = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	float k2 = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO);
	Vw = -((wheel_rpm[0]+wheel_rpm[2])/(2*k*k2));
	

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

// ����������
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
