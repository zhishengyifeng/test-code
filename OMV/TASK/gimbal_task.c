#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "delay.h"
#include "bsp_dwt.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()函数
#include "math.h"	//fabs()函数
#include "kalman_filter.h"
#include "filter.h"
#include "chassis_task.h"

#include "ladrc.h"
//外部调试
#include "bsp_vofa.h"
#include "stdio.h"

UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;
extern u8 INS_Init_Done;

int direction_change = 0;//换头检测用

ramp_t pit_ramp;
ramp_t yaw_ramp;

//LADRC_NUM Vision_angle;


/*调双环PID时先调内环，赋值1可断外环，调内环*/
uint8_t PID_Inner_Parameters;

#if (INFANTRY_CLASS == INFANTRY_MECANNUM)
{
////电机方向
// float pit_dir = 1.f;
// float yaw_dir = 1.f;
//  归中参数
float pit_init_pid[6] = {55, 0.01, 30, 50, 0, 0};
float yaw_init_pid[6] = {50, 0, 20, 90, 0, 5};

// 普通参数（小陀螺与普通模式共用一套参数）
float pit_pid[6] = {55, 0.01, 60, 40, 0, 0};
float yaw_pid[6] = {20, 0, 2, 90, 0, 5};

// 自瞄参数
float pit_vision_pid[6] = {18, 0.01, 5, 100, 0, 0};
float yaw_vision_pid[6] = {23, 0, 5, 75, 0, 8};

// 神符参数
float pit_buff_pid[6] = {30, 0.1, 1, 30, 0, 0};//{27, 0.13, 5, 100, 0.1, 0};
float yaw_buff_pid[6] = {30 , 0.2, 20, 60, 0, 0};
}
#elif (INFANTRY_CLASS == INFANTRY_OMV)

#ifdef DM_MOTOR_PITCH
//  归中参数
float pit_init_pid[6] = {0.5, 0.01f, 1, 7, 0, 0};//IMU反馈
float yaw_init_pid[6] = { 0.2f, 0, 0, 15000, 0, 0};//{ 0.8f, 0.0f, 0.5f, 30000, 0, 0};//{0.25f, 0, 0.5f, 40000, 0, 0};

// 普通参数（小陀螺与普通模式共用一套参数）
float pit_pid[6] = {0.5, 0.01f, 1, 7, 0, 0};//IMU反馈
float yaw_pid[6] = { 0.2f, 0, 0, 15000, 0, 0};//{ 0.8f, 0.0f, 0.5f, 30000, 0, 0};//{0.25f, 0, 0.5f, 40000, 0, 0};
// 自瞄参数
float pit_vision_pid[6] = {0.8f, 0.05f, 0.3f, 6, 0, 0};
float yaw_vision_pid[6] = { 3, 0, 30, 3000, 0, 0};// {0.2f, 0.0f, 0.0f, 20000, 0, 0};

// 神符参数
float pit_buff_pid[6] = {0.8f, 0.05f, 0.3f, 6, 0, 0};
float yaw_buff_pid[6] = {0.1f, 0.0f, 0.0f, 10000, 0, 0};
#else
//  归中参数
float pit_init_pid[6] = {0.3f, 0, 0, 12000, 0, 0};
float yaw_init_pid[6] = {0.2f, 0, 0, 15000, 0, 0};  

// 普通参数（小陀螺与普通模式共用一套参数）
float pit_pid[6] = {0.3f, 0.001f, 0.2f, 15000, 0, 0};//IMU反馈
float yaw_pid[6] = { 0.2f, 0, 0, 15000, 0, 0};//{ 0.8f, 0.0f, 0.5f, 30000, 0, 0};//{0.25f, 0, 0.5f, 40000, 0, 0};

// 自瞄参数
float pit_vision_pid[6] = {0.3f, 0.001f, 0.2f, 15000, 0, 0};//IMU反馈
float yaw_vision_pid[6] = { 3, 0, 20, 3000, 0, 0};// {0.2f, 0.0f, 0.0f, 20000, 0, 0};

// 神符参数
float pit_buff_pid[6] = {0.3f, 0.001f, 0, 12000, 0, 0};
float yaw_buff_pid[6] = {0.1f, 0.0f, 0.0f, 10000, 0, 0};
#endif

#elif (INFANTRY_CLASS == Test_Shoot)
{
////电机方向
// float pit_dir = 1.0f;
// float yaw_dir = 1.0f;
//  归中参数
float pit_init_pid[6] = {0, 0, 0, 0, 0, 0};
float yaw_init_pid[6] = {0, 0, 0, 0, 0, 0};

// 普通参数（小陀螺与普通模式共用一套参数）
float pit_pid[6] = {0, 0, 0, 0, 0, 0};
float yaw_pid[6] = {0, 0, 0, 0, 0, 0};


// 自瞄参数
//float pit_vision_pid[6] = {10, 0.01, 5, 170, 0, 0};
//float yaw_vision_pid[6] = {13, 0, 5, 75, 0, 8};

float pit_vision_pid[6] = {18, 0.01, 5, 100, 0, 0};
float yaw_vision_pid[6] = {23, 0, 5, 75, 0, 8};
// 神符参数
float pit_buff_pid[6] = {27, 0.13, 5, 100, 0.1, 0};
float yaw_buff_pid[6] = {30, 0.1, 5, 80, 0, 5};
// 拨盘参数
float trig_pid[6] = {130, 0, 10, 11, 0, 5};
}
#endif


// 左上为正是+号。左下为正为-号
float pit_dir = 1.f;
float yaw_dir = 1.f;

float pit_ctrl_ffc;
float yaw_ctrl_ffc;

#define PIT_ANGLE_MAX 40
#define PIT_ANGLE_MIN -15
#define YAW_ANGLE_MAX 50
#define YAW_ANGLE_MIN -50

/** @brief 	   LADRC 2024/5/21  自瞄模式
**	@attention 
**  @author
**/
LADRC_NUM Vision_Angle_Pit =
{ 
	.h=0.003,//定时时间及时间步长
	.r=70,//跟踪速度参数
};
LADRC_NUM Vision_Angle_Yaw =
{ 
	.h=0.003,//定时时间及时间步长
	.r=70,//跟踪速度参数
};

// void PRE_LADRC_TD(TD_LADRC *td_para, float Expect);

// kalman_filter_t pc_kalman_filter;
/* 卡尔曼滤波（pc） */
kalman_filter_init_t pc_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.001, 0, 1},
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {6000, 0, 0, 2000}};
kalman_filter_t pc_kalman_filter_pitch;
kalman_filter_t pc_kalman_filter_yaw;
void gimbal_kalman_qr_set()
{
	mat_init(&pc_kalman_filter_pitch.Q, 2, 2, pc_kalman_filter_para.Q_data);
	mat_init(&pc_kalman_filter_pitch.R, 2, 2, pc_kalman_filter_para.R_data);
	mat_init(&pc_kalman_filter_yaw.Q, 2, 2, pc_kalman_filter_para.Q_data);
	mat_init(&pc_kalman_filter_yaw.R, 2, 2, pc_kalman_filter_para.R_data);
}
/* 卡尔曼滤波（pc） */

gimbal_t gimbal;
uint32_t gimbal_time, last_gimbal_time;


void gimbal_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)INFO_GET_GIMBAL_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			if (Signal & INFO_GET_GIMBAL_SIGNAL)
			{
				if(INS_Init_Done == 1)
				{
					gimbal_time = HAL_GetTick() - last_gimbal_time;
					last_gimbal_time = HAL_GetTick();

					if (gimbal_mode == GIMBAL_TRACK_ARMOR)
					{
						PID_Struct_Init(&pid_vision_pit, pit_vision_pid[0], pit_vision_pid[1], pit_vision_pid[2], 30, 5, DONE);
						PID_Struct_Init(&pid_vision_pit_spd, pit_vision_pid[3], pit_vision_pid[4], pit_vision_pid[5], 25000, 0, DONE);

						PID_Struct_Init(&pid_vision_yaw, yaw_vision_pid[0], yaw_vision_pid[1], yaw_vision_pid[2], 30, 10, DONE);
						PID_Struct_Init(&pid_vision_yaw_spd, yaw_vision_pid[3], yaw_vision_pid[4], yaw_vision_pid[5], 25000, 0, DONE);
					}
					else if (gimbal_mode == GIMBAL_SHOOT_BUFF)
					{
						PID_Struct_Init(&pid_buff_pit, pit_buff_pid[0], pit_buff_pid[1], pit_buff_pid[2], 30, 10, DONE);
						PID_Struct_Init(&pid_buff_pit_spd, pit_buff_pid[3], pit_buff_pid[4], pit_buff_pid[5], 25000, 0, DONE);

						PID_Struct_Init(&pid_buff_yaw, yaw_buff_pid[0], yaw_buff_pid[1], yaw_buff_pid[2], 30, 10, DONE);
						PID_Struct_Init(&pid_buff_yaw_spd, yaw_buff_pid[3], yaw_buff_pid[4], yaw_buff_pid[5], 20000, 0, DONE);
					}
					else
					{
						/* pit 轴电机的PID参数 */
						PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 30, 10, DONE);
						PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 25000, 0, DONE);

						/* yaw 轴电机的PID参数 */
						PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 50, 10, DONE);
						PID_Struct_Init(&pid_yaw_spd, yaw_pid[3], yaw_pid[4], yaw_pid[5], 25000, 0, DONE);
					}
					if (gimbal_mode != GIMBAL_RELEASE)
					{
						if (gimbal.state == GIMBAL_INIT_NEVER)
						{
							gimbal_mode = GIMBAL_INIT;
						}
						switch (gimbal_mode)
						{
						case GIMBAL_INIT:
						{
							PID_Struct_Init(&pid_yaw, yaw_init_pid[0], yaw_init_pid[1], yaw_init_pid[2], 30, 10, DONE);
							PID_Struct_Init(&pid_yaw_spd, yaw_init_pid[3], yaw_init_pid[4], yaw_init_pid[5], 16384, 3000, DONE);

							PID_Struct_Init(&pid_pit, pit_init_pid[0], pit_init_pid[1], pit_init_pid[2], 1000, 500, DONE);
							PID_Struct_Init(&pid_pit_spd, pit_init_pid[3], pit_init_pid[4], pit_init_pid[5], 30000, 15000, DONE);
							init_mode_handler(); // 云台回中
						}
						break;

						/*dodge和normal合并为手动控制模式*/
						case GIMBAL_NORMAL_MODE:
						case GIMBAL_DODGE_MODE:
						{
							if ((last_gimbal_mode != GIMBAL_DODGE_MODE) && (last_gimbal_mode != GIMBAL_NORMAL_MODE))
							{
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
							nomarl_dodge_handler();
						}
						break;
						/*打能量机关模式*/
						case GIMBAL_SHOOT_BUFF:
						{
							if (last_gimbal_mode != GIMBAL_SHOOT_BUFF)
							{
								PID_Clear(&pid_buff_pit);
								PID_Clear(&pid_buff_yaw);
								PID_Clear(&pid_buff_pit_spd);
								PID_Clear(&pid_buff_yaw_spd);
							}
							shoot_buff_ctrl_handler();
						}
						break;
						/*自瞄模式*/
						case GIMBAL_TRACK_ARMOR:
						{
							if (last_gimbal_mode != GIMBAL_TRACK_ARMOR)
							{
								PID_Clear(&pid_vision_pit);
								PID_Clear(&pid_vision_yaw);
								PID_Clear(&pid_vision_pit_spd);
								PID_Clear(&pid_vision_yaw_spd);
							}
							track_aimor_handler();
						}
						break;

						default:
						{
						}
						break;
						}
					}
					else
					{
						memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
						gimbal.state = GIMBAL_INIT_NEVER;
					}
					
					
					/*pid计算-角度环*/
					if (gimbal_mode == GIMBAL_TRACK_ARMOR)
					{
						pid_calc(&pid_vision_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
						pid_calc(&pid_vision_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);
					}
					else if (gimbal_mode == GIMBAL_SHOOT_BUFF)
					{
						pid_calc(&pid_buff_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
						pid_calc(&pid_buff_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);
					}
					else
					{		
						pid_calc(&pid_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
						pid_calc(&pid_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);
					}
	 
					/*速度环给定（串级）*/
					if (gimbal_mode == GIMBAL_TRACK_ARMOR)
					{
						gimbal.pid.yaw_spd_ref = pid_vision_yaw.out;
						gimbal.pid.pit_spd_ref = pid_vision_pit.out;
						gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;
						gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;
					}
					else if (gimbal_mode == GIMBAL_SHOOT_BUFF)
					{
						gimbal.pid.yaw_spd_ref = pid_buff_yaw.out;
						gimbal.pid.pit_spd_ref = pid_buff_pit.out;
						gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;
						gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;
					}
					else
					{
						if(PID_Inner_Parameters)//断外环，调内环
						{
							gimbal.pid.yaw_spd_ref = rm.yaw_v*5;
							gimbal.pid.pit_spd_ref = rm.pit_v*5;
						}
						else
						{
							gimbal.pid.yaw_spd_ref = pid_yaw.out;
							gimbal.pid.pit_spd_ref = pid_pit.out;
						}
						gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;
						gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;
					}

					/*pid计算-速度环*/
					if (gimbal_mode == GIMBAL_TRACK_ARMOR)
					{
						//Yaw轴加前馈
						//gimbal.pid.yaw_spd_ref += FFC_OUT(gimbal.pid.yaw_angle_ref/100);
						if(gimbal.pid.yaw_spd_ref > 30)
							gimbal.pid.yaw_spd_ref = 30;
						if(gimbal.pid.yaw_spd_ref < -30)
							gimbal.pid.yaw_spd_ref = -30;
												
						pid_calc(&pid_vision_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
						pid_calc(&pid_vision_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
					}
					else if (gimbal_mode == GIMBAL_SHOOT_BUFF)
					{	
						//Yaw轴加前馈
						//gimbal.pid.yaw_spd_ref += FFC_OUT(gimbal.pid.yaw_angle_ref/100);
						if(gimbal.pid.yaw_spd_ref > 30)
							gimbal.pid.yaw_spd_ref = 30;
						if(gimbal.pid.yaw_spd_ref < -30)
							gimbal.pid.yaw_spd_ref = -30;
						
						pid_calc(&pid_buff_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
						pid_calc(&pid_buff_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
					}
					else
					{		
						//Yaw轴加前馈
						//gimbal.pid.yaw_spd_ref += FFC_OUT(gimbal.pid.yaw_angle_ref/100);
//						if(gimbal.pid.yaw_spd_ref > 30)
//							gimbal.pid.yaw_spd_ref = 30;
//						if(gimbal.pid.yaw_spd_ref < -30)
//							gimbal.pid.yaw_spd_ref = -30;
												
						pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
						pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
					}

					if (gimbal_is_controllable()) // 将pid计算的out值代入glb_cur.gimbal_cur[]，后续通过can发出
					{
						if (gimbal_mode == GIMBAL_TRACK_ARMOR)
						{
							glb_cur.gimbal_cur[0] = yaw_dir * pid_vision_yaw_spd.out;
							glb_cur.gimbal_cur[1] = pit_dir * pid_vision_pit_spd.out;
						}
						else if (gimbal_mode == GIMBAL_SHOOT_BUFF)
						{
							glb_cur.gimbal_cur[0] = yaw_dir * pid_buff_yaw_spd.out;
							glb_cur.gimbal_cur[1] = pit_dir * pid_buff_pit_spd.out;
						}
						else
						{
							glb_cur.gimbal_cur[0] = yaw_dir * pid_yaw_spd.out;
							glb_cur.gimbal_cur[1] = pit_dir * pid_pit_spd.out;
						}
					}
					else
					{
						memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
						gimbal_mode = GIMBAL_RELEASE;
					}					
				}
				else//四元数未更新好
				{
					memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
					gimbal_mode = GIMBAL_RELEASE;
				} 
//				if (PID_Inner_Parameters)//断外环，调内环,发速度期望
//				{
//					vofa_gimbal[0] = gimbal.pid.pit_spd_fdb;
//					vofa_gimbal[1] = rm.pit_v*5;
//					vofa_gimbal[2] = gimbal.pid.yaw_spd_fdb;
//					vofa_gimbal[3] = rm.yaw_v*5;
//				}
//				else
//				{
//					vofa_gimbal[0] = gimbal.pid.pit_angle_fdb;
//					vofa_gimbal[1] = gimbal.pid.pit_angle_ref;
//					vofa_gimbal[2] = gimbal.pid.yaw_angle_fdb;
//					vofa_gimbal[3] = gimbal.pid.yaw_angle_ref;
//				}
				
				/* vofa调试 */
				Vofa_Get_Info();
				DMA_Debug_USART_Tx_Data(Tx_Buffer_Num);
				if(Rx_Flag == 1)//vofa上位机发值控制
				{
					Rx_Flag = 0;
					Info_Proc();
				}

				last_gimbal_mode = gimbal_mode; // 获取上一次云台状态
				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
									 (uint32_t)GIMBAL_MOTOR_MSG_SIGNAL,
									 (eNotifyAction)eSetBits,
									 (uint32_t *)NULL);				
			}
		}
		gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

	}
}


void gimbal_param_init(void)
{
	memset(&gimbal, 0, sizeof(gimbal_t));

	gimbal.state = GIMBAL_INIT_NEVER;

	ramp_init(&pit_ramp, 1000);
	ramp_init(&yaw_ramp, 1000);

	// 自瞄PID
	PID_Struct_Init(&pid_pit, pit_vision_pid[0], pit_vision_pid[1], pit_vision_pid[2], 30, 10, INIT);
	PID_Struct_Init(&pid_pit_spd, pit_vision_pid[3], pit_vision_pid[4], pit_vision_pid[5], 20000, 3000, INIT);

	PID_Struct_Init(&pid_yaw, yaw_vision_pid[0], yaw_vision_pid[1], yaw_vision_pid[2], 30, 10, INIT);
	PID_Struct_Init(&pid_yaw_spd, yaw_vision_pid[3], yaw_vision_pid[4], yaw_vision_pid[5], 20000, 3000, INIT);

	// 神符PID
	PID_Struct_Init(&pid_pit, pit_buff_pid[0], pit_buff_pid[1], pit_buff_pid[2], 30, 10, INIT);
	PID_Struct_Init(&pid_pit_spd, pit_buff_pid[3], pit_buff_pid[4], pit_buff_pid[5], 20000, 3000, INIT);

	PID_Struct_Init(&pid_yaw, yaw_buff_pid[0], yaw_buff_pid[1], yaw_buff_pid[2], 23000, 10, INIT);
	PID_Struct_Init(&pid_yaw_spd, yaw_buff_pid[3], yaw_buff_pid[4], yaw_buff_pid[5], 23000, 3000, INIT);

	/* pit 轴电机的PID参数 */
	PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 30, 10, INIT);
	PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 20000, 3000, INIT);

	/* yaw 轴电机的PID参数 */
	PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 30, 10, INIT);
	PID_Struct_Init(&pid_yaw_spd, yaw_pid[3], yaw_pid[4], yaw_pid[5], 20000, 3000, INIT);

}

/*注意！！！！！！
		注意！！！！！！
		注意！！！！！！
		relative_angle（相对角度）指的是当前角度与flash记录的归中角度的相对，而不是相对于陀螺仪
		gyro_angle（陀螺仪角度，我们称为绝对角度）才是与陀螺仪的零度的差
		总的来说，相对角度基于flash记录的归中角度
				  绝对角度基于陀螺仪
		*/

static void init_mode_handler(void)
{
	// 将当前角度赋给反馈值
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
	// 目标值设为零（斜坡函数需要时可加）
	gimbal.pid.pit_angle_ref = 0;
	// 将当前相对角度赋给反馈值
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
	// 目标值设为当前相对角度
	gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_total_angle;

	gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;
	
	gimbal.state = GIMBAL_INIT_DONE;
}

/*判断yaw轴是否有输入*/
static gimbal_state_t remote_is_action(void)
{
	if ((abs(rc.ch3) >= 10) || (abs(rc.mouse.x) >= 1))
	{
		return IS_ACTION;
	}
	else
	{
		return NO_ACTION;
	}
}

LADRC_NUM ladrc_change_head = 
{
   .r = 30,     //速度因子
   .h = 0.002,            //积分步长
};

uint32_t debug_time = 1000;


/* creat manual control funtion.  add time:2025.1.5*/   
static void manual_control_funtion(gimbal_status NOW_MODE)
{
	static uint8_t confirm_time;
	gimbal.state = remote_is_action();												   // 判断yaw轴是否有输入

	if ((NOW_MODE != GIMBAL_TRACK_ARMOR) && (direction_change)) // 检测到鼠标中间的滚轮向下滑动，则进行云台角度的变化
	{
		// 当进入此判断时，gimbal.yaw_offset_angle会在get_gimbal_info函数中变化编码值4096（即6020的半圈）
//		gimbal.pid.yaw_angle_fdb =  gimbal.sensor.yaw_gyro_angle;
		if(direction_change == 1)
		{
			direction_change = 2;
			if(gimbal.pid.yaw_angle_ref < 180)
				gimbal.pid.yaw_angle_ref += 180;
			else
				gimbal.pid.yaw_angle_ref -= 180;
		}
//		LADRC_TD(&ladrc_change_head,gimbal.sensor.yaw_relative_angle);
//		gimbal.pid.yaw_angle_fdb =ladrc_change_head.v1;
		if ((abs((int)(gimbal.sensor.yaw_total_angle - gimbal.pid.yaw_angle_ref))%360) < 3)
		{
			direction_change++;
		}
		else
		{
			direction_change = 2;
		}
		if(direction_change == 102)//100ms delay
		{
			if(NOW_MODE == GIMBAL_NORMAL_MODE)//普通模式则底盘跟随甩头
				gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;
			direction_change = 0;
		}
	}																																																																	
	else if ((gimbal.last_state == NO_ACTION) && (gimbal.state == NO_ACTION))// && (HAL_GetTick() - no_action_time > debug_time)) 
	{
		switch(gimbal.trace_state)
		{
			case REF_CONFIRM://判断正反转
			{
				if(gimbal.pid.yaw_angle_ref>gimbal.pid.last_yaw_angle_ref)//判断当前为正转
					gimbal.trace_state = REF_ADD;
				else if(gimbal.pid.yaw_angle_ref<gimbal.pid.last_yaw_angle_ref)//判断当前为反转
					gimbal.trace_state = REF_SUB;
				gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 云台停止后刷新底盘0轴
			}break;
			case REF_ADD://正转，目标值向上阶跃
			{
				if(gimbal.sensor.yaw_palstance < 0.3f)
				{
					gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
					gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 云台停止后刷新底盘0轴
					gimbal.trace_state = REF_KEEP;//反馈跟随目标阶段
				}
			}break;
			case REF_SUB://反转，目标值向下阶跃
			{
				if(gimbal.sensor.yaw_palstance > (-0.3f))
				{
					gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
					gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 云台停止后刷新底盘0轴
					gimbal.trace_state = REF_KEEP;//反馈跟随目标阶段
				}
			}break;
			default:break;//完全停止后不刷新底盘0轴，此时底盘云台分开锁死
		}
		
	}
	else if (gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)//进入目标值跟踪阶段，消除动态超调
	{
		gimbal.trace_state = REF_CONFIRM;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 云台停止后刷新底盘0轴
	}
	else
	{
		gimbal.pid.last_yaw_angle_ref = gimbal.pid.yaw_angle_ref;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 在yaw轴动的情况下刷新0轴
		gimbal.trace_state = REF_KEEP;//反馈跟随目标阶段
	}

	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
	gimbal.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW + km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;

	/*pitch轴*/
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
	gimbal.pid.pit_angle_ref += rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
	/* 软件限制pitch轴角度 */
	VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
	gimbal.last_state = gimbal.state; // 获取上次输入的状态
	
}

/* merge normal and dodge together.  modification time:2025.1.5*/   
static void nomarl_dodge_handler(void)
{	
	if (last_gimbal_mode != GIMBAL_NORMAL_MODE && last_gimbal_mode != GIMBAL_DODGE_MODE )
	{
		// 刷新yaw零轴
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
		gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;
		gimbal.trace_state = REF_KEEP;//反馈跟随目标阶段
		gimbal.state = NO_ACTION;
		gimbal.last_state = NO_ACTION;
	}
	else
		manual_control_funtion(gimbal_mode);
}

float single_coordination = 0;
extern float pit_rec_real;
extern float yaw_rec_real;
extern WorldTime_RxTypedef PC_KF_Time;
/*激励信号所需量*/
float MyF = 0.5, Myt = 0, MyCnt = 0;

static void track_aimor_handler(void)
{
	pid_yaw.iout = 0;										// 清空其他模式yaw角度环iout累加值
	
	if (last_gimbal_mode != GIMBAL_TRACK_ARMOR)
	{		
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
		gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;
		gimbal.trace_state = REF_KEEP;//反馈跟随目标阶段
		gimbal.state = NO_ACTION;
		gimbal.last_state = NO_ACTION;
	}
	/*输入激励信号过程*/
	if (MyF < 10)
	{
		MyCnt = 100 * arm_sin_f32(2 * 3.14f * MyF * Myt);

		Myt += 0.002f;
		if (Myt > (1 / MyF * 10))
		{ // 变频过程
			MyF = MyF + 0.7f;
			Myt = 0;
		}
	}

	/* 跟踪微分器 */
	// PRE_LADRC_TD(&td_pit, pit_rec_real);
	// PRE_LADRC_TD(&td_yaw, yaw_rec_real);
	if (pc_recv_mesg.mode_Union.info.visual_valid == 1)
	{
//    /* TD跟踪微分处理 */
//		LADRC_TD(&Vision_Angle_Pit,  pc_recv_mesg.aim_pitch);
//		LADRC_TD(&Vision_Angle_Yaw,   pc_recv_mesg.aim_yaw);
		
//		gimbal.pid.yaw_angle_ref = Vision_Angle_Yaw.v1; //v1为角度，v2为导数
//		gimbal.pid.pit_angle_ref = Vision_Angle_Pit.v1;	
		
		gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;  // yaw轴用陀螺仪
		
		gimbal.pid.yaw_angle_ref = 360.0f*gimbal.sensor.yaw_cnt + pc_recv_mesg.aim_yaw;
		gimbal.pid.pit_angle_ref = pc_recv_mesg.aim_pitch;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;		
	}	
	/*2025.01.04 new visual invalid processing */
	else if (pc_recv_mesg.mode_Union.info.visual_valid == 0)//无效时可手动控制，防止变成呆瓜
		manual_control_funtion(gimbal_mode);

	// PC通讯出现问题,直接无法控制
	if (pc_recv_mesg.aim_yaw >= 180 || pc_recv_mesg.aim_yaw <= -180)
		gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
	if (pc_recv_mesg.aim_pitch >= 180 || pc_recv_mesg.aim_pitch <= -180)
		gimbal.pid.pit_angle_ref = gimbal.pid.pit_angle_fdb;

	//   /*跟踪微分器*/
	//   LADRC_TD(&Vision_angle,MyCnt);
	//   gimbal.pid.pit_angle_ref = Vision_angle.v1;
	//   gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_ref;

	/*输入激励信号代替自瞄，作用为向云台稳定输入激励信号采用数据用于系统辨识。使用时要把上面自瞄给定设0*/

	//		gimbal.pid.yaw_angle_ref = MyCnt;
	//		//想采样哪一周就把哪一轴给定设MyCnt.
	//		gimbal.pid.pit_angle_ref = 0;
}

static void shoot_buff_ctrl_handler(void)
{
	/*控制角度*/
	static float yaw_ctrl;
	static float pit_ctrl;

	/*获取反馈值*/
	gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 备份陀螺仪数据，以免退出自瞄时冲突
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;

	if (pc_recv_mesg.mode_Union.info.visual_valid == 1)
	{
		yaw_ctrl = pc_recv_mesg.aim_yaw;
		pit_ctrl = pc_recv_mesg.aim_pitch;
	}

	/*视觉无效处理*/
	else if (pc_recv_mesg.mode_Union.info.visual_valid == 0)
	{
		pit_ctrl = gimbal.pid.pit_angle_fdb;
		yaw_ctrl = gimbal.pid.yaw_angle_fdb;
	}

	gimbal.pid.yaw_angle_ref = yaw_ctrl;
	gimbal.pid.pit_angle_ref = pit_ctrl;

	/*给定角度限制*/
	VAL_LIMIT(gimbal.pid.yaw_angle_ref, -45, 45);
	VAL_LIMIT(gimbal.pid.pit_angle_ref, -30, 30); // 25,15
}

float speed_threshold = 10.0f;
// time_raw,pitch_angel_raw等都来自pc
// 此函数可算出速度，调用时position实参为pitch_angel_raw
// 函数实为给结构体speed_calc_data_t赋值，初始化参数
float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 1000; // 计算速度（当前位置-上次位置）/
#if 1
		if ((S->speed - S->processed_speed) < -speed_threshold) // speed_threshold=10.0f,S->processed_speed未赋过值
		{
			S->processed_speed = S->processed_speed - speed_threshold;
		}
		else if ((S->speed - S->processed_speed) > speed_threshold)
		{
			S->processed_speed = S->processed_speed + speed_threshold;
		}
		else
#endif
			S->processed_speed = S->speed; // 将前面计算出的速度赋予S->processed_speed

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if (S->delay_cnt > 200) // delay 200ms speed = 0
	{
		S->processed_speed = 0;
	}

	return S->processed_speed;
}

float FFC_OUT(float x_n)
{
	static float ts = 0.003;
	static float x_n_1 = 0.0f;
	static float x_n_2 = 0.0f;
	static float a1 = 96.89f/89.38f;
	static float a2 = 1/89.38f;
	float x_dot_2 = 0.0f;
	float x_dot_1 = 0.0f;
	float y_n;
	
	x_dot_1 = (x_n - x_n_1)/ts;
	x_dot_2 = (x_n - 2*x_n_1 + x_n_2)/(ts*ts);
	
	y_n = a1*x_dot_1 + a2*x_dot_2;
	
	x_n_2 = x_n_1;
	x_n_1 = x_n;
	return y_n;
}



/* 带相位超前的跟踪微分器 */
void PRE_LADRC_TD(TD_LADRC *td_para, float Expect)
{
	td_para->fh = -td_para->r * td_para->r * (td_para->v1 - Expect) - 2 * td_para->r * td_para->v2;
	td_para->v1 += td_para->v2 * td_para->h;
	td_para->v2 += td_para->fh * td_para->h;
	// 相位超前 v1+lamda*h*v2
	td_para->pre_v1 = td_para->v1 + td_para->lambda * td_para->fh * td_para->v2;
}

