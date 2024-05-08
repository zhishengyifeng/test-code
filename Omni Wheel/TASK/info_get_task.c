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
	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)MODE_SWITCH_INFO_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			if (Signal & MODE_SWITCH_INFO_SIGNAL)
			{
				taskENTER_CRITICAL(); // ����ϵͳ���ٽ籣����������ֹ����Ĳ����������л����ж������

				keyboard_global_hook();
				get_chassis_info();
				get_gimbal_info();
				get_shoot_info();
				get_global_last_info();

				taskEXIT_CRITICAL();			 // �˳��ٽ籣��
				if (global_mode == RELEASE_CTRL) // ��ң���ұߴ�����ʱ�����ȫ���ϵ�
				{
					direction = 1;
					gimbal.pit_center_offset = cali_param.pitch_offset;
					gimbal.yaw_center_offset = cali_param.yaw_offset;
					MEMSET(&glb_cur, motor_current_t);
					xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
									   (uint32_t)MODE_SWITCH_MSG_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
				}
				else // ����ִ����̨���̷�����3������
				{
					xTaskGenericNotify((TaskHandle_t)gimbal_Task_Handle,
									   (uint32_t)INFO_GET_GIMBAL_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
					xTaskGenericNotify((TaskHandle_t)chassis_Task_Handle,
									   (uint32_t)INFO_GET_CHASSIS_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
					xTaskGenericNotify((TaskHandle_t)shoot_Task_Handle,
									   (uint32_t)INFO_GET_SHOOT_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
				}

				info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
			}
		}
	}
}

static void get_chassis_info(void)
{
	/* �õ��������� */
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
	}
	/* get remote and keyboard chassis control information */
	keyboard_chassis_hook();
	remote_ctrl_chassis_hook();
}
int debug_yaw_spd = 1;
int debug_pit_spd = -1; // ����Ϊ����+�š�����Ϊ��Ϊ-��

static void get_gimbal_info(void)
{
	/* ת������ԽǶ� */
	static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_DECELE_RATIO / ENCODER_ANGLE_RATIO;
	static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR * PIT_DECELE_RATIO / ENCODER_ANGLE_RATIO;

	static int i = 0;
	if (direction_change == 1)
	{ // ��⵽��ͷָ��
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
	// ������ԽǶ�
	/*���ݺ���ǵı�����ֵ moto_yaw.ecd �ͺ���ǵ�����ƫ��ֵ gimbal.yaw_center_offset��
	���� get_relative_pos ��������ȡ����ǵ����λ�á���yaw_ecd_ratio�����λ�ã�������ֵ��
	��Ƕ�֮���ת�����ʣ�Ϊÿ����ֵ����Ӧ�ĽǶȣ���������λ�ã�����ֵ����Ϊ��ԽǶ�*/
	gimbal.sensor.yaw_relative_angle = yaw_ecd_ratio * get_relative_pos(moto_yaw.ecd, gimbal.yaw_center_offset);
	/*ƫ����ͬ��*/
	gimbal.sensor.pit_relative_angle = pit_ecd_ratio * get_relative_pos(moto_pit.ecd, gimbal.pit_center_offset);

	/* get gimbal relative palstance */
	// the Z axis(yaw) of gimbal coordinate system corresponds to the IMU Z axis
	//  gimbal.sensor.yaw_palstance = yaw_spd * mpu_data.yaw_spd; // 16.384f; //unit: dps
	//  //the Y axis(pitch) of gimbal coordinate sys em corresponds to the IMU -Y axis
	//  gimbal.sensor.pit_palstance = pit_spd * mpu_data.pit_spd;
	/*��ȡң�غͼ��������*/
	keyboard_gimbal_hook();
	remote_ctrl_gimbal_hook();
}

static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
	int16_t tmp = 0;
	tmp = raw_ecd - center_offset;//�õ���ǰ��ֵ
	if (raw_ecd - center_offset >= 4096)//��������Ȧ�� >=4096 �ͼ�ȥһȦ
		tmp -= 8192;
	else if (raw_ecd - center_offset <= -4096)//���ڸ���Ȧ�� <=-4096 �ͼ���һȦ
		tmp += 8192;
	return tmp;
}

static void get_shoot_info(void)
{
	remote_ctrl_shoot_hook();
	keyboard_shoot_hook();

	shoot.para_mode = SHOOTNOR_MODE;
}

static void get_global_last_info(void)
{
	glb_sw.last_sw1 = rc.sw1;
	glb_sw.last_sw2 = rc.sw2;
	glb_sw.last_iw = rc.iw;
}
