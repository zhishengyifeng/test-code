#ifndef _sys_config_H
#define _sys_config_H

#include "stm32f4xx.h"

#define INFANTRY_3 3
#define INFANTRY_4 4
#define INFANTRY_5 5   //��ȫ��
#define INFANTRY_24 24 //��ȫ��
#define TEST_SHOOT 6

#define INFANTRY_NUM INFANTRY_24 // �����޸Ĳ�������

/**********************ң�� ��������***************************/
/* ҡ�����ֵ */
#define RC_RESOLUTION 660.0f
/*************************���� ����*******************************/
/* ң��ģʽ  ���� �ٶ� ���� */
/* ǰ�� �ٶ� (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X 4000.0f
/* ���� �ٶ� (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y 4000.0f
/* ���� ��ת �ٶ� (deg/s) */
#define CHASSIS_RC_MAX_SPEED_R 480.0f

/* ����ģʽ  ���� �ٶ� ���� */
/* ǰ�� �ٶ� (mm/s) */
#define CHASSIS_KB_MAX_SPEED_X 4000.0f // �����ԣ�4000����Ч����ã��ٶ�Խ��Խ����ԭ�ش�

/* ���� �ٶ� (mm/s) */
#define CHASSIS_KB_MAX_SPEED_Y 4000.0f


/************************ ����Ӳ�� ���� ****************************/


#if (INFANTRY_NUM == INFANTRY_5 | INFANTRY_NUM == INFANTRY_24)

/* ȫ���ְ뾶(mm) */
#define RADIUS 77.0f
/* ȫ�����ܳ�(mm) */
#define PERIMETER 483.0f
/*ȫ���־ࣨ���ң�*/
#define WHEELTRACK 275.5f // 415
/*ȫ����ࣨǰ��*/
#define WHEELBASE 275.5f	 // 406

/**************************��̨ ����*******************************/
/* ң��ģʽ ��̨�ٶȿ��� */
/* pitch �� �ٶ� */
#define GIMBAL_RC_MOVE_RATIO_PIT  -1.4f // Сȫ���pit�������ͨ���ֲ���װ�䲻ͬ
/* yaw �� �ٶ� */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.6f

/* ����ģʽ ��̨�ٶȿ��� */
/* pitch �� �ٶ� */
#define GIMBAL_PC_MOVE_RATIO_PIT -0.6f // Сȫ���pit�������ͨ���ֲ���װ�䲻ͬ
/* yaw �� �ٶ� */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.5f

#else 

/* ���ְ뾶(mm) */
#define RADIUS 77.0f
/* �����ܳ�(mm) */
#define PERIMETER 483.0f
/*�����־ࣨ���ң�*/
#define WHEELTRACK 392.0f // 415
/*������ࣨǰ��*/
#define WHEELBASE 320.0f // 406

/**************************��̨ ����*******************************/
/* ң��ģʽ ��̨�ٶȿ��� */
/* pitch �� �ٶ� */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.4f
/* yaw �� �ٶ� */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.2f

/* ����ģʽ ��̨�ٶȿ��� */
/* pitch �� �ٶ� */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.6f
/* yaw �� �ٶ� */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.6f

#endif

/*��̨ƫ�Ƶ�������X�ᣨǰ�󣩾���*/
#define GIMBAL_X_OFFSET 0 // 130
/*��̨ƫ�Ƶ�������Y�ᣨ���ң�����*/
#define GIMBAL_Y_OFFSET 0 // 0

/* ���̵�� 3508 */
/* ���̵���ļ��ٱ� */
#define CHASSIS_DECELE_RATIO (1.0f / 19.0f)
/* ��һ 3508 ����� ��� ת��, unit is rpm */
#define MAX_WHEEL_RPM 8500 // 8347rpm = 3500mm/s
/* ���� ��� ƽ���ٶ� , unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3300 // 8000rpm
#define MAX_CHASSIS_VY_SPEED 3300
/* ���� ��� ��ת�ٶ� , unit is degree/s */
#define MAX_CHASSIS_VR_SPEED 236 // 5000rpm
/*С����ת��*/
#define LG_SPEED 200
/*С���ݻ�Сè�������VW*/
#define MAX_DODGE_SPEED 400

/************************** ��̨Ӳ�� ���� *****************************/
/* �������ֵ �� �Ƕȣ��ȣ� �ı��� */
#define ENCODER_ANGLE_RATIO (8192.0f / 360.0f)
/* pitch�� ��� �� ���ٱ� */
#define PIT_DECELE_RATIO 1.0f
/* yaw�� ��� �� ���ٱ� */
#define YAW_DECELE_RATIO 1.0f
/* pitch�� ��������� */
#define PIT_MOTO_POSITIVE_DIR 1.0f
/* yaw�� ��������� */
#define YAW_MOTO_POSITIVE_DIR 1.0f
/* ���� ������������ */
#define TRI_MOTO_POSITIVE_DIR 1.0f

/*********************** ϵͳ �����ӿ� ���� ****************************/

/* CAN ��� */
#define CHASSIS_CAN CAN1//��������
#define YAW_CAN CAN1	//yaw
#define POWER_CAN CAN1	//���ذ峬��

#define PITCH_CAN CAN2	//pitch
#define FRIC_CAN CAN2	//������������̡�Ħ���֣�
/* UART ��� */
/**
 * @attention
 * close usart DMA receive interrupt, so need add
 * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
 **/
#define DBUS_HUART huart1	  // ң�ؽ�����
#define JUDGE_HUART huart3	  // ����ϵͳ�ӿ�
#define COMPUTER_HUART huart6 // MINI���Խӿ�

/* ������� ��� */
#define DEFAULT_TUNE 0 // ����

/*���幦��*/
#define POWER_BUFFER 60

/*�������*/
#define SYSTEM_DELAY 0.3

/* IMU �¶ȿ��� */
#define DEFAULT_IMU_TEMP 50

/* ���� ��� */
/* ���� ϵ�� */
#define RADIAN_COEF 57.3f
/* Բ���� */
#define PI 3.142f

#define NATURAL_NUM_E 2.718282f

// �ж������Сֵ
#define VAL_LIMIT(val, min, max) \
	do                           \
	{                            \
		if ((val) <= (min))      \
		{                        \
			(val) = (min);       \
		}                        \
		else if ((val) >= (max)) \
		{                        \
			(val) = (max);       \
		}                        \
	} while (0)

#endif
