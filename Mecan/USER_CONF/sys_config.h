#ifndef _sys_config_H
#define _sys_config_H

#include "stm32f4xx.h"


#define INFANTRY_MECANNUM  1
#define INFANTRY_OMV  2
#define TEST_SHOOT 6

#define INFANTRY_CLASS INFANTRY_MECANNUM // 可以修改步兵种类

/**********************遥控 设置配置***************************/
/* 摇杆最大值 */
#define RC_RESOLUTION 660.0f
/*************************底盘 配置*******************************/
/* 遥控模式  底盘 速度 限制 */
/* 前后 速度 (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X 4000.0f
/* 左右 速度 (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y 4000.0f
/* 底盘 旋转 速度 (deg/s) */
#define CHASSIS_RC_MAX_SPEED_R 480.0f

/* 键盘模式  底盘 速度 限制 */
/* 前后 速度 (mm/s) */
#define CHASSIS_KB_MAX_SPEED_X 4000.0f // 经测试，4000左右效果最好，速度越快越容易原地打滑

/* 左右 速度 (mm/s) */
#define CHASSIS_KB_MAX_SPEED_Y 4000.0f


/************************ 底盘硬件 配置 ****************************/

#if (INFANTRY_CLASS == INFANTRY_MECANNUM)
/* 麦轮半径(mm) */
#define RADIUS 77.0f
/* 麦轮周长(mm) */
#define PERIMETER 483.0f
/*麦轮轮距（左右）*/
#define WHEELTRACK 392.0f // 415
/*麦轮轴距（前后）*/
#define WHEELBASE 320.0f // 406

#elif (INFANTRY_CLASS == INFANTRY_OMV)
/* 全向轮半径(mm) */
#define RADIUS 77.0f
/* 全向轮周长(mm) */
#define PERIMETER 483.0f
/*全向轮距（左右）*/
#define WHEELTRACK 275.5f // 415
/*全向轴距（前后）*/
#define WHEELBASE 275.5f	 // 406
#endif


/**************************云台 配置*******************************/
#if (INFANTRY_CLASS == INFANTRY_MECANNUM)
/* 遥控模式 云台速度控制 */
/* pitch 轴 速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.4f
/* yaw 轴 速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.6f

/* 键盘模式 云台速度控制 */
/* pitch 轴 速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.6f
/* yaw 轴 速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.6f

/**************************云台 配置*******************************/
#elif (INFANTRY_CLASS == INFANTRY_OMV)
/* 遥控模式 云台速度控制 */
/* pitch 轴 速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.4f
/* yaw 轴 速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.2f

/* 键盘模式 云台速度控制 */
/* pitch 轴 速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.6f
/* yaw 轴 速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.6f

#endif

/*云台偏移底盘中心X轴（前后）距离*/
#define GIMBAL_X_OFFSET 0 // 130
/*云台偏移底盘中心Y轴（左右）距离*/
#define GIMBAL_Y_OFFSET 0 // 0

/* 底盘电机 3508 */
/* 底盘电机的减速比 */
#define CHASSIS_DECELE_RATIO (1.0f / 19.0f)
/* 单一 3508 电机的 最大 转速, unit is rpm */
#define MAX_WHEEL_RPM 8500 // 8347rpm = 3500mm/s
/* 底盘 最大 平移速度 , unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3300 // 8000rpm
#define MAX_CHASSIS_VY_SPEED 3300
/* 底盘 最大 旋转速度 , unit is degree/s */
#define MAX_CHASSIS_VR_SPEED 236 // 5000rpm
/*小陀螺转速*/
#define LG_SPEED 200
/*小陀螺或小猫步下最大VW*/
#define MAX_DODGE_SPEED 400

/************************** 云台硬件 配置 *****************************/
/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO (8192.0f / 360.0f)
/* pitch轴 电机 的 减速比 */
#define PIT_DECELE_RATIO 1.0f
/* yaw轴 电机 的 减速比 */
#define YAW_DECELE_RATIO 1.0f
/* pitch轴 正方向参数 */
#define PIT_MOTO_POSITIVE_DIR 1.0f
/* yaw轴 正方向参数 */
#define YAW_MOTO_POSITIVE_DIR 1.0f
/* 拨盘 电机正方向参数 */
#define TRI_MOTO_POSITIVE_DIR 1.0f

/*********************** 系统 交互接口 配置 ****************************/

/* CAN 相关 */
#define CHASSIS_CAN CAN1//底盘轮子
#define YAW_CAN CAN1	//yaw
#define POWER_CAN CAN1	//功控板超电

#define PITCH_CAN CAN2	//pitch
#define FRIC_CAN CAN2	//发射机构（拨盘、摩擦轮）
/* UART 相关 */
/**
 * @attention
 * close usart DMA receive interrupt, so need add
 * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
 **/
#define DBUS_HUART huart1	  // 遥控接收器
#define JUDGE_HUART huart3	  // 裁判系统接口
#define COMPUTER_HUART huart6 // MINI电脑接口

/* 监测任务 相关 */
#define DEFAULT_TUNE 0 // 音调

/*缓冲功率*/
#define POWER_BUFFER 60

/*自瞄参数*/
#define SYSTEM_DELAY 0.3

/* IMU 温度控制 */
#define DEFAULT_IMU_TEMP 50

/* 计算 相关 */
/* 弧度 系数 */
#define RADIAN_COEF 57.3f
/* 圆周率 */
#define PI 3.142f

#define NATURAL_NUM_E 2.718282f

// 判断最大最小值
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
