//#ifndef __IMU_TASK_H__
//#define __IMU_TASK_H__

//#include "stm32f4xx.h"

///* imu task period time (ms) */
//#define IMU_TASK_PERIOD 1

//void imu_task(void *parm);


//#endif

#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stm32f4xx.h"
#include "bsp_imu.h"

/* imu task period time (ms) */
#define IMU_TASK_PERIOD 1

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

#if (INFANTRY_NUM == INFANTRY_5)
//竖直摩擦轮陀螺仪矩阵
#define BMI088_BOARD_INSTALL_SPIN_MATRIX   \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, -1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
		
#else 
////水平摩擦轮陀螺仪矩阵
#define BMI088_BOARD_INSTALL_SPIN_MATRIX   \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \		
		


#endif

#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

typedef struct
{
  int roll_cnt;
  int pitch_cnt;
  int yaw_cnt;
  
  float last_roll;
  float last_pitch;
  float last_yaw;

  float roll;
  float pitch;
  float yaw;
} imu_attitude_t;

void imu_task(void const *argu);

#endif



