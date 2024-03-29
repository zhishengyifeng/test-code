#include "imu_task.h"
#include "gimbal_task.h"
// #include "cmsis_os.h"
#include "bsp_imu.h"
#include "pid.h"
#include "sys_config.h"
// #include "bsp_io.h"
#include "math.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
#include "comm_task.h"
#include "bmi088driver.h"
#include "delay.h"
#include "MahonyAHRS.h"
#include "IST8310.h"
#include "filters.h"
#include "bsp_dwt.h"
#include "dma.h"
#include "detect_task.h"
#include "kalman_filter.h"

#include "ahrs.h"
#include "modeswitch_task.h"

#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

UBaseType_t imu_stack_surplus;
extern TaskHandle_t imu_Task_Handle;

volatile uint8_t imu_start_flag = 0;

extern IMU_Data_t BMI088;

uint32_t time = 0;
float dt = 0;
float EulerAngle[3] = {0};

float totalangle_transfer(float angle);

float imu_pid[3] = {200, 0.1, 0};

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

extern ist8310_data_t IST8310;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

// static uint8_t first_temperate;
// static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
// static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f; // tast run time , unit s.任务运行的时间 单位 s

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f}; // euler angle, unit rad.??? ?? rad
fp32 INS_angle_final[3] = {0.0f, 0.0f, 0.0f}; // 转换单位后的角度

void imu_temp_keep(void)
{
	PID_Struct_Init(&pid_imu_tmp, imu_pid[0], imu_pid[1], imu_pid[2], 5000, 1000, INIT);
	if (BMI088.Temperature > 45)
	{
		pid_calc(&pid_imu_tmp, BMI088.Temperature, 45);
		TIM_SetCompare1(TIM10, pid_imu_tmp.out);
	}
	else
	{
		TIM_SetCompare1(TIM10, 5000 - 1);
	}
}

/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], IMU_Data_t *bmi088)
{
	for (uint8_t i = 0; i < 3; i++)
	{
		gyro[i] = bmi088->Gyro[0] * gyro_scale_factor[i][0] + bmi088->Gyro[1] * gyro_scale_factor[i][1] + bmi088->Gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
		accel[i] = bmi088->Accel[0] * accel_scale_factor[i][0] + bmi088->Accel[1] * accel_scale_factor[i][1] + bmi088->Accel[2] * accel_scale_factor[i][2] + accel_offset[i];
//		mag[i] = ist8310->Mag[0] * mag_scale_factor[i][0] + ist8310->Mag[1] * mag_scale_factor[i][1] + ist8310->Mag[2] * mag_scale_factor[i][2] + mag_offset[i];
	}
}

void imu_task(void const *argu)
{
	uint32_t imu_wake_time = osKernelSysTick();
	imu_Task_Handle = xTaskGetHandle(pcTaskGetName(NULL));
	imu_start_flag = 1;

	AHRS_init(INS_quat, INS_accel, INS_mag);

	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
	accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
	accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

	while (1)
	{
		BMI088_read(BMI088.Gyro, BMI088.Accel, &BMI088.Temperature);
		// ReadIST8310Data(IST8310.Mag);//发现读了还是会温漂，而且会导致程序运行变慢，不如不读
		
		/* rotate and zero drift */
		imu_cali_slove(INS_gyro, INS_accel, INS_mag, &BMI088);

		/* accel low-pass filter */
		for (uint8_t i = 0; i < 3; ++i)
		{
			accel_fliter_1[i] = accel_fliter_2[i];
			accel_fliter_2[i] = accel_fliter_3[i];
			accel_fliter_3[i] = accel_fliter_2[i] * fliter_num[0] + accel_fliter_1[i] * fliter_num[1] + INS_accel[i] * fliter_num[2];
		}

		AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
		get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

		INS_angle_final[0] = INS_angle[0] * RAD_TO_ANGLE;
		INS_angle_final[1] = INS_angle[1] * RAD_TO_ANGLE;
		INS_angle_final[2] = INS_angle[2] * RAD_TO_ANGLE;

		gimbal.sensor.yaw_gyro_angle = INS_angle_final[0];

		gimbal.sensor.yaw_palstance = BMI088.Gyro[2] * 100;
		/////////////////////////////////////////////////
		gimbal.sensor.pit_gyro_angle = INS_angle_final[1];
		if (INFANTRY_NUM == INFANTRY_5) // 步兵五即小全向的pit电机与普通麦轮步兵装配不同
			gimbal.sensor.pit_palstance = BMI088.Gyro[0] * 100;
		else
			gimbal.sensor.pit_palstance = -BMI088.Gyro[0] * 100;
		/////////////////////////////////////////////////

		err_detector_hook(IMU_OFFLINE);
		imu_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

		vTaskDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
	}
}

void EXTI4_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line4) == 1)
	{
		if (imu_start_flag)
		{
			if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
			{
				static BaseType_t xHigherPriorityTaskWoken;
				vTaskNotifyGiveFromISR(imu_Task_Handle, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);
}
