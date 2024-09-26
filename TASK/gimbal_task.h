#ifndef _gimbal_task_H
#define _gimbal_task_H

#include "stm32f4xx.h"
#include "ramp.h"
typedef enum
{
  GIMBAL_INIT_NEVER,
  GIMBAL_INIT_DONE,
  NO_ACTION,
  IS_ACTION,
} gimbal_state_t;

typedef struct
{
  /* �ǶȻ��ĸ����ͷ��� */
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* �ٶȻ��ĸ����ͷ��� */
  float yaw_spd_ref;
  float pit_spd_ref;
  float yaw_spd_fdb;
  float pit_spd_fdb;
} gim_pid_t;

typedef struct
{
  /* ��ԽǶȣ�������ŷ���� */
  float pit_relative_angle;
  float yaw_relative_angle;
  float pit_gyro_angle;
  float yaw_gyro_angle;
	float roll_gyro_angle;
  /* ���ٶ� */
  float yaw_palstance;
  float pit_palstance;
	/* 多圈累加 */
	float yaw_total_angle;
	int32_t yaw_cnt;
} gim_sensor_t;

typedef struct
{
  uint8_t small_buff_ctrl;
  uint8_t big_buff_ctrl;
  uint8_t track_ctrl;
  uint8_t separate_ctrl;
  uint8_t chassis_stop;

  gim_pid_t pid;
  gim_sensor_t sensor;
  gimbal_state_t state;
  gimbal_state_t last_state;

  /*��FLASH�ж�����У׼����λ*/
  int32_t pit_center_offset;
  int32_t yaw_center_offset;
  float yaw_offset_angle;
  float pit_offset_angle;
  float yaw_dodge_angle;
} gimbal_t;

/*�������ã��ٶ���ؽṹ��*/
typedef struct // speed_calc_data_t
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

/* ����΢������pc�� */
typedef struct
{
  float r;
  float h;
  float fh;
  float v1, v2;
  float lambda, pre_v1;
} TD_LADRC;


extern gimbal_t gimbal;
extern uint8_t input_flag;
extern ramp_t pit_ramp;
extern ramp_t yaw_ramp;

void gimbal_task(void *parm);
void gimbal_param_init(void);

static void init_mode_handler(void);
static void nomarl_handler(void);
static void separate_handler(void);
static void dodge_handler(void);
static void track_aimor_handler(void);
static void shoot_buff_ctrl_handler(void);
//前馈控制
float FFC_OUT(float x_n);

float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position);

#endif
