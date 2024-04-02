#ifndef _chassis_task_H
#define _chassis_task_H


#include "stm32f4xx.h"
#include "pid.h"

//#define TWIST_ANGLE    45
#define CHASSIS_PERIOD 10
#define TWIST_PERIOD   1200//1300
typedef struct
{
  uint8_t         dodge_ctrl;
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // 
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
  int16_t         current[4];
  int16_t         position_ref;
	float  CapData[4];
}chassis_t;

extern float yaw_speed;
extern chassis_t chassis;
extern float cap_volt_limit;
void chassis_task(void *parm);
void chassis_param_init(void);

static void chassis_normal_handler(void);
static void chassis_separate_handler(void);
static void chassis_dodge_handler(void);
static void chassis_stop_handler(void);
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);
void spd_handler(void);
void buffer(float *a,float b,float high_parameter,float low_parameter);
void Cap_refresh(void);
#endif
