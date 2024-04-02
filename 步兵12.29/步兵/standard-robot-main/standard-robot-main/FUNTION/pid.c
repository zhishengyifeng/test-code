#include "pid.h"
#include "fuzzy_pid.h"

#define POSITION_PID 1 // λ��ʽ
#define DELTA_PID 2	   // ����ʽ
#define PID_MODE POSITION_PID

static void abs_limit(float *x, int32_t limit)
{
	if (*x > limit)
		*x = limit;
	if (*x < -limit)
		*x = -limit;
}

static void pid_init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxout = max_out;
	pid->integral_limit = integral_limit;
	pid->output_deadband = 5;
}

static void pid_reset(pid_t *pid, float p, float i, float d)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;

	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}

float pid_calc(pid_t *pid, float get, float set)
{
	pid->get = get;
	pid->set = set;
	pid->error[NOW_ERR] = set - get;

#if (PID_MODE == POSITION_PID)
	pid->pout = pid->kp * pid->error[NOW_ERR];
	pid->iout += pid->ki * pid->error[NOW_ERR];
	pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);

	abs_limit(&(pid->iout), pid->integral_limit);
	if (pid->error[NOW_ERR] < 200)
		pid->out = pid->pout + pid->iout + pid->dout;
	else
		pid->out = pid->pout + pid->dout;
	abs_limit(&(pid->out), pid->maxout);
#elif (PID_MODE == DELTA_PID)
	pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
	pid->iout = pid->ki * pid->error[NOW_ERR];
	pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]);

	pid->out += pid->pout + pid->iout + pid->dout;
	abs_limit(&(pid->out), pid->maxout);
#endif

	pid->error[LLAST_ERR] = pid->error[LAST_ERR];
	pid->error[LAST_ERR] = pid->error[NOW_ERR];

	if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
		return 0;
	else
		return pid->out;
}

float fuzzy_pid_calc(pid_t *pid, float get, float set)
{
	pid->get = get;
	pid->set = set;
	pid->error[NOW_ERR] = set - get;

	fuzzy_calc(&(*pid));

#if (PID_MODE == POSITION_PID)
	pid->pout = pid->kp * pid->error[NOW_ERR];
	pid->iout += pid->ki * pid->error[NOW_ERR];
	pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);

	abs_limit(&(pid->iout), pid->integral_limit);
	pid->out = pid->pout + pid->iout + pid->dout;
	abs_limit(&(pid->out), pid->maxout);

#elif (PID_MODE == DELTA_PID)
	pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
	pid->iout = pid->ki * pid->error[NOW_ERR];
	pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]);

	pid->out += pid->pout + pid->iout + pid->dout;
	abs_limit(&(pid->out), pid->maxout);

#endif

	pid->error[LLAST_ERR] = pid->error[LAST_ERR];
	pid->error[LAST_ERR] = pid->error[NOW_ERR];

	//  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
	//    return 0;
	//  else
	return pid->out;
}

void PID_Struct_Init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit, INIT_STATUS init_status)
{
	if (init_status == INIT) // ���ڳ�ʼ��
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_reset = pid_reset;

		pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
		pid->f_pid_reset(pid, p, i, d);
	}
	else // ����debug
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
	}
}

void PID_Clear(pid_t *pid)
{
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->error[NOW_ERR] = 0;
	pid->error[LAST_ERR] = 0;
}

pid_t pid_yaw = {0};
pid_t pid_pit = {0};
pid_t pid_yaw_spd = {0};
pid_t pid_pit_spd = {0};

pid_t pid_spd[4] = {0};
pid_t pid_chassis_angle = {0};
pid_t pid_trigger = {0};
pid_t pid_trigger_spd = {0};
pid_t pid_fric[2] = {0};
pid_t pid_Spd_limit = {0};
pid_t pid_heat_limit = {0};
pid_t pid_power = {0};
pid_t pid_heat_time = {0};
pid_t pid_imu_tmp = {0};

/*����ṹ��*/
pid_t pid_buff_yaw = {0};
pid_t pid_buff_pit = {0};
pid_t pid_buff_yaw_spd = {0};
pid_t pid_buff_pit_spd = {0};
/*����ṹ��*/
pid_t pid_vision_yaw = {0};
pid_t pid_vision_pit = {0};
pid_t pid_vision_yaw_spd = {0};
pid_t pid_vision_pit_spd = {0};
/*���нṹ��*/
pid_t pid_init_yaw = {0};
pid_t pid_init_pit = {0};
pid_t pid_init_yaw_spd = {0};
pid_t pid_init_pit_spd = {0};

pid_t pid_speedlimit = {0};

pid_t pid_chassis_vw = {0};

pid_t pid_chassis_power_buffer = {0};
