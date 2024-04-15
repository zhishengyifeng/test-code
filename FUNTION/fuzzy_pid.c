#include "fuzzy_pid.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"
#include "gimbal_task.h"

fuzzy_pid fuzzy_t;

// 模糊化论�?
float e_mf_paras[] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3};	// 误差的隶属度函数的参数对照表
float de_mf_paras[] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; // 误差变化率的隶属度函数的参数对照�?
float Kp_mf_paras[] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; // kp的隶属度函数的参照对照表
float Ki_mf_paras[] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; // ki的隶属度函数的参数对照表
float Kd_mf_paras[] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; // kd的隶属度函数的参数对照表

// pid模糊规则�?
int delta_Kp_Matrix[7][7] =
	{{PB, PB, PM, PM, PS, ZO, ZO},
	 {PB, PB, PM, PS, PS, ZO, NS},
	 {PM, PM, PM, PS, ZO, NS, NS},
	 {PM, PM, PS, ZO, NS, NM, NM},
	 {PS, PS, ZO, NS, NS, NM, NM},
	 {PS, ZO, NS, NM, NM, NM, NB},
	 {ZO, ZO, NM, NM, NM, NB, NB}};

int delta_Kd_Matrix[7][7] =
	{{PS, NS, NB, NB, NB, NM, PS},
	 {PS, NS, NB, NM, NM, NS, ZO},
	 {ZO, NS, NM, NM, NS, NS, ZO},
	 {ZO, NS, NS, NS, NS, NS, ZO},
	 {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
	 {PB, NS, PS, PS, PS, PS, PB},
	 {PB, PM, PM, PM, PS, PS, PB}};

int delta_Ki_Matrix[7][7] =
	{{NB, NB, NM, NM, NS, ZO, ZO},
	 {NB, NB, NM, NS, NS, ZO, ZO},
	 {NB, NM, NS, NS, ZO, PS, PS},
	 {NM, NM, NS, ZO, PS, PM, PM},
	 {NM, NS, ZO, PS, PS, PM, PB},
	 {ZO, ZO, PS, PS, PM, PB, PB},
	 {ZO, ZO, PS, PM, PM, PB, PB}};

/*
	1、确定基本论�?
	2、基本论�? 量化 模糊子集
	3、模糊逻辑推理
	4、反模糊�?
*/

// 模糊控制�?
void fuzzy_calc(pid_t *pid)
{
	//***********************确定基本论域
	fuzzy_t.percent = 1.13f; // 放大论域

	fuzzy_t.e_max = pid->maxout * fuzzy_t.percent,
	fuzzy_t.de_max = 2 * fuzzy_t.e_max, // 误差变化�?

		fuzzy_t.Kp_max = pid->kp * fuzzy_t.percent,
	fuzzy_t.Ki_max = pid->ki * fuzzy_t.percent,
	fuzzy_t.Kd_max = pid->kd * fuzzy_t.percent;

	//***********************确定 量化因子 �? 比例因子
	fuzzy_t.Ke = (NN / 2) / fuzzy_t.e_max;
	fuzzy_t.Kde = (NN / 2) / fuzzy_t.de_max;

	fuzzy_t.Ku_p = fuzzy_t.Kp_max / (NN / 2);
	fuzzy_t.Ku_i = fuzzy_t.Ki_max / (NN / 2);
	fuzzy_t.Ku_d = fuzzy_t.Kd_max / (NN / 2);

	//***********************输入 模糊�?
	fuzzy_t.e = fuzzy_t.Ke * pid->error[NOW_ERR];
	fuzzy_t.de = fuzzy_t.Kde * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);

	// 1到n的部分的最值和模糊因子，自己找到对应的调整
	float u_e[NN] = {0}, u_de[NN] = {0};			//, u_u[N]
	int u_e_index[NN] = {0}, u_de_index[NN] = {0};	// 假设一个输入最多激�?3个模糊子�?
	float delta_Kp = 0, delta_Ki = 0, delta_Kd = 0; // 反模糊化的出来的�?

	//***********************输入值的隶属�?
	// 找到e的隶属度
	int j = 0;
	for (int i = 0; i < NN; i++)
	{
		u_e[i] = trimf(fuzzy_t.e, e_mf_paras[i * 3], e_mf_paras[i * 3 + 1], e_mf_paras[i * 3 + 2]); // e的三角隶属函�?

		if (u_e[i] != 0) // 储存被激活的模糊子集的下标，，可以减少计算量
		{
			u_e_index[j] = i;
			j = j + 1;
		}
	}

	// 找到de的隶属度
	j = 0;
	for (int i = 0; i < NN; i++)
	{
		u_de[i] = trimf(fuzzy_t.de, de_mf_paras[i * 3], de_mf_paras[i * 3 + 1], de_mf_paras[i * 3 + 2]); // de的三角隶属函�?

		if (u_de[i] != 0) // 储存被激活的模糊子集的下标，可以减少计算�?
		{
			u_de_index[j] = i;
			j = j + 1;
		}
	}

	//***********************模糊逻辑推理 反模糊化
	float num, den;
	// 计算delta_Kp �? Kp反模糊化
	den = 0;
	num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * delta_Kp_Matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}

	if (den == 0)
		den = 0; // 避免出现分母�?0
	else
	{
		delta_Kp = num / den;
		delta_Kp = fuzzy_t.Ku_p * delta_Kp; // 反模糊化与比例因子的积的出模糊�?
		// 限制范围，防止过饱和
		if (delta_Kp >= fuzzy_t.Kp_max)
			delta_Kp = fuzzy_t.Kp_max;
		else if (delta_Kp <= -(fuzzy_t.Kp_max))
			delta_Kp = -(fuzzy_t.Kp_max);

		pid->kp += delta_Kp; // 得出比较合适的�?

		// 限制最值上下限，避免出现不在响�?
		if (pid->kp < fuzzy_t.Kp_max * 0.618f)
			pid->kp = fuzzy_t.Kp_max * 0.618f;
		if (pid->kp > fuzzy_t.Kp_max)
			pid->kp = fuzzy_t.Kp_max;
	}

	// 计算delta_Ki �? Ki反模糊化
	den = 0;
	num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * delta_Ki_Matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}

	if (den == 0)
		den = 0;
	else
	{
		delta_Ki = num / den;
		delta_Ki = fuzzy_t.Ku_i * delta_Ki;

		if (delta_Ki >= fuzzy_t.Ki_max)
			delta_Ki = fuzzy_t.Ki_max;
		else if (delta_Ki <= -(fuzzy_t.Ki_max))
			delta_Ki = -(fuzzy_t.Ki_max);
		pid->ki += delta_Ki;

		if (pid->ki < fuzzy_t.Ki_max * 0.618f)
			pid->ki = fuzzy_t.Ki_max * 0.618f;
		if ((pid->ki) > (fuzzy_t.Ki_max))
			pid->ki = fuzzy_t.Ki_max;
	}

	// 计算delta_Kd �? Kd反模糊化
	den = 0;
	num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * delta_Kd_Matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}

	if (den == 0)
		den = 0;
	else
	{
		delta_Kd = num / den;
		delta_Kd = fuzzy_t.Ku_d * delta_Kd;

		if (delta_Kd >= fuzzy_t.Kd_max)
			delta_Kd = fuzzy_t.Kd_max;
		else if (delta_Kd <= -(fuzzy_t.Kd_max))
			delta_Kd = -(fuzzy_t.Kd_max);

		pid->kd += delta_Kd;

		if (pid->kd < (fuzzy_t.Kd_max * 0.618f))
			pid->kd = fuzzy_t.Kd_max * 0.618f;
		if ((pid->kd) > (fuzzy_t.Kd_max))
			pid->kd = fuzzy_t.Kd_max;
	}
}

//

// 三角隶属度函�?
float trimf(float x, float a, float b, float c)
{

	float u;
	if (x >= a && x <= b)
		u = (x - a) / (b - a);
	else if (x > b && x <= c)
		u = (c - x) / (c - b);
	else
		u = 0.0;

	return u;
}
