#include "power_control.h"
#include "chassis_task.h"
#include "pid.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "math.h"

void get_buffer(float *chassis_power_buffer);		   // ��ȡ���役��
void get_chassis_max_power(uint16_t *max_power_limit); // ��ȡ�޶�����

uint8_t cap_state = 0;
float Charge_factor = 1;// 1-Charge_factor = ���ݳ��ٷֱ�

float Chassis_Power_Control(chassis_t *chassis_power_control)
{
	uint16_t max_power_limit = 40; // ��ʼ����40w(û����) -- ���ڼ����޶�����

	float chassis_max_power = 0; // ��������������
	float input_power = 0;		 // �޶����ʣ����Ե�ܣ�
	float initial_give_power[4]; // ԭ�Ƚ�Ҫ�ﵽ�Ĺ���
	float initial_total_power = 0;
	float scaled_give_power[4];
	float total_give = 0; // ���ո���Ĺ���

	float chassis_power_buffer = 0.0f;

	float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55 --- n*toque_coefficient = ת��/9.55 �� rpm*n*toque_coefficient = Pm(��е����)

	float k2 = 1.23e-07;  // k2
	float k1 = 1.453e-07; // k1
	float constant = 4.081f;


	get_buffer(&chassis_power_buffer);							   // ��ȡ���役��
	pid_calc(&pid_chassis_power_buffer, chassis_power_buffer, 30); // �������������ѻ��役��ѹ��30J�����Ϊ��
	get_chassis_max_power(&max_power_limit);					   // ��ȡ��ǰ�ȼ��Ĺ�������
	input_power = max_power_limit - pid_chassis_power_buffer.out;  // ���µ�ǰ�������������

	send_cap_power_can(input_power * 100); // �����ƹ��ʷ��͸����ذ�(��λ:w)

	// ѡ���Ƿ����ĵ���
	if (FAST_SPD || rc.ch2 == 660)
	{
		cap_state = 1;
	}
	else
	{
		cap_state = 0;
	}

	if (chassis_power_control->CapData[1] > 15) // ���ݵ�ѹ>15V
	{
		if (cap_state == 0) // �رյ��ݼ���
		{
				chassis_max_power = (input_power + 5)*Charge_factor; // �������̵������������Դ������ƹ��ʣ���ֹ����һֱ���絼�¹����˷�
		}
		else // �������ݼ���
		{
			chassis_max_power = input_power + 200;
		}
	}
	else // ���ݵ������㣬�������������� = ���ƹ���
	{
		chassis_max_power = input_power*Charge_factor;
	}

	for (uint8_t i = 0; i < 4; i++) // ����ÿ������ԭ����Ҫ�ﵽ�Ĺ���
	{
		// Pin = Pm + k1w^2 + k2tao^2 + constant
		// ��Ϲ�ʽ:Pin = Pm + k1*w^2 + k2*Icmd^2 + constant(�ø���3508��n����tao)
		initial_give_power[i] = chassis_power_control->current[i] * toque_coefficient * chassis_power_control->wheel_spd_fdb[i] +
								k1 * chassis_power_control->wheel_spd_fdb[i] * chassis_power_control->wheel_spd_fdb[i] +
								k2 * chassis_power_control->current[i] * chassis_power_control->current[i] + constant;

		if (initial_give_power[i] < 0) // ���������綯�ƣ��򲻼����ܹ��ʵļ���
			continue;
		initial_total_power += initial_give_power[i]; // ����ԭ�Ƚ�Ҫ�ﵽ�Ĺ���
	}
	total_give = initial_total_power;


	if (initial_total_power > chassis_max_power) // ���̽�Ҫ������--���й��ʿ��ƣ� ��û���������pid������������
	{
		float power_scale = chassis_max_power / initial_total_power; // ����ϵ������ÿ������ĵ�ǰ���ʳ�������ϵ�����õ�������������ʣ��������ܺ� = chassis_max_power��
		total_give = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // ��ȡÿ���������ź����������
			total_give += scaled_give_power[i];
			if (scaled_give_power[i] < 0) // �ų����ʵľͲ��������
			{
				continue;
			}
			//���ݵ�ǰת�٣���������õ���ﵽ�������ʵ�tao
			float a = k2;
			float b = toque_coefficient * chassis_power_control->wheel_spd_fdb[i];
			float c = k1 * chassis_power_control->wheel_spd_fdb[i] * chassis_power_control->wheel_spd_fdb[i] - scaled_give_power[i] + constant;

			if (chassis_power_control->current[i] > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					chassis_power_control->current[i] = 16000;
				}
				else
					chassis_power_control->current[i] = temp;
			}
			else
			{
				float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_power_control->current[i] = -16000;
				}
				else
					chassis_power_control->current[i] = temp;
			}
		}
	}

	return total_give;
}

void get_buffer(float *chassis_power_buffer)
{
	if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 45 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
		*chassis_power_buffer = judge_recv_mesg.power_heat_data.buffer_energy;
	else
		*chassis_power_buffer = 30; // �ý�����pid���Ϊ0
}
void get_chassis_max_power(uint16_t *max_power_limit)
{
	if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 45 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
		*max_power_limit = judge_recv_mesg.game_robot_state.chassis_power_limit;
	else
		*max_power_limit = Debug_Power; // û�������ϵͳ����ܹ�������޶�40w
}
