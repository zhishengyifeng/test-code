#include "power_control.h"
#include "chassis_task.h"
#include "pid.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "math.h"

void get_buffer(float* chassis_power_buffer);//��ȡ���役��
void get_chassis_max_power(uint16_t* max_power_limit);//��ȡ�޶�����

uint8_t cap_state = 0;

//deubg
float test_inti = 0;
float test_limit = 0;
float test_power_init = 0;
float test_power_give_0 = 0;
float test_power_give_1 = 0;
float test_power_give_2 = 0;
float test_power_give_3 = 0;
float test_total_give = 0;
float ob_cap_store;

float Chassis_Power_Control(chassis_t *chassis_power_control)
{

	uint16_t max_power_limit = 40;//��ʼ����40w(û����) -- ���ڼ����޶�����
	float chassis_max_power = 0;//��������������
	float input_power = 0;		 // �޶����ʣ����Ե�ܣ�
	float initial_give_power[4]; // ԭ�Ƚ�Ҫ�ﵽ�Ĺ���
	float initial_total_power = 0;
	float scaled_give_power[4];

	float chassis_power_buffer = 0.0f;

	float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55 --- n*toque_coefficient = ת��/9.55 �� rpm*n*toque_coefficient = Pm(��е����)
//	float k2 = -2.492e-06;						 // k2
//	float k1 = 2.107e-07;					 // k1
//	float constant = 1.97f;        //��̬���
	
	float k2 = 1.23e-07;						 // k2
	float k1 = 1.453e-07;					 // k1
	float constant = 4.081f;
	
	float cap_percent = (chassis_power_control->CapData[1]/25)*100; //���ݵ����ٷֱ�(����25V)

	get_buffer(&chassis_power_buffer);  //��ȡ���役��
	pid_calc(&pid_chassis_power_buffer,chassis_power_buffer, 30); //�������������ѻ��役��ѹ��30J�����Ϊ��
	get_chassis_max_power(&max_power_limit);  //��ȡ��ǰ�ȼ��Ĺ�������
	input_power = max_power_limit - pid_chassis_power_buffer.out; // ���µ�ǰ�������������

 
		send_cap_power_can(input_power*100);// �����ƹ��ʷ��͸����ذ�(��λ:w)

	//ѡ���Ƿ����ĵ���
	if (FAST_SPD)
	{
		cap_state = 1;
	}
	else
	{
		cap_state = 0;
	}
	
	if (cap_percent > 5)//���ݵ���>5%
	{
		if (cap_state == 0) // �رյ��ݼ���
		{
			chassis_max_power = input_power + 5; // �������̵������������Դ������ƹ��ʣ���ֹ����һֱ���絼�¹����˷ѣ�û�Ӳ���ϵͳ�������٣�С�����̹�������55w��
		}
		else // �������ݼ���
		{
			chassis_max_power = input_power + 200; 
		}
	}
	else // ���ݵ������㣬�������������� = ���ƹ���
	{
		chassis_max_power = input_power;
	}
	

	for (uint8_t i = 0; i < 4; i++) // ����ÿ������ԭ����Ҫ�ﵽ�Ĺ���
	{
		// Pin = Pm + k1w^2 + k2tao^2 + constant
		// ��Ϲ�ʽ:Pin = Pm + k1*w^2 + k2*Icmd^2 + constant(�ø���3508��n����tao)
		initial_give_power[i] = chassis_power_control->current[i] * toque_coefficient * chassis_power_control->wheel_spd_fdb[i] + 
								k1 * chassis_power_control->wheel_spd_fdb[i] * chassis_power_control->wheel_spd_fdb[i] +
								k2 * chassis_power_control->current[i] * chassis_power_control->current[i]+ constant;

		if (initial_give_power[i] < 0) // ���������綯�ƣ��򲻼����ܹ��ʵļ���
			continue;
		initial_total_power += initial_give_power[i]; // ����ԭ�Ƚ�Ҫ�ﵽ�Ĺ���
	}
	
	test_power_init = initial_give_power[0];
	test_inti = initial_total_power;
	test_limit = chassis_max_power;
	if (initial_total_power > chassis_max_power) // ���̽�Ҫ������--���й��ʿ��ƣ� ��û���������pid������������
	{
		float power_scale = chassis_max_power / initial_total_power; // ����ϵ������ÿ������ĵ�ǰ���ʳ�������ϵ�����õ�������������ʣ��������ܺ� = chassis_max_power��
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // ��ȡÿ���������ź����������
			if (scaled_give_power[i] < 0) // �ų����ʵľͲ��������
			{
				continue;
			}
			//���ݵ�ǰת�٣���������õ���ﵽ�������ʵ�tao
			float b = toque_coefficient * chassis_power_control->wheel_spd_fdb[i];
			float c = k2 * chassis_power_control->wheel_spd_fdb[i] * chassis_power_control->wheel_spd_fdb[i] - scaled_give_power[i] + constant;

			if (chassis_power_control->current[i] > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * k1 * c)) / (2 * k1);
				if (temp > 16000)
				{
					chassis_power_control->current[i] = 16000;
				}
				else
					chassis_power_control->current[i] = temp;
			}
			else
			{
				float temp = (-b - sqrt(b * b - 4 * k1 * c)) / (2 * k1);
				if (temp < -16000)
				{
					chassis_power_control->current[i] = -16000;
				}
				else
					chassis_power_control->current[i] = temp;
			}
		}
	}

	//debug�۲���--�۲��ĸ�3508����Ĺ���
	test_power_give_0 = chassis_power_control->current[0] * toque_coefficient * chassis_power_control->wheel_spd_fdb[0] + 
								k1 * chassis_power_control->wheel_spd_fdb[0] * chassis_power_control->wheel_spd_fdb[0] +
								k2 * chassis_power_control->current[0] * chassis_power_control->current[0]+ constant;
	
	test_power_give_1 = chassis_power_control->current[1] * toque_coefficient * chassis_power_control->wheel_spd_fdb[1] + 
								k1 * chassis_power_control->wheel_spd_fdb[1] * chassis_power_control->wheel_spd_fdb[1] +
								k2 * chassis_power_control->current[1] * chassis_power_control->current[1]+ constant;
	
	test_power_give_2 = chassis_power_control->current[2] * toque_coefficient * chassis_power_control->wheel_spd_fdb[2] + 
								k1 * chassis_power_control->wheel_spd_fdb[2] * chassis_power_control->wheel_spd_fdb[2] +
								k2 * chassis_power_control->current[2] * chassis_power_control->current[2]+ constant;
	
	test_power_give_3 = chassis_power_control->current[3] * toque_coefficient * chassis_power_control->wheel_spd_fdb[3] + 
								k1 * chassis_power_control->wheel_spd_fdb[3] * chassis_power_control->wheel_spd_fdb[3] +
								k2 * chassis_power_control->current[3] * chassis_power_control->current[3]+ constant;
	
	test_total_give = test_power_give_0 + test_power_give_1 + test_power_give_2 + test_power_give_3;
	
	ob_cap_store = chassis.CapData[1];
	
	return test_total_give;
}

void get_buffer(float* chassis_power_buffer)
{
	if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		*chassis_power_buffer = judge_recv_mesg.power_heat_data.chassis_power_buffer;
	else
		*chassis_power_buffer = 30;//�ý�����pid���Ϊ0
}
void get_chassis_max_power(uint16_t* max_power_limit)
{
	if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		*max_power_limit = judge_recv_mesg.game_robot_state.chassis_power_limit;
	else
		*max_power_limit = 39;//û�������ϵͳ����ܹ�������޶�39w
}

