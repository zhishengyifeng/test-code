#ifndef _Exp_Calculate_grade_H
#define _Exp_Calculate_grade_H


void Exp_Calculate_grade_Init(void);

typedef struct
{
int chassis_power_limit;//��������
int chassis_power_HPF;//Ѫ������ 
int shooter_17mm_cooling_rate_EF;//17mm������ȴ���ʱ�������
int shooter_17mm_cooling_rate_CF;//17mm������ȴ������ȴ����
int robot_level;//�ȼ�
int shooter_17mm_cooling_limit_EF;//��������ǹ����������
int shooter_17mm_cooling_limit_CF;//��ȴ����ǹ����������
}Calculate_grade_d;
extern Calculate_grade_d Calculate_grade;

//����ϵͳ������������
/*

	GAME_ROBOT_ALL_POS_ID              = 0x020B,  //���������λ������                     ����//�����ձ���
	GIME_ROBOT_RADAR_MARK_ID    		   = 0x020C,  //�״��ǽ�������                       ����//�����״������
	GAME_SENTRY_DECISION_SYNC_ID       = 0x020D,	//�ڱ��������������Ϣͬ��							  ����//�����ձ���
	GAME_RADAR_DECISION_SYNC_ID				 = 0x020E,	//�״�����������Ϣͬ��									  ����//�����״�����˵�

	CLIENT_MINIMAP_INTERACTIVE_RADAR_DATE_ID = 0x0305,//ѡ�ֶ�С��ͼ�����״�����           ����//
	CUSTOM_CONTROLLER_INTERACTIVE_DATE_ID    = 0x0306,//�Զ����������ѡ�ֶ˽�������				����//
	CUSTOM_MINIMAP_SENTRY_DATE_ID						 = 0x0307,//ѡ�ֶ�С��ͼ�����ڱ�����           ����//���ڱ��Ͱ��Զ����ƻ����˵ĺ��񲢲���Ҫ
	CLIENT_MINIMAP_INTERACTIVE_ROBOT_DATE_ID = 0x0308,//ѡ�ֶ�С��ͼ���ջ�������Ϣ         ����//

	������Ҳ�������Ķ����ļ����Ѿ�����

	����0x0301���ļ���ȷ����rx���ļ�����tx���ļ�

*/

#endif
