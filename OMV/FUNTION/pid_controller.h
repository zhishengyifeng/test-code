/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "stdint.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID �Ż�����ʹ�ܱ�־λ,ͨ��λ������ж����õ��Ż�����;Ҳ���Ըĳ�λ�����ʽ
typedef enum
{
    PID_IMPROVE_NONE = 1,                // 0000 0000
    PID_Integral_Limit = (1 << 1),              // 0000 0001
    PID_Derivative_On_Measurement = (1 << 2),   // 0000 0010
    PID_Trapezoid_Intergral = (1 << 3),         // 0000 0100
    PID_Proportional_On_Measurement = (1 << 4), // 0000 1000
    PID_OutputFilter = (1 << 5),                // 0001 0000
    PID_ChangingIntegrationRate = (1 << 6),     // 0010 0000
    PID_DerivativeFilter = (1 << 7),            // 0100 0000
    PID_ErrorHandle = (1 << 8),                 // 1000 0000
} PID_Improvement_e;

/* PID ��������ö��*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID�ṹ�� */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // �����޷�
    float CoefA;             // ���ٻ��� For Changing Integral
    float CoefB;             // ���ٻ��� ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // ����˲��� RC = 1/omegac
    float Derivative_LPF_RC; // ΢���˲���ϵ��

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* ����PID��ʼ���Ľṹ��*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // ����޷�
    float DeadBand; // ����

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // �����޷�
    float CoefA;         // ABΪ���ٻ��ֲ���,���ٻ���ʵ���Ͼ������˻��ַ���
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;

/**
 * @brief ��ʼ��PIDʵ��
 * @todo ���޸�Ϊͳһ��PIDRegister���
 * @param pid    PIDʵ��ָ��
 * @param config PID��ʼ������
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);
	
/**
 * @brief ����PID���
 *
 * @param pid     PIDʵ��ָ��
 * @param measure ����ֵ
 * @param ref     �趨ֵ
 * @return float  PID�������
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

//extern PIDInstance pid_yaw;
//extern PIDInstance pid_pit;
//extern PIDInstance pid_yaw_spd;
//extern PIDInstance pid_pit_spd;
//extern PIDInstance pid_spd[4];
//extern PIDInstance pid_chassis_angle;
//extern PIDInstance pid_trigger;
//extern PIDInstance pid_trigger_spd;
//extern PIDInstance pid_fric[2];
//extern PIDInstance pid_heat_limit;
//extern PIDInstance pid_power;
//extern PIDInstance pid_heat_time;
//extern PIDInstance pid_imu_tmp;
//  
///*����*/
//extern PIDInstance pid_buff_yaw;
//extern PIDInstance pid_buff_pit;
//extern PIDInstance pid_buff_yaw_spd;
//extern PIDInstance pid_buff_pit_spd;  	
///*����*/
//extern PIDInstance pid_vision_yaw;
//extern PIDInstance pid_vision_pit;
//extern PIDInstance pid_vision_yaw_spd;
//extern PIDInstance pid_vision_pit_spd;                     
///*����*/                     
//extern PIDInstance pid_init_yaw;             
//extern PIDInstance pid_init_pit;            
//extern PIDInstance pid_init_yaw_spd;         
//extern PIDInstance pid_init_pit_spd;                    
//                     
//extern PIDInstance pid_chassis_vw;			
//extern PIDInstance chassis_vw_cap;	

//extern PIDInstance pid_speedlimit;
//extern PIDInstance pid_chassis_power_buffer;		  

#endif
