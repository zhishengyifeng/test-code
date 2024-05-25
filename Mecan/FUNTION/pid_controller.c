/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID����������
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "pid_controller.h"
//#include "memory.h"

/* ----------------------------������pid�Ż����ڵ�ʵ��---------------------------- */

// ���λ���
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // �������ε����,(�ϵ�+�µ�)*��/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// ���ٻ���(���Сʱ�������ø�ǿ)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // ���ֳ��ۻ�����
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // �����ֵ,��ʹ�û���
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // ����ȴ�����ۻ�
        {
            pid->ITerm = 0; // ��ǰ����������
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// ΢������(��ʹ�÷���ֵ�����Ʋο������΢��)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// ΢���˲�(�ɼ�΢��ʱ,�˳���Ƶ����)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// ����˲�
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// ����޷�
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// �����ת���
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------������PID���ⲿ�㷨�ӿ�--------------------------- */

/**
 * @brief ��ʼ��PID,���ò��������õ��Ż�����,��������������
 *
 * @param pid    PIDʵ��
 * @param config PID��ʼ������
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // config�����ݺ�pid�Ĳ�����������������ͬ�ĵ�,���Կ���ֱ����memcpy
    // @todo: ������������,����չ�Բ�,��֪���Ŀ����߿��ܻ�����Ϊpid��config��ͬһ���ṹ��
    // �����޸�Ϊ�����ֵ
    memset(pid, 0, sizeof(PIDInstance));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    // set rest of memory to 0
    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // ��ȡ����pid�����ʱ����,���ڻ��ֺ�΢��
}

/**
 * @brief          PID����
 * @param[in]      PID�ṹ��
 * @param[in]      ����ֵ
 * @param[in]      ����ֵ
 * @retval         ���ؿ�
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // ��ת���
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // ��ȡ����pid�����ʱ����,���ڻ��ֺ�΢��

    // �����ϴεĲ���ֵ�����,���㵱ǰerror
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // �����������,�����PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // ������pid����,ʹ��λ��ʽ
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // ���λ���
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // ���ٻ���
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // ΢������
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // ΢���˲���
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // �����޷�
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // �ۼӻ���
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // �������

        // ����˲�
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // ����޷�
        f_Output_Limit(pid);
    }
    else // ��������, ����ջ��ֺ����
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // ���浱ǰ����,�����´μ���
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
