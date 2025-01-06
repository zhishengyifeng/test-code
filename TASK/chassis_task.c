#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
//#include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "data_packet.h"
#include "arm_math.h"

/*2022��������

��ʼ״̬			100 40

��������
        1��		150	60
        2��		200 80
        3��		250	100
Ѫ������
        1��		200	45
        2��		300	50
        3��		400	55

*/

UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

extern pid_t pid_chassis_vw;
extern pid_t pid_chassis_power_buffer;

chassis_t chassis;

//ǰ������תpid �������ٶ�pid
#if (INFANTRY_NUM == INFANTRY_1) 

//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_2))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_3))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_4))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.0, 0.0f, 0};
#elif((INFANTRY_NUM == INFANTRY_5))
//float chassis_pid[6] = {0,0, 0, 0, 0, 0};
float chassis_pid[6] = {10.0f, 0.0, 0.0f, 10.5, 0.0f, 0};
#else
		#error "INFANTRY_NUM define error!"
#endif

float cap_vol_pid[3] = {1.0f, 0, 0.0f}; //�������ݵ�ѹ����PID
float chassis_power_buffer_pid[3] = {1, 0, 0};//���役������PID
float chassis_vw_pid[3] = {0.1, 0, 0.1};//С����ת�ٿ���PID

float cap_store=24;//���ݴ洢��С(������ֵΪ25)
float cx=0,cy=0,cw=0;//��������ֵ����
uint8_t fast_flag = 0; //����״̬��־λ
int Speed_up = 1;//���ټ���־λ
float cap_ratio;//����ֵ�������������ݵ�ǰ����ֵ�����������������

void chassis_task(void *parm)
{
  uint32_t Signal;
  BaseType_t STAUS;

  while (1)
  {
    STAUS = xTaskNotifyWait((uint32_t)NULL,
                            (uint32_t)INFO_GET_CHASSIS_SIGNAL,
                            (uint32_t *)&Signal,
                            (TickType_t)portMAX_DELAY);
    if (STAUS == pdTRUE)
    {
//		if(INFANTRY_NUM != INFANTRY_2){//�������ڲ�������װ�����°�����飬��ѹ�����ȶ���25V���������ʵʱ���µ�������ֵ,��ȫ������������ѹ�󣬵���ʵʱ���������ĺ�������ɾ��
//			if((judge_recv_mesg.game_robot_state.mains_power_chassis_output==1&&chassis.CapData[1]>20)||chassis.CapData[1]>cap_store)
//			Cap_refresh();
//		}
			
      if (Signal & INFO_GET_CHASSIS_SIGNAL)
      {
        /*����vw��ת��pid*/
        PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, DONE);
        PID_Struct_Init(&pid_chassis_power_buffer, chassis_power_buffer_pid[0], chassis_power_buffer_pid[1], chassis_power_buffer_pid[2], 50, 10, DONE);
        PID_Struct_Init(&pid_chassis_vw, chassis_vw_pid[0], chassis_vw_pid[1], chassis_vw_pid[2], 500, 50, DONE);
				
				if(Speed_up&&chassis.CapData[1]!=0)
				cap_ratio=1-((cap_store-chassis.CapData[1])/(cap_store-10));
				else
				cap_ratio=1;
				
        /*����vx,vyƽ�Ƶ�pid*/
        for (int i = 0; i < 4; i++)
          PID_Struct_Init(&pid_spd[i], chassis_pid[3], chassis_pid[4], chassis_pid[5], 15000*cap_ratio, 1384*cap_ratio, DONE);

        if (chassis_mode != CHASSIS_RELEASE && gimbal.state != GIMBAL_INIT_NEVER) //��̨����֮����̲��ܶ�
        {
          switch (chassis_mode)
          {
          /*��̨���̸���ģʽ*/
          case CHASSIS_NORMAL_MODE:
          {
            chassis_normal_handler();
          }
          break;
          /*��̨���̷���ģʽ*/
          case CHASSIS_SEPARATE_MODE:
          {
            chassis_separate_handler();
          }
          break;
          /*С����ģʽ*/
          case CHASSIS_DODGE_MODE:
          {
            chassis_dodge_handler();
          }
          break;
          /*����ֹͣģʽ*/
          case CHASSIS_STOP_MODE:
          {
            chassis_stop_handler();
          }
          break;

          default:{}
          break;
          }

					if(FAST_SPD && fast_flag){
					cx=chassis.vx;
					cy=chassis.vy;
					cw=chassis.vw;
					}else{
					buffer(&cx,chassis.vx,50,100);	//������	
					buffer(&cy,chassis.vy,50,100);		
					buffer(&cw,chassis.vw, 1, 1);
					}
					
          mecanum_calc(cx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
		
          for (int i = 0; i < 4; i++)
            chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);

          if (!chassis_is_controllable())
            memset(chassis.current, 0, sizeof(chassis.current));

          memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
        }
        else
          memset(glb_cur.chassis_cur, 0, sizeof(glb_cur.chassis_cur));

        xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
                           (uint32_t)CHASSIS_MOTOR_MSG_SIGNAL,
                           (eNotifyAction)eSetBits,
                           (uint32_t *)NULL);
      }
    }

    chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

void Cap_refresh(){	//����ˢ�²���¼��ǰ������ܸ����ݳ������ѹ
//���ݵ����ѹֵ���ŵ�ظ������б仯�������Ҫ����ˢ��
	static int i=0;
	if(judge_recv_mesg.power_heat_data.chassis_power_buffer==60&&(chassis.CapData[0]-chassis.CapData[1]<2||chassis.CapData[1]>=20)){
	if(i!=10)i++;
	}else
		i=0;
	if(i==10)cap_store=chassis.CapData[1];
	if(chassis.CapData[1]>cap_store)cap_store=chassis.CapData[1];
	if(cap_store<20.f)cap_store=20;
};



extern int Speed_up;
//chassis_power_contorl(&pid_power,&chassis_vx,&chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
void chassis_power_contorl(pid_t *power_pid,float *power_vx,float *power_vy,float *power_yaw_speed,float real_time_Cap_remain,float real_time_Cap_can_store,float judge_power_limit)
{
    static  float max = 3000;
    static float min = 1150;//1000;
    static float parameter;            // ����ϵ��
    static float Cap_low = 14.0f;      // ��ʱֹͣ����
    float cap_flag = real_time_Cap_can_store - 18.0f; // 18~real_time_Cap_can_store��һ�ν��а�������ֵ
    if (real_time_Cap_remain < 15.0f)
    {
        *power_yaw_speed = (5 - (15.0f - real_time_Cap_remain)) / 5 * 0.01f;
    } // �͵��ݰ�����������������ƶ��ٶ�
    else
    {
        *power_yaw_speed = 0.01f;
    }

    if (real_time_Cap_remain > 18.0f && real_time_Cap_remain < real_time_Cap_can_store)
    {
          min = 1350 + (cap_flag - (real_time_Cap_can_store - real_time_Cap_remain)) / cap_flag * (judge_power_limit - 45) / 45 * 500;
		}
    else
    {
        min = 1150;//1000;
    }
        // �����ټ���Speed_up = 0,��3508������ٶȽ��г��
        if (Speed_up == 1)
        {

            if (real_time_Cap_remain == real_time_Cap_can_store)
            { // ��⵽���ݳ�����pid���м���
                pid_calc(power_pid, real_time_Cap_remain + 0.5f, real_time_Cap_can_store);
            }
            else
            {
							if(real_time_Cap_remain!=0)
                pid_calc(power_pid, real_time_Cap_remain, real_time_Cap_can_store);
            }
            if (power_pid->out > 0)
            {
                parameter = 1;
            } // ��������ɷֱ�ı����ٵ����ʻ��ٵ�����
            else
            {
                parameter = 1;
            }
        }
        else if (real_time_Cap_remain < Cap_low)
            Speed_up = 1; // һ����⵽���ݵ����Զ��رռ���

        if (km.vy > 0 || rm.vy > 0)
        {
            if (Speed_up == 0)
            {
                *power_vy = CHASSIS_KB_MAX_SPEED_Y;
            }
            else
            {
                if (*power_vy < 0)
                {
                    *power_vy = -*power_vy;
                }
                *power_vy -= parameter * power_pid->out;
                VAL_LIMIT(*power_vy, min * 0.7f, max); // �͵���ƽ�ƣ��ٶ����޸���
            }
        }
        else
        {
            if (km.vy < 0 || rm.vy < 0)
            {
                if (Speed_up == 0)
                    *power_vy = -CHASSIS_KB_MAX_SPEED_Y;
                else
                {
                    if (*power_vy > 0)
                    {
                        *power_vy = -*power_vy;
                    }
                    *power_vy += parameter * power_pid->out;
                    VAL_LIMIT(*power_vy, -max, -min * 0.7f);
                }
            }
            else
                *power_vy = 0;
        }

        if (km.vx > 0 || rm.vx > 0)
        {
            if (Speed_up == 0)
            {
                *power_vx = CHASSIS_KB_MAX_SPEED_X;
            }
            else
            {
                if (*power_vx < 0)
                {
                    *power_vx = -*power_vx;
                }
                *power_vx -= parameter * power_pid->out;
                VAL_LIMIT(*power_vx, min, max);
            }
        }
        else
        {
            if (km.vx < 0 || rm.vx < 0)
            {
                if (Speed_up == 0)
                    *power_vx = -CHASSIS_KB_MAX_SPEED_X;
                else
                {
                    if (*power_vx > 0)
                    {
                        *power_vx = -*power_vx;
                    }
                    *power_vx += parameter * power_pid->out;
                    VAL_LIMIT(*power_vx, -max, -min);
                }
            }
            else
            {
                *power_vx = 0;
            }
        }

    if (SLOW_SPD) // ����
    {
        if (*power_vx > 0)
            *power_vx = 1000;
        if (*power_vx < 0)
            *power_vx = -1000;
        if (*power_vy > 0)
            *power_vy = 1000;
        if (*power_vy < 0)
            *power_vy = -1000;
    }
    else if (FAST_SPD && fast_flag) // ����
    {
        Speed_up = 0;
    }
    else // �����ٶ�
    {
        Speed_up = 1;
    }

    if (real_time_Cap_remain > Cap_low && !FAST_SPD)
    {
        fast_flag = 1;
    } // ��ֹ���ٹ����з������ؼ���ģʽ���鴤��
    if (real_time_Cap_remain < Cap_low)
    {
        fast_flag = 0;
    }
}

static void chassis_normal_handler(void)
{
  float nor_chassis_vx,nor_chassis_vy;
  //vx,vy,yaw_speed power control
  chassis_power_contorl(&pid_power,&nor_chassis_vx,&nor_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
  
  int position_ref = 0;
  if (chassis_mode == CHASSIS_NORMAL_MODE)
  {
    if (input_flag == 1) 
		{chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));}
    else
		{chassis.vw = 0;}
  }
  chassis.vx=nor_chassis_vx;
  chassis.vy=nor_chassis_vy;
}


static void chassis_separate_handler(void)
{
  float cha_x, cha_y, separ_angle;
  separ_angle = gimbal.sensor.yaw_relative_angle;
  cha_x = (rm.vx + km.vx);
  cha_y = (rm.vy + km.vy);
  chassis.vy = (cha_y * arm_cos_f32(-PI / 180 * separ_angle) - cha_x * arm_sin_f32(-PI / 180 * separ_angle));
  chassis.vx = (cha_x * arm_cos_f32(-PI / 180 * separ_angle) + cha_y * arm_sin_f32(-PI / 180 * separ_angle));
  chassis.vw = -pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, 45);
}

static void chassis_dodge_handler(void)
{
	float chassis_vx_dodge,chassis_vy_dodge;
    float dodge_angle;
	float dodge_min=150;
	float dodge_max=500;
	float dodge_chassis_vx,dodge_chassis_vy;
	static int dodge_cap=0;
	
	if(last_chassis_mode!=CHASSIS_DODGE_MODE){
	chassis.vw=dodge_min+((float)judge_recv_mesg.game_robot_state.chassis_power_limit-45)/45*150;
	
	if(cap_store-chassis.CapData[1]<0.5)dodge_cap=chassis.CapData[1]-0.5;//��ֹ���ƽ��
		else 
			dodge_cap=chassis.CapData[1];
		
	}
	
  /*С����*/
  dodge_angle = gimbal.sensor.yaw_relative_angle;

  if ((rm.vx != 0 || rm.vy != 0) &&(km.vx == 0 || km.vy == 0))//С����ʱң�ط��򽵵��ٶ�
  {
    dodge_chassis_vx = rm.vx * 0.5f;
    dodge_chassis_vy = rm.vy * 0.5f;
  }
  else
		//����ʱ��ͨ�������㷨�������ٶȣ���ֹ������
		//vx,vy,yaw_speed power control
    	chassis_power_contorl(&pid_power,&dodge_chassis_vx,&dodge_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32( PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32( PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32( PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32( PI / 180 * dodge_angle));


  pid_calc(&pid_chassis_vw, chassis.CapData[1], dodge_cap);

  chassis.vw -= pid_chassis_vw.out;
  if (chassis.vw < dodge_min)
    chassis.vw = dodge_min;
  if (chassis.vw > dodge_max)
    chassis.vw = dodge_max;

}



static void chassis_stop_handler(void)
{
  for (int i = 0; i < 4; i++)
  {
    chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], 0);
  }

  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}

void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis));

  /*����vw��ת��pid*/
  PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, INIT);
  PID_Struct_Init(&pid_power, cap_vol_pid[0], cap_vol_pid[1], cap_vol_pid[2], 100, 20, INIT);
  /*����vx,vyƽ�Ƶ�pid*/
  for (int i = 0; i < 4; i++)
    PID_Struct_Init(&pid_spd[i], chassis_pid[3], chassis_pid[4], chassis_pid[5], 10000, 500, INIT);
}

/**
 * @brief mecanum chassis velocity decomposition
 * @param input : ��=+vx(mm/s)  ��=+vy(mm/s)  ccw=+vw(deg/s)
 *        output: every wheel speed(rpm)
 * @trans ���룺		ǰ�����ҵ���
 *				 �����		ÿ�����Ӷ�Ӧ���ٶ�
 * @note  1=FR 2=FL 3=BL 4=BR
 * @work	 �������㹫ʽ�����Ч��
 */
int rotation_center_gimbal = 0;
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;

  taskENTER_CRITICAL();
  if (chassis_mode == CHASSIS_DODGE_MODE || chassis.dodge_ctrl)
  {
    chassis.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis.rotate_y_offset = 0;
  }
  else
  {
    if (rotation_center_gimbal)
    {
      chassis.rotate_x_offset = glb_struct.gimbal_x_offset;
      chassis.rotate_y_offset = glb_struct.gimbal_y_offset;
    }
    else
    {
      chassis.rotate_x_offset = 0;
      chassis.rotate_y_offset = 0;
    }
  }
  //@works
  rotate_ratio_fr = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f - chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_fl = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f - chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_bl = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f + chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_br = ((glb_struct.wheel_base + glb_struct.wheel_track) / 2.0f + chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;

  wheel_rpm_ratio = 60.0f / (glb_struct.wheel_perimeter * CHASSIS_DECELE_RATIO);
  taskEXIT_CRITICAL();

  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
  /*С���������ģʽ��vw��������*/
  if (chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED); // deg/s
  /*С����ʱ��vw��������*/
  //	else
  //		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

  int16_t wheel_rpm[4];
  float max = 0;

  wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = (vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = (vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

  // find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  // equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)

      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}


void buffer(float *a, float b, float high_parameter, float low_parameter)
{

  if (((*a - b) < high_parameter) && ((*a - b) > -low_parameter))
  {
    *a = b;
  }
  else
  {
    if (*a < b)
      *a += high_parameter;
    if (*a > b)
      *a -= low_parameter;
  }
}
