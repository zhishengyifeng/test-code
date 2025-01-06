#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "delay.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()����
#include "math.h"   //fabs()����
#include "kalman_filter.h"

#include "chassis_task.h"

UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

int direction=1;
int direction_change=0;

ramp_t pit_ramp;
ramp_t yaw_ramp;

#if (INFANTRY_NUM == INFANTRY_1)//�޸� INFANTRY_NUM 
	//���в���
	float pit_init_pid[6] = {19.5,0.025,0,110,0,0};
	float yaw_init_pid[6] = {17.5,0.01,5,80,0,0};

	//��ͨ����
	float pit_pid[6] = {36, 0.1, 50, 140.55, 0.1, 50};//{27.8022998317653,0,0.280631669201902,90,0,0};//55,0,120,50,0.01,70//40,0,120,50,0.01,70//{19.5,0,7.5,95,0.01,15}//�൯pid{19.5,0,7.5,105,0.01,15};
	float yaw_pid[6] = {21.0864706788671,0,0.219976397416724,85,0,0};//19.5,0.01,5,90,0,10 //7.. 28��ǰ{17.5,0.01,5,80,0,0};//6.19{17.5,0.002,15,140,0,0};//6.16 //{15.5 ,0.005 ,7.5 ,150 ,0, 0};//5.27//

	//�������
	float pit_vision_pid[6] = {22,0,5,192,1,22};//{19.5,0.009,0.7,132,0.001,0.1};//{18,0.009,0,132,0,0};//{21,0.001,0,132,0,0};//{10,0,0,130,0,0};
	float yaw_vision_pid[6] = {25.5,0,0.6,125,0,0};//{7.5,0,0,125,0,0};//{21.5,0,0,96,0,0};//{10.5,0,0,90,0,0};//{15,0.001,0,40,0.01,0};//17.5,0,0,75,0,0//{19.5,0,60,75,0.05,20}//{10,0,20,110,0,10};//{20,0,5,40,0,7};//{31, 0, 10, 16, 0, 0};   

	//�������
	float pit_buff_pid[6] = {20,  0.3, 0,  140, 0, 5}//{25,  0.3, 0,  50, 0, 0};//{10, 0, 0 ,10, 0, 0};       
	float yaw_buff_pid[6] = {20.2, 0.2, 0, 180, 0, 0//{20.2, 0.2, 0, 100, 0, 0};//{10, 0, 0 ,10, 0, 0};

	//���̲���
	float trig_pid[6] = {125, 0, 100, 10, 0, 0};//{0, 0, 0 , 0, 0, 0};


#elif (INFANTRY_NUM == INFANTRY_2)
	//���в���
	float pit_init_pid[6] = {19.5,0.025,0,110,0,0};
	float yaw_init_pid[6] = {17.5,0.01,5,80,0,0};

	//��ͨ����
	float pit_pid[6] = {19.2295334121012,0,0.203704165044731,90,0,0};//55,0,120,50,0.01,70//40,0,120,50,0.01,70//{19.5,0,7.5,95,0.01,15}//�൯pid{19.5,0,7.5,105,0.01,15};
	float yaw_pid[6] = {23.9211951021423,0,0.246776245664673,85,0,10};//19.5,0.01,5,90,0,10//7.. 28��ǰ{17.5,0.01,5,80,0,0};//6.19{17.5,0.002,15,140,0,0};//6.16 //{15.5 ,0.005 ,7.5 ,150 ,0, 0};//5.27//

	//�������
	float pit_vision_pid[6] = {21,0.001,0,132,0,0};//{15.2,0.006,0,175,0,0};//{10,0,0,130,0,0};
	float yaw_vision_pid[6] = {21.5,0,0,96,0,0};//{14,0,0,133,0,0};//{10.5,0,0,90,0,0};//{15,0.001,0,40,0.01,0};//17.5,0,0,75,0,0//{19.5,0,60,75,0.05,20}//{10,0,20,110,0,10};//{20,0,5,40,0,7};//{31, 0, 10, 16, 0, 0};   
	
	//�������
	float pit_buff_pid[6] = {25,  0.3, 0,  50, 0, 0};//{10, 0, 0 ,10, 0, 0};       
	float yaw_buff_pid[6] = {20.2, 0.2, 0, 100, 0, 0};//{10, 0, 0 ,10, 0, 0};
	//���̲���
	
	float trig_pid[6] = {250, 0, 120, 10, 0, 0};//{0, 0, 0 , 0, 0, 0};


#elif (INFANTRY_NUM == INFANTRY_3)
	//���в���
	float pit_init_pid[6] = {19.5,0.025,0,110,0,0};
	float yaw_init_pid[6] = {17.5,0.01,5,80,0,0};

	//��ͨ����
	float pit_pid[6] = {55,0,120,50,0.01,70};//40,0,120,50,0.01,70//{19.5,0,7.5,95,0.01,15}//�൯pid{19.5,0,7.5,105,0.01,15};
	float yaw_pid[6] = {19.5,0.01,5,90,0,10};//7.. 28��ǰ{17.5,0.01,5,80,0,0};//6.19{17.5,0.002,15,140,0,0};//6.16 //{15.5 ,0.005 ,7.5 ,150 ,0, 0};//5.27//

	//�������
	float pit_vision_pid[6] = {10,0,0,130,0,0};
	float yaw_vision_pid[6] = {10.5,0,0,90,0,0};//{15,0.001,0,40,0.01,0};//17.5,0,0,75,0,0//{19.5,0,60,75,0.05,20}//{10,0,20,110,0,10};//{20,0,5,40,0,7};//{31, 0, 10, 16, 0, 0};   

	//�������
	float pit_buff_pid[6] = {25,  0.3, 0,  50, 0, 0};//{10, 0, 0 ,10, 0, 0};       
	float yaw_buff_pid[6] = {20.2, 0.2, 0, 100, 0, 0};//{10, 0, 0 ,10, 0, 0};
	
	//���̲���
	float trig_pid[6] = {250, 0, 120, 10, 0, 0};//{0, 0, 0 , 0, 0, 0};
 
 
#elif (INFANTRY_NUM == INFANTRY_4)
	//���в���
	float pit_init_pid[6] = {19.5,0.025,0,110,0,0};
	float yaw_init_pid[6] = {17.5,0.01,5,80,0,0};

	//��ͨ����
	float pit_pid[6] = {55,0,120,50,0.01,70};//40,0,120,50,0.01,70//{19.5,0,7.5,95,0.01,15}//�൯pid{19.5,0,7.5,105,0.01,15};
	float yaw_pid[6] = {19.5,0.01,5,90,0,10};//7.. 28��ǰ{17.5,0.01,5,80,0,0};//6.19{17.5,0.002,15,140,0,0};//6.16 //{15.5 ,0.005 ,7.5 ,150 ,0, 0};//5.27//

	//�������
	float pit_vision_pid[6] = {10,0,0,130,0,0};//{18,0.009,0,132,0,0};
	float yaw_vision_pid[6] = {10.5,0,0,90,0,0};//{7.5,0,0,125,0,0};//{15,0.001,0,40,0.01,0};//17.5,0,0,75,0,0//{19.5,0,60,75,0.05,20}//{10,0,20,110,0,10};//{20,0,5,40,0,7};//{31, 0, 10, 16, 0, 0};   

	//�������
	float pit_buff_pid[6] = {25,  0.3, 0,  50, 0, 0};//{10, 0, 0 ,10, 0, 0};       
	float yaw_buff_pid[6] = {20.2, 0.2, 0, 100, 0, 0};//{10, 0, 0 ,10, 0, 0};

	//���̲���
	float trig_pid[6] = {250, 0, 120, 10, 0, 0};//{0, 0, 0 , 0, 0, 0};
	
#elif (INFANTRY_NUM == INFANTRY_5)
	// ���в���
	float pit_init_pid[6] = {25, 0, 0, 170, 0.1, 0};
	float yaw_init_pid[6] = {20, 0, 0, 50, 0, 0};

	// ��ͨ������С��������ͨģʽ����һ�ײ�����
	float pit_pid[6] = {25,0.01,0,170,1,0};
	float yaw_pid[6] = {20, 0, 0, 90, 0, 5};
	// �������
	float pit_vision_pid[6] = {30, 0, 0, 170, 0, 0};
													
	float yaw_vision_pid[6] = {13,0,5,75,0,8};
	// �������
	float pit_buff_pid[6] = {0, 0, 0, 0, 0, 0}; 
	float yaw_buff_pid[6] = {0, 0, 0, 0, 0, 0};

	// ���̲���
	float trig_pid[6] = {130, 0, 10, 11, 0, 5};

#endif

	

//����Ϊ����+�š�����Ϊ��Ϊ-��
float pit_dir =  1.f;
float yaw_dir =  1.f;

float pit_ctrl_ffc;
float yaw_ctrl_ffc;

FFC p_ffc;
FFC y_ffc;

#define PIT_ANGLE_MAX       30               
#define PIT_ANGLE_MIN      -20
#define YAW_ANGLE_MAX       50
#define YAW_ANGLE_MIN      -50

/* ����΢������pc�� */
TD_LADRC td_yaw = {
	.r = 27,
	.h = 0.001,
	.lambda = 90};

TD_LADRC td_pit = {
	.r = 27,
	.h = 0.001,
	.lambda = 90};

void PRE_LADRC_TD(TD_LADRC *td_para, float Expect);	
/* ����΢������pc�� */
	
////�������˲���pc��
//kalman_filter_init_t pc_kalman_filter_para = {
//  .P_data = {2, 0, 0, 2},
//  .A_data = {1, 0.001, 0, 1},
//  .H_data = {1, 0, 0, 1},
//  .Q_data = {1, 0, 0, 1},
//  .R_data = {6000, 0, 0, 2000}
//};

//kalman_filter_t pc_kalman_filter;
/* �������˲���pc�� */
kalman_filter_init_t pc_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.001, 0, 1},
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {6000, 0, 0, 2000}};
kalman_filter_t pc_kalman_filter_pitch;
kalman_filter_t pc_kalman_filter_yaw;
void gimbal_kalman_qr_set()
{
	mat_init(&pc_kalman_filter_pitch.Q, 2, 2, pc_kalman_filter_para.Q_data);
	mat_init(&pc_kalman_filter_pitch.R, 2, 2, pc_kalman_filter_para.R_data);
	mat_init(&pc_kalman_filter_yaw.Q, 2, 2, pc_kalman_filter_para.Q_data);
	mat_init(&pc_kalman_filter_yaw.R, 2, 2, pc_kalman_filter_para.R_data);
}
/* �������˲���pc�� */

gimbal_t gimbal;
uint32_t gimbal_time,last_gimbal_time;

void gimbal_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_GIMBAL_SIGNAL)
			{
        gimbal_time = HAL_GetTick() - last_gimbal_time;
        last_gimbal_time = HAL_GetTick();
				
				//�������˲�				
				// mat_init(&pc_kalman_filter.Q,2,2, pc_kalman_filter_para.Q_data);
				// mat_init(&pc_kalman_filter.R,2,2, pc_kalman_filter_para.R_data); 
				// ������Q R�����������
				//gimbal_kalman_qr_set();
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
        {
			    PID_Struct_Init(&pid_vision_pit, pit_vision_pid[0], pit_vision_pid[1], pit_vision_pid[2], 6000, 500,DONE);
          PID_Struct_Init(&pid_vision_pit_spd, pit_vision_pid[3], pit_vision_pid[4], pit_vision_pid[5], 30000, 3000,DONE);
		
			    PID_Struct_Init(&pid_vision_yaw, yaw_vision_pid[0], yaw_vision_pid[1], yaw_vision_pid[2], 1000, 500,DONE);	
		      PID_Struct_Init(&pid_vision_yaw_spd, yaw_vision_pid[3], yaw_vision_pid[4], yaw_vision_pid[5], 30000, 15000,DONE);
	      }
        else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
	      {
		      PID_Struct_Init(&pid_buff_pit, pit_buff_pid[0], pit_buff_pid[1], pit_buff_pid[2], 1000, 500,DONE);
          PID_Struct_Init(&pid_buff_pit_spd, pit_buff_pid[3], pit_buff_pid[4], pit_buff_pid[5], 20000, 3000,DONE);//25000//6000  
		 
			    PID_Struct_Init(&pid_buff_yaw, yaw_buff_pid[0], yaw_buff_pid[1], yaw_buff_pid[2], 1000, 500,DONE);	
		      PID_Struct_Init(&pid_buff_yaw_spd, yaw_buff_pid[3], yaw_buff_pid[4], yaw_buff_pid[5], 30000, 15000,DONE);//25000
	      }
        else	 
	      {
         /* pit ������PID���� */
         PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 6000, 500, DONE);//6000,500
         PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 28000, 4000, DONE);//30000//22000,4000

         /* yaw ������PID���� */
         PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 5000, 500, DONE);
         PID_Struct_Init(&pid_yaw_spd, yaw_pid[3] , yaw_pid[4], yaw_pid[5], 30000, 15000, DONE); 
         /* ���� �����PID���� */
         PID_Struct_Init(&pid_trigger, trig_pid[0], trig_pid[1], trig_pid[2], 8000, 0, DONE);
         PID_Struct_Init(&pid_trigger_spd, trig_pid[3], trig_pid[4], trig_pid[5],8000, 3000,DONE);
	      }   
        if(gimbal_mode != GIMBAL_RELEASE)
        {
          if(gimbal.state == GIMBAL_INIT_NEVER)
          {
            gimbal_mode = GIMBAL_INIT;
          }
          switch(gimbal_mode)
          {
            case GIMBAL_INIT:
            {
              PID_Struct_Init(&pid_yaw, yaw_init_pid[0], yaw_init_pid[1], yaw_init_pid[2], 6000, 500,DONE);
              PID_Struct_Init(&pid_yaw_spd, yaw_init_pid[3], yaw_init_pid[4], yaw_init_pid[5], 30000, 3000,DONE);
              
              PID_Struct_Init(&pid_pit, pit_init_pid[0], pit_init_pid[1], pit_init_pid[2], 1000, 500,DONE);	
              PID_Struct_Init(&pid_pit_spd, pit_init_pid[3], pit_init_pid[4], pit_init_pid[5], 30000, 15000,DONE);
              init_mode_handler(); //��̨����
            }break;
            /*��̨���̸���ģʽ*/
            case GIMBAL_NORMAL_MODE:
            {
							if(last_gimbal_mode != GIMBAL_NORMAL_MODE ) 
							{	
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
              nomarl_handler();
            }break;
            /*��̨���̷���ģʽ*/
            case GIMBAL_SEPARATE_MODE:
            {
							if(last_gimbal_mode != GIMBAL_SEPARATE_MODE ) 
							{	
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
              separate_handler();
            }break;
            /*С����ģʽ*/
            case GIMBAL_DODGE_MODE:
            {
							if(last_gimbal_mode != GIMBAL_DODGE_MODE ) 
							{	
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
              dodge_handler();
            }break;
            /*����������ģʽ*/
            case GIMBAL_SHOOT_BUFF:
            {
							if(last_gimbal_mode != GIMBAL_SHOOT_BUFF ) 
							{	
								PID_Clear(&pid_buff_pit);
								PID_Clear(&pid_buff_yaw);
								PID_Clear(&pid_buff_pit_spd);
								PID_Clear(&pid_buff_yaw_spd);
							}
              shoot_buff_ctrl_handler();
            }break;
            /*����ģʽ*/
            case GIMBAL_TRACK_ARMOR:
            {
							if(last_gimbal_mode != GIMBAL_TRACK_ARMOR ) 
							{	
								PID_Clear(&pid_vision_pit);
								PID_Clear(&pid_vision_yaw);
								PID_Clear(&pid_vision_pit_spd);
								PID_Clear(&pid_vision_yaw_spd);
							}
              track_aimor_handler();
            }break;
            
            default:
            {
            }break;
          }
        }
        else
        {
          memset(glb_cur.gimbal_cur,0,sizeof(glb_cur.gimbal_cur));
          gimbal.state = GIMBAL_INIT_NEVER;
        }
        
        /*pid����-�ǶȻ�*/
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
				{
					pid_calc(&pid_vision_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
					pid_calc(&pid_vision_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);			
				}
				else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
				{
					pid_calc(&pid_buff_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
					pid_calc(&pid_buff_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);			
				}
				else
				{
					pid_calc(&pid_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
					pid_calc(&pid_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);
				}
        
        /*�ٶȻ�����*/
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
				{
					gimbal.pid.yaw_spd_ref = pid_vision_yaw.out;	 	
					gimbal.pid.pit_spd_ref = pid_vision_pit.out/2;
					gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance * 2;
					gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;
				}
				else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
				{
					gimbal.pid.yaw_spd_ref = pid_buff_yaw.out;	 
					gimbal.pid.pit_spd_ref = pid_buff_pit.out;
					gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;	    
					gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;					
				}
				else
				{
					gimbal.pid.yaw_spd_ref = pid_yaw.out;	 
					gimbal.pid.pit_spd_ref = pid_pit.out;
					gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;	    
					gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;					
				}
				

				
        

				/*pid����-�ٶȻ�*/
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
				{
					pid_calc(&pid_vision_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
					pid_calc(&pid_vision_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);			
				}
				else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
				{
					pid_calc(&pid_buff_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
					pid_calc(&pid_buff_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);					
				}
				else
				{
					pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);					
					pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref); 
        }					
        
        
        
        if (gimbal_is_controllable())              //�ⷢ��ṹʹ��ʱ��ע�͵���̨
        {
          if(gimbal_mode == GIMBAL_TRACK_ARMOR)
          {
						/*���� ��ǰ��*/
						pit_ctrl_ffc = getFeedforwardControl(&p_ffc, gimbal.pid.pit_angle_ref) + pid_vision_pit_spd.out;//
						yaw_ctrl_ffc = getFeedforwardControl(&y_ffc, gimbal.pid.yaw_angle_ref) + pid_vision_yaw_spd.out;// 
						glb_cur.gimbal_cur[0] = yaw_dir * yaw_ctrl_ffc;
						glb_cur.gimbal_cur[1] = pit_dir * pid_vision_pit_spd.out;
						glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
          }
					else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
          {
            glb_cur.gimbal_cur[0] = yaw_dir * pid_buff_yaw_spd.out;
            glb_cur.gimbal_cur[1] = pit_dir * pid_buff_pit_spd.out;
            glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
          }
          else
          {
            glb_cur.gimbal_cur[0] = yaw_dir * pid_yaw_spd.out;
            glb_cur.gimbal_cur[1] = pit_dir * pid_pit_spd.out;
            glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
          }
        }
        else
        {
          memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
          gimbal_mode = GIMBAL_RELEASE;
          pid_trigger.iout = 0;
        }
        
        last_gimbal_mode = gimbal_mode;//��ȡ��һ����̨״̬             
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                            (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
      }
    }
    
    gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}





void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));
  
  gimbal.state = GIMBAL_INIT_NEVER;
  
  ramp_init(&pit_ramp, 1000);
  ramp_init(&yaw_ramp, 1000);
	
     //����PID
			PID_Struct_Init(&pid_pit, pit_vision_pid[0], pit_vision_pid[1], pit_vision_pid[2], 1000, 500,INIT);
      PID_Struct_Init(&pid_pit_spd, pit_vision_pid[3], pit_vision_pid[4], pit_vision_pid[5], 6000, 3000,INIT);
		
			PID_Struct_Init(&pid_yaw, yaw_vision_pid[0], yaw_vision_pid[1], yaw_vision_pid[2], 1000, 500,INIT);	
		  PID_Struct_Init(&pid_yaw_spd, yaw_vision_pid[3], yaw_vision_pid[4], yaw_vision_pid[5], 25000, 15000,INIT);

     //���PID
		  PID_Struct_Init(&pid_pit, pit_buff_pid[0], pit_buff_pid[1], pit_buff_pid[2], 1000, 500,INIT);
      PID_Struct_Init(&pid_pit_spd, pit_buff_pid[3], pit_buff_pid[4], pit_buff_pid[5], 6000, 3000,INIT);  
		 
			PID_Struct_Init(&pid_yaw, yaw_buff_pid[0], yaw_buff_pid[1], yaw_buff_pid[2], 1000, 500,INIT);	
		  PID_Struct_Init(&pid_yaw_spd, yaw_buff_pid[3], yaw_buff_pid[4], yaw_buff_pid[5], 25000, 15000,INIT);
	 

	/* pit ������PID���� */
	PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 3000, 500, INIT);
  PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 6000, 4000, INIT);

  /* yaw ������PID���� */
  PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 5000, 500, INIT); 
  PID_Struct_Init(&pid_yaw_spd, yaw_pid[3] , yaw_pid[4], yaw_pid[5], 25000, 15000, INIT);  
  
  /* ���� �����PID���� */
  PID_Struct_Init(&pid_trigger, trig_pid[0], trig_pid[1], trig_pid[2], 8000, 0, INIT);
  PID_Struct_Init(&pid_trigger_spd, trig_pid[3], trig_pid[4], trig_pid[5],8000, 3000,INIT); 
	
//	//�������˲���ʼ��		 
//  kalman_filter_init(&pc_kalman_filter, &pc_kalman_filter_para);	
	/* �������˲���ʼ�� */
	kalman_filter_init(&pc_kalman_filter_pitch, &pc_kalman_filter_para);
	kalman_filter_init(&pc_kalman_filter_yaw, &pc_kalman_filter_para);
	
  /* ffc ��ʼ��*/
	initFeedforwardParam(&p_ffc, 210, 20);
	initFeedforwardParam(&y_ffc, 200, 40);
  
}

static void init_mode_handler(void)
{
  /* PIT���ȹ��� */
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  
	gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle * ( 1 - ramp_calc(&pit_ramp));//����б�º���ʹpitch�����ԽǶ��𽥱�Ϊ0
  /* ����YAW�᲻�� */
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
  gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
	
  if(gimbal.pid.pit_angle_fdb >= -1.0f && gimbal.pid.pit_angle_fdb <= 1.0f)
  {
    /*  ƫ���������*/   
    gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle *( 1 - ramp_calc(&yaw_ramp));
    
    if (gimbal.pid.yaw_angle_fdb >= -2.0f && gimbal.pid.yaw_angle_fdb <= 2.0f)
    {
      gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;  //ˢ������//��¼��ǰYAW�������ǵĽǶ�
			gimbal.pit_offset_angle = gimbal.sensor.pit_gyro_angle;//new	    		
      gimbal.pid.pit_angle_ref = 0;
      gimbal.pid.yaw_angle_ref = 0;

      gimbal.state = GIMBAL_INIT_DONE;
    }
  }
}

/*�ж�yaw���Ƿ�������*/
static gimbal_state_t remote_is_action(void)
{
  if ((abs(rc.ch3) >= 10) || (abs(rc.mouse.x) >= 1))
  {
      return IS_ACTION;
  }
  else
  {
      return NO_ACTION;
  }
}

uint8_t input_flag;
int no_action_time;
uint32_t debug_time = 1000;	
static void nomarl_handler(void)
{ 
	if(last_gimbal_mode != GIMBAL_NORMAL_MODE ) 
  {   
    gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;
		if(last_gimbal_mode == GIMBAL_INIT || last_gimbal_mode == GIMBAL_SHOOT_BUFF )
			gimbal.pid.yaw_angle_ref = 0;
		else
			gimbal.pid.yaw_angle_ref -= gimbal.yaw_offset_angle;
		
		input_flag = 1;
		no_action_time = HAL_GetTick();
  }
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;
  gimbal.state = remote_is_action(); //�ж�yaw���Ƿ�������

  if(direction_change==1)//��⵽����м�Ĺ������»������������̨�Ƕȵı仯
  {
		gimbal.pid.yaw_angle_fdb  = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;	
		input_flag = 0;
		if(gimbal.pid.yaw_angle_ref-gimbal.pid.yaw_angle_fdb<3&&gimbal.pid.yaw_angle_ref-gimbal.pid.yaw_angle_fdb>-3)
			direction_change=0;
  }
  else if((gimbal.last_state == NO_ACTION)        
        &&(gimbal.state == NO_ACTION)
        &&(HAL_GetTick() - no_action_time > debug_time)){
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
		gimbal.pid.yaw_angle_ref = 0;
		input_flag = 0;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;
  }
  else if (gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
  {
		no_action_time = HAL_GetTick();
		input_flag = 1;
  }else{
		
		gimbal.pid.yaw_angle_fdb  = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;	
	  gimbal.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW
                               +  km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;

    input_flag = 1;
	}
  

  /*pitch��*/
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.pit_angle_ref += rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT
                         + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
  /* �������pitch��Ƕ� */
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);  
  gimbal.last_state = remote_is_action();//��ȡ�ϴ������״̬
}


//7.27����ֲ����ģʽ
static void separate_handler(void)
{
  if(last_gimbal_mode != GIMBAL_SEPARATE_MODE)
  {
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref = gimbal.yaw_offset_angle;
  }
	gimbal.state = remote_is_action();
	if(gimbal.last_state == NO_ACTION && gimbal.state == NO_ACTION)
  {
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref = gimbal.yaw_offset_angle;
  }
  else if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
  {
    gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;
  }
  else
  {
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW
                              + km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;
  }
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.pit_angle_ref += rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT
														+ km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  
  gimbal.last_state = remote_is_action();
}

static void dodge_handler(void)
{ 
  
  if(last_gimbal_mode != GIMBAL_DODGE_MODE) //�Ӹ���ģʽ�л���С����ʱ��ˢ������
  {
    gimbal.pit_offset_angle = gimbal.sensor.pit_gyro_angle;//new
    gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref = gimbal.yaw_offset_angle;
    gimbal.pid.pit_angle_ref = gimbal.pit_offset_angle;//new
  }
  
  gimbal.state = remote_is_action(); //�ж�yaw���Ƿ�������
  if(gimbal.last_state == NO_ACTION && gimbal.state == NO_ACTION)    
  {
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref = gimbal.yaw_offset_angle;
  }
  else if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
  {
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle-3.5;//-3.5�ǵ���С����ʱ��̨yaw��ֹͣ�˶�ʱ�Ļس�
  }
  else
  {
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW
                              + km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;
  }
  
  /*pitch��*/
  //gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;//���
    gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;//new
  gimbal.pid.pit_angle_ref += rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT
                           +  km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
  /* �������pitch��Ƕ� */
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  
  gimbal.last_state = remote_is_action();//��ȡ�ϴ������״̬
}  

float yaw_a,pit_a;
extern float pit_rec_real;
extern float yaw_rec_real;
extern WorldTime_RxTypedef PC_KF_Time;

static void track_aimor_handler(void)
{
	/* ��¼��ʧ�Ƕ� */
 static float lost_pit;
 static float lost_yaw;
  /*���ƽǶ�*/
 static float yaw_ctrl;
 static float pit_ctrl;
 static uint8_t  last_vision_status;
	pid_yaw.iout = 0;//�������ģʽyaw�ǶȻ�iout�ۼ�ֵ
	gimbal.yaw_offset_angle  = gimbal.sensor.yaw_gyro_angle;//�������������ݣ������˳�����ʱ��ͻ
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;//yaw����������
	
	//	float Vision_Speed = target_speed_calc(&Vision_speed_Struct, PC_KF_Time.WorldTime, pc_recv_mesg.gimbal_control_data.yaw_ref);

	/* ����΢���� */
	// PRE_LADRC_TD(&td_pit, pit_rec_real);
	// PRE_LADRC_TD(&td_yaw, yaw_rec_real);
	/* �������˲���
	 * 0: �Ƕȸ��� 
	 * 1: �ٶȸ���  */
	//	float *pc_recv_result_pitch = kalman_filter_calc(&pc_kalman_filter_pitch,
	//																										td_pit.v1,
	//																										td_pit.v2);
	//	float *pc_recv_result_yaw = kalman_filter_calc  (&pc_kalman_filter_yaw,
	//																										td_yaw.v1,
	//	
	//																										td_yaw.v2);	
	if(last_gimbal_mode != GIMBAL_TRACK_ARMOR)//������ʱ����ֹ��ת��ͻ
		{
			  pit_ctrl = gimbal.sensor.pit_relative_angle;
			  yaw_ctrl = gimbal.sensor.yaw_gyro_angle;
			
		}	
  		
		
	
	if (pc_recv_mesg.gimbal_control_data.visual_valid == 1)
	{
			 
		if(chassis_mode == CHASSIS_DODGE_MODE)
			yaw_ctrl = gimbal.sensor.yaw_gyro_angle + yaw_rec_real-6.5;//����С��������ʱ���̵ķ�������
		else
			yaw_ctrl = gimbal.sensor.yaw_gyro_angle + yaw_rec_real;	
		if(INFANTRY_NUM == INFANTRY_5)//������̨�����ĵ���͵��ֱ�������Ƿ���	
			pit_ctrl =  gimbal.sensor.pit_relative_angle + pit_rec_real;		
		else		
			pit_ctrl =  gimbal.sensor.pit_relative_angle - pit_rec_real;
		last_vision_status = 1;
	}
	/*�Ӿ���Ч����*/
	 else if (pc_recv_mesg.gimbal_control_data.visual_valid == 0)		
	  {
			if (last_vision_status == 1)
			{		
				lost_yaw = gimbal.sensor.yaw_gyro_angle ;//pc_recv_result[0];//yaw_msg_t.filtered_value;			
				lost_pit = pit_ctrl;//-pc_recv_result[1];//pit_msg_t.filtered_value;//��ʧĿ�����̨����
				last_vision_status = 2;
				yaw_ctrl = lost_yaw;
				pit_ctrl = lost_pit;
				
			}
			last_vision_status = 0;
		}
	 //PCͨѶ��������
	if(pc_recv_mesg.gimbal_control_data.yaw_ref >= 180 || pc_recv_mesg.gimbal_control_data.yaw_ref <= -180)
		yaw_ctrl = gimbal.sensor.yaw_gyro_angle;
	if(pc_recv_mesg.gimbal_control_data.pit_ref >= 180 || pc_recv_mesg.gimbal_control_data.pit_ref <= -180)
		pit_ctrl = gimbal.sensor.pit_relative_angle;
	
	gimbal.pid.yaw_angle_ref = yaw_ctrl;
	gimbal.pid.pit_angle_ref = pit_ctrl;

	
	yaw_a = yaw_ctrl;
	pit_a = pit_ctrl;
	/* �������pitch��Ƕ� */
		VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
}



static void shoot_buff_ctrl_handler(void)
{
	/*��¼��ʧ�Ƕ�*/
	static float lost_pit;
	static float lost_yaw;
	
	/*���ƽǶ�*/
  static float yaw_ctrl;
  static float pit_ctrl;
	
	static uint8_t  last_vision_status;
	
	/*��ȡ����ֵ*/
	gimbal.yaw_offset_angle  = gimbal.sensor.yaw_gyro_angle; //�������������ݣ������˳�����ʱ��ͻ
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;

  //�������˲���
	
	//0: yaw����� ;  
  //1: pit����� ;	
//  float *pc_recv_result   = kalman_filter_calc(&pc_kalman_filter,   pc_recv_mesg.gimbal_control_data.yaw_ref,   pc_recv_mesg.gimbal_control_data.pit_ref);
 
	if(pc_recv_mesg.gimbal_control_data.visual_valid == 1)
	{
		last_vision_status = 1;		
		yaw_ctrl =  pc_recv_mesg.gimbal_control_data.yaw_ref;
 	  pit_ctrl = -pc_recv_mesg.gimbal_control_data.pit_ref;
	}	
  
	/*�Ӿ���Ч����*/
  else if(pc_recv_mesg.gimbal_control_data.visual_valid == 0)		
	{
    if (last_vision_status == 1)
    {
			lost_yaw =  pc_recv_mesg.gimbal_control_data.yaw_ref;			
			lost_pit = -pc_recv_mesg.gimbal_control_data.pit_ref;//��ʧĿ�����̨����
			last_vision_status = 2;
    }
		  yaw_ctrl = lost_yaw;		
			pit_ctrl = lost_pit;
		  
	}
	
	gimbal.pid.yaw_angle_ref = yaw_ctrl;
	gimbal.pid.pit_angle_ref = pit_ctrl;
	
  /*�����Ƕ�����*/	
 	VAL_LIMIT(gimbal.pid.yaw_angle_ref, -45, 45);
	VAL_LIMIT(gimbal.pid.pit_angle_ref, -28, 18);//25,15
}



float speed_threshold = 10.0f;
//time_raw,pitch_angel_raw�ȶ�����pc
//�˺���������ٶȣ�����ʱpositionʵ��Ϊpitch_angel_raw
//����ʵΪ���ṹ��speed_calc_data_t��ֵ����ʼ������
float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position)
{
  S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 1000;		//�����ٶȣ���ǰλ��-�ϴ�λ�ã�/
#if 1
    if ((S->speed - S->processed_speed) < -speed_threshold)		//speed_threshold=10.0f,S->processed_speedδ����ֵ
    {
        S->processed_speed = S->processed_speed - speed_threshold;
    }
    else if ((S->speed - S->processed_speed) > speed_threshold)
    {
        S->processed_speed = S->processed_speed + speed_threshold;
    }
    else 
#endif
      S->processed_speed = S->speed;													//��ǰ���������ٶȸ���S->processed_speed
    
    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }
  
  if(S->delay_cnt > 200) // delay 200ms speed = 0
  {
    S->processed_speed = 0;
  }

  return S->processed_speed;
}

void initFeedforwardParam(FFC *vFFC,float a,float b)
{
	//��ʼ��ǰ������
	vFFC->a = a;
	vFFC->b = b;
	vFFC->lastRin = 0;
	vFFC->perrRin = 0;
	vFFC->rin = 0;
}

/*ʵ��ǰ��������*/
float getFeedforwardControl(FFC* vFFC,float v)//yaw��
{
	vFFC->rin = v;
	float result = vFFC->a * (vFFC->rin - vFFC->lastRin) + vFFC->b * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);
	vFFC->perrRin = vFFC->lastRin;
	vFFC->lastRin = vFFC->rin;
	return result;
}

/* ����λ��ǰ�ĸ���΢���� */
void PRE_LADRC_TD(TD_LADRC *td_para, float Expect)
{
	td_para->fh = -td_para->r * td_para->r * (td_para->v1 - Expect) - 2 * td_para->r * td_para->v2;
	td_para->v1 += td_para->v2 * td_para->h;
	td_para->v2 += td_para->fh * td_para->h;
	// ��λ��ǰ v1+lamda*h*v2
	td_para->pre_v1 = td_para->v1 + td_para->lambda * td_para->fh * td_para->v2;
}
