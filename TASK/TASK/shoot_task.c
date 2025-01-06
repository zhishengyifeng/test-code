#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "string.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "pc_rx_data.h"
#include "rc.h"
#include "gimbal_task.h"
#include "math.h"

UBaseType_t shoot_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern gimbal_t gimbal;

shoot_t   shoot;
trigger_t trig;
uint16_t normal_speed;

#if (INFANTRY_NUM == INFANTRY_1) 
	/*Ħ����pid*/
	float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed_15 =	4329;
	uint16_t speed_18 = 4800;//4430
	uint16_t speed_30 = 7500;//8200

	/*���ָǿ���*/
	float ccr_open  = 500;
	float ccr_close = 2350;
#elif((INFANTRY_NUM == INFANTRY_2))
	/*Ħ����pid*/
	float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed_15 =	4150;//4450��3800
	uint16_t speed_18 = 4660;//4430
	uint16_t speed_30 = 8100;//8200
	/*���ָǿ���*/
	float ccr_open  = 500;
	float ccr_close = 2440;
#elif((INFANTRY_NUM == INFANTRY_3))
	/*Ħ����pid*/
	float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed_15 =	4160;//4450��3800
	uint16_t speed_18 = 4830;//4430
	uint16_t speed_30 = 7630;//8200

	/*���ָǿ���*/
	float ccr_open  = 500;
	float ccr_close = 2350;
#elif((INFANTRY_NUM == INFANTRY_4))
	/*Ħ����pid*/
	float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed_15 =	4150;//4450��3800
	uint16_t speed_18 = 4670;//4430
	uint16_t speed_30 = 7500;//8200
	/*���ָǿ���*/
	float ccr_open  = 500;
	float ccr_close = 2350;
#elif((INFANTRY_NUM == INFANTRY_5))
	/*Ħ����pid*/
	float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed_15 =	4000;//4450��3800
	uint16_t speed_18 = 4500;//4430
	uint16_t speed_30 = 8000;//8200
	/*���ָǿ���*/
	float ccr_open  = 500;
	float ccr_close = 2350;
#else
		#error "INFANTRY_NUM define error!"
#endif

	int close_down  = 1;     //���ָǹر���ɱ�־λ
	int open_down   = 1;      //���ָǴ���ɱ�־λ
	
	/* ����ת�� */
	int normal_cshoot				= 1000;
	int lspd;
	int rspd;	
	
	/*˳����ͨ-����-����-buff*/
	/*�������Ʋ���ת��*/
  float heat_limit_pid[3] = {30, 0, 10};
	/*������������ʱ����*/
	float heat_time_pid[3]  = {9, 0, 0};//{3, 0 ,3};
	
	
	//��̬����pid
	float speedlimit[3]={10,0,1};
	float last_shoot_speed=0;//��¼��һ���ӵ������٣���ΪPID����ֵ
	
int settime=5;//ʣ����������ֵ
float	heat_time_p=1,radio_freq=40;//һ������radio_freq55
float single_shoot_angle;
uint16_t single_time = 100; //������ʱʱ��������λms    200
uint16_t continue_time = 420;//������ʱʱ����           420
float surplus_heat;   //ʣ������
float extra_time; 
float shoot_delay;//����ķ����ӳ�
uint8_t switch_freq = 1;//��Ƶת����־
	
void shoot_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_SHOOT_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_SHOOT_SIGNAL)
			{
        if(shoot_mode != SHOOT_DISABLE)
        {

					continue_time = 1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10);//����ǹ����ȴ�����������ʱ����	
					surplus_heat = judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_limit - judge_recv_mesg.power_heat_data.shooter_id1_17mm_cooling_heat;//ʣ������
					radio_freq = (10000/judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_limit)-7;//����������������������С����ʱ������Ҳ���������Ƶ
					
          /*��������*/
          PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, DONE );
					/*�������ʱ�����*/
          PID_Struct_Init(&pid_heat_time, (1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10)-40)*0.01-heat_time_p, heat_time_pid[1], heat_time_pid[2], 
					1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10)-radio_freq, 0, DONE );//ԭ����heat_time_p1,radio_freq40
					pid_calc(&pid_heat_time,surplus_heat,settime);//0

					if((float)judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit-1-judge_recv_mesg.shoot_data.bullet_speed>5){
						speedlimit[0]=300;
						speedlimit[1]=0;
						speedlimit[2]=1;
					}else{
					  speedlimit[0]=10;
						speedlimit[1]=0;
						speedlimit[2]=1;
					}
				
					PID_Struct_Init(&pid_speedlimit, speedlimit[0], speedlimit[1], speedlimit[2], 7000, 0, DONE );
					
					/*PID��ʼ������*/
				  /*void PID_Struct_Init(pid_t *pid,float p,float i,float d,int32_t max_out,int32_t integral_limit,INIT_STATUS init_status)*/
					
				
		  //2021
					/*�������� 1�� 150 15 667 500*//*��ȴ���� 1�� 50  40 250 150*/
					/*�������� 2�� 280 25 400 300*//*��ȴ���� 2�� 100 60 167 100*/
					/*�������� 3�� 400 35 285 185*//*��ȴ���� 3�� 150 80 125 50*/
          
			//2022
					/*��ʼ		     50  10  		*/  //15m/s

					/*�������� 1�� 150 15 667 500*/ 	//15m/s   /*��ȴ���� 1�� 50  40 250 150*/			//15m/s
					/*�������� 2�� 280 25 400 300*/		//15m/s   /*��ȴ���� 2�� 100 60 167 100*/			//18m/s
					/*�������� 3�� 400 35 285 185*/		//15m/s	  /*��ȴ���� 3�� 150 80 125 50 */			//18m/s
					/*�������� 1�� 75  15  */			    //30m/s
					/*�������� 2�� 150 25  */		    	//30m/s
					/*�������� 3�� 200 35  */		    	//30m/s
					
          /*Ħ����*/
          for(int i=0;i<2;i++)
          {
            PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, DONE ); 
          }


					
					
          shoot_para_ctrl();						// ���ģʽ�л�
          ball_storage_ctrl();					// ������Ƶ��ָ�
          fric_wheel_ctrl();						// ����Ħ����
          
          if (shoot.fric_wheel_run)
          {
            shoot_bullet_handler();      
          }
          else             //Ħ����ʧ��ʱ��������־λ��ʼ��
          {
            shoot.shoot_cmd   = 0;
            shoot.c_shoot_cmd = 0;
            shoot.fric_wheel_spd = 1000;
            pid_trigger_spd.out = 0;
            trig.angle_ref = moto_trigger.total_angle; //��¼��ǰ���̵������λ
						trig.one_sta = TRIG_INIT;
          }
          
          get_last_shoot_mode();
        }
        else
        {
          pid_trigger_spd.out = 0;
          shoot.fric_wheel_spd = 0;
        }
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) SHOT_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
      }
    }
			
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
		
} 



void get_last_shoot_mode(void)
{
	shoot.last_para_mode = shoot.para_mode;
}


/*���ģʽѡ��
* @ SHOOTBUFF_MODE ���ģʽ
*	@ SHOOTTHROW_MODE �����ٵ���ģʽ
*	@ SHOOTMAD_MODE �����ٽ�սģʽ
* @ SHOOTNOR_MODE ��ͨ����ģʽ
**/

static void shoot_para_ctrl(void)
{
	if(judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit == 15)
		normal_speed = speed_15; 
	else if(judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit == 18)
		normal_speed = speed_18;
	else if(judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit == 30)
		normal_speed = speed_30;
	else
		normal_speed = speed_15;
	
	
	if(shoot.para_mode == SHOOTBUFF_MODE) 
		shoot.fric_wheel_spd = speed_30;
  else	
		shoot.fric_wheel_spd = normal_speed;
	
	/* ����ת�� */
	trig.shoot_spd			 = 3000;
	trig.c_shoot_spd		 = normal_cshoot;
}

/*Ħ���ֿ���*/
static void fric_wheel_ctrl(void)
{
	if (shoot.fric_wheel_run)
	{
		turn_on_friction_wheel(shoot.fric_wheel_spd, shoot.fric_wheel_spd);
	}
	else
	{
		turn_off_friction_wheel();
	}
}


/*��Ħ����*/
static void turn_on_friction_wheel(int16_t lspd,int16_t rspd)
{
		pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, -lspd);
		pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, rspd);
		glb_cur.fric_cur[0] = pid_fric[0].out;
		glb_cur.fric_cur[1] = pid_fric[1].out;
}

/*�ر�Ħ����*/
static void turn_off_friction_wheel(void)
{
	pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, 0);
	pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, 0);
	glb_cur.fric_cur[0] = pid_fric[0].out;
	glb_cur.fric_cur[1] = pid_fric[1].out;
}


/*���ָǿ���*/
static void ball_storage_ctrl(void)
{
  if (shoot.ball_storage_open)
  {
    TIM_SetCompare3(TIM8,ccr_open);
  }
  else
  {
    TIM_SetCompare3(TIM8,ccr_close);
  }
}

int Angle=45;
static void shoot_bullet_handler(void)
{
  if (shoot.shoot_cmd)//����
  {
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//û���Ӳ���ϵͳʱ
			single_time = 100;
		else
			single_time = 1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10 + judge_recv_mesg.game_robot_state.robot_level);
  

		if (trig.one_sta == TRIG_INIT)
		{
			trig.one_time = HAL_GetTick();
			single_shoot_angle = moto_trigger.total_angle + Angle;
			trig.one_sta = TRIG_PRESS_DOWN;
    }
		/* ���䴦�� */
    else if (trig.one_sta == TRIG_PRESS_DOWN)
    {			
      if (HAL_GetTick() - trig.one_time >= single_time)//������ʱ
      {
        trig.one_sta = TRIG_ONE_DONE;
      }
    }
		/* ������� */
    if (trig.one_sta == TRIG_ONE_DONE)
    {
      single_shoot_angle = moto_trigger.total_angle;     
      shoot.shoot_cmd = 0;
			trig.one_sta = TRIG_INIT;//���������־λ
			trig.c_sta = TRIG_INIT;
      shoot.shoot_bullets++;//�����ӵ�����
    }
		trig.angle_ref = single_shoot_angle;
		
  }
	
	else if (shoot.c_shoot_cmd)//��������ε���
  {
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//û���Ӳ���ϵͳʱ
		{
			if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* ���䴦�� */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= 200)  //��ʱ
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����
			}
			trig.angle_ref = single_shoot_angle;
		}
		else if(judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit == 30)//��������
		{
			if((judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_recv_mesg.power_heat_data.shooter_id1_17mm_cooling_heat) > 25)
				shoot_delay = 1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10+judge_recv_mesg.game_robot_state.robot_level);
			else
				shoot_delay = 1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10);
			
			if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* ���䴦�� */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= shoot_delay)  //��ʱ
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����
			}
			trig.angle_ref = single_shoot_angle;
		}
		else if(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate == 40 ||
						judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate == 60 ||
						judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate == 80)//��ȴ����
		{
			//���������Թ̶�����Ƶ�������
			if((judge_recv_mesg.shoot_data.bullet_freq < (judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10 + judge_recv_mesg.game_robot_state.robot_level*2)) && surplus_heat>20)
				shoot_delay = 500/((judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10)+judge_recv_mesg.game_robot_state.robot_level)+25;

			else
			{
				shoot_delay = 500/((judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10)-judge_recv_mesg.game_robot_state.robot_level);

			}
			
			if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* ���䴦�� */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= shoot_delay)  //��ʱ
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����
			}
			trig.angle_ref = single_shoot_angle;
		}
		else//��������
    {
			extra_time = pid_heat_time.out;//С��0
			
			if (trig.c_sta == TRIG_INIT)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle + Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
			}
			/* ���䴦�� */
			else if (trig.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig.one_time >= continue_time + extra_time)  //��ʱ
				{
					trig.c_sta = TRIG_ONE_DONE;
				}
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
//				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����

			}
			trig.angle_ref = single_shoot_angle;
		}
	}
	
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.power_heat_data.shooter_id1_17mm_cooling_heat 
		>= (judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-10)))
		trig.angle_ref = moto_trigger.total_angle;
	
	if(trig.angle_ref%45!=0){//�������������
	int i=trig.angle_ref%45;
		if(i>=25)trig.angle_ref+=45-i;
		else trig.angle_ref-=i;
	}

  pid_calc(&pid_trigger,moto_trigger.total_angle,trig.angle_ref);
  trig.spd_ref = pid_trigger.out;
	pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
	
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && judge_recv_mesg.game_robot_state.mains_power_shooter_output == 0)//Ħ���ֱ�����ϵͳ�ϵ���ò���ͣת
		pid_trigger_spd.out = 0;
	//��ֹ����ϵͳ�쳣���ֳ����������
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate == 0 || judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_limit == 0))
		pid_trigger_spd.out = 0;
}



void shoot_param_init(void)
{
  memset(&shoot, 0, sizeof(shoot_t));
  
  shoot.ctrl_mode      = SHOT_DISABLE;
	shoot.para_mode			 = SHOOTNOR_MODE;
  
  memset(&trig, 0, sizeof(trigger_t));
  
  trig.shoot_spd			 = 0;
  trig.c_shoot_spd     = 0;
  trig.one_sta         = TRIG_INIT;
	shoot.shoot_bullets = 0;
  
  /*��������*/
  PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, INIT );
	/*��������ʱ����*/
	PID_Struct_Init(&pid_heat_time, heat_time_pid[0], heat_time_pid[1], heat_time_pid[2], 400, 0, INIT );
  
  /*Ħ����*/
  for(int i=0;i<2;i++)
  {
    PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, INIT ); 
  }

}
//��̬����
void SpeedAdapt(void)
{
	
		if(judge_recv_mesg.shoot_data.bullet_speed-((float)judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit-1)>0.5||judge_recv_mesg.shoot_data.bullet_speed-((float)judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit-1)<-0.5){
		pid_calc(&pid_speedlimit, judge_recv_mesg.shoot_data.bullet_speed,(float)judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit-1);
		}
		
		switch(judge_recv_mesg.game_robot_state.shooter_id1_17mm_speed_limit){
			case 15:
				speed_15 += pid_speedlimit.out;
			break;
			case 18:
				speed_18 += pid_speedlimit.out;
			break;
			case 30:
				speed_30 += pid_speedlimit.out;
			break;
		}
			

}

