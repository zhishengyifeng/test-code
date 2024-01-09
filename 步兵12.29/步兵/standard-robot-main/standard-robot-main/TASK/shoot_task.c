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
#include "remote_ctrl.h"

UBaseType_t shoot_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern gimbal_t gimbal;

shoot_t   shoot;
trigger_t trig;
uint16_t normal_speed;
uint16_t Fric_Spd_Ajt;//3Ϊ��ʼδʹ��״̬

  
  #if(INFANTRY_NUM == INFANTRY_3)
  /*Ħ����pid*/
  float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed = 7300;//8200
	  /*���ָǿ���*/
	float ccr_open  = 500;
  float ccr_close = 2350;
  #elif(INFANTRY_NUM == INFANTRY_4)
  /*Ħ����pid*/
  float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed = 7000;//8200
	  /*���ָǿ���*/
	float ccr_open  = 500;
  float ccr_close = 2350;
    #elif(INFANTRY_NUM == INFANTRY_5)
  /*Ħ����pid*/
  float fric_pid[3] = {24, 0, 0};
	/* Ħ����ת�� */
	uint16_t speed = 7000;//8200

	  /*���ָǿ���*/
	float ccr_open  = 1550;
  float ccr_close = 430;
#else
		#error "INFANTRY_NUM define error!"
#endif

	int close_down  = 1;     //���ָǹر���ɱ�־λ
	int open_down   = 1;      //���ָǴ���ɱ�־λ
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
float	heat_time_p=1,radio_freq=40;
float single_shoot_angle;
uint16_t single_time = 100; //������ʱʱ������ʼֵ
uint16_t continue_time = 420;//������ʱʱ������ʼֵ
float surplus_heat;   //ʣ������
float extra_time; 
float shoot_delay;//�����ӳ�
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
					
          /*Ħ����*/
          for(int i=0;i<2;i++)
          {
            PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, DONE ); 
          }


					
					
          shoot_para_ctrl();						// ���ģʽ�л�
          ball_storage_ctrl();					// ������Ƶ��ָ�
          fric_wheel_ctrl();						// ����Ħ����
          
          if (shoot.fric_wheel_run)//��Ħ���ֿ���
          {
            shoot_bullet_handler();      
          }
          else
          {
            shoot.shoot_cmd   = 0;//������־λ
            shoot.c_shoot_cmd = 0;//������־λ
            shoot.fric_wheel_spd = 1000;//Ħ����ת��
            pid_trigger_spd.out = 0;//����
            trig.angle_ref = moto_trigger.total_angle; //��¼��ǰ���̵������λ
						trig.one_sta = TRIG_INIT;//��֤�´ο���Ħ����ʱ������״̬Ϊ��ʼ��
          }
          
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
	shoot.fric_wheel_spd = speed;	
}

/*Ħ���ֿ���*/
static void fric_wheel_ctrl(void)
{
	if (shoot.fric_wheel_run)
	{
		turn_on_friction_wheel(shoot.fric_wheel_spd,shoot.fric_wheel_spd);
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
  /* �ж��Ƿ��ڿ���״̬*/
  if(IF_trigger_stop(moto_trigger.speed_rpm))
  shoot_break_state = TRIGGER_MOVE_BACK;
  else
  shoot_break_state = NOBREAK;

  if (shoot.shoot_cmd)//����
  {
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//û���Ӳ���ϵͳʱ
			single_time = 100;
		else
			single_time = 1000/(judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate/10 + judge_recv_mesg.game_robot_state.robot_level);
  
        shoot_delay = single_time;
		if (trig.one_sta == TRIG_INIT)
		{
			trig.one_time = HAL_GetTick();
			single_shoot_angle = moto_trigger.total_angle + Angle;//���䣨single_shoot_angle��Ϊ��ǰ�Ƕȼ���Angle��
			trig.one_sta = TRIG_PRESS_DOWN;//���䰴���Ѱ���
        }
		/* ������ʱ */
    else if (trig.one_sta == TRIG_PRESS_DOWN)//�����䰴���Ѱ��±������ʱ
    {		
      if (HAL_GetTick() - trig.one_time >= shoot_delay)
      {
		if (shoot_break_state == NOBREAK)
		trig.c_sta = TRIG_ONE_DONE;
		if (shoot_break_state == TRIGGER_MOVE_BACK)
        trig.one_sta = TRIG_SHOOT_BREAK;//��ʱ��Ϻ�״̬��Ϊ���������
      }
    }
     
    if (trig.one_sta == TRIG_ONE_DONE)//������״̬Ϊ�����
    {
		single_shoot_angle = moto_trigger.total_angle;
        shoot.shoot_cmd = 0;//������־λ����
		trig.one_sta = TRIG_INIT;//����״̬��Ϊ��ʼ��
		trig.c_sta = TRIG_INIT;//����״̬��Ϊ��ʼ��
      shoot.shoot_bullets++;//�����ӵ�����
    }
	     /* ���� */
	if (trig.c_sta == TRIG_SHOOT_BREAK)
	{
		trig.one_time = HAL_GetTick();
		single_shoot_angle = moto_trigger.total_angle - 2*Angle;
		trig.c_sta = TRIG_PRESS_DOWN;
	}
		trig.angle_ref = single_shoot_angle;//����Ŀ��Ƕ���Ϊsingle_shoot_angle
		
  }
	
	else if (shoot.c_shoot_cmd)//��������ε���
  {
        //������ʱʱ��
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
					if (shoot_break_state == NOBREAK)
					trig.c_sta = TRIG_ONE_DONE;
					if (shoot_break_state == TRIGGER_MOVE_BACK)
					trig.c_sta = TRIG_SHOOT_BREAK;
				}
				
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����
			}
			/* ���� */
			if (trig.c_sta == TRIG_SHOOT_BREAK)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle - 2*Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
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
					if (shoot_break_state == NOBREAK)
					trig.c_sta = TRIG_ONE_DONE;
					if (shoot_break_state == TRIGGER_MOVE_BACK)
					trig.c_sta = TRIG_SHOOT_BREAK;
				}
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)//������״̬Ϊ�����
			{
				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����
			}
			/* ���� */
			if (trig.c_sta == TRIG_SHOOT_BREAK)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle - 2*Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
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
					if (shoot_break_state == NOBREAK)
					trig.c_sta = TRIG_ONE_DONE;
					if (shoot_break_state == TRIGGER_MOVE_BACK)
					trig.c_sta = TRIG_SHOOT_BREAK;
				}
			}
			/* ������� */
			if (trig.c_sta == TRIG_ONE_DONE)
			{
//				single_shoot_angle = moto_trigger.total_angle;
				trig.c_sta = TRIG_INIT;
				shoot.shoot_bullets++;//�����ӵ�����

			}
			/* ���� */
			if (trig.c_sta == TRIG_SHOOT_BREAK)
			{
				trig.one_time = HAL_GetTick();
				single_shoot_angle = moto_trigger.total_angle - 2*Angle;
				trig.c_sta = TRIG_PRESS_DOWN;
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
   //pid����
  pid_calc(&pid_trigger,moto_trigger.total_angle,trig.angle_ref);
	
	/*(trig.c_sta == TRIG_PRESS_DOWN && moto_trigger.speed_rpm==0)//�������ͷ�ת
	{
	     pid_calc(&pid_trigger,moto_trigger.total_angle,trig.angle_ref-Angle);
	}
	else*/
	    trig.spd_ref = pid_trigger.out;
	pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && judge_recv_mesg.game_robot_state.mains_power_shooter_output == 0)//Ħ���ֱ�����ϵͳ�ϵ���ò���ͣת
		pid_trigger_spd.out = 0;
	//��ֹ����ϵͳ�쳣���ֳ����������
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_rate == 0 || judge_recv_mesg.game_robot_state.shooter_id1_17mm_cooling_limit == 0))
		pid_trigger_spd.out = 0;
}


//��ʼ��
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


