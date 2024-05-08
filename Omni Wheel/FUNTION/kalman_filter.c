#include "kalman_filter.h"
#include "STM32_TIM_BASE.h"

// float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
// float P_data[4] = {2, 0,
//                    0, 2};
// float AT_data[4], HT_data[4];
// float A_data[4] = {1, 0.001,
//                    0, 1};
// float H_data[4] = {1, 0,
//                    0, 1};
// float Q_data[4] = {1, 0,
//                    0, 1};
// float R_data[4] = {2000, 0,
//                    0, 5000};

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	mat_init(&F->xhat, 2, 1, (float *)I->xhat_data);
	mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
	mat_init(&F->z, 2, 1, (float *)I->z_data);
	mat_init(&F->A, 2, 2, (float *)I->A_data);
	mat_init(&F->H, 2, 2, (float *)I->H_data);
	mat_init(&F->Q, 2, 2, (float *)I->Q_data);
	mat_init(&F->R, 2, 2, (float *)I->R_data);
	mat_init(&F->P, 2, 2, (float *)I->P_data);
	mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
	mat_init(&F->K, 2, 2, (float *)I->K_data);
	mat_init(&F->AT, 2, 2, (float *)I->AT_data);
	mat_trans(&F->A, &F->AT);
	mat_init(&F->HT, 2, 2, (float *)I->HT_data);
	mat_trans(&F->H, &F->HT);
}

/********************************************************
		 Q:过程噪声，Q增大，动态响应变快，收敛稳定性变�?
		 R:测量噪声，R增大，动态响应变慢，收敛稳定性变�?
		 F->z.pData  输入预测矩阵
		 P(k) 最优值对应的协方�?
		 Q用于求先验估计方差xhatminus，后验估计方差xhat
		 R用于求卡尔曼增益K
*********************************************************/
float *kalman_filter_calc(kalman_filter_t *F, float angle, float speed)
{
	float TEMP_data[4] = {0, 0, 0, 0};
	float TEMP_data21[2] = {0, 0};
	mat TEMP, TEMP21; // 定义mat浮点矩阵结构类型TEMP,TEMP21

	// mat_init	将浮点矩阵的参数初始化，确定行列，并将参数（矩阵数组TEMP_data�?4】）赋值给第一个参数TEMP
	mat_init(&TEMP, 2, 2, (float *)TEMP_data); // 指向TEMP矩阵�?2�?2列，矩阵数组为TEMP_data[4]
	mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);

	/*F->z.pData 预测矩阵 */
	F->z.pData[0] = angle;
	F->z.pData[1] = speed;

	/*求先验估计xhatminus*/
	// 1. xhat'(k)= A xhat(k-1)
	mat_mult(&F->A, &F->xhat, &F->xhatminus); // 浮点矩阵乘法	先验估计xhatminus = A * xhat(k-1)

	/*求先验方差估计Pminus*/
	// 2. P'(k) = A P(k-1) AT + Q
	mat_mult(&F->A, &F->P, &F->Pminus);	 //							先验方差估计Pminus = A * P(k-1)
	mat_mult(&F->Pminus, &F->AT, &TEMP); //							TEMP2*2矩阵 = 先验方差估计Pminus * AT
	mat_add(&TEMP, &F->Q, &F->Pminus);	 // 浮点矩阵加法	先验方差估计Pminus = TEMP + 过程噪声Q

	/*求Kalman增益K*/
	// 3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&F->H, &F->Pminus, &F->K); //							Kalman增益K = H * P'(k)
	mat_mult(&F->K, &F->HT, &TEMP);		//							TEMP = Kalman增益K * HT
	mat_add(&TEMP, &F->R, &F->K);		//							Kalman增益K = TEMP + 测量噪声R

	mat_inv(&F->K, &F->P);				 // 矩阵的�?			对矩阵K求逆得到P
	mat_mult(&F->Pminus, &F->HT, &TEMP); //							TEMP = P'(k) * HT
	mat_mult(&TEMP, &F->P, &F->K);		 //							Kalman增益K = TEMP * P

	/*求后验估计xhat*/
	// 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&F->H, &F->xhatminus, &TEMP21);   //							TEMP21 = H * 先验估计xhat'(k)
	mat_sub(&F->z, &TEMP21, &F->xhat);		   //							后验估计xhat = z(k) - TEMP21
	mat_mult(&F->K, &F->xhat, &TEMP21);		   //							TEMP21 = K(k) * xhat
	mat_add(&F->xhatminus, &TEMP21, &F->xhat); //							后验估计xhat = xhatminus + TEMP21

	/*求后验估计方差P*/
	// 5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&F->K, &F->H, &F->P);		//							K * H
	mat_sub(&F->Q, &F->P, &TEMP);		//							Q - K * H				(Q = 1)
	mat_mult(&TEMP, &F->Pminus, &F->P); //							P = (Q - K * H)	* Pminus

	F->filtered_value[0] = F->xhat.pData[0];
	F->filtered_value[1] = F->xhat.pData[1];
	return F->filtered_value;
}

/*
float A_data[4] =  {1, 0, dt, 0,
					0, 1, 0 , dt,
					0, 0, 1 , 0,
					0, 0, 0 , 1};

float H_data[4] =  {1, 0, 0, 0,
					0, 1, 0, 0};

float Q_data[4] =  {1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1};

float R_data[4] =  {R, 0, 0, 0,
					0, R, 0, 0,
					0, 0, R, 0,
					0, 0, 0, R};
*/

void kalman_filter_init_two(kalman_filter_t *F, kalman_filter_init_t *I)
{
	mat_init(&F->xhat, 4, 1, (float *)I->xhat_data);
	mat_init(&F->xhatminus, 4, 1, (float *)I->xhatminus_data);
	mat_init(&F->z, 2, 1, (float *)I->z_data);
	mat_init(&F->A, 4, 4, (float *)I->A_data);
	mat_init(&F->H, 2, 4, (float *)I->H_data);
	mat_init(&F->Q, 4, 4, (float *)I->Q_data);
	mat_init(&F->R, 2, 2, (float *)I->R_data);
	mat_init(&F->P, 4, 4, (float *)I->P_data);
	mat_init(&F->Pminus, 4, 4, (float *)I->Pminus_data);
	mat_init(&F->K, 4, 2, (float *)I->K_data);
	mat_init(&F->AT, 4, 4, (float *)I->AT_data);
	mat_trans(&F->A, &F->AT);
	mat_init(&F->HT, 4, 2, (float *)I->HT_data);
	mat_trans(&F->H, &F->HT);
}

/********************************************************
 * 函数名：kalman_filter_calc_two
		 Q:过程噪声，Q增大，动态响应变快，收敛稳定性变�?
		 R:测量噪声，R增大，动态响应变慢，收敛稳定性变�?
		 F->z.pData  输入预测矩阵
		 P(k) 最优值对应的协方�?
		 Q用于求先验估计方差xhatminus，后验估计方差xhat
		 R用于求卡尔曼增益K
*********************************************************/
float *kalman_filter_calc_two(kalman_filter_t *F, float Pitch, float Pitch_speed, float Yaw, float Yaw_speed)
{
	//	float TEMP_data21[2] = {0, 0};
	//  mat TEMP,TEMP21;
	float TEMP_data44[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float TEMP_data24[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	float TEMP_data22[4] = {0, 0, 0, 0};
	float TEMP_data_22[4] = {0, 0, 0, 0};
	float TEMP_data42[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	float TEMP_data41[4] = {0, 0, 0, 0};
	float TEMP_data21[2] = {0, 0};
	mat TEMP44, TEMP24, TEMP22, TEMP_22, TEMP42, TEMP41, TEMP21; // 定义mat浮点矩阵结构类型

	// mat_init	将浮点矩阵的参数初始化，确定行列数，指向数组
	mat_init(&TEMP44, 4, 4, (float *)TEMP_data44); // 指向TEMP矩阵，n行n列，矩阵数组为TEMP_data[mxn]
	mat_init(&TEMP24, 2, 4, (float *)TEMP_data24);
	mat_init(&TEMP22, 2, 2, (float *)TEMP_data22);
	mat_init(&TEMP_22, 2, 2, (float *)TEMP_data_22);
	mat_init(&TEMP42, 4, 2, (float *)TEMP_data42);
	mat_init(&TEMP41, 4, 1, (float *)TEMP_data41);
	mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);

	/*F->z.pData 预测矩阵 */
	F->z.pData[0] = Pitch;
	F->z.pData[1] = Yaw;

	/*求先验估计xhatminus*/
	// 1. xhat'(k)= A xhat(k-1)
	mat_mult(&F->A, &F->xhat, &F->xhatminus); // 浮点矩阵乘法	先验估计xhatminus = A * xhat(k-1)

	/*求先验方差估计Pminus*/
	// 2. P'(k) = A P(k-1) AT + Q
	mat_mult(&F->A, &F->P, &F->Pminus);	   //							先验方差估计Pminus = A * P(k-1)
	mat_mult(&F->Pminus, &F->AT, &TEMP44); //							TEMP2*2矩阵 = 先验方差估计Pminus * AT
	mat_add(&TEMP44, &F->Q, &F->Pminus);   // 浮点矩阵加法	先验方差估计Pminus = TEMP + 过程噪声Q
	{
		/*求Kalman增益K*/
		// 3. K(k) = P'(k) HT / (H P'(k) HT + R)
		mat_mult(&F->H, &F->Pminus, &TEMP24); //							H * P'(k)
		mat_mult(&TEMP24, &F->HT, &TEMP22);	  //							H * P'(k) * HT
		mat_add(&TEMP22, &F->R, &TEMP22);	  //							H * P'(k) * HT + 测量噪声R

		mat_inv(&TEMP22, &TEMP_22);			   // 矩阵的�?  (H * P'(k) * HT + 测量噪声R)的�?
		mat_mult(&F->Pminus, &F->HT, &TEMP42); //							TEMP42 = P'(k) * HT
		mat_mult(&TEMP42, &TEMP_22, &F->K);	   //							Kalman增益K = TEMP * (H * P'(k) * HT + 测量噪声R)的�?

		/*求后验估计xhat*/
		// 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
		mat_mult(&F->H, &F->xhatminus, &TEMP21);   //							       H * 先验估计xhat'(k)
		mat_sub(&F->z, &TEMP21, &TEMP21);		   //			          z(k) - H * 先验估计xhat'(k)
		mat_mult(&F->K, &TEMP21, &TEMP21);		   //							K(k) * (z(k) - H * 先验估计xhat'(k))
		mat_add(&F->xhatminus, &TEMP21, &F->xhat); //							后验估计xhat = xhatminus + K(k) * (z(k) - H * 先验估计xhat'(k))

		/*求后验估计方差P*/
		// 5. P(k) = (1-K(k)H)P'(k)
		mat_mult(&F->K, &F->H, &F->P);		  //							K * H
		mat_sub(&F->Q, &F->P, &TEMP44);		  //							Q - K * H				(Q = 1)
		mat_mult(&TEMP44, &F->Pminus, &F->P); //							P = (Q - K * H)	* Pminus

		F->filtered_value[0] = F->xhat.pData[0];
		F->filtered_value[1] = F->xhat.pData[1];
		return F->filtered_value;
	}
}

#ifdef IIR_FILTER
double NUM[7] =
	{
		0.000938328292536,
		-0.005375225952906,
		0.01306656496967,
		-0.01725922344534,
		0.01306656496967,
		-0.005375225952906,
		0.000938328292536};
double DEN[7] =
	{
		1,
		-5.733703351811,
		13.70376013927,
		-17.47505866491,
		12.53981348666,
		-4.800968865471,
		0.7661573674342};

iir_filter_t yaw_msg_t, pit_msg_t;

double iir_filter(iir_filter_t *F)
{
	int i;
	for (i = 6; i > 0; i--)
	{
		F->xbuf[i] = F->xbuf[i - 1];
		F->ybuf[i] = F->ybuf[i - 1];
	}

	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];

	for (i = 1; i < 7; i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}

	F->filtered_value = F->ybuf[0];

	return F->filtered_value;
}
#endif

static uint32_t FPS_Calculate(uint16_t deltaTime)
{												   // FPS计算函数�?
	return (1.0f / (double)(deltaTime)) * 1000.0f; // 先转换为浮点数，否则会有精度丢失
}

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS) // 获取当前系统时钟节拍并算出FPS
{
	time->WorldTime = osKernelSysTick() * portTICK_PERIOD_MS;
	*FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime);
	time->Last_WorldTime = time->WorldTime;
}
