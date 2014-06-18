#include "motor.h"
#define THROTTLE_OFFSET 120
static uint8_t motorThrottle[4] = {0}; //储存相对值，相对值+THROTTLE_OFFSET=绝对值
static uint8_t isStart = 0; //是否启动标志

void MyMotor_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_IS;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	GPIO_IS.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_IS.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_IS);
}

void MyMotor_TIM2_PWM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_IS;
	TIM_OCInitTypeDef TIM_OC_IS;
	TIM_TimeBaseStructInit(&TIM_IS);
	TIM_OCStructInit(&TIM_OC_IS);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	TIM_IS.TIM_Period = 2000;
	TIM_IS.TIM_Prescaler = 720;//PWM周期20ms 50Hz
	TIM_IS.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_IS.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_IS);

	TIM_OC_IS.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_IS.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_IS.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC_IS.TIM_Pulse = THROTTLE_OFFSET;
	TIM_OC1Init(TIM2,&TIM_OC_IS);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC_IS.TIM_Pulse = THROTTLE_OFFSET;
	TIM_OC2Init(TIM2,&TIM_OC_IS);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC_IS.TIM_Pulse = THROTTLE_OFFSET;
	TIM_OC3Init(TIM2,&TIM_OC_IS);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC_IS.TIM_Pulse = THROTTLE_OFFSET;
	TIM_OC4Init(TIM2,&TIM_OC_IS);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
}
//初始化，开启TIM2
void MyMotor_Init(void)
{
	MyMotor_GPIO_Config();
	MyMotor_TIM2_PWM_Config();
	TIM_Cmd(TIM2,ENABLE);
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
}
//开始后才能驱动电机
void MyMotor_Start(void)
{
	isStart = 1;
	MyMotor_Update();
}
//停止电机，不管当前throttle为多少
void MyMotor_Stop(void)
{
	isStart = 0;
	MyMotor_Update();
}

void MyMotor_SetThrottle(enum MOTORS motor,uint8_t throttle)
{
	motorThrottle[motor] = throttle>40?40:throttle;
}

void MyMotor_GetThrottle(enum MOTORS motor,uint8_t* throttle)
{
	*throttle = motorThrottle[motor];
}

void MyMotor_Update(void)
{
	if(isStart)
	{
		TIM_SetCompare1(TIM2,motorThrottle[MOTOR1] + THROTTLE_OFFSET);
		TIM_SetCompare2(TIM2,motorThrottle[MOTOR2] + THROTTLE_OFFSET);
		TIM_SetCompare3(TIM2,motorThrottle[MOTOR3] + THROTTLE_OFFSET);
		TIM_SetCompare4(TIM2,motorThrottle[MOTOR4] + THROTTLE_OFFSET);
	}
	else
	{
		TIM_SetCompare1(TIM2,THROTTLE_OFFSET);
		TIM_SetCompare2(TIM2,THROTTLE_OFFSET);
		TIM_SetCompare3(TIM2,THROTTLE_OFFSET);
		TIM_SetCompare4(TIM2,THROTTLE_OFFSET);
	}
}
