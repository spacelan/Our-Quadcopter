#include "time.h"

volatile uint32_t currentTime_ms = 0;
uint64_t timer_us = 0;

//��ʼ����Ĭ��72MHz
void MyTime_Init(void)
{
	currentTime_ms = 0;
	SysTick_Config(SystemCoreClock / 1000);  //72000 ticks = 1s / 1000 = 1 ms �ж�һ��
}

//��ȡ��ǰʱ�䣬��λ����
uint32_t MyTime_GetCurrentTime_ms(void)
{
	return currentTime_ms;
}

//��ȡ��ǰʱ�䣬��λ΢��
uint64_t MyTime_GetCurrentTime_us(void)
{
	return currentTime_ms * 1000 + (SysTick->LOAD - SysTick->VAL) / 72000;
}

//1ms <= nms
void Delay_ms(uint32_t nms)
{
	uint32_t target = currentTime_ms + nms;
	while(currentTime_ms < target) ;
}

//1us <= nus < 1ms
void Delay_us(uint32_t nus)
{
	uint64_t target = MyTime_GetCurrentTime_us() + nus;
	while(MyTime_GetCurrentTime_us() < target) ;
}

//��ʱ����ʼ
void MyTime_TimerStart(void)
{
	timer_us = MyTime_GetCurrentTime_us();
}

//��ʱ��ֹͣ������ʱ�䣬��λ΢��
uint32_t MyTime_TimerStop(void)
{
	return MyTime_GetCurrentTime_us() - timer_us;
}

// SysTick�жϡ�
void SysTick_Handler(void)
{
    currentTime_ms++;
}
