#ifndef TIME_H
#define TIME_H

#include "sys.h"
void MyTime_Init(void);
uint32_t MyTime_GetCurrentTime_ms(void);
uint64_t MyTime_GetCurrentTime_us(void);
void Delay_ms(uint32_t nms);
void Delay_us(uint32_t nus);
void MyTime_TimerStart(void);
uint32_t MyTime_TimerStop(void);

#endif
