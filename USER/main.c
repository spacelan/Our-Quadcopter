#include "sys.h"
#include "usart.h"
#include "time.h"
#include "led.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "communication.h"

int main(void)
{
	SystemInit(); 			 //系统时钟初始化为72M	  SYSCLK_FREQ_72MHz 
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	MyTime_Init();	    	 //延时函数初始化	 
	MyUSART_Init(9600);
	MyI2C_Init();
	MyLED_Config();
	while(mpu_init());
	while(1)
	{
		u8 type;
		short accel[3];
		MyCOM_GetData(accel,&type);
		if(type) MyCOM_SendData(accel,type);
		Delay_ms(100);
		MyLED_Toggle();
	}
}

