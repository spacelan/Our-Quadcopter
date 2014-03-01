#include "sys.h"
#include "usart.h"
#include "time.h"
#include "led.h"
#include "i2c.h"
//#include "inv_mpu.h"
#include "communication.h"
#include "mpu6050.h"

int main(void)
{
	SystemInit(); 			 //系统时钟初始化为72M	  SYSCLK_FREQ_72MHz 
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	MyTime_Init();	    	 //延时函数初始化	 
	MyUSART_Init(9600);
	MyI2C_Init();
	MyLED_Config();
	MPU_Config();
	while(1)
	{
		long quat[4] = {0xaaaaaaaa,0xbbbbbbbb,0xcccccccc,0xdddddddd};
		MPU_ReadDMPFifo();
		MPU_GetQuat(quat);
		MyCOM_SendData(quat,DATA_TYPE_QUAT);
		Delay_ms(500);
		MyLED_Toggle();
	}
}

