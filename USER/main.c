#include "sys.h"
#include "usart.h"
#include "time.h"
#include "led.h"
#include "i2c.h"
#include "communication.h"
#include "mpu6050.h"

int main(void)
{
	long quat[4];
	short accel[3],gyro[3];
	uint8_t CMD = COMMAND_TYPE_SEND_QUAT;
	SystemInit(); 			 //系统时钟初始化为72M	  SYSCLK_FREQ_72MHz 
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	MyTime_Init();	    	 //延时函数初始化	 
	MyLED_Config();
	MyUSART_Init(115200);
	MyCOM_Init();
	MyI2C_Init();
	Delay_ms(20);
	MPU_Config();
	MyLED(ON);
	while(1)
	{
		Delay_ms(50);
		MyCOM_GetData();
		MyCOM_GetCOMMAND(&CMD);
		if((CMD & (uint8_t)COMMAND_TYPE_RESTART))
		{
			NVIC_SystemReset();
		}
		if((CMD & (uint8_t)COMMAND_TYPE_RUN) == 0)
		{
			MyLED(ON);
			continue;
		}
		MPU_ReadDMPFifo();
		if((CMD & (uint8_t)COMMAND_TYPE_SEND_QUAT))
		{
			MPU_GetQuat(quat);
			MyCOM_SendData(quat,DATA_TYPE_QUAT);
		}
		if((CMD & (uint8_t)COMMAND_TYPE_SEND_ACCEL))
		{
			MPU_GetAccel(accel);
			MyCOM_SendData(accel,DATA_TYPE_ACCEL);			
		}
		if((CMD & (uint8_t)COMMAND_TYPE_SEND_GYRO))
		{
			MPU_GetGyro(gyro);
			MyCOM_SendData(gyro,DATA_TYPE_GYRO);	
		}
		MyLED_Toggle();
	}
}

