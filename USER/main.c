#include "sys.h"
#include "usart.h"
#include "time.h"
#include "led.h"
#include "i2c.h"
#include "communication.h"
#include "mpu6050.h"
#include "control.h"
#include "motor.h"
int main(void)
{
	long quat[4];
	float target[4] = {1,0,0,0};
	uint8_t throttle[4];
	short accel[3],gyro[3];
	uint8_t CMD = COMMAND_TYPE_SEND_QUAT;
//	SystemInit(); 			 //系统时钟初始化为72M	  SYSCLK_FREQ_72MHz 
//	SCB->VTOR = FLASH_BASE | 0x8000; /* Vector Table Relocation in Internal FLASH. */
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
		control(target);
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
		if((CMD & (uint8_t)COMMAND_TYPE_SEND_THROTTLE))
		{
			MyMotor_GetThrottle(MOTOR1,&throttle[0]);
			MyMotor_GetThrottle(MOTOR2,&throttle[1]);
			MyMotor_GetThrottle(MOTOR3,&throttle[2]);
			MyMotor_GetThrottle(MOTOR4,&throttle[3]);
			MyCOM_SendData(throttle,DATA_TYPE_THROTTLE);	
		}
		MyLED_Toggle();
	}
}

