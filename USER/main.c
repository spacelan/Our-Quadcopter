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
	uint8_t cmd;
	SystemInit(); 			 //ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz 
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	MyTime_Init();	    	 //��ʱ������ʼ��	 
	MyLED_Config();
	MyUSART_Init(115200);
	MyCOM_Init();
	MyI2C_Init();
	Delay_ms(5);
	MPU_Config();
	MyLED(ON);
	while(1)
	{
		Delay_ms(30);
		MyCOM_GetData();
		MyCOM_GetCOMMAND(&cmd);
		switch(cmd)
		{
			case COMMAND_TYPE_RUN:
				break;
			case COMMAND_TYPE_PAUSE:
				MyLED(ON);
				continue;
			case COMMAND_TYPE_RESTART:
				NVIC_SystemReset();
				break;
		}
		MPU_ReadDMPFifo();
		MPU_GetQuat(quat);
		MyCOM_SendData(quat,DATA_TYPE_QUAT);
		MyLED_Toggle();
	}
}

