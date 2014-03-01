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
	SystemInit(); 			 //ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz 
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	MyTime_Init();	    	 //��ʱ������ʼ��	 
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

