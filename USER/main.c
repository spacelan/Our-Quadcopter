#include "sys.h"
#include "usart.h"
#include "time.h"
#include "led.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "communication.h"

int main(void)
{
	SystemInit(); 			 //ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz 
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	MyTime_Init();	    	 //��ʱ������ʼ��	 
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

