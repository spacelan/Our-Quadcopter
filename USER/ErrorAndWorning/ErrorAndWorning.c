#include "ErrorAndWorning.h"
#include "led.h"
#include "time.h"

void MyError(u8 n)
{
	u8 t = n;
	while(1)
	{
		while(t--)
		{
			MyLED(ON);
			Delay_ms(50);
			MyLED(OFF);
			Delay_ms(100);
		}
		Delay_ms(1000);
		t = n;
	}
}

void MyWorning(void)
{
	MyLED(ON);
	Delay_ms(500);
	MyLED(OFF);
}
