#ifndef LED_H
#define	LED_H

#include "sys.h"

/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1

#define MyLED(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_15);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_15)


void MyLED_Config(void);
void MyLED_Toggle(void);
	
#endif /* __MYLED_H */
