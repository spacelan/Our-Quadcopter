#ifndef MOTOR_H
#define MOTOR_H
#include "sys.h"
#define THROTTLE_BASE 20
enum MOTORS
{
	MOTOR1 = 0,
	MOTOR2,
	MOTOR3,
	MOTOR4
};

void MyMotor_Init(void);
void MyMotor_Start(void);
void MyMotor_Stop(void);
void MyMotor_SetThrottle(enum MOTORS motor,uint8_t throttle);
void MyMotor_GetThrottle(enum MOTORS motor,uint8_t* throttle);
void MyMotor_Update(void);

#endif
