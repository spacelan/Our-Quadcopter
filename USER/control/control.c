#include "control.h"
#include "mpu6050.h"
#include "quaternion.h"
#include "pid.h"
#include "motor.h"
void control(const float targetQuat[4])
{
	float currentQuat[4],errorQuat[4];
	float output[3];
	long temp[4];
	MPU_GetQuat(temp);
	currentQuat[0] = temp[0];
	currentQuat[1] = temp[1];
	currentQuat[2] = temp[2];
	currentQuat[3] = temp[3];
	Quat_Normalize(currentQuat);
	Quat_Inverse(currentQuat);
	Quat_Mult(errorQuat,currentQuat,targetQuat);
	
	pid(&errorQuat[1],output);
	
	MyMotor_SetThrottle(MOTOR1,THROTTLE_BASE             - output[1] + (output[2] /10));
	MyMotor_SetThrottle(MOTOR2,THROTTLE_BASE + output[0]             - (output[2] /10));
	MyMotor_SetThrottle(MOTOR3,THROTTLE_BASE             + output[1] + (output[2] /10));
	MyMotor_SetThrottle(MOTOR4,THROTTLE_BASE - output[0]             - (output[2] /10));
	MyMotor_Update();
}
