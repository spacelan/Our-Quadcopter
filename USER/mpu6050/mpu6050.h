#ifndef MPU6050_H
#define MPU6050_H

#include "sys.h"

void MPU_Config(void);
void MPU_ReadDMPFifo(void);
void MPU_GetQuat(long *needQuat);
void MPU_GetAccel(short *needAccel);
void MPU_GetGyro(short *needGyro);
#endif
