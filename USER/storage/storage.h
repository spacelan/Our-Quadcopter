#ifndef STORAGE_H
#define STORAGE_H

#include "sys.h"
//其实是数据的起始地址
enum STORAGE_DATA_TYPE
{
	STORAGE_DATA_TYPE_HEAD = 0,//2bytes
	STORAGE_DATA_TYPE_ACCEL_BIAS = STORAGE_DATA_TYPE_HEAD + 2,//6bytes
	STORAGE_DATA_TYPE_GYRO_BIAS = STORAGE_DATA_TYPE_ACCEL_BIAS + 6,//6bytes
	STORAGE_DATA_TYPE_MOTOR_BASE = STORAGE_DATA_TYPE_GYRO_BIAS + 6,//4bytes
	STORAGE_DATA_TYPE_P = STORAGE_DATA_TYPE_MOTOR_BASE + 4,//2bytes
	STORAGE_DATA_TYPE_I = STORAGE_DATA_TYPE_P + 2,//2bytes
	STORAGE_DATA_TYPE_D = STORAGE_DATA_TYPE_I + 2//2bytes
};

void MyStorage_Init(void);
void MyStorage_Write(void *data,uint8_t type);
void MyStorage_Read(void *data,uint8_t type);
void MyStorage_Program(void);

#endif


