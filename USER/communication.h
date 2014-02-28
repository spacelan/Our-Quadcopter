#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "sys.h"
enum DATA_TYPE
{
	DATA_TYPE_NONE = 0,
	DATA_TYPE_QUAT,
	DATA_TYPE_ACCEL,
	DATA_TYPE_GYRO,
	DATA_TYPE_COMTROL,
	DATA_TYPE_PARAM,
	DATA_TYPE_MESSAGE
};
void MyCOM_SendData(const void *data,uint8_t dataType);
void MyCOM_GetData(void *data,uint8_t *dataType);

#endif
