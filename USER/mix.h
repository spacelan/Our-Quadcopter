#ifndef MIX_H
#define MIX_H

#include "sys.h"

void attitude_init(void);
void attitude_updateAttitude(long quat[],short accel[],short gyro[],long timestamp);

#endif
