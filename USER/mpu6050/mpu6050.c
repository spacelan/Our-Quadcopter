#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ErrorAndWorning.h"
#include "communication.h"
//#include "math.h"

#define q30  1073741824.0f

short gyro[3], accel[3], sensors;
long quat[4];
unsigned long timestamp = 0;

void NO_ACTION()
{
}

void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x03) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] = (long)(accel[0] * accel_sens);
        accel[1] = (long)(accel[1] * accel_sens);
        accel[2] = (long)(accel[2] * accel_sens);
        dmp_set_accel_bias(accel);
    }
}

void MPU_Config(void)
{
	mpu_init() == 0 ? NO_ACTION() : MyError(1);
	mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO) == 0 ? NO_ACTION() : MyError(2);
//	mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO) == 0 ? NO_ACTION() : MyError(3);
	mpu_set_sample_rate(200) == 0 ? NO_ACTION() : MyError(4);

    dmp_load_motion_driver_firmware() == 0 ? NO_ACTION() : MyError(5);
    dmp_enable_feature(  DMP_FEATURE_6X_LP_QUAT
    					|DMP_FEATURE_GYRO_CAL
    					|DMP_FEATURE_SEND_CAL_GYRO
    					|DMP_FEATURE_SEND_RAW_ACCEL
    					) == 0 ? NO_ACTION() : MyError(6);
    dmp_set_fifo_rate(50) == 0 ? NO_ACTION() : MyError(7);
    mpu_set_dmp_state(ENABLE) == 0 ? NO_ACTION() : MyError(8);
//    run_self_test();
}

void MPU_ReadDMPFifo(void)
{
	unsigned char more;
    while(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) || more) ;
}

void MPU_GetQuat(long *needQuat)
{
	needQuat[0] = quat[0];
	needQuat[1] = quat[1];
	needQuat[2] = quat[2];
	needQuat[3] = quat[3];
}
