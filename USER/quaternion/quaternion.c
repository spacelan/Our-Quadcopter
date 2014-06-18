#include "quaternion.h"

float rsqrt(float num);

void Quat_Normalize(float quat[4])
{
	float norm = rsqrt(quat[0]*quat[0]+quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3]);
	quat[0] *= norm;
	quat[1] *= norm;
	quat[2] *= norm;
	quat[3] *= norm;
	return;
}

void Quat_Inverse(float quat[4])
{
	quat[1] = -quat[1];
	quat[2] = -quat[2];
	quat[3] = -quat[3];
	return;
}

void Quat_Mult(float result[4],const float quat1[4],const float quat2[4])
{
	result[0] = quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] - quat1[3] * quat2[3];
    result[1] = quat1[1] * quat2[0] + quat1[0] * quat2[1] + quat1[2] * quat2[3] - quat1[3] * quat2[2];
    result[2] = quat1[2] * quat2[0] + quat1[0] * quat2[2] + quat1[3] * quat2[1] - quat1[1] * quat2[3];
    result[3] = quat1[3] * quat2[0] + quat1[0] * quat2[3] + quat1[1] * quat2[2] - quat1[2] * quat2[1];
}

float rsqrt(float num)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = num * 0.5F;
    y = num;
    i = *(long *)&y;                       // evil floating point bit level hacking（对浮点数的邪恶位级hack）
    i = 0x5f3759df - (i >> 1);               // what the fuck?（这他妈的是怎么回事？）
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));   // 1st iteration （第一次牛顿迭代）
    y = y * (threehalfs - (x2 * y * y));   // 2nd iteration, this can be removed（第二次迭代，可以删除）

    return y;
}