#ifndef QUATERNION_H
#define QUATERNION_H

void Quat_Normalize(float quat[4]);
void Quat_Inverse(float quat[4]);
void Quat_Mult(float result[4],const float quat1[4],const float quat2[4]);

#endif
