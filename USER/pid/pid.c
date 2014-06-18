#include "pid.h"

float preInput[3] = {0};
float integral[3] = {0};

void pid(float input[3],float output[3])
{
	uint8_t i;
	float pOutput,iOutput,dOutput;
	float P = 100,I = 0,D = 0;
	for(i=0;i<3;i++)
	{
		pOutput = input[i] * P;
		integral[i] += input[i] * I;
		iOutput = integral[i];
		dOutput = (input[i] - preInput[i]) * D;
		output[i] = pOutput + iOutput + dOutput;
		preInput[i] = input[i];
	}
	return;
}
