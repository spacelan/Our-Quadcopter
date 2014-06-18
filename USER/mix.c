#define NUMOFHISTORY 16
#define CHANGERAD 57.29577951

#include "mix.h"
#include "math.h"


#define q30  1073741824.0f

//those vars below are used to figure out attitude
float quater[4],gyr[3],interval,acc[3];
short indexOfHistory,numOfData,firstuse;
long sumOfAcc[3];
long accHistory[NUMOFHISTORY][3];

///////////////////////////////////////////////////////
float attitude_rsqrt(float number)
{
	 long i;
	 float x2, y;
	 const float threehalfs = 1.5F;

	 x2 = number * 0.5F;
	 y  = number;
	 i  = * ( long * ) &y;                       // evil floating point bit level hacking
	 i  = 0x5f3759df - ( i >> 1 );
	 y  = * ( float * ) &i;
	 y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	 y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	 return y;
}

void attitude_init(void)
{
	sumOfAcc[0]=0;
	sumOfAcc[1]=0;
	sumOfAcc[2]=0;

	quater[0]=1.0f;
	quater[1]=0.0f;
	quater[2]=0.0f;
	quater[3]=0.0f;

	for(indexOfHistory=0;indexOfHistory<NUMOFHISTORY;indexOfHistory++)
	{
		accHistory[indexOfHistory][0]=0;
		accHistory[indexOfHistory][1]=0;
		accHistory[indexOfHistory][2]=0;
	}

	indexOfHistory=0;
	numOfData=1;
	interval = 0;
	firstuse = 1;
}

void attitude_steepestDescentMethod(void)
{
		//some useful vars
    float w_q = quater[0];
    float x_q = quater[1];
    float y_q = quater[2];
    float z_q = quater[3];
    float x_q_2 = x_q * 2;
    float y_q_2 = y_q * 2;
    float z_q_2 = z_q * 2;
	  float a_rsqrt;
    
	  //figure out difference
    float x_da = x_q*z_q_2 - w_q*y_q_2     - acc[0];
    float y_da = y_q*z_q_2 + w_q*x_q_2     - acc[1];
    float z_da = 1 - x_q*x_q_2 - y_q*y_q_2 - acc[2];
    
	  //figure out steep
    float w_pf =  - x_da*y_q + y_da*x_q;
    float x_pf = x_da*z_q + y_da*w_q - z_da*x_q;
    float y_pf = - x_da*w_q + y_da*z_q - z_da*y_q;
    float z_pf = x_da*x_q + y_da*y_q;
    
	  // fix quat[]

    const float factor = (x_da + y_da + z_da) * 0.05;
    quater[0] -= w_pf * factor;
    quater[1] -= x_pf * factor;
    quater[2] -= y_pf * factor;
    quater[3] -= z_pf * factor;
    
		//normalize quat[]
	  a_rsqrt = attitude_rsqrt(quater[0]*quater[0]+quater[1]*quater[1]+quater[2]*quater[2]+quater[3]*quater[3]);
	  quater[0] *= a_rsqrt;
	  quater[1] *= a_rsqrt;
	  quater[2] *= a_rsqrt;
	  quater[3] *= a_rsqrt;
}

void attitude_firstUse()
{
	float from[3] = {0,0,1},a_rsqrt ;
    float from_norm = sqrtf(from[0]*from[0] + from[1]*from[1] + from[2]*from[2]);
    float to_norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    
    float cos_theta = (from[0]*acc[0] + from[1]*acc[1] + from[2]*acc[2]) / (from_norm*to_norm);
    
    float sin_half_theta = sqrtf((1.0f - cos_theta) / 2.0f);
    float cross_x = from[1]*acc[2] - from[2]*acc[1];
    float cross_y = from[2]*acc[0] - from[0]*acc[2];
    float cross_z = from[0]*acc[1] - from[1]*acc[0];
		float sin_half_theta_div_cross_norm ;

		quater[0] = sqrtf((1.0f + cos_theta) / 2.0f); // cos(theta/2)

    if(cos_theta < 0)
    {
        cross_x = - cross_x;
        cross_y = - cross_y;
        cross_z = - cross_z;
    }
		
    sin_half_theta_div_cross_norm = sin_half_theta/sqrtf(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
		
    quater[1] = -cross_x * sin_half_theta_div_cross_norm;
    quater[2] = -cross_y * sin_half_theta_div_cross_norm;
    quater[3] = cross_z * sin_half_theta_div_cross_norm;
		
		a_rsqrt = attitude_rsqrt(quater[0]*quater[0]+quater[1]*quater[1]+quater[2]*quater[2]+quater[3]*quater[3]);
	  quater[0] *= a_rsqrt;
	  quater[1] *= a_rsqrt;
	  quater[2] *= a_rsqrt;
	  quater[3] *= a_rsqrt;
}

void attitude_updateAttitude(long quat[],short accel[],short gyro[],long timestamp)
{
	 const static float FACTOR = 0.001f;	//factor of filter
	 u8 i = 0;

	 // some useful const var
	 float w_q = quater[0];
	 float x_q = quater[1];
	 float y_q = quater[2];
	 float z_q = quater[3];
	 float x_q_2 = x_q * 2;
	 float y_q_2 = y_q * 2;
	 float z_q_2 = z_q * 2;

	 //figure out const acc , according to the current quat
	 float x_ac = x_q*z_q_2 - w_q*y_q_2;
	 float y_ac = y_q*z_q_2 + w_q*x_q_2;
	 float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;

	 float a_rsqrt=0,x_ca,y_ca,z_ca,delta_x,delta_y,delta_z;
	
	 // get acc and divided by 1000
	 acc[0] = accel[0]*0.001f;
	 acc[1] = accel[1]*0.001f;
	 acc[2] = accel[2]*0.001f;
	 
	 //nomalize acc
	 a_rsqrt = attitude_rsqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
	 acc[0] *= a_rsqrt;
	 acc[1] *= a_rsqrt;
	 acc[2] *= a_rsqrt;
	 
	 if(firstuse == 1)//if this is first time to use this function,we need initialize quater by current status
	 {
		 firstuse = 0;//change flag
		 
		 attitude_firstUse();//exe function to initialize quater
		 
		 quat[0] = quater[0]*10000;
		 quat[1] = quater[1]*10000;
		 quat[2] = quater[2]*10000;
		 quat[3] = quater[3]*10000;
		 
		 return;
	 }

	 //get gyr and fix it
	 gyr[0] = gyro[0]  / CHANGERAD / 16.4;
	 gyr[1] = gyro[1]  / CHANGERAD / 16.4;
	 gyr[2] = gyro[2]  / CHANGERAD / 16.4;
	
	 //figure out time interval
	 interval = (timestamp - interval)/1000;
	
	 //acc filter
	 for(i=0;i<3;i++)
	 {
		 sumOfAcc[i] += acc[i]-accHistory[indexOfHistory][i];
		 accHistory[indexOfHistory][i] = acc[i];
	 }
	 
	 if(numOfData > NUMOFHISTORY)
	 {
		acc[0] = sumOfAcc[0]/NUMOFHISTORY;
		acc[1] = sumOfAcc[1]/NUMOFHISTORY;
		acc[2] = sumOfAcc[2]/NUMOFHISTORY;
	 }
	 else
	 {
		 acc[0] = sumOfAcc[0]/numOfData;
		 acc[1] = sumOfAcc[1]/numOfData;
		 acc[2] = sumOfAcc[2]/numOfData;
		 numOfData++;
	 }
	 
	 x_ca = acc[1] * z_ac - acc[2] * y_ac;
   y_ca = acc[2] * x_ac - acc[0] * z_ac;
	 z_ca = acc[0] * y_ac - acc[1] * x_ac;

	 //
	 delta_x = gyr[0] * interval / 2 + x_ca * FACTOR;
	 delta_y = gyr[1] * interval / 2 + y_ca * FACTOR;
	 delta_z = gyr[2] * interval / 2 + z_ca * FACTOR;

	 //figure out quater[]
	 quater[0] = w_q         - x_q*delta_x - y_q*delta_y - z_q*delta_z;
	 quater[1] = w_q*delta_x + x_q         + y_q*delta_z - z_q*delta_y;
	 quater[2] = w_q*delta_y - x_q*delta_z + y_q         + z_q*delta_x;
	 quater[3] = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;

	 //normalize quater[]
	 a_rsqrt = attitude_rsqrt(quater[0]*quater[0]+quater[1]*quater[1]+quater[2]*quater[2]+quater[3]*quater[3]);
	 quater[0] *= a_rsqrt;
	 quater[1] *= a_rsqrt;
	 quater[2] *= a_rsqrt;
	 quater[3] *= a_rsqrt;
	 
	 indexOfHistory++;
	 if(indexOfHistory == 16)//fix quater[] according acc every 16 times entering this function(except first use this function)
	 {
		 indexOfHistory = 0;
		 attitude_steepestDescentMethod();
	 }
	 
	 //update quat[]
	 quat[0] = quater[0]*10000;
	 quat[1] = quater[1]*10000;
	 quat[2] = quater[2]*10000;
	 quat[3] = quater[3]*10000;
	 
	 interval = timestamp;//record last timestamp
}
///////////////////////////////////////////////////////

