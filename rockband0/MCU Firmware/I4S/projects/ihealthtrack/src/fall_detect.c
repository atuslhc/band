/*****************************************************************************
 * The fall detect module will calculate the accelerometer 3-axis data to
 * energy curve and report the result. The caller can gather all of them and 
 * issue a signal. In the module do not support the timer to keep or ignore
 * the result in a certain duration. User can use a external variable and timer
 * to do it.
 * Generally, the sensor have a buffer(FIFO) store the data, or user can store'
 * the sampling data by external array while reading each sampling. We just need 
 * call the module per second to keep the data process up to date. The caller 
 * should call Init_Fall_Detect() after system initialize, then parsing the
 * data to module by array to Calc_axis_rms() to prepare the first stage data.
 * If the system already have a function calculated the 3-axis RMS value, he can
 * skip the Calc_axis_rms() and call the Fall_Detect(), and then call
 * post_Fall_Detect() adjust external axis_rms[] index.
 *****************************************************************************/

#include <stdbool.h>

#include "arm_math.h"
#include "fall_detect.h"

#include <stdlib.h>

//int16_t MEMS_BUFF[3][MEMS_BUFF_SIZE]; //to store accelerometer x,y,z data.

int32_t axis_rms[XYZ_BUFF_SIZE];
int16_t axis_rms_index = 0;  //the index of axis_rms[]


#if FALL_DETECT_SUPPORT		//[BG012] add compiler flag.
#define FALLING_HI_THRESHOLD	23000	//the RMS hi threshold of FD. [BG008-5] move to mems_tracking.c 24000 >> 23000
#define FALLING_LOW_THRESHOLD	6360	//the RMS low threshold of FD. [BG008-5] move to mems_tracking.c 6000 >> 6360
short int axis_filter_index = 0; //the index of axis_filter[]
float axis_filter[AXIS_FILTER_BUFF_SIZE];    //XYZ_BUFF_SIZE (32*4=128 too large oversize)
uint8_t FD_result=0;                          //global flag for algo result.
uint8_t oldFallStatus=0;	//[BG033] move CheckFall() static to global.
float fd_hi_threshold = FALLING_HI_THRESHOLD;   //[BG008-4] add vars, reserve can adjust by user setting.
float fd_low_threshold = FALLING_LOW_THRESHOLD; //[BG008-4] add vars, reserve can adjust by user setting.
#define BG008_7         1
#define FD_HI_DURATION  5       //[BG008-7] third condition
#define FD_LOW_DURATION 10      //[BG008-7] third condition
#define FD_HI_PEAK      36000   //[BG008-7] fourth condition


#if INTEG_MBIENT
/* Fall_Detect initialize. It should be called before use the Fall_Detect().
 * params: none.
 */
void Init_Fall_Detect(void)
{
  	/* reset the algo result */
	FD_result = 0;
	oldFallStatus = 0;
	/* clear buffer and index */
	axis_filter_index = 0;
	memset(axis_filter, 0x00, sizeof(axis_filter));
}
/* Calculate the x, y, z to rms value and store to an array.
 * params:
 * 	*x: the accelerometer axis-x values, it contains len element one second.
 *	*y: the accelerometer axis-y values, it contains len element one second.
 *	*z: the accelerometer axis-z values, it contains len element one second.
 *	len: the elments passing to in the array, generally is sampling rate.
 * The result will auto store into the axis_rms and but without update the
 *  index until call post_Fall_Detect().
 */
int16_t Calc_axis_rms(int16_t *x, int16_t *y, int16_t *z, uint8_t len)
{
  	int i;
	
	for(i = 0; i < len; i++)
	{
		axis_rms[axis_rms_index + i] = (int32_t) sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
	}
	return axis_rms_index;
}

/* post_Fall_Detect use after call Fall_Detect() for update the buffer index and action if need.
 *
 */
int16_t post_Fall_Detect(uint8_t len)
{
	axis_rms_index += len;
	if(axis_rms_index >= XYZ_BUFF_SIZE) //4sec turn around.
	{
		axis_rms_index = 0;
	}
	return axis_rms_index;
}
#endif

uint8_t Fall_Detect(int32_t *axis_rms, uint8_t len) //[BG008] add
{
	float a[]={(float)1.0, (float)-3.330669073875470e-16, (float)0.171572875253810}; //cofet a/b
	float b[]={(float)0.292893218813452, (float)0.585786437626905, (float)0.292893218813452}; //cofet
	uint8_t result=0;
	uint8_t fd_low_duration;
	int i,j,k;
	int l;        //[BG008-7] add. [BG025] h not used, remark.
	static int fdscanbackoffset=0; //[BG008-7] add, change check range from last fd trigger until FALLING_SCAN2_BUFF.
        //static int8_t fd_temp = 0; //[BG008-7] add.[BG025] not used, remark.
        //static int8_t fd_hi_duration=0; //[BG025] not used, remark.
	/* calculate the axis_filter[] base on axis_rms[] */
        for (i=0,j=axis_rms_index,k=axis_filter_index ; i<len ; i++,j++,k++)
             axis_filter[k] = b[0]*axis_rms[j] + b[1]*axis_rms[(j-1>=0)?j-1:j-1+XYZ_BUFF_SIZE] - a[1]*axis_filter[(k-1>=0)?k-1:k-1+AXIS_FILTER_BUFF_SIZE] + b[2]*axis_rms[(j-2>=0)?j-2:j-2+XYZ_BUFF_SIZE] - a[2]*axis_filter[(k-2>=0)?k-2:k-2+AXIS_FILTER_BUFF_SIZE];
	/* scan the axis_filter[] with threshold checking */
	for (j=axis_filter_index ; (j-axis_filter_index)<len && result==0 ; j++)
	{
		if (axis_filter[j]>fd_hi_threshold) //[BG008-4] support dynamic threshold. FALLING_HI_THRESHOLD >> fd_hi_threshold
		{
			if (fdscanbackoffset<FALLING_SCAN2_BUFF && j-axis_filter_index+fdscanbackoffset<FALLING_SCAN2_BUFF) //[BG008-7] add k not fix -FALLING_SCAN2_BUFF, also refer last fd occurs.
				k = axis_filter_index-fdscanbackoffset;
			else
				k = j-FALLING_SCAN2_BUFF; //backwards scan distance for low threshold.
			if (k<0) //adjust round index
				k=k+AXIS_FILTER_BUFF_SIZE;
			for (i=j-1 ; i!=k && result==0 ; i--) //[BG008-5] backwards scan
			{
				if (i<0) //round back to array tail 
					i=AXIS_FILTER_BUFF_SIZE-1;
#if 1   //[BG008-7] remark because fdscanbackoffset initial 0 can cover this condition.
				if (axis_filter[i]<=0) { //not store element yet.
					break;
				}
#endif
				if (axis_filter[i]<fd_low_threshold) // && systemStatus.blSkinTouched==true) //[BG008-4] FALLING_LOW_THRESHOLD >> fd_low_threshold. TODO: && systemStatus.blSkinTouched==true
				{
#if (BG008_7)
					for (l=i,fd_low_duration=0 ; l!=k ; l--) {
						if (l<0) //round back to array tail
							l = AXIS_FILTER_BUFF_SIZE-1;
						if (axis_filter[l] < fd_low_threshold)
							fd_low_duration++;
						else
							break;
					}
					if (fd_low_duration<=FD_LOW_DURATION) {
#endif
						result ++;  //count trigger, also break loop
						fdscanbackoffset=len-(j-axis_filter_index); //[BG008-7] add
					}
				}
			}
		}
	}
	/* update the axis_filter_index */
	if ((axis_filter_index += len) >= AXIS_FILTER_BUFF_SIZE)
		axis_filter_index = 0;
	if (result==0 && fdscanbackoffset<FALLING_SCAN2_BUFF) { //[BG008-7] add
		fdscanbackoffset += len;
		if (fdscanbackoffset >  FALLING_SCAN2_BUFF)
			fdscanbackoffset = FALLING_SCAN2_BUFF;
	}
	return result;
}
/* [BG008] add block end */
#endif
