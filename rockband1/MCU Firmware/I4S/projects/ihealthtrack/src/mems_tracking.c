#include <stdbool.h>

#include "arm_math.h"
#include "mems_tracking.h"
#include "common_vars.h"

#include <stdlib.h>

#define MEMS_MOTION_THRESHOLD           350
#if GESTURE_DISP_SUPPORT	//[BG012] add compiler flag.
#define Z_THRESHOLD             5000
uint8_t trigger_on, trigger_off;
bool trigger_state;
#if BGXXX==2
uint32_t gesturecount=0; //debug the gesture trigger counts.
#endif
#endif
int16_t data_index;
int16_t last_mems_sample = 0, val_at_top_peak = 0, val_at_bottom_peak = 0;
bool step_sign = false;
bool walk_state;
uint8_t inside_iSteps = 0;
uint8_t prev_inside_iSteps[2];
uint8_t steps_time = 1;
int16_t XYZ_BUFF[XYZ_BUFF_SIZE][3];
int16_t XYZ_BUFF_Wp = 0;
int XYZ_ROOT = 0;
int MEMS_XYZ_DC_REG[3];
int32_t mems_P2P_Val_Delta;
int16_t pos_at_top_peak, pos_at_bottom_peak;
uint8_t step_count;

extern float BMR_PER_SECOND, BMR_RATE;
extern volatile uint32_t iCalories;
extern volatile uint32_t iSteps, iDistance;

void StepsTrack(int32_t mems_sample, int16_t i);
void Init_StepsTrack(void);
short XYZDcEstimator(int *p, short x, unsigned char d);

short XYZDcEstimator(int *p, short x, unsigned char d)
{
  *p += ((((long) x << 16) - *p) >> d);
  return(*p >> 16);
}

void Init_StepsTrack(void)
{
  int i;
  last_mems_sample = 0;
  val_at_top_peak = 0;	
  val_at_bottom_peak = 0;
  step_count = 0;
  step_sign = true;
  pos_at_top_peak = 0;
  pos_at_bottom_peak = 0;
  inside_iSteps = 0;
  for (i = 0; i < 3; i++)
  {
    MEMS_XYZ_DC[i] = 0;
    MEMS_XYZ_DC_REG[i] = 0;
    Window_ACT_LEVEL[i] = 0;
  }
  prev_inside_iSteps[0] = 0;
  prev_inside_iSteps[1] = 0;
  walk_state = false;
#if GESTURE_DISP_SUPPORT	//[BG012] add compiler flag.
  trigger_on = 0;
  trigger_off = 0;
  trigger_state = false;
  #if BGXXX==2
  gesturecount = 0;
  #endif
#endif
#if FALL_DETECT_SUPPORT		//[BG012] add compiler flag.
  for (i=0 ; i<AXIS_FILTER_BUFF_SIZE ; i++) {
    axis_filter[i] = 0;
  }
  //FD_result = 0;
  //oldFallStatus = 0;
  axis_filter_index = 0;
#endif
}

void UpdatePedometerInfor(void)
{ 
  static float iCalories_bak = 0, iDistance_bak = 0;
  float step_speed, step_distance, speed, calories;
  //===================================
  step_speed = inside_iSteps * MEMS_FRQ / XYZ_BUFF_SIZE / steps_time;
  // step_distance is the distance estimation of one step
  // unit: m/s
  // 1.43 X
  if(step_speed >= 4.0)
    step_distance = systemSetting.userProfile.height / 160.0;
  else if(step_speed >= 3.0)
    step_distance = systemSetting.userProfile.height / 200.0;
  else if(step_speed >= 2.5)
    step_distance = systemSetting.userProfile.height / 200.0;
  else if(step_speed >= 2.0)
    step_distance = systemSetting.userProfile.height / 200.0;
  else if(step_speed >= 1.5)
    step_distance = systemSetting.userProfile.height / 240.0;
  else if(step_speed >= 1.0)
    step_distance = systemSetting.userProfile.height / 240.0;
  else
    step_distance = systemSetting.userProfile.height / 240.0;
  iDistance_bak += (step_distance * (float)inside_iSteps);
  
  speed = step_distance * step_speed * 3.6; // m/s * 3600 / 1000
  
  if(speed >= 20.0)
    calories = 4.0 * 2.5 * 132.5 * systemSetting.userProfile.weight / 60000;
  else if(speed >= 16.0)
    calories = 4.0 * 2 * 132.5 * systemSetting.userProfile.weight / 60000;
  else if(speed >= 12.0)
    calories = 4.0 * 1.5 * 132.5 * systemSetting.userProfile.weight / 60000;
  else if(speed >= 8.0)
    calories = 4.0 * 132.5 * systemSetting.userProfile.weight / 60000;
  else if(speed >= 6.0)
    calories = 4.0 * 77.5 * systemSetting.userProfile.weight / 60000;
  else if(speed >= 4.0)
    calories = 4.0 * 61.0 * systemSetting.userProfile.weight / 60000;
  else if(speed >= 2.0)
    calories = 4.0 * 44.5 * systemSetting.userProfile.weight / 60000;
  else
    calories = 4.0 * BMR_PER_SECOND;
  if(steps_time == 3)
    calories = 3.0 * calories - 8.0 * BMR_PER_SECOND;
  if(systemStatus.blSkinTouched == false)
    calories = 0.0;
  //===================================

  iSteps += inside_iSteps;
  
  if(iDistance_bak >= 1.0)
  {
    iDistance += (UINT)iDistance_bak;
    iDistance_bak = iDistance_bak - (UINT)iDistance_bak;
  }
  //iDistance = (UINT)(iDistance_bak * 1000);
		 	
  iCalories_bak += calories;
  if(iCalories_bak >= 1.0)
  {
    iCalories += (UINT)iCalories_bak;
    iCalories_bak = iCalories_bak - (UINT)iCalories_bak;
  }
  //iCalories = (UINT)(iDistance_bak * 100);
  
  int XYZ_WINDOW_ENG = XYZ_ROOT >> 6;
  XYZ_WINDOW_ENG = XYZ_WINDOW_ENG / MEMS_FRQ / 3;
  active_level += XYZ_WINDOW_ENG;

  MotionModel = inside_iSteps;
  if(MotionModel == 0)
  {
    if(XYZ_WINDOW_ENG > 3)
      MotionModel = 255;
  }
  inside_iSteps = 0;	 
}

int16_t top_value[(XYZ_BUFF_SIZE >> 2) + 5];
uint8_t top_index = 0;
void StepsTrack(int32_t mems_sample, int16_t index)
{
  int16_t i;
  int c;
  if (step_sign == true)
  {
    if (mems_sample > val_at_top_peak)
    {
      val_at_top_peak = mems_sample;
      pos_at_top_peak = (int16_t)(index + 256);
      step_count = 0;
      top_index = 0;
      top_value[top_index++] = mems_sample;
    }
    else if ((mems_sample < val_at_bottom_peak) && ((val_at_top_peak - val_at_bottom_peak) < val_threshold))
    {
      val_at_bottom_peak = mems_sample;
      pos_at_bottom_peak = (int16_t)(index + 256);
      step_count = 0;
      step_sign = false;
    }
    else
    {
      step_count++;
      if (top_index < ((XYZ_BUFF_SIZE >> 2) + 5))
        top_value[top_index++] = mems_sample;
      if(step_count > 3)
      {
        val_at_bottom_peak = mems_sample;
        pos_at_bottom_peak = (int16_t)(index + 256);
        step_count = 0;
        step_sign = false;
      }
    }
  }
  else
  {
    if (top_index < ((XYZ_BUFF_SIZE >> 2) + 5))
      top_value[top_index++] = mems_sample;
    if (mems_sample < val_at_bottom_peak)
    {
      val_at_bottom_peak = mems_sample;
      pos_at_bottom_peak = (int16_t)(index + 256);
      step_count = 0;
    }
    else if ((mems_sample > val_at_top_peak) && ((val_at_top_peak - val_at_bottom_peak) < val_threshold))
    {
      val_at_top_peak = mems_sample;
      pos_at_top_peak = (int16_t)(index + 256);
      step_count = 0;
      top_index = 0;
      step_sign = true;
    }
    else
    {
      step_count++;
      if (step_count > 3)
      {
        i = (int16_t)(pos_at_bottom_peak - pos_at_top_peak);
        if ((i > (pos_min_interval >> 1)) && (i < pos_max_interval))
        {
          mems_P2P_Val_Delta = abs(val_at_top_peak - val_at_bottom_peak);
          if (mems_P2P_Val_Delta > val_threshold)
          {
            c = 0;
            for (i = 0; i < (top_index - 5); i++)
            {
              if (top_value[i + 1] > top_value[i])
                c = c + top_value[i + 1] - top_value[i];
            }
            if (c < (mems_P2P_Val_Delta >> 1))
              inside_iSteps++;
          }
        }
        data_index = (int16_t)(index - 4);
        if (data_index < 0)
          data_index = 0;
        val_at_top_peak = val_at_bottom_peak;
        pos_at_top_peak = (int16_t)(data_index + 256);
        step_count = 0;
        step_sign = true;
      }
    }
  }
  last_mems_sample = mems_sample;
}

void UnLockScreen(bool raiseEvent);
int32_t axis_rms[XYZ_BUFF_SIZE];
int16_t axis_rms_index = 0;  //the index of axis_rms[]

uint32_t MEMS_TRACKING(int16_t *x, int16_t *y, int16_t *z, uint8_t len)
//uint32_t MEMS_TRACKING(int16_t *FIFO, uint8_t len);
{
  int16_t i, j;
  uint32_t act_energy = 0;
  uint8_t temp_step;
  int int_temp;
  int xxyyzz[3];
  uint8_t which_DC_max = 0, which_act_max = 0;
  for(i = 0; i < len; i++)
  {
    axis_rms[axis_rms_index + i] = (int32_t)
      sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
    xxyyzz[0] = x[i];
    xxyyzz[1] = y[i];
    xxyyzz[2] = z[i];
    for(j = 0; j < 3; j++)  
    {
      MEMS_XYZ_DC[j] = XYZDcEstimator(&MEMS_XYZ_DC_REG[j], xxyyzz[j], 5);
      XYZ_BUFF[XYZ_BUFF_Wp][j] = xxyyzz[j] - MEMS_XYZ_DC[j];
      Window_ACT_LEVEL[j] += abs(XYZ_BUFF[XYZ_BUFF_Wp][j]);
      act_energy += abs(XYZ_BUFF[XYZ_BUFF_Wp][j]);
    }
    XYZ_BUFF_Wp++;
#if GESTURE_DISP_SUPPORT	//[BG012] add compiler flag.
    if((systemStatus.blSkinTouched == true) && (z[i] < -Z_THRESHOLD))
    {
      trigger_off = 0;
      if(trigger_on < 32)
        trigger_on++;
      else if(trigger_state == false)
      {
        trigger_state = true;
        UnLockScreen(true);
#if BGXXX==2
        gesturecount++;
#endif
      }
    }
    else
    {
      trigger_on = 0;
      if(trigger_off < 32)
        trigger_off++;
      else if(trigger_state == true)
        trigger_state = false;
    }
#endif
  }
#if FALL_DETECT_SUPPORT		//[BG012] add compiler flag.
//  if (FD_result < 255) //[BG008] add. [BG008-4] remove 255 check.
    FD_result += Fall_Detect(axis_rms, len); //[BG008] add call FD, [BG008-3] &axis_rms[axis_rms_index] >> axis_rms
//  else
//    Fall_Detect(axis_rms, len); //[BG008] add call FD, [BG008-3] &axis_rms[axis_rms_index] >> axis_rms
#endif
  axis_rms_index += len;
  if(axis_rms_index >= XYZ_BUFF_SIZE) //4sec turn around.
  {
    axis_rms_index = 0;
    XYZ_BUFF_Wp = 0;
    which_DC_max = 0;
    int_temp = 0;
    for(i = 0; i < 3; i++) // find which axie dc max
    {		 		  
      if(int_temp < abs(MEMS_XYZ_DC[i]))
      {
        int_temp = abs(MEMS_XYZ_DC[i]);
        which_DC_max = i;
      }
    }
    int_temp = 0;
    which_act_max = 0;
    for(i = 0; i < 3; i++) // find  which axie is nearest to zero ,but act_level is max
    {
      if(i == which_DC_max)
        continue;
      if(int_temp < Window_ACT_LEVEL[i])
      {
        int_temp = Window_ACT_LEVEL[i];
        which_act_max = i;
        //which_act_avg_level = int_temp;
      }
    }
    XYZ_ROOT = 0;
    for(i = 0; i < 3; i++)
    {
      XYZ_ROOT += Window_ACT_LEVEL[i];
      Window_ACT_LEVEL[i] = 0;
    }
    //which_act_avg_level /= (2 * XYZ_BUFF_SIZE);
    which_ax_near_zero = which_act_max;
    if (pos_at_bottom_peak > 128)
      pos_at_bottom_peak = (int16_t)(pos_at_bottom_peak - 128);
    if (pos_at_top_peak > 128)
      pos_at_top_peak = (int16_t)(pos_at_top_peak - 128);

    int_temp = 0;
    for (i = 0; i < XYZ_BUFF_SIZE; i++)
      int_temp += (int)axis_rms[i];
    int_temp = int_temp >> 7;
    for(i = 0; i < XYZ_BUFF_SIZE; i++)
    {
      data_index = i;
      StepsTrack(axis_rms[i] - int_temp, i);
      //StepsTrack(XYZ_BUFF[i][which_ax_near_zero]);
      i = data_index;
    }

    steps_time = 1;
    if (walk_state == true)
    {
      if (inside_iSteps == 0)
      {
        prev_inside_iSteps[0] = 0;
        prev_inside_iSteps[1] = 0;
        walk_state = false;
      }
    }
    else
    {
      temp_step = (uint8_t)(prev_inside_iSteps[0] + prev_inside_iSteps[1] + inside_iSteps);
      if (temp_step > 10)
      {
        walk_state = true;
        inside_iSteps = (uint8_t)(inside_iSteps + prev_inside_iSteps[0] + prev_inside_iSteps[1]);
        prev_inside_iSteps[0] = 0;
        prev_inside_iSteps[1] = 0;
        steps_time = 3;
      }
      else
      {
        prev_inside_iSteps[0] = prev_inside_iSteps[1];
        prev_inside_iSteps[1] = inside_iSteps;
        inside_iSteps = 0;
      }
    }
#if SOS_HIT_SUPPORT
extern int8_t SOS_hit_count;
extern uint8_t SOS_result;
#if 0
    if (inside_iSteps<SOS_HIT_LOW_THRESHOLD || inside_iStep>SOS_HIT_HI_THRESHOLD)
    {
      prev_SOS_hit_count[0] = 0;
      prev_SOS_hit_count[1] = 0;
    }
    else
    {
      
    }
#endif
//    if (SOS_hit_count < inside_iSteps)

      SOS_hit_count = inside_iSteps;
    SOS_result = walk_state;
#endif
    UpdatePedometerInfor();
  }
  return act_energy;
}
#if SOS_HIT_SUPPORT
#define SOS_HIT_LOW_THRESHOLD   6
#define SOS_HIT_HI_THRESHOLD    15
uint8_t SOS_result=0;   //[BG014] add.
int8_t SOS_hit_count=0;
int8_t prev_SOS_hit_count[2]={0,0};
#endif

/*****************************************************************************
 * The fall detect module will calculate the accelerometer 3-axis data to
 * energy curve and report the result. The caller can gather all of them and 
 * issue a signal. In the module do not support the timer to keep or ignore
 * the result in a certain duration. User can use a external variable and timer
 * to do it.
 *****************************************************************************/
#if FALL_DETECT_SUPPORT		//[BG012] add compiler flag.
/* [BG008] add block begin */
#define FALLING_HI_THRESHOLD	23000	//the RMS hi threshold of FD. [BG008-5] move to mems_tracking.c 24000 >> 23000
#define FALLING_LOW_THRESHOLD	6360	//the RMS low threshold of FD. [BG008-5] move to mems_tracking.c 6000 >> 6360
short int axis_filter_index = 0; //the index of axis_filter[]
float axis_filter[AXIS_FILTER_BUFF_SIZE];    //XYZ_BUFF_SIZE (32*4=128 too large oversize)
uint8_t FD_result=0;                          //global flag for algo. result, increment 1 while detect.
uint8_t oldFallStatus=0;	//[BG033] move CheckFall() static to global.
float fd_hi_threshold = FALLING_HI_THRESHOLD;   //[BG008-4] add vars, reserve can adjust by user setting.
float fd_low_threshold = FALLING_LOW_THRESHOLD; //[BG008-4] add vars, reserve can adjust by user setting.
#define BG008_7         1
#define FD_HI_DURATION  5       //[BG008-7] third condition
#define FD_LOW_DURATION 10      //[BG008-7] third condition
#define FD_HI_PEAK      36000   //[BG008-7] fourth condition
#define INTEG_MBIENT	1	// for API with 3rd party integration.

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
		XYZ_BUFF_Wp = 0;
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
	int l;        //[BG008-7] add.
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
