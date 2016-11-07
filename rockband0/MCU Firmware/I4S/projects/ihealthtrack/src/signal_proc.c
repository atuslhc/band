#include "arm_math.h"
#include "Signal_proc.h"
#include "common_vars.h"

#define FIR_BLOCK_SIZE  32
#define PPG_8S_LEN      (FIR_BLOCK_SIZE*8)
#define PPG_1S_LEN      (FIR_BLOCK_SIZE*1)
#define PPG_4S_LEN      (FIR_BLOCK_SIZE*4)

#define FFTLEN          (PPG_8S_LEN)
#define ACCFFT_LEN      FFTLEN 
#define OriMagLen       64

int16_t PPG_OUT_DATA;
arm_status status;
arm_rfft_fast_instance_f32 INS;

float PPG_8S[PPG_8S_LEN];
float ACC_8S[PPG_8S_LEN];

int32_t PPG_RESULT_OUT[PPG_8S_LEN];

float TEMPBUFF[FFTLEN];
float PPG_Mag_ORI[OriMagLen];
float ACC_Mag_ORI[OriMagLen];
float PPG_FFT_COMPLEX[FFTLEN];//*2];

uint16_t LEN_WP = 0;
int PPG_RESULT_OUT_Len, PPG_RESULT_OUT_Rp;
uint32_t SecondPakCount = 0;
uint8_t debug_info[5];
time_t heart_time = 0;

bool SendBuffEmpty = false;

#define FIR_FILTER_ORDER     12
#define integral_size 4 //??????????????

//  pass band =3HZ , stop band =6HZ ,	40dB, 50dB
const int Band_PassFIR_Coefs[FIR_FILTER_ORDER] =
{84, 77, 103, 127, 145, 154, 154, 145, 127, 103, 77, 84};
extern uint16_t  afe_sample_counter;    //Atus: uint32_t >> uint16_t
uint16_t PPG_IN_1S_WP = 0;
void smooth_hr_output(float hr);
void  Track_INIT(void);
int32_t Integral_ppg(int32_t ac_ppg);
int32_t Filter32_Execution(int* pi32ActualSample, int* pi32BufferLimit, int* pi32Coefptr, unsigned char u8Filter_Order);
int32_t PPG_Filter(int32_t NewSample);
void Tracking_heart_rate(int32_t heart_ac_signal);
//int32_t DSP_PROCESSING(int32_t s_ppg);
void XYZ_FIR_INIT(uint8_t freq);
void FFT_PROC();

void XYZ_FIR_INIT(uint8_t freq)
{
}

uint32_t XYZFilter_TRACK(void)
{
  uint32_t t = MEMS_TRACKING(&MEMS_BUFF[0][0],
    &MEMS_BUFF[1][0],
    &MEMS_BUFF[2][0], PPG_1S_LEN);
  return t; 
}

void Set_OutDebugInfo_Len(int len)
{
  PPG_RESULT_OUT_Len=len;
}
 
void OutDebugInfo(void)
{
  int i;
#if 0
  for(i = 0; i < PPG_8S_LEN; i++)
  {
    PPG_RESULT_OUT[16+i]=PPG_8S[i];
  }
#else
  for(i = 0; i < 64; i++)
  {
    PPG_RESULT_OUT[16+i] = (int32_t)PPG_Mag_ORI[i];
  }
#endif
  PPG_RESULT_OUT_Rp = 0;
  SendBuffEmpty = false;
  //==============
}

int32_t TimingCount = 0;
void PPG_PROCESSING(void)
{
  //int i;
  extern int16_t axie[];	
  TimingCount++;
  /*
  for(i = 0; i < 3; i++)
    MEMS_BUFF[i][PPG_IN_1S_WP] = axie[i];
  PPG_IN_1S_WP++;
  if(PPG_IN_1S_WP == PPG_1S_LEN)
  {
    XYZFilter_TRACK();
    PPG_IN_1S_WP = 0;
  }
  */
  PPG_OUT_DATA = DSP_PROCESSING(read_IR_reg_22bit, read_AB_reg_22bit);
  PPG_8S[LEN_WP] = PPG_OUT_DATA;//read_IR_reg_22bit;
  ACC_8S[LEN_WP] = axie[0] + axie[1] + axie[2];
  LEN_WP++;
  LEN_WP %= PPG_8S_LEN;
  //extern int16_t AxieOUT[3];
  //for(i = 0; i < 3; i++)
  //  AxieOUT[i] = axie[i];
  if((LEN_WP%PPG_1S_LEN)==0)
  {
    SecondPakCount++;
    //------------------------------------------
    if(SecondPakCount>10)
    {	
      FFT_PROC();
    }		
#ifndef NDEBUG
    if((LEN_WP%PPG_4S_LEN)==0)
    {
      //Set_OutDebugInfo_Len(PPG_8S_LEN);
      OutDebugInfo();
    }
#endif
    debug_info[0] = iHeartRate.component.heart;
    debug_info[1] = 0;
    debug_info[2] = 0;
    debug_info[3] = 0;
    extern uint8_t Ref_F;
    debug_info[4] = Ref_F;
  }
}

void PPG_INIT(void)
{
  //systemStatus.blHeartBeatLock = false;
  LEN_WP = 0;
  SecondPakCount = 0;
  TimingCount = 0;
  INS.Sint.fftLen = FFTLEN;
  //INS.Sint.pBitRevTable
  status = arm_rfft_fast_init_f32(&INS, FFTLEN);
  if (status != ARM_MATH_SUCCESS)
  {
    while (1) ;
  }
#ifdef TEST_DATA
  produce_test_data();
#endif
  Track_INIT();
}
 
void FFT_PROC(void)
{
  int i, rp, rp_cpy;	  
  rp = LEN_WP;
  rp_cpy = rp;
  for(i = 0; i < PPG_8S_LEN; i++)
  {
    TEMPBUFF[i] = ACC_8S[rp];
    rp++;
    rp %= PPG_8S_LEN;	
  }
  arm_rfft_fast_f32(&INS, TEMPBUFF, PPG_FFT_COMPLEX, 0);
  arm_cmplx_mag_f32(PPG_FFT_COMPLEX, ACC_Mag_ORI, OriMagLen);
  for(i = 0; i < OriMagLen; i++)
    ACC_Mag_ORI[i] = ACC_Mag_ORI[i]/(FFTLEN/2);
  rp = rp_cpy;
  for(i = 0; i < PPG_8S_LEN; i++)
  {
    TEMPBUFF[i] = PPG_8S[rp];
    rp++;
    rp%=PPG_8S_LEN;
  }			  
  arm_rfft_fast_f32(&INS,TEMPBUFF, PPG_FFT_COMPLEX, 0);
  arm_cmplx_mag_f32(PPG_FFT_COMPLEX, PPG_Mag_ORI, OriMagLen);
  for(i = 0; i < OriMagLen; i++)
    PPG_Mag_ORI[i] = PPG_Mag_ORI[i]/(FFTLEN/2);
  Set_OutDebugInfo_Len(PPG_4S_LEN);
  //---------------------------------------------------  
}

//=============================
#define min_interval_samplepoint  6  // as a threshold to filte some noise
#define max_interval_samplepoint  64 // as a threshold to filte some noise
//#define confirm_threshold  8
#define confirm_threshold  4
#define peak2peak_threshold 150 //400
#define peak_buff_size 8
#define Sample_freq 32
#define HR_INIT_VAL  75

uint8_t peak_interval[peak_buff_size];
uint8_t peak_buff_wpr = 0;

uint32_t bottom_peak_position, last_bottom_peak_position;
uint32_t top_peak_position, last_top_peak_position;
uint16_t ppg_avg_peak2peak = 0;
uint8_t t2b_peak_pos_dela, b2t_peak_pos_dela;
int32_t bottom_peak_val, top_peak_val;
int32_t last_heart_ac_signal = 0;
int16_t avg_pos_dela;
uint8_t up_dir = 0, hr_confirm_count = 0, peak_good_count = 0;	

int16_t top_pos_dela, last_top_pos_dela = 0;
int16_t bottom_pos_dela, last_bottom_pos_dela = 0;
uint8_t heart_display;
uint8_t heart_buffer[3];
uint8_t heart_timecount = 0;
void smooth_hr_output(float hr)
{
  uint8_t temp_hr;
  int up, down;
  time_t currentTime;
  temp_hr = (uint8_t)(hr + 0.5);
  currentTime = time(NULL);
  
  if((currentTime - heart_time) < 5 * 60 * 60)
  {
    //up = 105 * (int)iHeartRate.component.heart / 100;
    //down = 95 * (int)iHeartRate.component.heart / 100;
    if(iHeartRate.component.heart < 150)
    {
      up = iHeartRate.component.heart + 1;
      down = iHeartRate.component.heart - 1;
    }
    else
    {
      //up = (101 * (int)iHeartRate.component.heart + 50) / 100;
      //down = (99 * (int)iHeartRate.component.heart + 50) / 100;
      up = iHeartRate.component.heart + 2;
      down = iHeartRate.component.heart - 2;
    }
    if(up > 200)
      up = 200;
    if(down < 40)
      down = 40;
    if(((int)temp_hr <= up) && ((int)temp_hr >= down))
      iHeartRate.component.heart = temp_hr;
    else
    {
      if(temp_hr > iHeartRate.component.heart)
        iHeartRate.component.heart = (uint8_t)up;
      else
        iHeartRate.component.heart = (uint8_t)down;
    }
  }
  else
  {
    if((int)temp_hr > 200)
      temp_hr = 200;
    if((int)temp_hr < 40)
      temp_hr = 40;
    iHeartRate.component.heart = temp_hr;
    heart_display = iHeartRate.component.heart;
    heart_timecount = 0;
  }
  systemStatus.blHeartBeatLock = true;
  heart_time = currentTime;
}

void Track_INIT(void)
{
  int i;
  systemStatus.blHeartBeatLock = false;
  iHeartRate.component.heart = HR_INIT_VAL;
  bottom_peak_position = 0;
  last_bottom_peak_position = 0;
  top_peak_position = 0;
  last_top_peak_position = 0;
  ppg_avg_peak2peak = 0;
  t2b_peak_pos_dela = 0;
  bottom_peak_val = 0;
  top_peak_val = 0;
  last_heart_ac_signal = 0;
  up_dir = 0;
  hr_confirm_count = 0;
  peak_good_count = 0;
  top_pos_dela = 0;
  last_top_pos_dela = 0;
  bottom_pos_dela = 0;
  last_bottom_pos_dela = 0;
  for(i = 0; i < peak_buff_size; i++)
    //peak_interval[i] = Sample_freq;
    peak_interval[i] = 0;
}

int32_t Integral_ppg(int32_t ac_ppg)
{
  static int32_t ppg_buff[integral_size];
  static  int ppg_wp = 0;
  int32_t ppg_sum;
  short i;

  ppg_buff[ppg_wp] = ac_ppg;
  ppg_wp++;
  ppg_wp %= integral_size;

  ppg_sum = 0;

  for(i = 0; i < integral_size; i++)
  {
    ppg_sum += ppg_buff[i];
  }
  return (ppg_sum);
}

int32_t Filter32_Execution(int* pi32ActualSample, int* pi32BufferLimit, int* pi32Coefptr, unsigned char u8Filter_Order)
{
  volatile unsigned char u8Filter_Order_Copy = u8Filter_Order;
  volatile int32_t i32Accumulate = 0;
  volatile int32_t i32Multiply = 0;

  while (u8Filter_Order--)
  {
    i32Multiply = (int) ((*pi32ActualSample++) * (*pi32Coefptr++));
    i32Accumulate += i32Multiply;
    if(pi32ActualSample == pi32BufferLimit)
      pi32ActualSample -= u8Filter_Order_Copy;
  }
  return (i32Accumulate);
}

int32_t PPG_Filter(int32_t NewSample)
{
  static unsigned char SampleIndex = 0;
  static int32_t PPG_FIR_Buff[FIR_FILTER_ORDER] = {0};
  int32_t FilteredValue;
  *(PPG_FIR_Buff + SampleIndex) = NewSample / 2; ///30;
  FilteredValue = Filter32_Execution((PPG_FIR_Buff + SampleIndex), (PPG_FIR_Buff + FIR_FILTER_ORDER), (int*)&Band_PassFIR_Coefs[0], FIR_FILTER_ORDER);
  if(SampleIndex < (FIR_FILTER_ORDER - 1))
  {
    SampleIndex++;
  }
  else
  {
    SampleIndex = 0;
  }
  return (FilteredValue >> 10);
}


void Tracking_heart_rate(int32_t heart_ac_signal)
{
  int16_t temp_short;
  short i;
  //float temp_float;
  float heart_rate_float;
  int16_t ppg_peak_range;
  uint8_t count;

  if(up_dir)
  {
    if(last_heart_ac_signal <= heart_ac_signal)
    {
      top_peak_position = TimingCount;
      top_peak_val = heart_ac_signal;
      hr_confirm_count = 0;
    }
    else
    {
      hr_confirm_count++;
      if(hr_confirm_count >= confirm_threshold) // changned the direction really
      {
        up_dir = 0;
        hr_confirm_count = 0;
        top_pos_dela = top_peak_position - last_top_peak_position;
        if((top_pos_dela < max_interval_samplepoint) && (top_pos_dela > min_interval_samplepoint) && ((top_peak_val - bottom_peak_val) > peak2peak_threshold))
          peak_good_count++;
        else
          peak_good_count = 0;
        last_top_pos_dela = top_pos_dela;
        b2t_peak_pos_dela = top_peak_position - last_bottom_peak_position;
        last_top_peak_position = top_peak_position;
        bottom_peak_position = TimingCount;
        bottom_peak_val = heart_ac_signal;
      }
    }
  }
  else
  {
    if(heart_ac_signal <= last_heart_ac_signal)
    {
      bottom_peak_position = TimingCount;
      bottom_peak_val = heart_ac_signal;
      hr_confirm_count = 0;
    }
    else
    {
      hr_confirm_count++;
      if(hr_confirm_count >= confirm_threshold) // changned the direction really
      {
        up_dir = 1;
        hr_confirm_count = 0;
        bottom_pos_dela = bottom_peak_position - last_bottom_peak_position;
        last_bottom_pos_dela = bottom_pos_dela;
        t2b_peak_pos_dela = bottom_peak_position - last_top_peak_position;
        last_bottom_peak_position = bottom_peak_position;
        top_peak_position = TimingCount;
        top_peak_val = heart_ac_signal;
      }
    }
  }
  if(peak_good_count > 0)
  {
    peak_good_count = 0;
    //char error = 0;
    ppg_avg_peak2peak = (ppg_avg_peak2peak + (top_peak_val - bottom_peak_val)) / 2;
    //temp_float = (float)(t2b_peak_pos_dela) / b2t_peak_pos_dela;
    //if((temp_float < 0.333) || (temp_float > 3))
    //  error++;
    ppg_peak_range = abs(top_peak_val - bottom_peak_val);
    //if(ppg_peak_range < peak2peak_threshold)
    //  error++;
    //if(ppg_peak_range > (ppg_avg_peak2peak + peak2peak_threshold))
    //  error++;
    if(ppg_peak_range > peak2peak_threshold)
    {
      avg_pos_dela = (top_pos_dela + bottom_pos_dela) / 2;
      peak_interval[peak_buff_wpr++] = top_pos_dela;//avg_pos_dela;
      peak_buff_wpr %= peak_buff_size;
      temp_short = 0;
      count = 0;
      for(i = 0; i < peak_buff_size; i++)
      {
        temp_short += peak_interval[i];
        if(peak_interval[i] != 0)
          count++;
      }	  		 
      if(count == peak_buff_size)
      {
        heart_rate_float = 60.0 * Sample_freq * peak_buff_size / temp_short;
        smooth_hr_output(heart_rate_float);
      }
    }    
  }
  last_heart_ac_signal = heart_ac_signal;
}

int32_t DSP_PROCESSING(int32_t s_ppg, int32_t s_amb)
{
  static int32_t PPG_BAK = 0;
  int32_t ppg_temp;

#if BG013RAW
  extern int16_t AxieOUT[3];
  AxieOUT[0] = (int16_t)(s_ppg>>6);
#endif
	
  ppg_temp = Integral_ppg(PPG_BAK - s_ppg);
#if BG013RAW
  extern int16_t AxieOUT[3];
  //AxieOUT[1] = (int16_t)(ppg_temp>>6);
  AxieOUT[1] = (int16_t)(s_amb>>6);
#endif
  ppg_temp = PPG_Filter(ppg_temp);
#if BG013RAW
  extern int16_t AxieOUT[3];
  AxieOUT[2] = (int16_t)(ppg_temp); //>>6
#endif
  PPG_BAK = s_ppg;	   
  //PPG_AVG = gUtilDcEstimator((long*)&PPG_AVG_REGISTER, (short)ppg_temp, 5);
  //PPG_OUT_DATA=ppg_temp;//-PPG_AVG;
  Tracking_heart_rate(ppg_temp);//(PPG_OUT_DATA);
#if 0
  extern int32_t ecg_val;
  #if 1
  ECG_OUT_DATA =- Integral_ecg(ecg_val);
  #else
  FilteredECG = ECG_Filter(ecg_val);
  ECG_OUT_DATA =- Integral_ecg(FilteredECG);
  ECG_BAK = FilteredECG;
  #endif
#endif

  return ppg_temp;
}