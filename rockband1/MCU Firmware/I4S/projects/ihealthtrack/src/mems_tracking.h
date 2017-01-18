#ifndef _mems_tracking_H_
#define _mems_tracking_H_

#define MEMS_FIFO_SIZE 32

#define MEMS_BUFF_SIZE  (MEMS_FIFO_SIZE)

#define valid_first_steps  5
#define XYZ_BUFF_SIZE (MEMS_FIFO_SIZE * 4)
#if FALL_DETECT_SUPPORT
//#define FALLING_HI_THRESHOLD	24000	//the RMS hi threshold of FD. [BG008-5] move to mems_tracking.c
//#define FALLING_LOW_THRESHOLD	6000	//the RMS low threshold of FD. [BG008-5] move to mems_tracking.c
#define AXIS_FILTER_BUFF_SIZE	(MEMS_FIFO_SIZE*3)
#define FALLING_SCAN2_BUFF	48	//1.5 sec, samplerate(32)*1.5=48
#endif

extern uint8_t Variance_Val_threshold;
extern uint8_t MEMS_FRQ;
extern int16_t pos_max_interval, pos_min_interval;
extern int16_t val_threshold;
extern uint8_t which_ax_near_zero;
extern volatile uint32_t  active_level;
extern int Window_ACT_LEVEL[3];
extern int MEMS_XYZ_DC[3];
extern uint8_t  MotionModel;
#if FALL_DETECT_SUPPORT
extern uint8_t FD_result;   //[BG008] add
extern uint8_t oldFallStatus; //[BG033] static change to global can initialize.
extern short int axis_filter_index; //[BG008] add
extern float axis_filter[AXIS_FILTER_BUFF_SIZE];    //[BG008] add, XYZ_BUFF_SIZE (32*4=128 too large oversize)
#if 1 //for api integration. 
void Init_Fall_Detect(void);
int16_t Calc_axis_rms(int16_t *x,int16_t *y,int16_t *z,uint8_t len);
int16_t post_Fall_Detect(uint8_t len);
#endif
uint8_t Fall_Detect(int32_t *axis_rms, uint8_t len); //[BG008] add
#endif

void UpdatePedometerInfor(void);
uint32_t MEMS_TRACKING(int16_t *x,int16_t *y,int16_t *z,uint8_t len);
#endif
