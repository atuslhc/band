#ifndef _fall_detect_H_
#define _fall_detect_H_

#define FALL_DETECT_SUPPORT		1	//turn on compiler flag.
#define INTEG_MBIENT			1	// for API with 3rd party integration.

#define MEMS_FIFO_SIZE 32  //assume max. sample rate

#define MEMS_BUFF_SIZE  (MEMS_FIFO_SIZE)

#define XYZ_BUFF_SIZE (MEMS_FIFO_SIZE * 4)
#if FALL_DETECT_SUPPORT
#define AXIS_FILTER_BUFF_SIZE	(MEMS_FIFO_SIZE*3)
#define FALLING_SCAN2_BUFF	48	//1.5 sec, samplerate(32)*1.5=48
#endif

#if FALL_DETECT_SUPPORT
extern uint8_t FD_result;
extern uint8_t oldFallStatus;
extern short int axis_filter_index;
extern float axis_filter[AXIS_FILTER_BUFF_SIZE];
#if 1 //for api integration. 
void Init_Fall_Detect(void);
int16_t Calc_axis_rms(int16_t *x,int16_t *y,int16_t *z,uint8_t len);
int16_t post_Fall_Detect(uint8_t len);
#endif
uint8_t Fall_Detect(int32_t *axis_rms, uint8_t len);
#endif

#endif
