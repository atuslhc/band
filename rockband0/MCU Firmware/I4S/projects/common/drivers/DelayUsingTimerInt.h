#ifndef __DelayUsingTimerInt_H
#define __DelayUsingTimerInt_H

#include <stdbool.h>
#include <stdint.h>

// ============================
//使用延时定时器的各个部件标志。
// 注意：调整此处定义时，请确保 delayusingtimerint.c 中 MAX_TIMER_FLAGS 能包含所有定义
//       同时，请调整定义使 MAX_TIMER_FLAGS 尽可能小，以节省空间
//       优先级高的定义请放前面
typedef enum
{
	TIMER_FLAG_KeyEvent 		= 0,
//	TIMER_FLAG_Beep        		,
	TIMER_FLAG_StopWatch        ,
	TIMER_FLAG_TEMP        		,
	TIMER_FLAG_LeUart      		,
	TIMER_FLAG_BLE           	,
//	TIMER_FLAG_WIFI				,
//	TIMER_FLAG_USBClose    		,
	TIMER_FLAG_LEDFlashing 		,
} TIMER_FLAG;
//实际上他们都使用同一个硬件定时器，当定时到了之后就去调用DelayTimerCallback(),这里面计算各个软件定时器的状态，做相应的处理。

typedef void (*TIMER_HANDLER)(void*);			// 定时器回调函数定义


typedef struct
{
//	bool enabled;
	bool repeat;
	uint16_t top;
	uint16_t counter;
	TIMER_HANDLER handler;
	void* userData;
} TIMER_DEFINE;



void EnableDelayTimer(uint16_t flag, bool repeat, uint16_t interval, TIMER_HANDLER callback, void* userData);
void DisableDelayTimer(uint16_t flag);
void ResetDelayTimer(uint16_t flag);

#endif
