#ifndef __DelayUsingTimerInt_H
#define __DelayUsingTimerInt_H

#include <stdbool.h>
#include <stdint.h>

// ============================
//ʹ����ʱ��ʱ���ĸ���������־��
// ע�⣺�����˴�����ʱ����ȷ�� delayusingtimerint.c �� MAX_TIMER_FLAGS �ܰ������ж���
//       ͬʱ�����������ʹ MAX_TIMER_FLAGS ������С���Խ�ʡ�ռ�
//       ���ȼ��ߵĶ������ǰ��
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
//ʵ�������Ƕ�ʹ��ͬһ��Ӳ����ʱ��������ʱ����֮���ȥ����DelayTimerCallback(),�����������������ʱ����״̬������Ӧ�Ĵ���

typedef void (*TIMER_HANDLER)(void*);			// ��ʱ���ص���������


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
