// Using this Timer to do interrupt dealy , the minium dealy uint is 100MS 
// Because the timer can't run  at EM2 , inform OS by "DelayTimer_RUNNING" flag

#include "em_int.h"
#include "em_timer.h"

#include "main.h"
#include "common_vars.h"
#include "device_task.h"
#include "DelayUsingTimerInt.h"


// =================================================================
//#define MAX_TIMER_FLAGS	6					// maximum of timer, upper limit is 16 (uint16_t support 16 bits). Move to DelayUsingTimerInt.h enum.

volatile uint16_t DelayTimerStatus = 0;		// timer status, one bit per timer

// 按位置对应每个定时器的定义，如果为 handler=NULL，则向 device task 发送 timer 消息
TIMER_DEFINE timerDefs[MAX_TIMER_FLAGS] = {NULL};	


// =================================================================
// interval 单位为 ms,自动换算为 top
// top = ceil(interval / timer_interval)
// 因此如果 interval 未对齐 timer_interval，则实际 interval 可能会略长
void SetDelayTimerFlag(uint16_t flag, bool repeat, uint16_t interval, TIMER_HANDLER callback, void* userData)
{
	INT_Disable();
	DelayTimerStatus |= (1 << flag);
	timerDefs[flag].repeat = repeat;
	timerDefs[flag].top = (uint16_t) ceil(interval / timer_interval);	//counter; [BG025] add convert uint16_t
	timerDefs[flag].counter = 0;
	timerDefs[flag].handler = callback;
	timerDefs[flag].userData = userData;
	INT_Enable();
}

void DisableDelayTimer(uint16_t flag)
{
	INT_Disable();
	DelayTimerStatus &= ~((1 << flag));

	if(DelayTimerStatus)
	{
		INT_Enable();
		return; //as long as others use the timer, can't close it
	}

	INT_Enable();

	ClearSysEnergyModeFlag(DelayTimer_RUNNING);

	TIMER_Enable(TIMER_DEALY, false);
	CMU_ClockEnable(cmuClock_TIMER_DEALY, false);
}

void ClearDelayTimerFlag(uint16_t flag)
{
	INT_Disable();
	DelayTimerStatus &= ~((1 << flag));
	INT_Enable();
}

uint16_t GetDelayTimerFlag(void)
{
	return (DelayTimerStatus);
}

void ResetDelayTimer(uint16_t flag)
{
	timerDefs[flag].counter = 0;
}

void EnableDelayTimer(uint16_t flag, bool repeat, uint16_t counter, TIMER_HANDLER callback, void* userData)
{
	SetSysEnergyModeFlag(DelayTimer_RUNNING);
	
	//
	if(DelayTimerStatus)
	{
		SetDelayTimerFlag(flag, repeat, counter, callback, userData);
		return; //as long as the timer is running , don't need to enable it again
	}
	SetDelayTimerFlag(flag, repeat, counter, callback, userData);
	
	//
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale512,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
		
	};
	
	CMU_ClockEnable(cmuClock_TIMER_DEALY, true);  
	
	TIMER_Init(TIMER_DEALY, &timerInit);
	
	//TIMER_TopSet(TIMER_DEALY, CMU_ClockFreqGet(cmuClock_TIMER_DEALY)/512/10);// 100ms unit
	TIMER_TopSet(TIMER_DEALY, CMU_ClockFreqGet(cmuClock_TIMER_DEALY)/512/50);// 20ms unit
	
	TIMER_Enable(TIMER_DEALY, true);
	
	
	NVIC_SetPriority(TIMER2_IRQn,TIMER2_IRQn_LEVEL);
	
	TIMER_IntEnable(TIMER_DEALY,TIMER_DEALY_IF);
	NVIC_EnableIRQ(TIMER_DEALY_IRQn);
}


// =================================================================
//当timer2定时到后，在timer2中断中调用该函数。
void DelayTimerCallback(void)
{
  	MESSAGE msg;
	
	if(DelayTimerStatus == 0)
	{
	  	//	0号表示Key_event_flg
		DisableDelayTimer(0);
	}
	else
	{
		for (int i = 0; i < MAX_TIMER_FLAGS; i++)
		{
			// 检查标志位是否有效
			if ((DelayTimerStatus >> i) & 0x01 == 0x01)
			{
				TIMER_DEFINE *td = &(timerDefs[i]);
				td->counter++;
				
				if (td->counter >= td->top)
				{
					// 自动重复或禁用
					if (td->repeat)
						td->counter = 0;
					else
						DisableDelayTimer(i);
					
					// 到达计数点
					if (td->handler)
						td->handler(td->userData);
					else
					{
//						osMessagePut(hMsgInterrupt, MESSAGE_TIMER +  ((long)i << 16), 0);
//						uint32_t msg = MESSAGE_TIMER +  ((long)i << 16);
//						xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
						msg.params.type = MESSAGE_TIMER;
						msg.params.param = i;
						xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
					}
				}
			}
		}
	}
}


