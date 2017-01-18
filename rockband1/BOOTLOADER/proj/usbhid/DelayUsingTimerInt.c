
// Using this Timer to do interrupt dealy , the minium dealy uint is 100MS 
// Because the timer can't run  at EM2 , inform OS by "DelayTimer_RUNNING" flag

#include "sleep.h"
#include "DelayUsingTimerInt.h"
#include "globaldata.h"

//使用定时器2来构成延时定时器
volatile uint32_t DelayTimerStatus=0;//注意这个变量的类型

void EnableDelayTimer(uint32_t flag)
{


 SetSysEnergyModeFlag(DelayTimer_RUNNING);
 
 if(DelayTimerStatus)
 	    {
 	     DelayTimerStatus |= flag;
 	     return; //as long as the timer is running , don't need to enable it again
 	     }

 DelayTimerStatus |= flag;
 
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


void DisableDelayTimer(uint32_t flag)
{
   DelayTimerStatus &= ~(flag);	
   if(DelayTimerStatus)
		 return; //as long as others use the timer, can't close it 		

  ClearSysEnergyModeFlag(DelayTimer_RUNNING);
  
  TIMER_Enable(TIMER_DEALY, false);
  CMU_ClockEnable(cmuClock_TIMER_DEALY, false); 
 
}

void SetDelayTimerFlag(uint32_t flag)
{
  DelayTimerStatus |= flag;
}

void ClearDelayTimerFlag(uint32_t flag)
{
  DelayTimerStatus &= ~(flag);
}

uint32_t GetDelayTimerFlag(void)
{
  return (DelayTimerStatus);
}


extern void LEUARTCallback(void);

 void DelayTimerCallback(void)
  {
   static uint8_t delaycount=0;
   if(DelayTimerStatus==0)
	  {
	   delaycount=0;
	   DisableDelayTimer(0);
		}
	else
	  {
		//20ms interval
		if(DelayTimerStatus&LeUartUsingDelayTimer)
			  LEUARTCallback();
	
//		delaycount++;
//		if(delaycount>=5)
//		  {
//		   delaycount=0;
//		   if(DelayTimerStatus&USBCloseUsingDelayTimer)
//			  USBCloseCallback();
//							
//		   if(DelayTimerStatus&VibrateUsingDelayTimer)
//			  VibrateCallback();
//  
//		   if(DelayTimerStatus&TEMPUsingDelayTimer)
//			  {
//			   extern osMessageQId hMsgInterrupt;
//			   osMessagePut(hMsgInterrupt,TEMP_Message,0);	  
//			  }
//		  }
//		
		}
   
   }
 
 
extern void DelayTimerCallback(void);

void TIMER2_IRQHandler(void)
{
  uint32_t IntFlag, NextTime;
  
  IntFlag = TIMER_DEALY->IF;
  TIMER_IntClear(TIMER_DEALY, IntFlag);

  if (IntFlag & TIMER_IF_OF)
  	        DelayTimerCallback();

 }


void GPIO_EVEN_IRQHandler(void)
{
  uint32_t status;
  
  status = GPIO->IF;
  GPIO->IFC = status;
//  
//  if (status & (1 << KEY_PIN)) {
//     KeyIntHandler();
//  }
  extern void EnableLeUart(void);
  if (status & (1 << BLE_INT_PIN)) {
     EnableLeUart();
  }

//  if (status & (1 << CHARGER_STA_PIN)) 
//  {
//   if(GPIO_PinInGet(CHARGER_STA_GPIOPORT,CHARGER_STA_PIN)==0)
//   	 systemStatus.blBatteryCharging=true;
//    else
//	 systemStatus.blBatteryCharging=false;			   
//  } 
}

