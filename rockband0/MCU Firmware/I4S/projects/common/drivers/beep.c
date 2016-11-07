/**************************************************************************//**
 * @file
 * @brief LED driver code for Energy Micro EFM32_G8xx_STK starter kit
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#include "beep.h"
#include "main.h"


uint32_t BeepTimerClock;

uint16_t sound_ontime,sound_offtime,sound_times,sound_ontime_bak,sound_offtime_bak;
bool sound_disable=false,sound_task=false;


/**************************************************************************//**
 * @brief Initialize Beep interface
 *****************************************************************************/
void BeepInit(bool enable)
{
   /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER3 module */
  CMU_ClockEnable(cmuClock_BEEP_TIMER, true);
  
  /* Set PE14 as output to control 1 the sound volume */
  GPIO_PinModeSet(BEEP_CON1_PORT, Beep_CON1_PIN, gpioModePushPull, 0);

  /* Set PB1 as output to control 2 the sound volume */
  GPIO_PinModeSet(BEEP_CON2_PORT, Beep_CON2_PIN, gpioModePushPull, 0);
  

   /* Set TIMER3 CC1 location 0 pin (PE15) as output */
  GPIO_PinModeSet(BEEP_PWM_PORT, BEEP_PWM_PIN, gpioModePushPull, 0);
 
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModePWM,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,

   };

   /* Configure CC channel 1 */
  TIMER_InitCC(BEEP_TIMER,BEEP_PWM_CHANNLE, &timerCCInit);

   /* Route CC0 to location 0 (PE15) and enable pin */  
  BEEP_TIMER->ROUTE |= (TIMER_ROUTE_CCXPEN | TIMER_ROUTE_LOCATION); 
    
  /* Select timer parameters */ 
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = enable,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
    	
  };
  
  /* Configure timer */
  TIMER_Init(BEEP_TIMER, &timerInit);

}

void BeepStart(uint32_t hz)
{
  CMU_ClockEnable(cmuClock_BEEP_TIMER, true);
  
  EnableBEEPPWM;

  BeepTimerClock = CMU_ClockFreqGet(cmuClock_BEEP_TIMER);

  int top_val=BeepTimerClock/hz;
  
  TIMER_TopSet(BEEP_TIMER,top_val); 
  
  TIMER_CompareBufSet(BEEP_TIMER,BEEP_PWM_CHANNLE,top_val/2); 
    
  TIMER_Enable(BEEP_TIMER, true);

  EnableDelayTimer(BeepUsingDelayTimer);
 
  //TEST_H();
}

/**************************************************************************//**
 * @brief  BeepStop
 * @param void
 *****************************************************************************/
void BeepStop(void)
{
  BEEP_CON1_L();
  BEEP_CON2_L();
		
  TIMER_Enable(BEEP_TIMER, false);
 
  CMU_ClockEnable(cmuClock_BEEP_TIMER, false);

  DisableDelayTimer(BeepUsingDelayTimer);
  
  DisableBEEPPWM;
  sound_task =false;
 
  //TEST_L();

}


/**************************************************************************//**
 * @brief  Sound freq -- hz time (on+off time) repeat -- beep cnt
 * @param void
 *****************************************************************************/
void Sound(uint16_t freq,uint16_t ontime,uint16_t offtime,uint16_t times,uint16_t volume)
{  
	if((sound_task==true)||(sound_disable==true))return;
	switch(volume)
	{
		case BEEP_MUTE:
			BEEP_CON1_L();
			BEEP_CON2_L();
			break;

		case BEEP_SMALL_VOLUME:
			BEEP_CON1_L();
			BEEP_CON2_H();
			break;

		case BEEP_MEDIUM_VOLUME:
			BEEP_CON1_H();
			BEEP_CON2_L();
			break;

		case BEEP_LARGE_VOLUME:
			BEEP_CON1_H();
			BEEP_CON2_H();
			break;

	}
  
//	BEEP_CON1_H();
//	BEEP_CON2_H();
			
  sound_task =true;
  if(ontime)
     sound_ontime=ontime-1;
    else
	 sound_ontime=ontime;
	
  sound_offtime=offtime; //200ms unit
  sound_ontime_bak=ontime;
  sound_offtime_bak=offtime; 
  sound_times=times;
  BeepStart(freq);
  
}

void BeepCallback(void)
{
    if (sound_times)sound_times--;
    if (sound_ontime)
    {
        sound_ontime--;
        if (sound_ontime == 0)
        {
            DisableBEEPPWM;
        }
    }
    else if (sound_times == 0)
    {
        BeepStop();
    }

    else if (sound_offtime)
    {
        sound_offtime--;
        if (sound_offtime == 0)
        {
            if (sound_times == 0)
            {
                BeepStop();
            }
            else
            {
                EnableBEEPPWM;
                sound_ontime = sound_ontime_bak;
                sound_offtime = sound_offtime_bak;
            }
        }
    }
}




