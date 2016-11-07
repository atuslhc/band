#include "vibrate.h"
#include "main.h"

uint16_t vibrate_ontime,vibrate_offtime,vibrate_times,vibrate_ontime_bak,vibrate_offtime_bak;
bool vibrate_disable=false,vibrate_task=false;


void VibrateInit(bool enable)
{
 GPIO_PinModeSet(VIBRATE_CON_PORT, VIBRATE_CON_PIN, gpioModePushPull,0);
 DisableVIBRATE();
}

void VibrateStart(uint32_t hz)
{
 
}

/**************************************************************************//**
 * @brief  VibrateStop
 * @param void
 *****************************************************************************/
void VibrateStop(void)
{
 

}


/**************************************************************************//**
 * @brief  Sound freq -- hz time (on+off time) repeat -- beep cnt
 * @param void
 *****************************************************************************/
void VibrateACT(uint16_t freq,uint16_t ontime,uint16_t offtime,uint16_t times)
{
 
}

void VibrateCallback(void)
{
 
	   
}



