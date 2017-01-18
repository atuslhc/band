#ifndef __BEEP_H
#define __BEEP_H

#include "efm32.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "bsp.h"

void VibrateInit(bool enable);
void VibrateStart(uint32_t hz);
void VibrateStop(void);
void VibrateACT(uint16_t freq,uint16_t ontime,uint16_t offtime,uint16_t times);



#endif



