#ifndef __DelayUsingTimerInt_H
#define __DelayUsingTimerInt_H

#include "efm32.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "main.h"
#include "common_vars.h"

#define BeepUsingDelayTimer          (1 << 0)
#define VibrateUsingDelayTimer      (1 << 1)
#define TEMPUsingDelayTimer      (1 << 2)
#define LeUartUsingDelayTimer      (1 << 3)
#define EventSimulate       (1 << 4)
#define USBCloseUsingDelayTimer      (1 << 5)





void EnableDelayTimer(uint32_t flag);
void DisableDelayTimer(uint32_t flag);


#endif
