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
#ifndef __BEEP_H
#define __BEEP_H

#include "efm32.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"

#include "bsp.h"

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Beep
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


extern uint16_t sound_ontime,sound_offtime,sound_times,sound_ontime_bak,sound_offtime_bak;
extern bool sound_disable,sound_task;


void BeepInit(bool enable);
void BeepStart(uint32_t hz);
void BeepStop(void);


void Sound(uint16_t freq,uint16_t ontime,uint16_t offtime,uint16_t times,uint16_t volume);


#ifdef __cplusplus
}
#endif

/** @} (end group Leds) */
/** @} (end group Drivers) */

#endif
