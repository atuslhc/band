/**************************************************************************//**
 * @file
 * @brief Capacitive sense driver
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
#ifndef __CAPLESENSE_H_
#define __CAPLESENSE_H_

#include <stdint.h>
#include <stdbool.h>

//#include "cmsis_os.h"

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup CapSense
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//extern bool  CapSensorModel;
extern uint16_t SkinChVal;
#define SKINSCANONLY 1
#define KEYsSCAN 0

extern uint8_t current_touchsensor_feq;
extern 	bool ModelSwitched;

void CAPLESENSE_Init(void);
void Close_ALLCAPLESENSE(void);

void CAPLESENSE_setupInit(void);

void CAPLESENSE_setupScaners(bool bottom_only);
void CAPLESENSE_setupCallbacks(void (*scanCb)(void), void (*chCb)(void));
void CAPLESENSE_Sleep(void);



void capSenseSwitchModel(bool model);


void LockScreen(void);
void AutoLockScreen(void);
void UnLockScreen(bool raiseEvent);


#ifdef __cplusplus
}
#endif

/** @} (end group CapSense) */
/** @} (end group Drivers) */

#endif /* __CAPSENSE_H_ */
