/***************************************************************************//**
 * @file
 * @brief CLOCK header file
 * @author Energy Micro AS
 * @version 1.23
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
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
 ******************************************************************************/

#ifndef __CLOCK_H
#define __CLOCK_H

#include <time.h>

/* Function prototypes*/
void clockInit(struct tm * timeptr);
void clockSetCalendar(struct tm * timeptr);
void clockSetStartTime(time_t offset);
time_t clockGetStartTime(void);
uint32_t clockOverflow(void);
void clockSetOverflowCounter(uint32_t of);
uint32_t clockGetOverflowCounter(void);

#endif
