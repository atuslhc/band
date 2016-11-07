/***************************************************************************//**
 * @file
 * @brief CALENDAR header file
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

#ifndef _CLOCKAPP_H_
#define _CLOCKAPP_H_

#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_burtc.h"
#include "clock.h"

/* Scheduler includes. */
//#include "cmsis_os.h"

/* Clock defines for display update */
#define UPDATE_INTERVAL       1  /* in seconds */
#define LFXO_FREQUENCY        32768
#define BURTC_PRESCALING      128
#define COUNTS_PER_SEC        (LFXO_FREQUENCY/BURTC_PRESCALING)
#define COUNTS_BETWEEN_UPDATE ((UPDATE_INTERVAL*LFXO_FREQUENCY)/BURTC_PRESCALING)

/* Calendar struct */
//extern struct tm calendar;
//extern struct tm lastTime;

/* Function prototypes */
void burtcSetup(void);

void clockAppInit(struct tm initialCalendar,bool needchanging);
//void clockAppDisplay(void);  
void clockAppBackup(void);
void clockAppRestore(uint32_t burtcCountAtWakeup);
void clockAppUpdate(void);
void clockAppOverflow(void);

struct tm * GetRTCCalendar(void);
void SetRTCCalendar(struct tm *settime);

//extern int32_t SetTimeSecond(bool ctrl);
//extern int32_t SetTimeMinute(bool ctrl);
//extern int32_t SetTimeHour(bool ctrl);
//extern int32_t SetTimeDay(bool ctrl);
//extern int32_t SetTimeMonth(bool ctrl);
//extern int32_t SetTimeYear(bool ctrl);

// 是否为闰年
bool isLeapYear(int year);

// 获得某天为星期几
int getWeekDay(int year, int month, int day);

// 获得某天在一年中的天数
int getYearDay(int year, int month, int day);

// 获得某个月的天数
int getDaysOfMonth(int year, int month);

//计算两个日期之间的天数差
int subDate(int y1, int m1, int d1, int y2, int m2, int d2);

#endif


