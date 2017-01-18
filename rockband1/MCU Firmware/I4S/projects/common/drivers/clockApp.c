/***************************************************************************//**
 * @file
 * @brief Application handling calendar display and user input in 
 *        EFM32 Backup Power Domain Application Note
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
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
	 
#include "em_device.h"
#include "em_gpio.h"
#include "em_burtc.h"
#include "em_cmu.h"
	 
#include "clock.h"
#include "clockApp.h"

#include "common_vars.h"
	 
#include "main.h"

/* Calendar struct */
//struct tm calendar;
//struct tm lastTime = {0};

/* Declare variables for time keeping */
static uint32_t  burtcCount = 0;
static uint32_t  burtcOverflowCounter = 0;
static time_t    startTime; 
static uint32_t  burtcOverflowIntervalRem;
static uint32_t  burtcTimestamp;
                      
/* Declare variables */
static uint32_t resetcause = 0;

/* Declare BURTC variables */
static uint32_t burtcCountAtWakeup = 0;

/***************************************************************************//**
 * @brief Set up backup domain.
 ******************************************************************************/
void budSetup(void)
{
  /* Assign default TypeDefs */
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_BUPDInit_TypeDef bupdInit = EMU_BUPDINIT_DEFAULT; 

  /*Setup EM4 configuration structure */
  em4Init.lockConfig = true;
  em4Init.osc = emuEM4Osc_LFXO;
  em4Init.buRtcWakeup = true;
  em4Init.vreg = true;

  /* Setup Backup Power Domain configuration structure */                                          
  bupdInit.probe = emuProbe_Disable;
  bupdInit.bodCal = false;
  bupdInit.statusPinEnable = false;
  bupdInit.resistor = emuRes_Res0;
  bupdInit.voutStrong = false;
  bupdInit.voutMed = false;
  bupdInit.voutWeak = false;

  #ifdef PowerTesting  // 关闭电容充电，便于功耗测量
  bupdInit.inactivePower = emuPower_NoDiode;//emuPower_None;//emuPower_NoDiode;//emuPower_MainBU;//
  #else
  bupdInit.inactivePower = emuPower_MainBU;//
  #endif

  bupdInit.activePower = emuPower_None;
  bupdInit.enable = true;

  /* Unlock configuration */
  EMU_EM4Lock( false );
   
  /* Initialize EM4 and Backup Power Domain with init structs */
  EMU_BUPDInit( &bupdInit );
  EMU_EM4Init( &em4Init );
  
  /* Release reset for backup domain */
  RMU_ResetControl( rmuResetBU, false );

  /* Lock configuration */
  EMU_EM4Lock( true );
}


/******************************************************************************
 * @brief   Configure backup RTC
 *****************************************************************************/
void burtcSetup(void)
{
  
  /* Create burtcInit struct and fill with default values */ 
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

  /* Set burtcInit to proper values for this application */
  /* To make this example easier to read, all fields are listed, 
     even those which are equal to their default value */
  burtcInit.enable = true;
  burtcInit.mode = burtcModeEM4;
  burtcInit.debugRun = false;
  burtcInit.clkSel = burtcClkSelLFXO;
  burtcInit.clkDiv = burtcClkDiv_128;
  burtcInit.timeStamp = true;
  burtcInit.compare0Top = false;
  burtcInit.lowPowerMode = burtcLPEnable;//burtcLPDisable;
  
  /* Initialize BURTC with burtcInit struct */
  BURTC_Init( &burtcInit );
  
  /* Enable BURTC interrupt on compare match and counter overflow */
  //BURTC_IntEnable( BURTC_IF_COMP0 | BURTC_IF_OF );
  BURTC_IntEnable( BURTC_IF_OF ); //只设置了溢出的中断。
}


/***************************************************************************//**
 * @brief RTC Interrupt Handler, invoke callback if defined.
 *        The interrupt table is in assembly startup file startup_efm32gg.s
 *        Do critical tasks in interrupt handler. Other tasks are handled in main
 *        while loop.
 ******************************************************************************/
void BURTC_IRQHandler(void)
{  
  /* Interrupt source: compare match */
  /*   Increment compare value and
   *   update TFT display            */
  if ( BURTC_IntGet() & BURTC_IF_COMP0 )
  {
    BURTC_CompareSet( 0, BURTC_CompareGet(0) + COUNTS_BETWEEN_UPDATE );
    BURTC_IntClear( BURTC_IF_COMP0 );
  }

  /* Interrupt source: counter overflow */
  /*   Increase overflow counter 
   *   and backup calendar              */
  if ( BURTC_IntGet() & BURTC_IF_OF )
  {
    clockOverflow( );
    clockAppBackup();
    BURTC_IntClear( BURTC_IF_OF );
  }
}

/***************************************************************************//**
 * @brief  Backup CALENDAR to retention registers
 *
 *   RET[0].REG : number of BURTC overflows
 *   RET[1].REG : epoch offset
 *
 ******************************************************************************/
void clockAppBackup(void)
{
  /* Write overflow counter to retention memory */
  BURTC_RetRegSet( 0, clockGetOverflowCounter() );

  /* Write local epoch offset to retention memory */
  BURTC_RetRegSet( 1, clockGetStartTime() );
}

/***************************************************************************//**
 * @brief  Restore CALENDAR from retention registers
 *
 *  @param[in] burtcCountAtWakeup BURTC value at power up. Only used for printout
 *
 ******************************************************************************/
void clockAppRestore(uint32_t burtcCountAtWakeup)
{
  (void) burtcCountAtWakeup;
  
  uint32_t burtcStart;
  uint32_t nextUpdate ;

  /* Store current BURTC value for consistency in display output within this function */
  burtcCount = BURTC_CounterGet();

  /* Timestamp is BURTC value at time of main power loss */
  burtcTimestamp = BURTC_TimestampGet();

  /* Read overflow counter from retention memory */  
  burtcOverflowCounter = BURTC_RetRegGet( 0 );

  /* Check for overflow while in backup mode 
     Assume that overflow interval >> backup source capacity 
     i.e. that overflow has only occured once during main power loss */
  if ( burtcCount < burtcTimestamp )
  {
    burtcOverflowCounter++;
  }
  
  /* Restore epoch offset from retention memory */
  clockSetStartTime( BURTC_RetRegGet( 1 ) );

  /* Restore clock overflow counter */
  clockSetOverflowCounter(burtcOverflowCounter);

  /* Calculate start point for current BURTC count cycle 
     If (COUNTS_BETWEEN_UPDATE/burtcOverflowInterval) is not an integer,
     BURTC value at first update is different between each count cycle */
  burtcStart = (burtcOverflowCounter * (COUNTS_BETWEEN_UPDATE - burtcOverflowIntervalRem)) % COUNTS_BETWEEN_UPDATE;

  /*  Calculate next update compare value 
      Add 1 extra UPDATE_INTERVAL to be sure that counter doesn't 
      pass COMP value before interrupts are enabled */
  nextUpdate = burtcStart + ((burtcCount / COUNTS_BETWEEN_UPDATE) +1 ) * COUNTS_BETWEEN_UPDATE ;
  BURTC_CompareSet( 0, nextUpdate );
}

/***************************************************************************//**
 * @brief Initialize application
 *
 ******************************************************************************/
void clockAppInit(struct tm initialCalendar,bool needchanging)
{   
  /* Calendar struct for initial date setting */
  //struct tm initialCalendar;

  /* Read and clear RMU->RSTCAUSE as early as possible */

  /*   RMU_ResetCauseGet() cannot yet (as of emlib version 2.4.1) be used as it masks out RMU_RSTCAUSE_BUMODERESET. 
       This will be fixed in an upcoming release of emlib */
  resetcause = RMU->RSTCAUSE;
  RMU_ResetCauseClear();

  /* Enable clock to low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Read Backup Real Time Counter value */
  burtcCountAtWakeup = BURTC_CounterGet();

  /* Configure Backup Domain */
  budSetup();

  /* Start LFXO and wait until it is stable */
  //CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

 
  /* Set the calendar */
  clockInit(&initialCalendar);  
  
  /* Compute overflow interval (integer) and remainder */
  burtcOverflowIntervalRem = ((uint64_t)UINT32_MAX+1)%COUNTS_BETWEEN_UPDATE;

  BURTC_CompareSet( 0, COUNTS_BETWEEN_UPDATE );
  
/* If waking from backup mode, restore time from retention registers */
  uint32_t regBURTC_CTL =BURTC->CTRL;
  if ( (resetcause & RMU_RSTCAUSE_BUBODBUVIN)   ||(needchanging==true) ||  (regBURTC_CTL &0X00000008))
    //  RMU_RSTCAUSE_BUBODBUVIN BURTC低电压复位    强制复位BURTC             BURTC_CTL.RSTEN 第一次上电。 
    //都要对BURTC进行初始化。
    // if ( !(resetcause & RMU_RSTCAUSE_BUBODBUVIN))// && resetcause & RMU_RSTCAUSE_BUMODERST )
  {
    /* Setup BURTC */
    burtcSetup();
    
    /* Backup initial calendar (also to initialize retention registers) */
    clockAppBackup();  
  }
  else//不用对BURTC进行复位。 
  {
    /* Restore time from backup RTC + retention memory and print backup info*/
    clockAppRestore( burtcCountAtWakeup );
    
    /* Reset timestamp */
    BURTC_StatusClear();
  }
  
  //NVIC_SetPriority(BURTC_IRQn,BURTC_IRQn_LEVEL);

  /* Enable BURTC interrupts */
  NVIC_ClearPendingIRQ( BURTC_IRQn );
  NVIC_EnableIRQ( BURTC_IRQn );
}

/*********************************************************************************************************** 
* Function Name: GetRTCCalendar()
* Input parameters: void
* Output parameters: struct tm *
* Descriptions: -- Get RTC Time
* Time: 2013-6-29 
* Creat by: Alan
************************************************************************************************************/ 
struct tm * GetRTCCalendar(void) 
{ 
  static time_t    currentTime;
  
  currentTime = time( NULL );
  return (localtime( &currentTime ));
}

/*********************************************************************************************************** 
* Function Name: SetRTCCalendar()
* Input parameters: struct tm *calendar
* Output parameters: void
* Descriptions: -- Set RTC Calendar
* Time: 2013-6-29 
* Creat by: Alan
************************************************************************************************************/ 
void SetRTCCalendar(struct tm *settime) 
{ 
  struct tm calendar= *settime;

  BURTC_CounterReset();
  
  startTime = mktime( &calendar );
  clockSetStartTime( startTime );
  clockAppBackup( );
} 


int getWeekDay(int year, int month, int day)
{
	int c;

	if ( month <= 2 )
	{
		month += 12;
		year--;
	}

	c = year / 100;
	year %= 100;

	int w = ( year + (year / 4) + (c / 4) - 2 * c + (26 * (month + 1) / 10) + day - 1) % 7;
	if ( w <= 0 )
		w += 7;

	return w;
}

bool isLeapYear(int year)
{
	if ((year % 4 == 0) && (year % 100 != 0) || (year % 400 == 0))
		return true;
	else
		return false;
}

int getDaysOfMonth(int y, int m)
{
	int data[] = {31, isLeapYear(y) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,};
	return data[m - 1];
}

int getYearDay(int y, int m, int d)
{
	int sum = d;
	for (int i = 1; i < m; i++)
	{
		sum += getDaysOfMonth(y, i);
	}
	return   sum;
}

int subDate(int y1, int m1, int d1, int y2, int m2, int d2)
{
	int sum = getYearDay(y1, m1, d1) - getYearDay(y2, m2, d2);
	if (y1 != y2)
	{
		for (int i = y2; i < y1; i++)
		{
			sum += isLeapYear(i) ? 366 : 365;
		}
	}
	return   sum;
}

void DateTime2TM(DATE_TIME* dt, struct tm* tm)
{
	CCASSERT(dt != 0 && tm != 0);
	
	tm->tm_year	= dt->Year  - 1900;
	tm->tm_mon	= dt->Month - 1;
	tm->tm_mday	= dt->Day;
	tm->tm_hour	= dt->Hour;
	tm->tm_min	= dt->Minute;
	tm->tm_sec	= dt->Second;
}

void TM2DateTime(struct tm* tm, DATE_TIME* dt)
{
	CCASSERT(dt != 0 && tm != 0);
	
	dt->Year 		= tm->tm_year + 1900;
	dt->Month 		= tm->tm_mon + 1;
	dt->Day 		= tm->tm_mday;
	dt->DayOfWeek	= tm->tm_wday; // 从0开始，以sunday为一周开始
	dt->Hour 		= tm->tm_hour;
	dt->Minute 		= tm->tm_min;
	dt->Second 		= tm->tm_sec;
}

