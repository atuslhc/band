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

/*
 * 功能：板级初始化
 * 函数：
 *     BspInit()--时钟初始化、蜂鸣器初始化、按键初始化
 * 资源：48MHz 外部晶体 32.768KHz外部晶体
 */

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Boadr Init
 * @{
 ******************************************************************************/

#include "bsp.h"
#include "main.h"
#include "common_vars.h"
#include "device_task.h"

/**************************************************************************//**
 * @brief Initialize Beep interface
 *****************************************************************************/

void CHARGER_STA_INIT(void)
{
#if 0
	/* Enable GPIO */
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, gpioModeInputPull, 1);

	GPIO_IntConfig(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, true, true, true);

	GPIO_IntClear(1 << CHARGER_STA_PIN);

	NVIC_SetPriority(GPIO_EVEN_IRQn, GPIO_EVEN_INT_LEVEL);

	/* Enabling Interrupt from GPIO_EVEN */
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	if(GPIO_PinInGet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN) == 0)
		systemStatus.blBatteryCharging = true;
	else
		systemStatus.blBatteryCharging = false;

	osMessagePut(hMsgInterrupt, MESSAGE_BATTERY_CHARGING, 0);
#else

#if (CHARGER_SUPPORT==1)
	/* Enable GPIO */
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, gpioModeInputPull, 1);
#endif

#endif
}

bool BURTC_ERR;

void BspInit(void)
{
	struct tm initialCalendar;
	/* Setting up a structure to initialize the calendar
	  for January 1 2012 12:00:00
	  The struct tm is declared in time.h
	  More information for time.h library in http://en.wikipedia.org/wiki/Time.h */
	initialCalendar.tm_sec	 = 0;	 /* 0 seconds (0-60, 60 = leap second)*/
	initialCalendar.tm_min	 = 0;	/* 0 minutes (0-59) */
	initialCalendar.tm_hour	 = 12;	 /* 12 hours (0-23) */
	initialCalendar.tm_mday	 = 1;	/* 1st day of the month (1 - 31) */
	initialCalendar.tm_mon	 = 0;	/* January (0 - 11, 0 = January) */
	initialCalendar.tm_year	 = 112;  /* Year 2012 (year since 1900) */
	initialCalendar.tm_wday	 = 0;	 /* Sunday (0 - 6, 0 = Sunday) */
	initialCalendar.tm_yday	 = 0;  /* 1st day of the year (0-365) */
	initialCalendar.tm_isdst  = -1;	  /* Daylight saving time; enabled (>0), disabled (=0) or unknown (<0) */



	/* initialize 48MHz */
	//CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	//CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);


	/* initialize 32.768KHz oscillator */
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	//============2014.08.16===================
#if 0
	clockAppInit(initialCalendar, false);
#else
	BURTC_ERR = false;

	for(int i = 0; i < 3; i++)
	{
		clockAppInit(initialCalendar, false);

		if(BURTC_ERR == false)
			break;

		BURTC_ERR = false;
	}

#endif
	//======================================================

	CMU_ClockEnable(cmuClock_GPIO, true);

	// init test pin
#ifdef DEBUG_MODE
	GPIO_PinModeSet(TEST_GPIOPORT, TEST_PIN, gpioModePushPull, 1);
	TEST_H();

	GPIO_PinModeSet(TEST_PORTB, TEST_PIN2, gpioModePushPull, 1);
	TEST_PORT2_CLEAR();

	GPIO_PinModeSet(TEST_PORTA, TEST_PIN4, gpioModePushPull, 1);
	TEST_PORT4_CLEAR();

	GPIO_PinModeSet(TEST_PORTA, TEST_PIN15, gpioModePushPull, 1);
	TEST_PORT15_CLEAR();
#endif


#if (AFE44XX_SUPPORT==1) //(BOARD_TYPE==0 || BOARD_TYPE==1) 
    //turn off the afe4400 external power.
	GPIO_PinModeSet(AFE_POWER_CON_PORT, AFE_POWER_CON_PIN, gpioModePushPull, 0);
	BOTTOM_POWER_OFF();
#endif
    
	// led pin
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	GPIO_PinModeSet(LED_GPIOPORT, LED_PIN, gpioModePushPull, 1);
#if defined(DEBUG)
    LED_ON();
    SysCtlDelay(5000);
#endif
	LED_OFF();

#elif (BOARD_TYPE==2)
    GPIO_PinModeSet(LED_GPIOPORT, LEDC_PIN, gpioModePushPull, 1);  //Atus: should be input config. In power test can check if disable.
#if (1) //(DEBUG)
    unsigned int ledc=0;
    ledc=GetLEDC();
#endif
    /* set the LED GPIO */
	GPIO_PinModeSet(LED_GPIOPORT, LEDR_PIN, gpioModePushPull, 1);
#if (1) //defined(DEBUG)
    LEDR_ON();
    SysCtlDelay(900000);
#endif
//    while (1);
	LEDR_OFF();
    
	GPIO_PinModeSet(LED_GPIOPORT, LEDG_PIN, gpioModePushPull, 1);
#if (1) //defined(DEBUG)
    LEDG_ON();
    SysCtlDelay(900000);
#endif
//    while (1);
//	LEDG_OFF(); //debug keep it on with green
    
	GPIO_PinModeSet(LED_GPIOPORT, LEDB_PIN, gpioModePushPull, 1);
#if (1) //defined(DEBUG)
    LEDB_ON();
    SysCtlDelay(900000);
#endif
//    while (1);
	LEDB_OFF();
    
	/* Configure GPIO port PA2(SW1),PA5(SW2) as Key input */
	GPIO_PinModeSet(KEY_PORT, KEY2_PIN, gpioModeInput, 1);
	GPIO_IntConfig(KEY_PORT, KEY2_PIN, false, true, true);
	GPIO_IntClear(1 << KEY2_PIN); 	
	NVIC_SetPriority(GPIO_ODD_IRQn,GPIO_ODD_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	GPIO_PinModeSet(KEY_PORT, KEY1_PIN, gpioModeInput, 1);
	GPIO_IntConfig(KEY_PORT, KEY1_PIN, false, true, true);	
	GPIO_IntClear(1 << KEY1_PIN); 	
	NVIC_SetPriority(GPIO_EVEN_IRQn,GPIO_EVEN_INT_LEVEL);  
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    
    /* The debug test use with DAUIO_x */
    GPIO_PinModeSet(gpioPortD, DAUIO_1_PIN, gpioModePushPull, 0);  //Atus: set input pin, read in BLE_RESET() filter.
    //GPIO_PinModeSet(gpioPortC, DAUIO_2_PIN, gpioModePushPull, 1);
    //GPIO_PinModeSet(gpioPortC, DAUIO_3_PIN, gpioModePushPull, 1);
    //GPIO_PinModeSet(gpioPortC, DAUIO_4_PIN, gpioModePushPull, 1);
    //GPIO_PinModeSet(gpioPortA, DAUIO_5_PIN, gpioModePushPull, 1);
    //GPIO_PinModeSet(gpioPortA, DAUIO_6_PIN, gpioModePushPull, 1);
    //GPIO_PinModeSet(gpioPortB, DAUIO_7_PIN, gpioModePushPull, 1);
    //GPIO_PinModeSet(gpioPortB, DAUIO_8_PIN, gpioModePushPull, 1);
    
    //value = GetDAUIO_1();
    
#endif

	//
	LoadSystemSettings();
}


void initLETIMER(void)
{
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	//GPIO_PinModeSet(LCD_EXTCOM_PORT,LCD_EXTCOM_PIN, gpioModePushPull, 0);

	/* Set initial compare values for COMP0 */
	LETIMER_CompareSet(LETIMER0, 0, 32768);

	//i2 only
	//LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_LOCATION_LOC1;

	/* The current version of emlib (3.0.0) does not properly set
	* REP0 to a nonzero value while enabling LETIMER in Free mode.
	* Therefore we set REP0 manually here */
	LETIMER0->REP0 = 0x01;

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
		.enable         = true,                   /* Don't start counting when init completed - only with RTC compare match */
		.debugRun       = false,                  /* Counter shall not keep running during debug halt. */
		.rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
		.rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
		.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
		.bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
		.out0Pol        = 0,                      /* Idle value for output 0. */
		.out1Pol        = 0,                      /* Idle value for output 1. */
		.ufoa0          = letimerUFOAToggle,      /* Pulse output on output 0 */
		.ufoa1          = letimerUFOANone,        /* No output on output 1*/
		.repMode        = letimerRepeatFree       /* Repeat indefinitely */
	};


	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);

	NVIC_SetPriority(LETIMER0_IRQn, LETIMER0_IRQn_LEVEL);

	/* Enable LETIMER0 interrupt vector in NVIC*/
	NVIC_EnableIRQ(LETIMER0_IRQn);

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);
}


/** @} (end group Leds) */
/** @} (end group Drivers) */
