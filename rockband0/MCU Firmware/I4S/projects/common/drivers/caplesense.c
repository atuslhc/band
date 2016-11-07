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

/* EM header files */
#include "em_device.h"

/* Drivers */
#include "caplesense.h"
#include "em_emu.h"
#include "em_acmp.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_lesense.h"
#include "main.h"
#include "common_vars.h"

/* Capacitive sense configuration */
#include "caplesenseconfig.h"

#include "CapTouchDetect.h"

#include "device_task.h"

/**************************************************************************//**
 * @brief This vector stores the latest read values from LESENSE
 * @param LESENSE_CHANNELS Vector of channels.
 *****************************************************************************/
uint16_t touchValues;

//bool  CapSensorModel = SKINSCANONLY;
uint16_t SkinTouchVal=0;//, max_SkinTouchVal=0;

uint8_t current_touchsensor_feq;
bool ModelSwitched=false;
bool isTouchSensorOn=false;
/**************************************************************************//**
 * @brief  A bit vector which represents the channels to iterate through
 * @param LESENSE_CHANNELS Vector of channels.
 *****************************************************************************/

/**************************************************************************//**
 * Prototypes
 *****************************************************************************/
void CAPLESENSE_setupCMU(void);
void CAPLESENSE_setupGPIO(void);
void CAPLESENSE_setupACMP(void);
void capSenseScanComplete(void);
void capSenseChTrigger(void);


void capSenseChTrigger(void)
{
}

/**************************************************************************//**
 * Local variables
 *****************************************************************************/
/** Callback function for LESENSE interrupts. */
static void (*lesenseScanCb)(void);
/** Callback function for LESENSE interrupts. */
static void (*lesenseChCb)(void);

/**************************************************************************//**
 * @brief  Setup the CMU
 *****************************************************************************/
void CAPLESENSE_setupCMU(void)
{
	/* Ensure core frequency has been updated */
	SystemCoreClockUpdate();

	/* Select clock source for LFA clock. */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	/* Select clock source for LFB clock. */
	//CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

	/* Enable clock for ACMP0. */
	//CMU_ClockEnable(cmuClock_ACMP0, 1);

	/* Enable clock for ACMP1. */
	CMU_ClockEnable(cmuClock_ACMP1, 1);

	/* Enable CORELE clock. */
	CMU_ClockEnable(cmuClock_CORELE, 1);

	/* Enable clock for LESENSE. */
	CMU_ClockEnable(cmuClock_LESENSE, 1);

	/* Enable clock divider for LESENSE. */
	CMU_ClockDivSet(cmuClock_LESENSE, cmuClkDiv_1);
}

/**************************************************************************//**
 * @brief  Setup the GPIO
 *****************************************************************************/
void CAPLESENSE_setupGPIO(void)
{
	/* Configure the drive strength of the ports for the light sensor. */
	GPIO_DriveModeSet(CAPLESENSE_KEY_PORT, gpioDriveModeStandard);

	/* Initialize the 5 GPIO pins of the touch slider for using them as LESENSE
	 * scan channels for capacitive sensing. */
	GPIO_PinModeSet(CAPLESENSE_KEY_PORT, TOUCH_KEY1_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(CAPLESENSE_KEY_PORT, TOUCH_KEY2_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(CAPLESENSE_KEY_PORT, TOUCH_KEY3_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(CAPLESENSE_KEY_PORT, TOUCH_KEY4_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(CAPLESENSE_KEY_PORT, TOUCH_KEY5_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(CAPLESENSE_KEY_PORT, TOUCH_REF_PIN, gpioModeDisabled, 0);
}


/**************************************************************************//**
 * @brief  Setup the ACMP
 *****************************************************************************/
void CAPLESENSE_setupACMP(void)
{
	/* ACMP capsense configuration constant table. */
	static const ACMP_CapsenseInit_TypeDef initACMP =
	{
		.fullBias                 = true,//false,
		.halfBias                 = true,//false,
		.biasProg                 = 0x05,//                 0x7,
		.warmTime                 = acmpWarmTime512,
		.hysteresisLevel          = acmpHysteresisLevel5,//acmpHysteresisLevel7,
		.resistor                 = acmpResistor0,
		.lowPowerReferenceEnabled = false,
		.vddLevel                 = 0x3D,//  0x30,
		.enable                   = false
	};

	/* Configure ACMP locations, ACMP output to pin disabled. */
	// ACMP_GPIOSetup(ACMP0, 0, false, false); // only the second 8 channels used
	ACMP_GPIOSetup(ACMP1, 0, false, false);

	/* Initialize ACMPs in capacitive sense mode. */
	// ACMP_CapsenseInit(ACMP0, &initACMP); // only the second 8 channels used
	ACMP_CapsenseInit(ACMP1, &initACMP);

	/* Don't enable ACMP, LESENSE controls it! */
}

/**************************************************************************//**
 * @brief  CAPLESENSE_setupInit
 *****************************************************************************/
void CAPLESENSE_setupInit(void)
{
	/* LESENSE central configuration constant table. */
	static const LESENSE_Init_TypeDef  initLESENSE =
	{
		.coreCtrl         =
		{
			.scanStart    = lesenseScanStartPeriodic,
			.prsSel       = lesensePRSCh0,
			.scanConfSel  = lesenseScanConfDirMap,
			.invACMP0     = false,
			.invACMP1     = false,
			.dualSample   = false,
			.storeScanRes = false,
			.bufOverWr    = true,
			.bufTrigLevel = lesenseBufTrigHalf,
			.wakeupOnDMA  = lesenseDMAWakeUpDisable,
			.biasMode     = lesenseBiasModeDutyCycle,
			.debugRun     = false
		},

		.timeCtrl         =
		{
			.startDelay     =  0U
		},

		.perCtrl          =
		{
			.dacCh0Data     = lesenseDACIfData,
			.dacCh0ConvMode = lesenseDACConvModeDisable,
			.dacCh0OutMode  = lesenseDACOutModeDisable,
			.dacCh1Data     = lesenseDACIfData,
			.dacCh1ConvMode = lesenseDACConvModeDisable,
			.dacCh1OutMode  = lesenseDACOutModeDisable,
			.dacPresc       =                        0U,
			.dacRef         = lesenseDACRefBandGap,
			.acmp0Mode      = lesenseACMPModeMuxThres,
			.acmp1Mode      = lesenseACMPModeMuxThres,
			.warmupMode     = lesenseWarmupModeNormal
		},

		.decCtrl          =
		{
			.decInput  = lesenseDecInputSensorSt,
			.chkState  = false,
			.intMap    = true,
			.hystPRS0  = false,
			.hystPRS1  = false,
			.hystPRS2  = false,
			.hystIRQ   = false,
			.prsCount  = true,
			.prsChSel0 = lesensePRSCh0,
			.prsChSel1 = lesensePRSCh1,
			.prsChSel2 = lesensePRSCh2,
			.prsChSel3 = lesensePRSCh3
		}
	};

	/* Only initialize main LESENSE parameters once. */
	/* Initialize LESENSE interface with RESET. */
	LESENSE_Init(&initLESENSE, true);

	/* Setup capSense callbacks. */
	CAPLESENSE_setupCallbacks(&capSenseScanComplete, &capSenseChTrigger);
}

/**************************************************************************//**
 * @brief  Setup the LESENSE for capavitive sensing
 * @param sleep
 *	If true, go into sleep mode.
 *****************************************************************************/
void CAPLESENSE_setupScaners(bool bottom_only)
{
	/* LESENSE channel configuration constant table in sense mode. */
	static const LESENSE_ChAll_TypeDef initChsSense = LESENSE_CAPSENSE_SCAN_CONF_SENSE;

	//static const LESENSE_ChAll_TypeDef initBottomSensor = LESENSE_CAPSENSE_SCAN_CONF_SLEEP; //[BG025] remark.

	/* Stop LESENSE before configuration. */
	LESENSE_ScanStop();

	/* Wait until the currently active scan is finished. */
	while (LESENSE_STATUS_SCANACTIVE & LESENSE_StatusGet()) ;

	/* Clean scan complete interrupt flag. */
	LESENSE_IntClear(LESENSE_IEN_SCANCOMPLETE);

	/* Clear result buffer. */
	LESENSE_ResultBufferClear();

	/* Set clock divisor for LF clock. */
	LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);//lesenseClkDiv_

	if(bottom_only == true)
	{
		/* Set scan frequency (in Hz). */
		current_touchsensor_feq=LESENSE_ScanFreqSet(0U, LESENSE_SLEEP_FREQUENCY);//64U);

		/* Configure scan channels. */
		//LESENSE_ChannelAllConfig(&initBottomSensor);
	}
	else
	{
		/* Set scan frequency (in Hz). */
		current_touchsensor_feq=LESENSE_ScanFreqSet(0U, LESENSE_SCAN_FREQUENCY);//64U);

		/* Configure scan channels. */
		//LESENSE_ChannelAllConfig(&initChsSense);
	}

	/* Configure scan channels. */
	LESENSE_ChannelAllConfig(&initChsSense);

	/* Enable scan complete interrupt. */
	LESENSE_IntEnable(LESENSE_IEN_SCANCOMPLETE);

    NVIC_SetPriority(LESENSE_IRQn,LESENSE_INT_LEVEL);
		
	/* Enable LESENSE interrupt in NVIC. */
	NVIC_EnableIRQ(LESENSE_IRQn);


	/* Start scanning LESENSE channels. */
	LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  capSenseSwitchModel
 *****************************************************************************/
void capSenseSwitchModel(bool model)
{
	if(isTouchSensorOn==true)
	{
	 //CapSensorModel = model;
	 CAPLESENSE_setupScaners(model); // true  = triggle CH12  only , false = scan CH8-CH12   
     ModelSwitched=true;
	}
}

/**************************************************************************//**
 * @brief  LESENSE callback setup
 * @param  scanCb Scan callback
 * @param  chCb Channel callback
 *****************************************************************************/
void CAPLESENSE_setupCallbacks(void (*scanCb)(void), void (*chCb)(void))
{
	lesenseScanCb = scanCb;
	lesenseChCb   = chCb;
}

/**************************************************************************//**
 * @brief  LESENSE callback scan complete
 *****************************************************************************/


//extern osMessageQId hMsgInterrupt;

void capSenseScanComplete(void)
{
	SkinTouchVal= (uint16_t)LESENSE_ScanResultDataGet();//skin
	
	touchValues=(uint16_t)LESENSE_ScanResultDataGet();

#if 0
    static char error_count=0; //[BG025] move variable into the debug scope.
    if(TouchEventACK==true)
  	{
  	 error_count++;
	 if(error_count>40)
	 	RESET_MCU(); // reset the device
    }
     else {
   	      TouchEventACK=true;
		   error_count=0;
		 }
#endif

	//if(isMemsSleeping==false)

//    osMessagePut(hMsgInterrupt, TouchSensorMsg, 0);

	uint32_t msg = TouchSensorMsg;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);

}



/**************************************************************************//**
 * @brief  LESENSE interrupt handler
 *****************************************************************************/
void LESENSE_IRQHandler(void)
{
	//if(isTouchSensorOn==false)
	//	{
		// LESENSE_IntClear(CAPLESENSE_CHANNEL_INT);
		 //return;
		 //}

	/* LESENSE scan complete interrupt. */
	if (LESENSE_IF_SCANCOMPLETE & LESENSE_IntGetEnabled())
	{
		LESENSE_IntClear(LESENSE_IF_SCANCOMPLETE);

		/* Call callback function. */
		if (lesenseScanCb != 0x00000000)
		{
			lesenseScanCb();
		}
	}

	/* LESENSE channel interrupt. */
	if (CAPLESENSE_CHANNEL_INT & LESENSE_IntGetEnabled())
	{
		/* Clear flags. */
		LESENSE_IntClear(CAPLESENSE_CHANNEL_INT);

		/* Call callback function. */
		if (lesenseChCb != 0x00000000)
		{
			lesenseChCb();
		}
	}
}



/**************************************************************************//**
 * @brief Send the capacative sense system to sleep mode.
 *****************************************************************************/
void CAPLESENSE_Sleep(void)
{
	/* Go to EM2 and wait for the measurement to complete. */
// EMU_EnterEM2(true);
}

/**************************************************************************//**
 * @brief Initializes the capacative sense system without LESENSE.
 *****************************************************************************/
void CAPLESENSE_Init(void)
{
 if(isTouchSensorOn==false)
 	{
   	/* Disable interrupts */
   	INT_Disable();
   
   	/* Setup CMU. */
   	CAPLESENSE_setupCMU();
   	/* Setup GPIO. */
   	CAPLESENSE_setupGPIO();
   	/* Setup ACMP. */
   	CAPLESENSE_setupACMP();
   	/* Setup LESENSE. */
   	CAPLESENSE_setupInit();
   
    isTouchSensorOn=true;
   	capSenseSwitchModel(SKINSCANONLY); //

    /* Initialization done, enable interrupts globally. */
   	INT_Enable();
   
   	}

}

/**************************************************************************//**
 * @brief Close ALL CAPLESENSE
 *****************************************************************************/
void Close_ALLCAPLESENSE(void)
{
 if(isTouchSensorOn==true)
   {
	static const LESENSE_ChAll_TypeDef initChsClose = LESENSE_CAPSENSE_CLOSE;

	/* Disable interrupts */
   	INT_Disable();
	
	isTouchSensorOn=false;
	
	/* Disable scan complete interrupt. */
	LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);

	/* Disable LESENSE interrupt in NVIC. */
	NVIC_DisableIRQ(LESENSE_IRQn);

	LESENSE_Reset();

	LESENSE_ChannelAllConfig(&initChsClose);

	/* Disable clock for ACMP1. */
	CMU_ClockEnable(cmuClock_ACMP1, false);

	/* Disable clock for LESENSE. */
	CMU_ClockEnable(cmuClock_LESENSE, false);

    /* Initialization done, enable interrupts globally. */
   	INT_Enable();
	
	}

}
