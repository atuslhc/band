#include <string.h>
#include <stdbool.h>
#include "efm32.h"
#include "boot.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "em_adc.h"
#include "em_wdog.h"
#include "em_rmu.h"
#include "em_rmu.h"
#include "GlobalData.h"
#include "bootld.h"
#include "main.h"

#ifndef BOARD_TYPE
#error "Please specify BOARD_TYPE value in Options/Preprocessor !"
#endif

//After reset, the HFRCO frequency is 14 MHz
#define CORE_FREQ_HIGH cmuHFRCOBand_14MHz
#define CORE_FREQ_LOW  cmuHFRCOBand_1MHz
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
#define BAT_GO_LEVEL 3.5 //3.6
#elif (BOARD_TYPE==2)
#define BAT_GO_LEVEL 2.6
#endif


//这个是绝对地址访问，需要在linker文件中在加上
//place at address mem: 0x000097FC    { readonly section ConstSection1 };就可以在具体位置上写上版本号。

#pragma location = "ConstSection1"
__root const char abc1[4] = {0, 0, BOOT_VER_M , BOOT_VER_S};


void initGPIO(void)
{

	CMU_ClockEnable(cmuClock_GPIO, true); //turn on GPIO clock.
//	GPIO_PinModeSet(KEY_GPIOPORT, KEY_PIN, gpioModeInputPull, 1);
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	GPIO_PinModeSet(LED_GPIOPORT, LED_PIN, gpioModePushPull, 1); //set LED gpio mode.
#elif (BOARD_TYPE==2)
	GPIO_PinModeSet(LED_GPIOPORT, LEDR_PIN, gpioModePushPull, 1); //set LED gpio mode.
	GPIO_PinModeSet(LED_GPIOPORT, LEDB_PIN, gpioModePushPull, 1); //set LED gpio mode.
	GPIO_PinModeSet(LED_GPIOPORT, LEDG_PIN, gpioModePushPull, 1); //set LED gpio mode.
	GPIO_PinModeSet(LED_GPIOPORT, LED_PIN, gpioModePushPull, 0); //set LED gpio mode.
#endif
    SysCtlDelay(5000);
    LED_OFF();
//	GPIO_IntConfig(KEY_GPIOPORT, KEY_PIN, false, true, true);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	//NVIC_EnableIRQ(GPIO_ODD_IRQn);

#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	GPIO_PinModeSet(AFE_POWER_CON_PORT, AFE_POWER_CON_PIN, gpioModePushPull, 0); //把AFE的电也关掉了
	BOTTOM_POWER_OFF();

	//BOTTOM_POWER_ON();

	GPIO_PinModeSet(BACKLIGHT_PORT, BACKLIGHT_PIN, gpioModePushPull, 0);
	BACKLIGHT_ON();
	BACKLIGHT_OFF();

	GPIO_PinModeSet(MOTOR_PORT, MOTOR_PIN, gpioModePushPull, 0);
	MOTOR_ON();
	MOTOR_OFF();
#endif

#if 0   //Atus: can not find out what board use this.
	EXTVCC_ON();
	EXTVCC_OFF();
#endif
    
//	GPIO_PinModeSet(PWM1_PORT, PWM1_PIN, gpioModePushPull, 0);
//	GPIO_PinModeSet(PWM2_PORT, PWM2_PIN, gpioModePushPull, 0);
//	PWM1_OFF();
//	PWM2_OFF(); //这两组pwm仅仅在Firmware中用做调试信息用的。


}

#define BAT_VSIZE    4
long bat_val[BAT_VSIZE] = {0, 0, 0, 0}, bat_avg = 0; //电池数据

float BAT_VCC = 0;

unsigned char bat_index = 0, bat_checked = 0;

int timerCount = 0;//定时器计数器

/* 976 Hz -> 1Mhz (clock frequency) / 1024 (prescaler)
  Setting TOP to 488 results in an overflow each 0.1 seconds */

#define TOP 100



//Low Energy Timer0 interrupt handler
void LETIMER0_IRQHandler(void)
{
	LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0); //clear COMP0 interrupt flag.
	timerCount++;


	while (ADC0->STATUS & ADC_STATUS_SINGLEACT);

	bat_val[bat_index++] = (long)ADC_DataSingleGet(ADC0); //single get ADC value fill to array

	if(bat_index >= 4) //interrupt count 4, we could make it as array size as done the battery check.
	{
		bat_index = 0;
		bat_checked = 1; //set battery checked done.
	}

	ADC_Start(ADC0, adcStartSingle);//开始扫描或单次转发，@p1,ADC类型，@p2ADC开始的类型。

}

const	ADC_InitSingle_TypeDef BAT_ADC_INIT =
{
	adcPRSSELCh0, /*   PRS ch0 (if enabled). */ 		   \
	adcAcqTime8,  /* 1 ADC_CLK cycle acquisition time. */	  \
	adcRef2V5,	/* 2.5V internal reference. */ 		  \
	adcRes12Bit, /* 12 bit resolution. */			   \
	adcSingleInpCh7, /* CH0 input selected. */		   \
	false, /* Single ended input. */			   \
	false,		/* PRS disabled. */ 		\
	false,	/* Right adjust. */ 								 \
	false  /* Deactivate conversion after one scan sequence. */ \
};

void ADC_INIT(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;//use default setting.

	/* Init common settings for both single conversion and scan mode */
	init.timebase = ADC_TimebaseCalc(0);//计算时基，这里设置为0表示用当前设置的高频外设时钟

	// init.lpfMode =adcLPFilterRC;//adcLPFilterDeCap;

	//init.warmUpMode=adcWarmupKeepScanRefWarm;

	/* Set ADC clock to 7 MHz, use default HFPERCLK */
	init.prescale = ADC_PrescaleCalc(7000000, 0);

	/* Set oversampling rate */
	//init.ovsRateSel = adcOvsRateSel32;//adcOvsRateSel32;

	ADC_Init(ADC0, &init);

	ADC_InitSingle(ADC0, &BAT_ADC_INIT);

}

// charger state initialize
void CHARGER_STA_INIT(void)
{
	/* Enable GPIO */
	CMU_ClockEnable(cmuClock_GPIO, true);

#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	/* Configure GPIO port A2 as Key input,PF4 as Charger state pin*/
	GPIO_PinModeSet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, gpioModeInputPull, 1);

	GPIO_IntConfig(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, true, true, true);

	GPIO_IntClear(1 << CHARGER_STA_PIN);

	/* Enabling Interrupt from GPIO_EVEN */
	NVIC_EnableIRQ(GPIO_EVEN_IRQn); //turn on GPIO interrupt
#endif

}

unsigned char Charging = 0;

void GPIO_EVEN_IRQHandler(void)
{
	uint32_t status;
    /* charger interrupt service */
	status = GPIO->IF;
	GPIO->IFC = status;

#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	if (status & (1 << CHARGER_STA_PIN))
	{
		if(GPIO_PinInGet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN) == 0) //if state low, indicate in charging.
			Charging = 1;
		else
			Charging = 0;
	}
#endif
}


//unsigned char charging_Counter = 0; //charge counter replace with timerCount

/**************************************************************************//**
 * The main entry point.
 *****************************************************************************/

int main(void)
{
	/* Initialize watchdog structure */
	WDOG_Init_TypeDef init =
	{
		.enable     = true,               /* Start watchdog when init done */
		.debugRun   = false,              /* WDOG not counting during debug halt */
		.em2Run     = true,               /* WDOG counting when in EM2 */
		.em3Run     = true,               /* WDOG counting when in EM3 */
		.em4Block   = false,              /* EM4 can be entered */
		.swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */
		.lock       = false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
		.clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */
		.perSel     = wdogPeriod_4k,      /* Set the watchdog period to 4097 clock periods (ie about 4 seconds)*/ //persel=10
	};

	//T = (2^(3+perSel)+1)/f; f is the watchdog clock, f=1Khz, perSel=10; T=8s

	CHIP_Init();

	initGPIO();

	//---config LETimer0, use in LED blink freq at difference case and watchdog feed freq.
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true); //wait low osc enable.
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); // external Low Frequency Crystal Osc(LFXO=32.768Hz) as clock source.
	CMU_ClockEnable(cmuClock_CORELE, true); //Low Energy clock module for RTC and LE equipment, and watchdog source.
	CMU_ClockEnable(cmuClock_LETIMER0, true); //turn on Timer0

	LETIMER_Init_TypeDef leTimerinit = LETIMER_INIT_DEFAULT; //initialize

	leTimerinit.debugRun = true;
	leTimerinit.comp0Top = true;
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0); //comparer0 interrupt
	NVIC_EnableIRQ(LETIMER0_IRQn); //NestedVectorInterruptController enable
	LETIMER_CompareSet(LETIMER0, 0, 32768 / 10); //10HZ
	LETIMER_Init(LETIMER0, &leTimerinit);


	WDOG_Init(&init); //after init, watchdog run.

	//====================================

	ADC_INIT();

	SysCtlDelay(8000);// delay for ADC initialize to sync operation.

	//====================================
	CHARGER_STA_INIT();

#if (BOARD_TYPE==0 || BOARD_TYPE==1)
    //scan first time to avoid the GPIO_EVEN_IRQHandler interrupt not active without change state.
	if(GPIO_PinInGet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN) == 0)  //if state low, indicate in charging.
		Charging = 1;
	else
		Charging = 0;
#endif

	//  ===================================

	while(1)
	{
		EMU_EnterEM2(true);

		WDOG_Feed();


		if(bat_checked) //set in LETIMER0_IRQHandler set bat_val[4] done set flag.
		{
			bat_checked = 0;
			bat_avg = 0;

			for(int i = 0; i < BAT_VSIZE; i++)
			{
				bat_avg += bat_val[i]; //bat_val[] is global variable, sampling in LETIMER0_IRQHandler interrupt.
			}

			bat_avg = bat_avg / BAT_VSIZE;

			BAT_VCC = 2 * bat_avg * 2.5 / 4096;
		}

		if(BAT_VCC > BAT_GO_LEVEL) //check battery good, break
			break;
		else if(Charging == 1)//显示充电的画面
		{
			if(timerCount > 4)
			{
				timerCount = 0;
				LED_TOGGLE();
			}
		}
		else
		{
			if(timerCount >= 19)
			{
				timerCount = 0;
				LED_TOGGLE();
			}
		}
	}

	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);

	__set_MSP( ( 0x20000000 + sizeof( bootloader ) + 0x800 ) & 0xFFFFFFF0 ); //and 0xfffffff0 alignment 256 bytes (should be align 16 bytes)
	//setup stack pointer


	WDOG_Feed(); //dogfeed one time before copy.

	/* Load the entire bootloader into SRAM. */
	memcpy( (void*)0x20000000, bootloader, sizeof( bootloader ) ); //bootloader[] in bootld.h, built by usbhhidkbd project with bin to hex text.

	/* Start executing the bootloader. */

    //Why disable watchdog while transfer to loader.
	WDOG_Enable(false);

#if (BOARD_TYPE==2  && defined(SHOWLED))
    LEDG_ON();  //for debug only.
#endif
	SysCtlDelay(50); //delay for disable watchdog?

	BOOT_jump( *(uint32_t*)0x20000000, *(uint32_t*)0x20000004 );//@p1=sp,@p2=pc, execute start at 0x20000004
	/* reset vector address is 0x00000004, not the PC=0x200000004 */
}
