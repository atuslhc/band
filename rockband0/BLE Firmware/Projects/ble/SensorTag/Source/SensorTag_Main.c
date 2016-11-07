/**************************************************************************************************
  Filename:       SensorTag_Main.c
  Revised:        $Date: 2012-08-08 17:04:23 -0700 (Wed, 08 Aug 2012) $
  Revision:       $Revision: 31161 $

  Description:    This file contains the main and callback functions for
                  the Sensor Tag sample application.

  Copyright 2012  Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"
#include "hal_assert.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

/* Application */
#include "SensorTag.h"
#include "CC2540_bitdef.h"

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */

#if 0
//这些主要是在I4B中判断外部晶振起不起振。
void Delay_115200(void)
{

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

}
void Delay_9600(void)
{
	int delay = 53;

	while(delay)
	{
		delay--;
	}

	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
}

void Delay100MS(uint8 count)
{
	int delay = 60000;

	for(uint8 k = 0; k < count; k++)
		while(delay--);
}

void SimulateUart_9600( unsigned char data)
{
	char i = 8;
	unsigned char temp = 0;

	P1_0 = 0; //起始位
	Delay_9600();

	while(i--)
	{
		temp = data;
		temp &= 0x01;

//发送1位数据
		if(temp == 1)
		{
			P1_0 = 1;
		}
		else
		{
			P1_0 = 0;
		}

		Delay_9600();
		data = data >> 1;
	}

	P1_0 = 1; //结束位

	Delay_9600();
}

bool CheckCrystalStart(void)
{
	uint8 count = 0;

	CLKCONCMD = (CLKCONCMD & ~0x80) | OSC_32KHZ;

	while ( ((CLKCONSTA & 0x80) != OSC_32KHZ ) && (count < 8))
	{
		//起震时间设置为1s
		count++;
		Delay100MS(1);
	}

	if(count >= 8)
		return false;//晶振起震失败
	else
		return true;//
}
#endif

/***
P0_2是串口的rxd 针对于cc2540
p0_3是串口的txd
***/


int main(void)
{
#if 0
	bool     blSuccessStart = true;


	blSuccessStart = CheckCrystalStart();

	if(blSuccessStart == false)
	{
          //如果起震失败，则在串口发送引脚产生中断。
		P0SEL &= ~0x08;
		P0DIR |= 0x08;

		while(1)
		{
			P0_3 = 0;
			Delay100MS(5);
			P0_3 = 1;
			Delay100MS(5);
		}

	}
#endif
    
	{
	  // SET_32KHZ_OSC();
	  // InitDelay();
	}

	/* Initialize hardware */
	HAL_BOARD_INIT();

	// Initialize board I/O and Interrupt
	InitBoard( OB_COLD );  

	/* Initialze the HAL driver: include TIMER,ADC,DMA,AES,LCD,UART,KEY,SPI,HID */
	HalDriverInit();

	/* Initialize NV system */
	osal_snv_init();

	/* Initialize the operating system */
	osal_init_system();


	/* Enable interrupts */
	HAL_ENABLE_INTERRUPTS();


	// Final board initialization
	InitBoard( OB_READY );

#if defined ( POWER_SAVING )
	osal_pwrmgr_device( PWRMGR_BATTERY );
#endif

	/* Start OSAL */
	osal_start_system(); // No Return from here

	return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
