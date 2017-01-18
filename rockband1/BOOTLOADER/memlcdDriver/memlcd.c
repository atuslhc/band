/**************************************************************************//**
 * @file memlcd.c
 * @brief Sharp Memory LCD Serial Interface
 * @author Energy Micro AS
 * @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
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
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
//#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "memlcd.h"
#include "main.h"
#include "em_letimer.h"


/* Globals */
//static int            timerCyclesPerMicrosecond;
static uint8_t        comPolarity = 0;
unsigned int sysclk=1; //1M 


void SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}



void initLETIMER(void)
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  
  GPIO_PinModeSet(LCD_EXTCOM_PORT,LCD_EXTCOM_PIN, gpioModePushPull, 0);
   
  /* Set initial compare values for COMP0 */
  LETIMER_CompareSet(LETIMER0, 0, 32768);//比较器设置成32768.

  /* Route LETIMER to location 3 (OUT0 - PC4) and enable output */
  //LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_LOCATION_LOC3;
 //PB11 For CNV BOARD
  LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_LOCATION_LOC1;
  
  /* The current version of emlib (3.0.0) does not properly set 
   * REP0 to a nonzero value while enabling LETIMER in Free mode. 
   * Therefore we set REP0 manually here */
  LETIMER0->REP0 = 0x01;//重新计数器0
  
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
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}



/***********************************************************
 * Initialize driver
 **********************************************************/
void MEMLCD_Init(void)
{
   USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;
  
   /* Setup clocks */
   CMU_ClockEnable( cmuClock_GPIO, true );
   CMU_ClockEnable( LCD_SPI_CMUCLOCK, true );

   /* Setup GPIO's */
   GPIO_PinModeSet(LCD_SPI_GPIOPORT, LCD_SPI_CLKPIN, gpioModePushPull, 0 );
   GPIO_PinModeSet(LCD_SPI_GPIOPORT, LCD_SPI_MOSIPIN,   gpioModePushPull, 0 );
   GPIO_PinModeSet(LCD_CS_PORT, LCD_CS_PIN,  gpioModePushPull, 0 );

   GPIO_PinModeSet(LCD_EXTCOM_PORT,LCD_EXTCOM_PIN, gpioModePushPull, 0 );
   GPIO_PinModeSet(LCD_DISP_PORT,LCD_DISP_PIN,gpioModePushPull, 0 );  

   /* Setup USART */
   usartInit.baudrate = 1100000;
   usartInit.databits = usartDatabits16;
   
   USART_InitSync(LCD_SPI_USART, &usartInit );
   LCD_SPI_USART->ROUTE = (USART_ROUTE_CLKPEN | USART_ROUTE_TXPEN |LCD_SPI_LOCATION);
  
   GPIO_PinOutSet(LCD_DISP_PORT,LCD_DISP_PIN);

   MEMLCD_Clear();
  
}

/***********************************************************
 * Enable or disable the display. Disabling the display
 * does not make it lose it's data.
 **********************************************************/
void MEMLCD_Enable(bool enable)
{
  if (enable)
  {
     GPIO_PinOutSet(LCD_DISP_PORT,LCD_DISP_PIN);
  }
  else
  {
    GPIO_PinOutClear(LCD_DISP_PORT,LCD_DISP_PIN);
  }
}


/***********************************************************
 * Clear display
 **********************************************************/
void MEMLCD_Clear( void )
{
   uint8_t cmd;

   /* Set SCS */
   LCD_CS_H();
   
   /* SCS setup time: min 6us */
    SysCtlDelay(6/3*sysclk);

   /* Send command */
   cmd = (MEMLCD_CMD_ALL_CLEAR | comPolarity);
   USART_TxDouble(LCD_SPI_USART, cmd );

   /* Wait for transfer to finish */
   while ( !(LCD_SPI_USART->STATUS & USART_STATUS_TXC) );

   /* SCS hold time: min 2us */
   SysCtlDelay(3/3*sysclk);
   
   /* Clear SCS */
   LCD_CS_L();
}



/** @brief Update the display
  * @param firstLine The first line to update
  * @param lastLine The last line to update
  */
EMSTATUS MEMLCD_Update(  uint16_t *data, int firstLine, int lastLine )
{
  int i,j;
  
  /* Assert SCS */
  LCD_CS_H();

  /* SCS setup time: min 6us */
   SysCtlDelay(6/3*sysclk);
  
  /* Send update command and first line address */
  USART_TxDouble(LCD_SPI_USART, MEMLCD_CMD_UPDATE | (firstLine + 1) << 8);
  
  /* Get start address to draw from */
  uint16_t *p = (uint16_t *)data;
  
  p += firstLine * ((MEMLCD_SIZE_X + 16 + 32)/16);

  
  for ( i=firstLine; i<=lastLine; i++ ) {
        
    /* Send pixels for this line */

    for ( j=0; j<((MEMLCD_SIZE_X+16)/16); j++ ) {

      USART_TxDouble(LCD_SPI_USART, *p);
      p++;
    }
       
    /* Skip padding data in frame buffer */
    p += 2;
  }
  
  
  /* Wait for USART to finish */
  while (!(LCD_SPI_USART->STATUS & USART_STATUS_TXC)) ;
  
  /* SCS hold time: min 2us */
   SysCtlDelay(3/3*sysclk);
  
  /* De-assert SCS */
  LCD_CS_L();

  return MEMLCD_OK;
}



