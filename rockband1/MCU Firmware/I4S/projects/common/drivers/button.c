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


/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Button
 * @{
 ******************************************************************************/
#include "button.h"
#include "main.h"

#include "cmsis_os.h"



void KeyInit(void)
{
  /* Enable GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure GPIO port PA2(SW1), PA5(SW2) as Key input */
  #if (BOARD_TYPE==2)
  GPIO_PinModeSet(KEY_PORT, KEY2_PIN, gpioModeInputPull, 1);
  #endif
  GPIO_PinModeSet(KEY_PORT, KEY_PIN, gpioModeInputPull, 1);
  
  GPIO_IntConfig(KEY_PORT, KEY_PIN, false, true, true);
  
  GPIO_IntClear(1 << KEY_PIN); 

  NVIC_SetPriority(GPIO_EVEN_IRQn,GPIO_EVEN_INT_LEVEL);
   	
  /* Enabling Interrupt from GPIO_EVEN */
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
 
}

void KeyIntHandler(void)
{

 extern osMessageQId hMsgInterrupt;
 osMessagePut(hMsgInterrupt,HardKey_Message,0);	
}


/**************************************************************************//**
 * @brief Initialize Beep interface
 *****************************************************************************/
void ButtonInit(void)
{
  KeyInit();
 
}

/** @} (end group Leds) */
/** @} (end group Drivers) */
