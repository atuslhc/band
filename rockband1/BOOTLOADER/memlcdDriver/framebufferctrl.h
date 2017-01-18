/**************************************************************************//**
 * @file framebuffer_control.h
 * @brief Contains functions to manipulate and retrieve the frame buffer
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

#ifndef _FRAMEBUFFER_CTRL_H_
#define _FRAMEBUFFER_CTRL_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "memlcd.h"


#define DISPLAY_WIDTH  MEMLCD_SIZE_X
#define DISPLAY_HEIGHT MEMLCD_SIZE_Y

/* Width of the frame buffer. 4 extra bytes are added to each line. 
 * 2 bytes are added to accommodate the control signals and dummy bits
 * for the SPI protocol. The 2 last bytes are added because of a bug
 * in the current emWin version (v 5.16), which does not handle a frame buffer
 * width of 144 correctly. 
*/
#define BUFFER_VIRTUAL_WIDTH (MEMLCD_SIZE_X+16+32)

/* Size defines, defined for readability in code below */

#define DISPLAY_HALFWORDS_PER_LINE     (DISPLAY_WIDTH / 16)

#define DISPLAY_BYTES_PER_LINE         (DISPLAY_HALFWORDS_PER_LINE * 2)

#define BUFFER_HALFWORDS_PER_LINE      (BUFFER_VIRTUAL_WIDTH / 16)
#define BUFFER_BYTES_PER_LINE      		(BUFFER_HALFWORDS_PER_LINE * 2)

#define BUFFER_SIZE_HALFWORDS          (BUFFER_HALFWORDS_PER_LINE * DISPLAY_HEIGHT)

#define BUFFER_SIZE_BYTES              (BUFFER_SIZE_HALFWORDS * 2)



///* Two frame buffers are used. While transferring one buffer to the display
// * the next frame is begin drawn to the other frame buffer. */
//extern uint16_t frameBuffer[BUFFER_SIZE_HALFWORDS];
//
//extern uint16_t backgroundBuffer[2][BUFFER_HALFWORDS_PER_LINE * 15];



///** Direct emWin drawing output to the frame buffer */
//extern void FB_activateBuffer(void);

///** Write the SPI control signals (line adresses and dummy data) to frame buffer */
//extern void FB_writeControlSignals(uint8_t *buffer);

//extern void FB_copyBuffer(uint8_t *pdest, uint8_t *psrc, uint8_t startline, uint8_t endline);

uint16_t *FB_getActiveBuffer(void);

/** Direct emWin drawing output to the frame buffer */
//void FB_activateFirstBuffer(void);

//void FB_activateSecondBuffer(void);

/** Direct emWin drawing output to the background buffer */
//void FB_activateBackgroundBuffer(void);

/** Copy the entire background buffer to the frame buffer */
//void FB_copyBackgroundToFrameBuffer(void);

//void FB_flipBuffer(void);

/** Clears the current buffer */
void FB_clearBuffer(void);

void DrawBMP(const unsigned char * src, unsigned short x, unsigned short y);

#endif
