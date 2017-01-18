/**************************************************************************//**
 * @file dma.h
 * @brief Copy pixels to Memory LCD with DMA
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

#ifndef _DMA_H_
#define _DMA_H_

#include "em_device.h"
#include "em_dma.h"
#include "em_emu.h"
#include "em_ebi.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "dmactrl.h"


#include "bsp.h"

/* The DMA channel to use */
#define LCD_DMA_CHANNEL 0

/* Max number of elements the DMA can transfer in one cycle */
#define DMA_MAX_TRANSFER_SIZE 1024

/* Frame buffer width, including control signals. 
 * W = 128 px + 8 addr bits + 8 dummy bits */
 
#define FRAME_BUFFER_WIDTH (MEMLCD_SIZE_X+8+8) //(144+8+8)


/* Total width of each line in frame buffer, in bits. 
 * Includes 128 pixels + 8 address bits + 8 dummy bits for SPI
 * + 16 bits of padding required because of a bug in emWin (v 5.16)
 * that requires the display width to be divisble by 32 */
 
#define FRAME_BUFFER_STRIDE (MEMLCD_SIZE_X+8+8+32)//(144+8+8+32)




/* DMA structures */
extern DMA_Init_TypeDef dmaInit;
extern DMA_CB_TypeDef dmaCallback;

/* Flags to check if DMA is currently active */
extern volatile bool spiTransferActive;

/* Configuration structure */
//extern MEMLCD_Config *pConf;



/** Called by DMA when transfer is complete */
static void transferComplete(unsigned int channel, bool primary, void *user);

/**  
  * Configure DMA to send part of, or the entire frame buffer over SPI
  * to the memory LCD. Rectangle copy is used because it can transfer
  * the entire frame in one DMA cycle. 
  * (In addition, the extra padding bits needed because emWin v5.16 does 
  * not handle a 144 px width correctly are skipped by rectangle copy
  * by setting the width to 144 px and stride to 160 px.) 
  */
extern void configRectangleCopy(USART_TypeDef *usart);

/** Initialize DMA operation. 
  * Enables the clock and sets up descriptors and 
  * registers for dma frame buffer transfer */
//extern void initDMA(MEMLCD_Config *config);

extern void dmaStartFrameTransfer(int firstLine, int lastLine);

extern 	void initDMA4LCD(void);
#endif

