/**************************************************************************//**
 * @file dma.c
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

#include "dma.h"
#include "main.h"
#include "globaldata.h"

#include "framebufferctrl.h"

/* DMA structures */
DMA_Init_TypeDef dmaInit;
DMA_CB_TypeDef dmaCallback;

/* Configuration structure */
//MEMLCD_Config *pConf;

/** Called by DMA when transfer is complete */
static void transferComplete(unsigned int channel, bool primary, void *user) 
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Wait for USART to finish */
  while (!(LCD_SPI_USART->STATUS & USART_STATUS_TXC)) ;
  
  /* De-assert SCS */
  LCD_CS_L();
    
    /* Reset flag to indicate that transfer is done */
  ClearSysEnergyModeFlag(DISPLAY_RUNNING);
}
/**  
  * Configure DMA to send part of, or the entire frame buffer over SPI
  * to the memory LCD. Rectangle copy is used because it can transfer
  * the entire frame in one DMA cycle. 
  * (In addition, the extra padding bits needed because emWin v5.16 does 
  * not handle a 144 px width correctly are skipped by rectangle copy
  * by setting the width to 144 px and stride to 160 px.) 
  */
void configRectangleCopy(USART_TypeDef *usart)
{
  DMA_CfgChannel_TypeDef   channelConfig;
  DMA_CfgDescr_TypeDef     descriptorConfig;

  /* Setting callback function */  
  dmaCallback.cbFunc = transferComplete;
  dmaCallback.userPtr = NULL;

  /* Setting up channel */
  channelConfig.highPri   = false;                /* No high priority */
  channelConfig.enableInt = true;                 /* Enable interrupt */
  
  /* Select USARTx peripheral */
  if ( usart == USART0 ) {
    channelConfig.select = DMAREQ_USART0_TXBL;   
  } else if ( usart == USART1 ) {
    channelConfig.select = DMAREQ_USART1_TXBL;   
  } else if ( usart == USART2 ) {
    channelConfig.select = DMAREQ_USART2_TXBL; 
  }
  
  channelConfig.cb        = &dmaCallback;         /* Callback routine */
  DMA_CfgChannel(DMA_CHANNEL, &channelConfig);

  /* Configure descriptor */
  descriptorConfig.dstInc   = dmaDataIncNone;     /* Do not increase destination */
  descriptorConfig.srcInc   = dmaDataInc2;        /* Increase source by 2 bytes */
  descriptorConfig.size     = dmaDataSize2;       /* Element size is 2 bytes */
  descriptorConfig.arbRate  = dmaArbitrate1;      /* Arbiratrate after each transfer */
  descriptorConfig.hprot    = 0;                  /* Non-privileged access */
  
  /* Configure the LOOP0 register for 2D copy */
  DMA_CfgLoop_TypeDef loopConfig;
  loopConfig.enable = false;
  loopConfig.nMinus1 = FRAME_BUFFER_WIDTH/16-1;  /* Number of elements (-1) to transfer */
  DMA_CfgLoop(DMA_CHANNEL, &loopConfig);
  
  /* Configure the RECT0 register for 2D copy */
  DMA_CfgRect_TypeDef rectConfig;
  rectConfig.dstStride = 0;
  rectConfig.srcStride = FRAME_BUFFER_STRIDE / 8; /* Width of the total frame buffer, in bytes */
  
  rectConfig.height = MEMLCD_SIZE_Y;
  DMA_CfgRect(DMA_CHANNEL, &rectConfig);
  
  /* Create the descriptor */
  DMA_CfgDescr(DMA_CHANNEL, true, &descriptorConfig); 
}

/** Initialize DMA operation. 
  * Enables the clock and sets up descriptors and 
  * registers for dma frame buffer transfer */
/** Initialize DMA operation. 
  * Enables the clock and sets up descriptors and 
  * registers for dma frame buffer transfer */
void initDMA4LCD(void) 
{
  
  CMU_ClockEnable(cmuClock_DMA, true);
  
  dmaInit.hprot = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Configure registers and descriptors for frame buffer transfer */
  configRectangleCopy(LCD_SPI_USART);
}

void dmaStartFrameTransfer(int firstLine, int lastLine)
{
    /* Get address of first line */
  uint16_t *startAddr = FB_getActiveBuffer();
  
#if 0
  startAddr += firstLine * 10;
#else
  startAddr += firstLine * ((MEMLCD_SIZE_X+16+32)/16);
#endif

  
  /* Create update command and address of first line */
  uint16_t cmd = MEMLCD_CMD_UPDATE | ((firstLine+1) << 8); 
  
  /* Enable chip select */
  LCD_CS_H();
  
  /* Set number of lines to copy */
  DMA->RECT0 = (DMA->RECT0 & ~_DMA_RECT0_HEIGHT_MASK) | (lastLine - firstLine);
  
  /* Indicate to the rest of the program that SPI transfer is in progress */
  SetSysEnergyModeFlag(DISPLAY_RUNNING);
  
  /* Send the update command */
  USART_TxDouble(LCD_SPI_USART, cmd);
    
  /* Start the transfer */
  DMA_ActivateBasic(DMA_CHANNEL,
                    true,                               /* Use primary channel */
                    false,                              /* No burst */
                    (void *)&(LCD_SPI_USART->TXDOUBLE),  /* Write to USART */
                    startAddr,                          /* Start address */
                    FRAME_BUFFER_WIDTH/16-1);           /* Width -1 */  
  
}


