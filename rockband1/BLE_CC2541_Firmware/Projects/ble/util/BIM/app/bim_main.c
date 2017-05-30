/******************************************************************************

 @file  bim_main.c

 @brief This module contains the definitions for the main functionality of an
        Boot Image Manager.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2012-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"

#include "ioCC254x_bitdef.h"
#include "bim_main.h"
/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define FBIT0 0xFE
#define FBIT1 0xFD
#define FBIT2 0xFB
#define FBIT3 0xF7
#define FBIT4 0xEF
#define FBIT5 0xDF
#define FBIT6 0xBF
#define FBIT7 0x7F
#define CLR_WDT WDCTL = WD_RESET1;WDCTL = WD_RESET2; //Çå¿´ÃÅ¹·
#define checkSumAdd         0xF600
#define crcCheck_Page       123
#define appStart_Page	    0
#define appEnd_Page         122
#define SPI_DataStart       '<'
#define SPI_DataStop        '>'
#define SPI_ID_LEN          0x01
#define SPI_ID_DATA         0x03

#define SPI_DATA_START      0x00
#define SPI_DATA_CMD        0x01
#define SPI_DATA_LEN        0x02
#define SPI_DATA_DATA       0x03

#define SPI_ADVER_CMD       SPI_DATA_DATA
#define SPI_ADVER_DATA      SPI_DATA_DATA+1

#define ADVER_EN            0x01
#define ADVER_DIS           0x00


#define SPI_CMD_DEVIN        0x81
#define SPI_CMD_UPDATA        0x10
#define SPI_CMD_DATA	      0x11

#define SPI_DATA_UPDATA_CMD	0x03
#define SPI_DATA_UPDATA_CHECKSUM        0x04


#define SPI_LEN_DEVIN        0x04
#define SPI_LEN_INFOR        0x05
#define SPI_LEN_COM          0x04

#define ADVER_EN            0x01
#define ADVER_DIS           0x00

#define INFOR_STARTED       0x00
#define INFOR_ADVERTISING   0x01
#define INFOR_CONNECTED     0x02

#define TxFlage_EN          0x01
#define TxFlage_DIS         0x00

#define BLE_STATE_IDLE              0x00
#define BLE_STATE_ADVERTISING       0x01
#define BLE_STATE_CONNECTED         0x02

#define Dev_Infor_ChipID    0x40
#define Dev_Infor_Cap       0x08
#define Dev_Infor_Ver1      0x02
#define Dev_Infor_Ver2      0x02

#define BOOTLOAD  0
#define BLE_APP 1


// Define size of buffer
#define BUFFER_SIZE 32

#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20
#define SPI_BAUD_E 16
#define SPI_BAUD_M 0

// 115200
#define UART_BAUD_M  216
#define UART_BAUD_E  11

// 11400
//#define UART_BAUD_M  216
//#define UART_BAUD_E  8

// 9600
//#define UART_BAUD_M  59
//#define UART_BAUD_E  8



/***********************************************************************************
* LOCAL VARIABLES
*/

#define rxBufferSlaveSize 240
#define CRC_CHECK_VALUE 0X2013
__no_init  uint8 rxBufferSlave[150];

uint8 RxLen=0;

uint8 spiFlage = 0;

uint16 uartDelay = 0;


#define UARTDELAY_LIM                   2000
#define  URX0_VECTORboot    VECT(  3, 0x0091 )   /*  USART1 RX Complete                          */

uint16 crcCheck(void);
void SPI_READ(void);
void uart0Send(uint8* uartTxBuf, uint8 uartTxBufLength);
/* ------------------------------------------------------------------------------------------------
*                                       Global Variables
* ------------------------------------------------------------------------------------------------
*/

__no_init halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite().
/* ------------------------------------------------------------------------------------------------
*                                       Local Variables
* ------------------------------------------------------------------------------------------------
*/

__no_init uint8 pgBuf[HAL_FLASH_PAGE_SIZE];

__no_init __data uint8 JumpToImageAorB @ 0x09;

#pragma vector = URX0_VECTORboot
__interrupt void uart1RX_isr(void)
{
  
  URX0IF = 0;
  uartDelay=UARTDELAY_LIM;
  
  rxBufferSlave[RxLen] = U0DBUF;
  RxLen++;
  if (((rxBufferSlave[UART_ID_LEN]+6) == RxLen) && (RxLen > UART_ID_LEN))
  {
    spiFlage = 0x01;
    RxLen = 0;
  }
}

/**************************************************************************************************
 * @fn          crcCheck
 *
 * @brief       Calculate the image CRC and set it ready-to-run if it is good.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      None, but no return from this function if the CRC check is good.
 **************************************************************************************************
 */
uint16 crcCheck(void)
{
  uint8 dataBuf[128];
  uint8 app_Page = 0;
  uint16 offset = 0;
  uint8 i = 0, j;
  uint16 checkSumCon = 0;
  
  for (app_Page = (appStart_Page + 1); app_Page < (appEnd_Page+1); app_Page++)
  {
    for (j=0; j<16; j++)
    {
      HalFlashRead(app_Page,offset,dataBuf,128);
      for (i=0; i<128; i++)
      {
        checkSumCon  = (checkSumCon >> 8) | (checkSumCon << 8);
        checkSumCon ^= dataBuf[i];
        checkSumCon ^= (checkSumCon & 0xff) >> 4;
        checkSumCon ^= checkSumCon << 12;
        checkSumCon ^= (checkSumCon & 0xff) << 5;
      }
      offset += 128;
    }
    offset = 0;	
  }
  return checkSumCon;
}

__no_init XDATA uint8 mac_buf[6]@0x780E;
uint8 devInfor[21];
void SPI_READ(void)
{	
  devInfor[3] = UART_DATA_START;
  devInfor[4] = 18;
  devInfor[5] = UART_CMD_INFOR;
  devInfor[6] = 0x01;
  devInfor[7] = Dev_Infor_ChipID;
  devInfor[8] = Dev_Infor_Cap;
  devInfor[9] = Dev_Infor_Ver1;
  devInfor[10] = Dev_Infor_Ver2;
  devInfor[11] = mac_buf[0];
  devInfor[12] = mac_buf[1];
  devInfor[13] = mac_buf[2];
  devInfor[14] = mac_buf[3];
  devInfor[15] = mac_buf[4];
  devInfor[16] = mac_buf[5];
  
  devInfor[17] = BOOTLOAD;
  
  devInfor[20] = UART_DATA_STOP;
}
uint8 ackData[11]={
  0x00,0x00,0x00,UART_DATA_START,0x08,0x05,0x05,0x02,0x00,0x00,UART_DATA_STOP
};

/**************************************************************************************************
 * @fn          main
 *
 * @brief       C-code main function.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
volatile char ii,jj;
void main(void)
{
  static uint8 writePage = 0;
  static uint16 writeOffset = 0;
  static uint16 writeAdd = 0;
  static uint16 checkSumCode=0;
  static uint16 checkSumCon=0;
  
  CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
  while (CLKCONSTA & CLKCON_OSC);   // Wait until clock source has changed		
 // checkSumCon =*(uint16 *)&mac_buf[4]; // so copy the memory ,but mac is different , protection!!!
  HalFlashRead(crcCheck_Page, 0, (uint8 *)&checkSumCode, 2);
 // if (checkSumCode == checkSumCon)
  if (checkSumCode == CRC_CHECK_VALUE)
  {
    JumpToImageAorB = 0;
    asm("LJMP 0x0830");
    HAL_SYSTEM_RESET();
  }
#if (0)  //Atus: force write flag.
          //checkSumCon = crcCheck();
  if (checkSumCode == 0xFFFF)
  {
     EA = 1; //HAL_ENABLE_INTERRUPTS();  
     HAL_DMA_SET_ADDR_DESC0(&dmaCh0);
     HalFlashErase(crcCheck_Page);
     while (FCTL & 0x80);
     checkSumCode =CRC_CHECK_VALUE;
     HalFlashWrite(checkSumAdd,(uint8 *)&checkSumCode,0x01);
     HalFlashRead(crcCheck_Page, 0, (uint8 *)&checkSumCode, 2);
  }
  
#endif
  JumpToImageAorB = 1;
  SPI_READ();
  P0SEL = BIT2 | BIT3;
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;
  
  U0CSR |= U0CSR_MODE;
  U0UCR= 0x02;
  
  U0GCR &= ~U0GCR_ORDER;
  URX0IF = 0;
  U0CSR |= 0x40;
  URX0IE = 1;
  EA = 1; 
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);
  
  uart0Send(devInfor,21);
  
  while (1)
  {
    if (spiFlage)
    {
      switch (rxBufferSlave[UART_ID_CMD])
      {
      case UART_CMD_UPDATA:
        
        if (rxBufferSlave[UART_ID_DATA])
        {
          
          ((uint8 *)&checkSumCode)[0] = rxBufferSlave[UART_ID_DATA+1];
          ((uint8 *)&checkSumCode)[1] = rxBufferSlave[UART_ID_DATA+2];
          
          checkSumCon = crcCheck();
          if (checkSumCon == checkSumCode)
          {
            HalFlashErase(crcCheck_Page);
            while (FCTL & 0x80);
            checkSumCode =CRC_CHECK_VALUE;
            HalFlashWrite(checkSumAdd,(uint8 *)&checkSumCode,0x01);
            uart0Send(ackData,11);
            HAL_SYSTEM_RESET();
          }
        }
        writePage = 0;
        writeOffset = 0;
        writeAdd = 0;
        uart0Send(ackData,11);
        
        break;	
        
      case UART_CMD_DATA:
        {
          if (writeOffset == 0)
          {
            if(writePage>appEnd_Page)writePage=appEnd_Page;
            HalFlashErase(writePage+1);
            while (FCTL & 0x80);
          }
          writeAdd = (writeOffset >> 2) + ((uint16)(writePage+1) << 9);
          HalFlashWrite(writeAdd,&rxBufferSlave[UART_ID_DATA],128/4);//
          writeOffset += 128;
          ackData[8]=writePage+1;
          uart0Send(ackData,11);
          ackData[8]=0;
          if (writeOffset >= 2048)
          {
            writeOffset = 0;
            writePage++;
          }
          break;	
        }
        
      case UART_CMD_INFOR:
        {
          uart0Send(devInfor,21);
          break;
        }
      default : break;
      }
      spiFlage = 0x00;
    }
    if (uartDelay)
    {
      uartDelay--;
      if(uartDelay==0)
        RxLen=0;			
    }
  }
}

void uart0Send(uint8* uartTxBuf, uint8 uartTxBufLength)
{
  uint8 uartTxIndex1;
  
  U0CSR &= ~U0CSR_TX_BYTE;
  for (uartTxIndex1 = 0; uartTxIndex1 < uartTxBufLength; uartTxIndex1++)
  {
    U0DBUF = uartTxBuf[uartTxIndex1];
    while(! (U0CSR & U0CSR_TX_BYTE) );
    U0CSR &= ~U0CSR_TX_BYTE;
  }
}


/**************************************************************************************************
*/
