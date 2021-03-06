/**************************************************************************************************
Filename:       SsensorTag.h
Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
Revision:       $Revision: 33575 $

Description:    This file contains the Sensor Tag sample application
definitions and prototypes.

Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

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
PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef SENSORTAG_H
#define SENSORTAG_H

#ifdef __cplusplus
extern "C"
{
#endif

#define HEX_TO_ASCII_H(x) (((x>>4)&0xf)<0xa? ((x>>4)+'0'):((x>>4)-10+'A'))
#define HEX_TO_ASCII_L(x) (((x)&0xf)<0xa? ((x&0xf)+'0'):((x&0xf)-10+'A'))

/*********************************************************************
* INCLUDES
*/

/*********************************************************************
* CONSTANTS
*/

// Sensor Tag Task Events
#define ST_START_DEVICE_EVT                             0x0001
#define ST_UART_READ_EVT                                0x0002
#define ST_SPI_DELAY_EVT                                0x0004
#define ST_SPI_BLE_EVT                                  0x0008
#define ST_DELAY_EVT					                0x0010
#define USER_UART_RX_EVT				                0x0020
#define BP_START_DISCOVERY_EVT                          0x0040
#define ST_SYS_RESET_EVT                                0x0080
#define WAIT_CRYSTAL_START_EVT                          0x0100
//#define PERIOD_EVT                                    0X0040
/*********************************************************************
* MACROS
*/

/*********************************************************************
* FUNCTIONS
*/

/*
* Task Initialization for the BLE Application
*/
extern void SensorTag_Init( uint8 task_id );

/*
* Task Event Processor for the BLE Application
*/
extern uint16 SensorTag_ProcessEvent( uint8 task_id, uint16 events );

/*
* Power on self test
*/
extern uint16 sensorTag_test(void);
void BLEDtatMOV(void);

void CONN_INTERVAL_UPDATA_RES(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAG_H */
