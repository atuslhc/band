/******************************************************************************

 @file  sensorTag.h

 @brief This file contains the Sensor Tag sample application definitions and
        prototypes.

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

#ifndef SENSORTAG_H
#define SENSORTAG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// only number
#define FIRMWARE_VERSION    "00.06"

// Sensor Tag Task Events
#define ST_START_DEVICE_EVT                              0x0001
#define ST_IRTEMPERATURE_READ_EVT                        0x0002
#define ST_ACCELEROMETER_SENSOR_EVT                      0x0004
#define ST_HUMIDITY_SENSOR_EVT                           0x0008
#define ST_MAGNETOMETER_SENSOR_EVT                       0x0010
#define ST_BAROMETER_SENSOR_EVT                          0x0020
#define ST_GYROSCOPE_SENSOR_EVT                          0x0040
#define ST_SYS_RESET_EVT                                 0x0080
#define SBP_SEND_EVT                                     0x0100
#define ST_SPI_BLE_EVT                                   0x0200
#define ST_DELAY_EVT                                     0x0400
#if defined( PLUS_BROADCASTER )     //atus add
#define ST_ADV_IN_CONNECTION_EVT                         0x0800
#endif

#define SBP_BURST_EVT_PERIOD            20
/*********************************************************************
 * MACROS
 */
#define HEX_TO_ASCII_H(x) (((x>>4)&0xf)<0xa? ((x>>4)+'0'):((x>>4)-10+'A'))
#define HEX_TO_ASCII_L(x) (((x)&0xf)<0xa? ((x&0xf)+'0'):((x&0xf)-10+'A'))

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8 sensorTag_TaskID;   // Task ID for task/event processing

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


extern uint8 rxDtatCheck(void);
extern void BLEDtatMOV(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAG_H */
