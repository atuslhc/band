/**************************************************************************************************
Filename:       timeapp_ind.c
Revised:        $Date: 2011-06-22 20:44:48 -0700 (Wed, 22 Jun 2011) $
Revision:       $Revision: 26428 $

Description:    Time App indication and notification handling routines.

Copyright 2011 Texas Instruments Incorporated. All rights reserved.

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
PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
* INCLUDES
*/

#include "string.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "timeapp.h"
#include "OnBoard.h"
#include "serialapp.h"
#include "SensorTagUser.h"
/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/
//Ancs EventID Values
#define EVENTID_ADDED                     0x00
#define EVENTID_MODIFIED                  0x01
#define EVENTID_REMOVED                   0x02

//
//#define ANCS_CATEGORYID_INCOMINGCALL      0x01           // CategoryID Incoming Call
//#define ANCS_CATEGORYID_MISSEDCALL        0x02 // CategoryID Missed Call
#define ANCS_CATEGORY_ID_OTHER               0
#define ANCS_CATEGORY_ID_INCOMING_CALL       1
#define ANCS_CATEGORY_ID_MISSED_CALL         2
#define ANCS_CATEGORY_ID_VOICE_MAIL          3
#define ANCS_CATEGORY_ID_SOCIAL              4
#define ANCS_CATEGORY_ID_SCHEDULE            5
#define ANCS_CATEGORY_ID_EMAIL               6
#define ANCS_CATEGORY_ID_NEWS                7
#define ANCS_CATEGORY_ID_HEALTH_FITNESS      8
#define ANCS_CATEGORY_ID_BUSINESS_FINACE     9
#define ANCS_CATEGORY_ID_LOCATION            10
#define ANCS_CATEGORY_ID_ENTERTAINMENT       11

#define UID_NUM_LEN                       10

//Ancs Uid Flag value
#define UID_FLAG_REMOVED                  0x11
#define UID_FLAG_ADDED                    0xAA

// ANCS Notification info   ÏÂÃæµÄ½á¹¹ÊÇ±ê×¼
typedef struct
{
	uint8 EventID;//±íÃ÷¸øIOSµÄnotificationÊÇ·ñadd,modify,remove
	uint8 EventFlag;
	uint8 CategoryID;
	uint8 CategoryCount;
	uint8 NotificationUID[4];//IOSµÄnotificationµÄUIDÊÇ32Î»µÄ£¬ÕâÀï¶¨Òå4byte.
} Ancs_Noti_t;

// ANCS Notification info
typedef struct
{
	uint8 Flag;
	uint8 UID[4];
} Ancs_Uid_t;

Ancs_Uid_t Ancs_Noti[UID_NUM_LEN];

uint16 Ancs_Data_Len = ANCS_DATA_LOCATION;

uint8 Ancs_Data_Buff[ANCS_DATA_MAX] = {0X3C};
uint8 g_ucFlag = 0;
/*********************************************************************
* LOCAL FUNCTIONS
*/
static void Ancs_Notification_Explain( Ancs_Noti_t* noti );
static void Ancs_Date_Explain( attHandleValueNoti_t* noti );
static void simpleBLEWriteChar( uint8* data );
/*********************************************************************
* @fn      timeAppIndGattMsg
*
* @brief   Handle indications and notifications.
*
* @param   pMsg - GATT message.
*
* @return  none
*/
void timeAppIndGattMsg( gattMsgEvent_t* pMsg )
{
	uint8 i;

	// Look up the handle in the handle cache
	for ( i = 0; i < HDL_CACHE_LEN; i++ )
	{
		if ( pMsg->msg.handleValueNoti.handle == timeAppHdlCache[i] )
		{
			break;
		}
	}

	// Send confirm for indication   Ö»ÓÐÊÇindicationÊ±£¬²ÅÐèÒª·¢ËÍÈ·ÈÏ£¬notification²»ÐèÒªÈ·ÈÏÖ±½Ó·¢ËÍ³öÈ¥
	if ( pMsg->method == ATT_HANDLE_VALUE_IND )
	{
		ATT_HandleValueCfm( pMsg->connHandle );
	}

	// Perform processing for this handle
	switch ( i )
	{
		case HDL_ANCS_NOTI_START:
			//DEBUG_PRINT("Notify@\r\n",0);
			Ancs_Notification_Explain( (Ancs_Noti_t*)pMsg->msg.handleValueNoti.value );
			break;

		case HDL_ANCS_DATA_START:
			//DEBUG_PRINT("Data@\r\n",0);
			Ancs_Date_Explain( &pMsg->msg.handleValueNoti );
			break;

		default:
			break;
	}

}

/*********************************************************************
* @fn      Ancs_Notification_Explain
*
* @brief   Ancs Notification Explain.
*
* @param   Ancs_Noti_t - noti.
*
* @return  none
*/
static void Ancs_Notification_Explain( Ancs_Noti_t* noti )
{
#if 0

	switch( noti->EventID )
	{
		case EVENTID_ADDED:

			// case EVENTID_MODIFIED:
			if( ANCS_CATEGORY_ID_INCOMING_CALL == noti->CategoryID ||
			        ANCS_CATEGORY_ID_MISSED_CALL == noti->CategoryID || 4 == noti->CategoryID )
			{
				for(uint8 i = 0; i < UID_NUM_LEN + 1; i++ )
				{
					if( UID_NUM_LEN == i )
					{
						for( uint8 j = 0; j < UID_NUM_LEN; j++ )
							Ancs_Noti[i].Flag = UID_FLAG_REMOVED;

						osal_memcpy( Ancs_Noti[0].UID, noti->NotificationUID, 4 );
						break;
					}

					if( UID_FLAG_ADDED != Ancs_Noti[i].Flag )
					{
						Ancs_Noti[i].Flag = UID_FLAG_ADDED;
						osal_memcpy( Ancs_Noti[i].UID, noti->NotificationUID, 4 );
						break;
					}
				}

				osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_DATA_EVT, 100 );
			}

			break;

		case EVENTID_REMOVED:
			break;

		default:
			break;
	}

#endif

// if(noti->CategoryID ==ANCS_CATEGORY_ID_SOCIAL)
	{
		if(noti->EventID == EVENTID_ADDED)
		{
//Èç¹ûANCSÊÂ¼þIDÊÇÌí¼ÓÊÂ¼þ£¬ÄÇÃ´¾Í»ñÈ¡UID
			osal_memcpy( Ancs_Noti[0].UID, noti->NotificationUID, 4 );

//Æô¶¯Ò»¸ö¶ÁÈ¡ANCSÊÂ¼þµÄ¶¨Ê±Æ÷¡£
			osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_DATA_EVT, BP_READ_ANCS_TIMEOUT );
			g_ucFlag = 1;

		}
		else if(noti->EventID == EVENTID_REMOVED)
		{

		}
		else
		{
		}
	}

	/*  uint8 EventID;
	uint8 EventFlag;
	uint8 CategoryID;
	uint8 CategoryCount;
	uint8 NotificationUID[4];
	*/
	uint8 ucDataArry[13] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_ANCS, 1, \
	                        noti->EventID, noti->EventFlag, noti->CategoryID, \
	                        noti->CategoryCount, noti->NotificationUID[0], \
	                        noti->NotificationUID[1], noti->NotificationUID[2], \
	                        noti->NotificationUID[3], UART_DATA_STOP
	                       };
	txDtatSend(ucDataArry, sizeof(ucDataArry));

}

/*********************************************************************
* @fn      ReadAncsAppNameTask
*
* @brief   Read the App Name on Ancs.
*
* @param   none.
*
* @return
*/
void ReadAncsAppNameTask( void )
{
	simpleBLEWriteChar( Ancs_Noti[0].UID );
	//  for( uint8 i=0; i<UID_NUM_LEN; i++ )
	//  {
	//    if( UID_FLAG_ADDED == Ancs_Noti[i].Flag )
	//    {
	//      simpleBLEWriteChar( Ancs_Noti[i].UID );
	//      Ancs_Noti[i].Flag = UID_FLAG_REMOVED;
	//      break;
	//    }
	//  }
}

/*********************************************************************
* @fn      simpleBLEWriteCha()
*
* @brief   Perform the characteristic configuration read or
*          write procedure.
*
* @param   state - Configuration state.
*
* @return  none.
*/
static void simpleBLEWriteChar( uint8* data )
{
	attWriteReq_t writeReq;
#if 0
	writeReq.len = 8;
	writeReq.value[0] = 0;
	osal_memcpy( &writeReq.value[1], data, 4 );
	writeReq.value[5] = 3;
	writeReq.value[6] = 0xff;
	writeReq.value[7] = 0xff;
	writeReq.sig = 0;
	writeReq.cmd = 0;
	//  for (uint8 k=0;k<4;k++)
	//  {
	//    DEBUG_PRINT_HEX(data[k]);
	//  }
#else
	writeReq.len = 12;
	writeReq.value[0] = 0;
	osal_memcpy( &writeReq.value[1], data, 4 );
	writeReq.value[5] = 1;
	writeReq.value[6] = 0xff;
	writeReq.value[7] = 0xff;
	writeReq.value[8] = 3;
	writeReq.value[9] = 0xff;
	writeReq.value[10] = 0xff;
	writeReq.value[11] = 4;
	writeReq.sig = 0;
	writeReq.cmd = 0;
#endif
	writeReq.handle = timeAppHdlCache[HDL_ANCS_COMM];

	GATT_WriteCharValue( gapConnHandle, &writeReq, sensorTag_TaskID );
//¸Ãº¯Êý½«characteristicÖµÐ´Èëµ½serverÖÐ
}

/*********************************************************************
* @fn      Ancs_Data_Explain
*
* @brief   Ancs Data Explain.
*
* @param   *data.
*
* @return  none
*/
static void Ancs_Date_Explain( attHandleValueNoti_t* noti )
{
#if 0

	if( Ancs_Data_Len + noti->len >= ANCS_DATA_MAX )
		Ancs_Data_Len = ANCS_DATA_LOCATION;

	osal_memcpy( &Ancs_Data_Buff[Ancs_Data_Len], noti->value, noti->len );
	Ancs_Data_Len += noti->len;

	osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_CML_EVT, BP_READ_ANCS_TIMEOUT );
#endif

	if( Ancs_Data_Len + noti->len >= ANCS_DATA_MAX)
		Ancs_Data_Len = ANCS_DATA_LOCATION;



	switch(g_ucFlag)
	{
		case 0:
			DEBUG_PRINT("NO Noti$\r\n", 0);
			break;

		case 1://¿ªÊ¼½ÓÊÕ¡£
			osal_memcpy( &Ancs_Data_Buff[Ancs_Data_Len], noti->value, noti->len );
			Ancs_Data_Len += noti->len;
			osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_CML_EVT, BP_READ_ANCS_TIMEOUT );
			g_ucFlag = 2;
			break;

		case 2:
			osal_memcpy( &Ancs_Data_Buff[Ancs_Data_Len], noti->value, noti->len );
			osal_start_reload_timer( sensorTag_TaskID, BP_READ_ANCS_CML_EVT, BP_READ_ANCS_TIMEOUT );

			Ancs_Data_Len += noti->len;
			break;
//ÕâÀïÊÇ²»ÊÇÒòÎªANCSµÄÊý¾Ý¶àÓÚÒ»¸ö°üµÄÐèÒª¶à´Î·¢ËÍ£¿£¿
		default:
			break;

	}

	extern uint16 g_uiAddr, g_uiLen;
	g_uiAddr = 0;

	g_uiLen = Ancs_Data_Len;
}

/*********************************************************************
* @fn      Ancs_Data_Explain
*
* @brief   Ancs Data Explain.
*
* @param   *data.
*
* @return  none
*/
void Ancs_Date_Complete( void )
{


	//Ancs_Data_Len = ANCS_DATA_LOCATION;
	HalUARTWrite( HAL_UART_PORT_0, "\r\n", 2 );

	// osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_DATA_EVT, 200 );
}
/*********************************************************************
*********************************************************************/