/**************************************************************************************************
  Filename:       heartrateservice.c
  Revised:        $Date: 2013-08-15 15:28:40 -0700 (Thu, 15 Aug 2013) $
  Revision:       $Revision: 34986 $

  Description:    This file contains the Heart Rate sample service
                  for use with the Heart Rate sample application.

  Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 鎻係 IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "testAlertNotification.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of heart rate measurement value in attribute array
//#define HEARTRATE_MEAS_VALUE_POS            2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// My Alert Notiifcation service
CONST uint8 myAlertServUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(MYALERT_SERV_UUID), HI_UINT16(MYALERT_SERV_UUID)
};

// My Alert Characteristic value Data UUID
CONST uint8 myAlertDataUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(MYALERT_DATA_UUID), HI_UINT16(MYALERT_DATA_UUID)
};

//----------------------------------------------------------------------------

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static myAlertServiceCB_t myALertServiceCB;


/*********************************************************************
 * Profile Attributes - variables
 */

// Heart Rate Service attribute
static CONST gattAttrType_t myALertService = { ATT_BT_UUID_SIZE, myAlertServUUID };

// Heart Rate Measurement Characteristic
// Note characteristic value is not stored here
static uint8 myAlertMeasProps = GATT_PROP_NOTIFY;
static uint8 myAlertMeas = 0;
static gattCharCfg_t myAlertMeasClientCharCfg[GATT_MAX_NUM_CONN];
//#if (SERVICE_DESCRIBE_OPEN==TRUE)
// Heart Rate Measurement Characteristic User Description
static uint8 myAlertDataUserDesp[] = "AlertNotification. Data";
//#endif

// Position of heart rate measurement value in attribute array
#define MYALERT_MEAS_VALUE_POS            2

/*********************************************************************
 * Profile Attributes - Table
 */

gattAttribute_t myAlertAttrTbl[] =
{
	// My Alert Service
	{
		{ ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
		GATT_PERMIT_READ,                         /* permissions */
		0,                                        /* handle */
		(uint8*)& myALertService                /* pValue */
	},

	// My Alert Measurement Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&myAlertMeasProps
	},

	// My Alert Measurement Value
	{
		{ ATT_BT_UUID_SIZE, myAlertDataUUID },
		0,
		0,
		&myAlertMeas
	},

	// My Alert Measurement Client Characteristic Configuration
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8*)& myAlertMeasClientCharCfg
	},
	// Characteristic User Description
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		myAlertDataUserDesp
	},

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 myAlert_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                 uint8* pValue, uint8* pLen, uint16 offset, uint8 maxLen );
static bStatus_t myAlert_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                      uint8* pValue, uint8 len, uint16 offset );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Heart Rate Service Callbacks
CONST gattServiceCBs_t myAlertCBs =
{
	myAlert_ReadAttrCB,  // Read callback function pointer
	myAlert_WriteAttrCB, // Write callback function pointer
	NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_AddService
 *
 * @brief   Initializes the Heart Rate service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t MyAlert_AddService( uint32 services )
{
	uint8 status = SUCCESS;

	// Initialize Client Characteristic Configuration attributes
	GATTServApp_InitCharCfg( INVALID_CONNHANDLE, myAlertMeasClientCharCfg );

	if ( services & MY_ALERT_SERVICE )
	{
		// Register GATT attribute list and CBs with GATT Server App
		status = GATTServApp_RegisterService( myAlertAttrTbl,
		                                      GATT_NUM_ATTRS( myAlertAttrTbl ),
		                                      &myAlertCBs );
	}

	return ( status );
}

/*********************************************************************
 * @fn      HeartRate_Register
 *
 * @brief   Register a callback function with the Heart Rate Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void MyAlert_Register( myAlertServiceCB_t pfnServiceCB )
{
	myALertServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      HeartRate_SetParameter
 *
 * @brief   Set a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t MyAlert_SetParameter( uint8 param, uint8 len, void* value )
{
	bStatus_t ret = SUCCESS;

	switch ( param )
	{
		case MYALERT_MEAS_CHAR_CFG:
			// Need connection handle
			//heartRateMeasClientCharCfg.value = *((uint16*)value);
			break;

		default:
			ret = INVALIDPARAMETER;
			break;
	}

	return ( ret );
}

/*********************************************************************
 * @fn      HeartRate_GetParameter
 *
 * @brief   Get a Heart Rate parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t MyAlert_GetParameter( uint8 param, void* value )
{
	bStatus_t ret = SUCCESS;

	switch ( param )
	{
		case MYALERT_MEAS_CHAR_CFG:
			// Need connection handle
			//*((uint16*)value) = heartRateMeasClientCharCfg.value;
			break;

//    case HEARTRATE_SENS_LOC:
//      *((uint8*)value) = heartRateSensLoc;
//      break;
//
//    case HEARTRATE_COMMAND:
//      *((uint8*)value) = heartRateCommand;
//      break;

		default:
			ret = INVALIDPARAMETER;
			break;
	}

	return ( ret );
}

/*********************************************************************
 * @fn          HeartRate_MeasNotify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t MyAlert_MeasNotify( uint16 connHandle, attHandleValueNoti_t* pNoti )
{
	uint16 value = GATTServApp_ReadCharCfg( connHandle, myAlertMeasClientCharCfg );

	// If notifications enabled
	if ( value & GATT_CLIENT_CFG_NOTIFY )
	{
		// Set the handle
		pNoti->handle = myAlertAttrTbl[MYALERT_MEAS_VALUE_POS].handle;

		// Send the notification
		return GATT_Notification( connHandle, pNoti, FALSE );
	}

	return bleIncorrectMode;
}

/*********************************************************************
 * @fn          heartRate_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 myAlert_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                 uint8* pValue, uint8* pLen, uint16 offset, uint8 maxLen )
{
	bStatus_t status = SUCCESS;

	// Make sure it's not a blob operation (no attributes in the profile are long)
	if ( offset > 0 )
	{
		return ( ATT_ERR_ATTR_NOT_LONG );
	}

	uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

	if (uuid == BODY_SENSOR_LOC_UUID)
	{
		*pLen = 1;
		pValue[0] = *pAttr->pValue;
	}
	else
	{
		status = ATT_ERR_ATTR_NOT_FOUND;
	}

	return ( status );
}

/*********************************************************************
 * @fn      heartRate_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t myAlert_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                      uint8* pValue, uint8 len, uint16 offset )
{
	bStatus_t status = SUCCESS;

	uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

	switch ( uuid )
	{
//    case HEARTRATE_CTRL_PT_UUID:
//      if ( offset > 0 )
//      {
//        status = ATT_ERR_ATTR_NOT_LONG;
//      }
//      else if (len != 1)
//      {
//        status = ATT_ERR_INVALID_VALUE_SIZE;
//      }
//      else if (*pValue != HEARTRATE_COMMAND_ENERGY_EXP)
//      {
//        status = HEARTRATE_ERR_NOT_SUP;
//      }
//      else
//      {
//        *(pAttr->pValue) = pValue[0];
//
//        (*heartRateServiceCB)(HEARTRATE_COMMAND_SET);
//
//      }
//      break;

		case GATT_CLIENT_CHAR_CFG_UUID:
			status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
			         offset, GATT_CLIENT_CFG_NOTIFY );

			if ( status == SUCCESS )
			{
				uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

				(*myALertServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
				                     MYALERT_MEAS_NOTI_DISABLED :
				                     MYALERT_MEAS_NOTI_ENABLED );
			}

			break;

		default:
			status = ATT_ERR_ATTR_NOT_FOUND;
			break;
	}

	return ( status );
}

/*********************************************************************
 * @fn          HeartRate_HandleConnStatusCB
 *
 * @brief       Heart Rate Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
void MyAlert_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
	// Make sure this is not loopback connection
	if ( connHandle != LOOPBACK_CONNHANDLE )
	{
		// Reset Client Char Config if connection has dropped
		if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
		        ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
		          ( !linkDB_Up( connHandle ) ) ) )
		{
			GATTServApp_InitCharCfg( connHandle, myAlertMeasClientCharCfg );
		}
	}
}


/*********************************************************************
*********************************************************************/
