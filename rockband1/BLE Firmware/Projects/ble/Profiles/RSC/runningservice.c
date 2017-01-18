/**************************************************************************************************
  Filename:       runningservice.c
  Revised:        $Date: 2013-04-04 15:28:09 -0700 (Thu, 04 Apr 2013) $
  Revision:       $Revision: 33765 $

  Description:    This file contains the Running Speed and Cadence (RSC) service
                  for use with the RunningSensor sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "runningservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Running Service Task Events
#define RSC_CMD_IND_SEND_EVT   0x0001

#define RSC_MEAS_VALUE_POS     2
#define RSC_MEAS_CFG_POS       3
#define RSC_COMMAND_VALUE_POS  9
#define RSC_COMMAND_CFG_POS    10
#define COMMAND_IND_LENGTH     2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// RSC service
CONST uint8 runningServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_SERV_UUID), HI_UINT16(RSC_SERV_UUID)
};

// RSC measurement characteristic
CONST uint8 runningMeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_MEAS_UUID), HI_UINT16(RSC_MEAS_UUID)
};

// RSC feature characteristic
CONST uint8 runningFeatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_FEATURE_UUID), HI_UINT16(RSC_FEATURE_UUID)
};

// RSC sensor location characteristic
CONST uint8 runningSensLocUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_SENS_LOC_UUID), HI_UINT16(RSC_SENS_LOC_UUID)
};

// RSC command characteristic
CONST uint8 runningCommandUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_COMMAND_UUID), HI_UINT16(RSC_COMMAND_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static runningServiceCB_t runningServiceCB = NULL;

static bool scOpInProgress = FALSE;

// Variables used in RSC command processing

/*********************************************************************
 * Profile Attributes - variables
 */

// TaskID


// RSC Service attribute
static CONST gattAttrType_t runningService = { ATT_BT_UUID_SIZE, runningServUUID };

// Available sensor locations

// Running Measurement Characteristic
// Note characteristic value is not stored here
static uint8 runningMeasProps = GATT_PROP_NOTIFY;
static uint8 runningMeas = 0;
static gattCharCfg_t runningMeasClientCharCfg[GATT_MAX_NUM_CONN];

// Feature Characteristic
static uint8 runningFeatureProps = GATT_PROP_READ;
static uint16 runningFeatures = RSC_TOTAL_DIST_SUPP|RSC_WALKING_RUNNING_SUPP|RSC_SENSOR_CALIB_SUPP;

// Sensor Location Characteristic

static uint8 runningSensLoc = RSC_NO_SENSOR_LOC;

// Command Characteristic
static uint8 runningCommand = 0;
static gattCharCfg_t runningCommandClientCharCfg[GATT_MAX_NUM_CONN];

/*********************************************************************
 * Profile Attributes - Table
 */

gattAttribute_t runningAttrTbl[] =
{
  // RSC Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&runningService                  /* pValue */
  },

    // RSC Measurement Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &runningMeasProps
    },

      // Measurement Value
      {
        { ATT_BT_UUID_SIZE, runningMeasUUID },
        0,
        0,
        &runningMeas
      },

      // Measurement Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &runningMeasClientCharCfg
      },

    // RSC Feature Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &runningFeatureProps
    },

      // Feature Value
      {
        { ATT_BT_UUID_SIZE, runningFeatureUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)&runningFeatures
      },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 running_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t running_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// RSC Service Callbacks
CONST gattServiceCBs_t runningCBs =
{
  running_ReadAttrCB,  // Read callback function pointer
  running_WriteAttrCB, // Write callback function pointer
  NULL                 // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */



/*********************************************************************
 * @fn      Running_AddService
 *
 * @brief   Initializes the RSC service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Running_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, runningMeasClientCharCfg );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, runningCommandClientCharCfg);

  if ( services & RUNNING_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( runningAttrTbl,
                                          GATT_NUM_ATTRS( runningAttrTbl ),
                                          &runningCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      Running_Register
 *
 * @brief   Register a callback function with the RSC Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void Running_Register( runningServiceCB_t pfnServiceCB )
{
  runningServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Running_SetParameter
 *
 * @brief   Set a RSC parameter.
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
bStatus_t Running_SetParameter( uint8 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case RSC_SENS_LOC:
    {
      runningSensLoc = *((uint8*)pValue);
    }
    break;

    case RSC_FEATURE:
    {
      runningFeatures = *((uint16*)pValue);
    }
    break;

    case RSC_AVAIL_SENS_LOCS:

    break;

    default:
      ret = INVALIDPARAMETER;
    break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      Running_GetParameter
 *
 * @brief   Get a RSC parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Running_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case RSC_FEATURE:
      *((uint16*)value) = runningFeatures;
      break;

    case RSC_SENS_LOC:
      *((uint8*)value) = runningSensLoc;
      break;

    case RSC_COMMAND:
      *((uint8*)value) = runningCommand;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          Running_MeasNotify
 *
 * @brief       Send a notification containing a RSC
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t Running_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, runningMeasClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = runningAttrTbl[RSC_MEAS_VALUE_POS].handle;

    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          Running_HandleConnStatusCB
 *
 * @brief       RSC Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
void Running_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, runningMeasClientCharCfg );
      GATTServApp_InitCharCfg( connHandle, runningCommandClientCharCfg );
    }
  }
}

/*********************************************************************
 * @fn          running_ReadAttrCB
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
static uint8 running_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                         uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch ( uuid )
  {
    // Read Sensor Location
    case RSC_SENS_LOC_UUID:
    {
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
    }
    break;

    // Read Running Feature List
    case RSC_FEATURE_UUID:
    {
      *pLen = 2;
      pValue[0] = LO_UINT16( runningFeatures );
      pValue[1] = HI_UINT16( runningFeatures );
    }
    break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
    break;
  }

  // Notify app
  if ( runningServiceCB != NULL )
  {
    VOID (*runningServiceCB)( RSC_READ_ATTR);
  }

  return ( status );
}

/*********************************************************************
 * @fn      running_WriteAttrCB
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
static bStatus_t running_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                      uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if ( offset > 0 )
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  switch ( uuid )
  {
    case RSC_COMMAND_UUID:
      // Make sure Control Point Cfg is not already in progress
      if ( scOpInProgress == TRUE )
      {
        status = RSC_ERR_PROC_IN_PROGRESS;
      }
      // Make sure Control Point Cfg is configured for Indications
      else if ( (runningCommandClientCharCfg[connHandle].value & GATT_CLIENT_CFG_INDICATE) == FALSE )
      {
         status = RSC_ERR_CCC_IMPROPER_CFG;
      }
      else
      {
        // Process RSC command
     
      }
      break;

    // For Measure and Commands CCC
    case GATT_CLIENT_CHAR_CFG_UUID:
//      if ( pAttr->handle == runningAttrTbl[RSC_COMMAND_CFG_POS].handle )
//      {
//        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
//                                                 offset, GATT_CLIENT_CFG_INDICATE );
//        // Notify app
//        if ( runningServiceCB != NULL )
//        {
//          VOID (*runningServiceCB)( RSC_WRITE_ATTR, NULL );
//        }
//      }
//      else 
      if ( pAttr->handle == runningAttrTbl[RSC_MEAS_CFG_POS].handle )
      {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS )
        {
          // Notify app
          if ( runningServiceCB != NULL )
          {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

            VOID (*runningServiceCB)( ((charCfg == GATT_CFG_NO_OPERATION) ?
                                        RSC_MEAS_NOTI_DISABLED :
                                        RSC_MEAS_NOTI_ENABLED ) );
          }
        }
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/