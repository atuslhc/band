#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "immediateAlert.h"


extern void ImmediateAlertCB(uint8 ucValue);

// ImmediateAlertServUUID
CONST uint8 immediateAlertServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IMMEDIATE_ALERT_SERVICE_UUID), HI_UINT16(IMMEDIATE_ALERT_SERVICE_UUID)
};

// ImmediateAlertUUID
CONST uint8 immediateAlertUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IMMEDIATE_ALERT_UUID), HI_UINT16(IMMEDIATE_ALERT_UUID)
};

// Battery Service attribute
static CONST gattAttrType_t immediateAlertService = { ATT_BT_UUID_SIZE, immediateAlertServUUID };
// Battery level characteristic
static uint8 immediateAlertProps = GATT_PROP_WRITE_NO_RSP;//GATT_PROP_READ | GATT_PROP_NOTIFY;
uint8 ucImmediateAlert=0;
#if (SERVICE_DESCRIBE_OPEN==TRUE)
static uint8 ImmediatAlertUserDesp[]="ImmediatAlert";
#endif

static gattAttribute_t immediatAlertAttrTbl[] =
{
  // Battery Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&immediateAlertService                     /* pValue */
  },

    // Battery Level Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &immediateAlertProps
    },

      // Battery Level Value
      {
        { ATT_BT_UUID_SIZE, immediateAlertUUID },
        GATT_PERMIT_WRITE,
        0,
        &ucImmediateAlert
      },
#if (SERVICE_DESCRIBE_OPEN==TRUE)
            // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        ImmediatAlertUserDesp
      },
#endif
};

static bStatus_t ImmediateAlertWriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset );
static gattCharCfg_t ImmediateAlertClientCharCfg[GATT_MAX_NUM_CONN];
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Battery Service Callbacks
CONST gattServiceCBs_t ImmediateAlertCBs =
{
  NULL,  // Read callback function pointer
  ImmediateAlertWriteAttrCB, // Write callback function pointer
  NULL             // Authorization callback function pointer
};

static bStatus_t ImmediateAlertWriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
    case IMMEDIATE_ALERT_UUID:

      if ( status == SUCCESS )
      {
        uint8 ucValue=pValue[0];
        ImmediateAlertCB(ucValue);
//        uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

//        if ( battServiceCB )
//        {
//          (*battServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
//                            BATT_LEVEL_NOTI_DISABLED :
//                            BATT_LEVEL_NOTI_ENABLED);
//        }
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}
/*********************************************************************
 * @fn      Batt_AddService
 *
 * @brief   Initializes the Battery Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t ImmediateAlert_AddService( void )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ImmediateAlertClientCharCfg );

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( immediatAlertAttrTbl,
                                        GATT_NUM_ATTRS( immediatAlertAttrTbl ),
                                        &ImmediateAlertCBs );

  return ( status );
}

