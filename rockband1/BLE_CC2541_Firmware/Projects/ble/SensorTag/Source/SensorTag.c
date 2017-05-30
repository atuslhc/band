/******************************************************************************

 @file  SensorTag.c

 @brief This file contains the Sensor Tag sample application for use with the
        TI Bluetooth Low Energy Protocol Stack.

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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "string.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_keys.h"
#include "hal_i2c.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#if defined ( PLUS_BROADCASTER )
//  #include "peripheralBroadcaster.h"
  #include "peripheral.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

// Services
#include "st_util.h"
#include "devinfoservice-st.h"
#include "battservice.h"
//#include "irtempservice.h"
#include "accelerometerservice.h"
//#include "humidityservice.h"
#include "magnetometerservice.h"
//#include "barometerservice.h"
#include "gyroservice.h"
#if (AA60_PATCH==1)
#include "myalertnotification.h"  //[AlertNotification]
#endif
#if defined FEATURE_TEST
#include "testservice.h"
#endif
#include "simplekeys.h"
#include "ccservice.h"

#include "serialInterface.h"
// Sensor drivers
#include "sensorTag.h"
#include "hal_sensor.h"

#include "hal_irtemp.h"
#include "hal_acc.h"
#include "hal_humi.h"
#include "hal_mag.h"
#include "hal_bar.h"
#include "hal_gyro.h"
#include "gatt_profile_uuid.h"
#include "proxreporter.h"
#include "irtempservice.h"

#if (BOND_PATCH==1)
#include "linkdb.h"
#include "timeapp.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define TEMP_DEFAULT_PERIOD                   1000
#define HUM_DEFAULT_PERIOD                    1000
#define BAR_DEFAULT_PERIOD                    1000
#define MAG_DEFAULT_PERIOD                    2000
#define ACC_DEFAULT_PERIOD                    1000
#define GYRO_DEFAULT_PERIOD                   1000
#if (NOTI2_PATCH==1)
#define SBP_DELAY_EVT_PERIOD                  200
#define DEFAULT_DISCOVERY_DELAY               1000
#endif

// Constants for two-stage reading
#define TEMP_MEAS_DELAY                       275   // Conversion time 250 ms
#define BAR_FSM_PERIOD                        80
#define ACC_FSM_PERIOD                        20
#define HUM_FSM_PERIOD                        20
#define GYRO_STARTUP_TIME                     60    // Start-up time max. 50 ms

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         8

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// Side key bit
#define SK_KEY_SIDE                           0x04

// Test mode bit
#define TEST_MODE_ENABLE                      0x80

// Common values for turning a sensor on and off + config/status
#define ST_CFG_SENSOR_DISABLE                 0x00
#define ST_CFG_SENSOR_ENABLE                  0x01
#define ST_CFG_CALIBRATE                      0x02
#define ST_CFG_ERROR                          0xFF

// System reset
#define ST_SYS_RESET_DELAY                    3000

// How often to perform periodic event
#define SBP_SEND_EVT_PERIOD                       7

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
__no_init XDATA uint8 mac_buf[6]@0x780E;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 sensorTag_TaskID;   // Task ID for task/event processing

/*********************************************************************
 * LOCAL VARIABLES
 */

static bool connected_flag = FALSE;  //whether to try to send data or not from SerialBuffer

uint16 gapConnHandle; //for notification external access.
gaprole_States_t gapProfileState = GAPROLE_INIT;
#if (BOND_PATCH==1)
// Service discovery state
static uint8 timeAppDiscState = DISC_IDLE;

// Service discovery complete
static uint8 timeAppDiscoveryCmpl = FALSE;

// Characteristic configuration state
static uint8 timeAppConfigState = TIMEAPP_CONFIG_START;

// TRUE if pairing started
static uint8 timeAppPairingStarted = FALSE;

// TRUE if discovery postponed due to pairing
static uint8 timeAppDiscPostponed = FALSE;

// Bonded state
static bool timeAppBonded = FALSE;

// Bonded peer address
static uint8 timeAppBondedAddr[B_ADDR_LEN];

// Last connection address
static uint8 lastConnAddr[B_ADDR_LEN] = {0xf, 0xf, 0xf, 0xf, 0xf, 0xe};
#endif

static uint16 buffer_tail = 0;  //last data byte sent from SerialBuffer
uint8 timFlag = 0; //ATUS: copy from CC2540, but seems not specify function.

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0f,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'R', // 'S'
  'o', // 'e'
  'c', // 'n'
  'k', // 's'
  'b', // 'o'
  'a', // 'r'
  'n', // 'T'
  'd', // 'a'
  '1', // 'g'
  '_',
  'x',
  'x',
  'x',
  'x',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
//static uint8 advertData[] =
uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  /*
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  */
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  0x07,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16( 0xAA50 ),//0xAA50  GYROSCOPE_SERV_UUID
  HI_UINT16( 0xAA50 ), //0xAA50
  LO_UINT16( THERMOMETER_SERV_UUID ),//Health Thermometer  0x1809
  HI_UINT16( THERMOMETER_SERV_UUID ), //Health Thermometer  0x1809
  LO_UINT16( HEARTRATE_SERV_UUID ),//Heart Rate 0x180D
  HI_UINT16( HEARTRATE_SERV_UUID ), //Heart Rate 0x180D

  0x08,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x00,
  0x0D,
  0xFF,  //[15]
  0xAA,  //[16]
  0xFF,  //[17]
  0xFF,  //[18]
  0xFF,  //[19]
};

// GAP GATT Attributes
static uint8 attDeviceName[] = "Rockband1_xxxx";

extern uint8 DeviceInfo[16];

// Sensor State Variables
#if (AA00_PATCH==0)
static bool   irTempEnabled = FALSE;
#endif
static bool   magEnabled = FALSE;
static uint8  accConfig = ST_CFG_SENSOR_DISABLE;
//static bool   barEnabled = FALSE;
//static bool   humiEnabled = FALSE;
static bool   gyroEnabled = FALSE;

//static bool   barBusy = FALSE;
//static uint8  humiState = 0;

static bool   sysResetRequest = FALSE;

static uint16 sensorMagPeriod = MAG_DEFAULT_PERIOD;
static uint16 sensorAccPeriod = ACC_DEFAULT_PERIOD;
#if (AA00_PATCH==0)
static uint16 sensorTmpPeriod = TEMP_DEFAULT_PERIOD;
#endif
//static uint16 sensorHumPeriod = HUM_DEFAULT_PERIOD;
//static uint16 sensorBarPeriod = BAR_DEFAULT_PERIOD;
static uint16 sensorGyrPeriod = GYRO_DEFAULT_PERIOD;

static uint8  sensorGyroAxes = 0;
static bool   sensorGyroUpdateAxes = FALSE;
static uint16 selfTestResult = 0;
//static bool   testMode = FALSE;
extern uint8 BLE_State;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

//static void readIrTempData( void );
//static void readHumData( void );
static void readAccData( void );
static void readMagData( void );
//static void readBarData( void );
//static void readBarCalibration( void );
static void readGyroData( void );

//static void barometerChangeCB( uint8 paramID );
static void irTempChangeCB( uint8 paramID );
static void accelChangeCB( uint8 paramID );
//static void humidityChangeCB( uint8 paramID);
static void magnetometerChangeCB( uint8 paramID );
static void gyroChangeCB( uint8 paramID );
#if defined FEATURE_TEST
static void testChangeCB( uint8 paramID );
#endif
static void ccChangeCB( uint8 paramID );
static void ccUpdate( void );
static void gapRolesParamUpdateCB( uint16 connInterval, uint16 connSlaveLatency,
                                  uint16 connTimeout );

//static void resetSensorSetup( void );
//static void sensorTag_HandleKeys( uint8 shift, uint8 keys );
static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint8 value, 
                                     uint8 paramLen );
static void resetCharacteristicValues( void );
//static uint8 sendData(uint16 diff); //AA10 patch.
static void sendData(void);

void sendStatus(void);
void delayUS(uint16 us);

#if (BOND_PATCH==1)
static void timeAppPasscodeCB( uint8* deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs );
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );
#endif
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t sensorTag_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t sensorTag_BondMgrCBs =
{
#if (BOND_PATCH==1)
	timeAppPasscodeCB,
	timeAppPairStateCB
#else
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
#endif
};

// Simple GATT Profile Callbacks
//static sensorCBs_t sensorTag_BarometerCBs =
//{
//  barometerChangeCB,        // Characteristic value change callback
//};


static sensorCBs_t sensorTag_IrTempCBs =
{
  irTempChangeCB,           // Characteristic value change callback
};

static sensorCBs_t sensorTag_AccelCBs =
{
  accelChangeCB,            // Characteristic value change callback
};
/*
static sensorCBs_t sensorTag_HumidCBs =
{
  humidityChangeCB,         // Characteristic value change callback
};
*/
static sensorCBs_t sensorTag_MagnetometerCBs =
{
  magnetometerChangeCB,     // Characteristic value change callback
};

static sensorCBs_t sensorTag_GyroCBs =
{
  gyroChangeCB,             // Characteristic value change callback
};

#if defined FEATURE_TEST
static testCBs_t sensorTag_TestCBs =
{
  testChangeCB,             // Characteristic value change callback
};
#endif

static ccCBs_t sensorTag_ccCBs =
{
 ccChangeCB,               // Characteristic value change callback
};

static gapRolesParamUpdateCB_t paramUpdateCB =
{
  gapRolesParamUpdateCB,
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void setCONN_INTERVAL(uint8 conn_interval);
extern void DevInfoInit(void);
/*********************************************************************
 * @fn      SensorTag_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SensorTag_Init( uint8 task_id )
{
  sensorTag_TaskID = task_id;
  DevInfoInit();
  scanRspData[12] = attDeviceName[10] = HEX_TO_ASCII_H(mac_buf[1]);
  scanRspData[13] = attDeviceName[11] = HEX_TO_ASCII_L(mac_buf[1]);
  scanRspData[14] = attDeviceName[12] = HEX_TO_ASCII_H(mac_buf[0]);
  scanRspData[15] = attDeviceName[13] = HEX_TO_ASCII_L(mac_buf[0]);

    // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  // Setup the GAP Peripheral Role Profile
  {
    // Device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;
    BLE_State = BLE_STATE_ADVERTISING;
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = FALSE; //TRUE; //FALSE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Add services
#if 1 // for new assign service sequence with fix handler.
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
	GATT_RegisterForInd(task_id);
  DevInfo_AddService();                           // Device Information Service
  IRTemp_AddService();                            // IR Temperature Service
  Accel_AddService();                             // Accelerometer Service
  //Humidity_AddService();                          // Humidity Service
  Gyro_AddService();                              // Gyro Service
  Batt_AddService( );     // Battery Service
  //SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
#if (AA60_PATCH==1)
  MyAlert_AddService(); //[AlertNotification]
#endif
#if defined FEATURE_TEST
  Test_AddService();                              // Test Profile
#endif
  ProxReporter_AddService( GATT_ALL_SERVICES );  // Proximity Reporter Profile
  Magnetometer_AddService();                      // Magnetometer Service
  //Barometer_AddService();                         // Barometer Service
  CcService_AddService();                         // Connection Control Service
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

#else //original service sequence.
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  ProxReporter_AddService( GATT_ALL_SERVICES );  // Proximity Reporter Profile
#if (AA60_PATCH==1)
  MyAlert_AddService(); //[AlertNotification]
#endif
  Batt_AddService( );     // Battery Service
  IRTemp_AddService();                            // IR Temperature Service
  Accel_AddService();                             // Accelerometer Service
  //Humidity_AddService();                          // Humidity Service
  Magnetometer_AddService();                      // Magnetometer Service
  //Barometer_AddService();                         // Barometer Service
  Gyro_AddService();                              // Gyro Service
  //SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
#if defined FEATURE_TEST
  Test_AddService();                              // Test Profile
#endif
  CcService_AddService();                         // Connection Control Service
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
  
#endif
  // Setup the Sensor Profile Characteristic Values
  resetCharacteristicValues();

  // Register for all key events - This app will handle all key events
  RegisterForKeys( sensorTag_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // Initialize sensor drivers
  /*
  HALIRTempInit();
  HalHumiInit();
  HalMagInit();
  HalAccInit();
  HalBarInit();
  HalGyroInit();
  */

  // Register callbacks with profile
  VOID IRTemp_RegisterAppCBs( &sensorTag_IrTempCBs );
  VOID Magnetometer_RegisterAppCBs( &sensorTag_MagnetometerCBs );
  VOID Accel_RegisterAppCBs( &sensorTag_AccelCBs );
  //VOID Humidity_RegisterAppCBs( &sensorTag_HumidCBs );
  //VOID Barometer_RegisterAppCBs( &sensorTag_BarometerCBs );
  VOID Gyro_RegisterAppCBs( &sensorTag_GyroCBs );
#if (AA60_PATCH==1)
  VOID MyAlert_Register( &myAlertCB ); //[AlertNotification]
#endif
#if defined FEATURE_TEST  
  VOID Test_RegisterAppCBs( &sensorTag_TestCBs );
#endif
  VOID CcService_RegisterAppCBs( &sensorTag_ccCBs );
  VOID GAPRole_RegisterAppCBs( &paramUpdateCB );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( sensorTag_TaskID, ST_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SensorTag_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SensorTag_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that is not used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( sensorTag_TaskID )) != NULL )
    {
      //sensorTag_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Handle system reset (long press on side key)
  if ( events & ST_SYS_RESET_EVT )
  {
    if (sysResetRequest)
    {
      HAL_SYSTEM_RESET();
    }
    return ( events ^ ST_SYS_RESET_EVT );
  }

  if ( events & ST_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &sensorTag_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &sensorTag_BondMgrCBs );
    
    txEncodeSend(DeviceInfo, DeviceInfo[UART_ID_LEN_POS]); //HalUARTWrite(HAL_UART_PORT_0, DeviceInfo, DeviceInfo[UART_ID_LEN_POS]);
    delayUS(1000);
    sendStatus();

    return ( events ^ ST_START_DEVICE_EVT );
  }

  //////////////////////////
  //    IR TEMPERATURE    //
  //////////////////////////
  /*
  if ( events & ST_IRTEMPERATURE_READ_EVT )
  {
    if ( irTempEnabled )
    {
      if (HalIRTempStatus() == TMP006_DATA_READY)
      {
        readIrTempData();
        osal_start_timerEx( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT, sensorTmpPeriod-TEMP_MEAS_DELAY );
      }
      else if (HalIRTempStatus() == TMP006_OFF)
      {
        HalIRTempTurnOn();
        osal_start_timerEx( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT, TEMP_MEAS_DELAY );
      }
    }
    else
    {
      //Turn off Temperature sensor
      VOID HalIRTempTurnOff();
      VOID resetCharacteristicValue(IRTEMPERATURE_SERV_UUID,SENSOR_DATA,0,IRTEMPERATURE_DATA_LEN);
      VOID resetCharacteristicValue(IRTEMPERATURE_SERV_UUID,SENSOR_CONF,ST_CFG_SENSOR_DISABLE,sizeof ( uint8 ));
    }
    return (events ^ ST_IRTEMPERATURE_READ_EVT);
  }
  */


  //////////////////////////
  //    Accelerometer     //
  //////////////////////////
  if ( events & ST_ACCELEROMETER_SENSOR_EVT )
  {
    if(accConfig != ST_CFG_SENSOR_DISABLE)
    {
      readAccData();
      osal_start_timerEx( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT, sensorAccPeriod );
    }
    else
    {
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_DATA, 0, ACCELEROMETER_DATA_LEN );
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_ACCELEROMETER_SENSOR_EVT);
  }

  //////////////////////////
  //      Humidity        //
  //////////////////////////
  /*
  if ( events & ST_HUMIDITY_SENSOR_EVT )
  {
    if (humiEnabled)
    {
      HalHumiExecMeasurementStep(humiState);
      if (humiState == 2)
      {
        readHumData();
        humiState = 0;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, sensorHumPeriod );
      }
      else
      {
        humiState++;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, HUM_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_DATA, 0, HUMIDITY_DATA_LEN);
      resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_HUMIDITY_SENSOR_EVT);
  }
  */

  //////////////////////////
  //      Magnetometer    //
  //////////////////////////
  if ( events & ST_MAGNETOMETER_SENSOR_EVT )
  {
    if(magEnabled)
    {
      if (HalMagStatus() == MAG3110_DATA_READY)
      {
        readMagData();
      }
      else if (HalMagStatus() == MAG3110_OFF)
      {
        HalMagTurnOn();
      }

      osal_start_timerEx( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT, sensorMagPeriod );
    }
    else
    {
      HalMagTurnOff();
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_MAGNETOMETER_SENSOR_EVT);
  }

  //////////////////////////
  //        Barometer     //
  //////////////////////////
  /*
  if ( events & ST_BAROMETER_SENSOR_EVT )
  {
    if (barEnabled)
    {
      if (barBusy)
      {
        barBusy = FALSE;
        readBarData();
        osal_start_timerEx( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT, sensorBarPeriod );
      }
      else
      {
        barBusy = TRUE;
        HalBarStartMeasurement();
        osal_start_timerEx( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT, BAR_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_DATA, 0, BAROMETER_DATA_LEN);
      resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
      resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_CALB, 0, BAROMETER_CALI_LEN);
    }

    return (events ^ ST_BAROMETER_SENSOR_EVT);
  }
  */

  //////////////////////////
  //      Gyroscope       //
  //////////////////////////
  if ( events & ST_GYROSCOPE_SENSOR_EVT )
  {
    uint8 status;

    status = HalGyroStatus();

    if(gyroEnabled)
    {
      if (status == HAL_GYRO_STOPPED)
      {
        HalGyroSelectAxes(sensorGyroAxes);
        HalGyroTurnOn();
        osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_STARTUP_TIME);
      }
      else
      {
        if(sensorGyroUpdateAxes)
        {
          HalGyroSelectAxes(sensorGyroAxes);
          sensorGyroUpdateAxes = FALSE;
        }

        if (status == HAL_GYRO_DATA_READY)
        {
          readGyroData();
          osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, sensorGyrPeriod - GYRO_STARTUP_TIME);
        }
        else
        {
          // Gyro needs to be activated;
          HalGyroWakeUp();
          osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_STARTUP_TIME);
        }
      }
    }
    else
    {
      HalGyroTurnOff();
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA, 0, GYROSCOPE_DATA_LEN);
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
    }

    return (events ^ ST_GYROSCOPE_SENSOR_EVT);
  }

#if defined ( PLUS_BROADCASTER )
  if ( events & ST_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ ST_ADV_IN_CONNECTION_EVT);
  }
#endif

#if 0  //ATUS: remark not use   
  if ( events & SBP_SEND_EVT )
  {
    // Restart timer
    if ( SBP_SEND_EVT_PERIOD )
    {
      osal_start_timerEx( sensorTag_TaskID, SBP_SEND_EVT, SBP_SEND_EVT_PERIOD );
    }

    // Send data if not caught up and connected
    if ((buffer_tail != serialBufferOffset) && (connected_flag == TRUE))
    {
      //calculate how many bytes can be sent
      uint16 diff = circular_diff(serialBufferOffset, buffer_tail);
      //send data and update tail
      uint8 bytes_sent = sendData(diff);
      buffer_tail = circular_add(buffer_tail,bytes_sent);
      //if we sent anything over-the-air, let host MCU know it can send more bytes
      if (bytes_sent)
      {
        //keep trying to send ACK until it is a success. this may not be the desirable approach
        while(sendAckMessage(bytes_sent));
      }
    }
    return (events ^ SBP_SEND_EVT);
  }
#endif
#if (NOTI_PATCH==1) 
  if ( events & ST_SPI_BLE_EVT ) //ATUS: add for AA10 response
	{
		sendData(); //send notification to RF, the host side will get the response notification.

		//if ((BLE_State == BLE_STATE_CONNECTED)/* && (rxDtatCheck())*/)
		if (send_flage)
		{
			osal_start_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT, BLE_DELAY );
		}
		else
		{
			timFlag = 0;
#if (NOTI2_PATCH==1)
            /* ATUS: add to terminate ST_SPI_BLE_EVT */
            osal_start_reload_timer( sensorTag_TaskID, ST_DELAY_EVT, SBP_DELAY_EVT_PERIOD );
            (void)osal_pwrmgr_task_state(sensorTag_TaskID, PWRMGR_HOLD);
#endif
		}

		return (events ^ ST_SPI_BLE_EVT);
	}

	if ( events & ST_DELAY_EVT )
	{
		send_flage = 0x00;
		osal_stop_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT );
		(void)osal_pwrmgr_task_state(sensorTag_TaskID, PWRMGR_CONSERVE); //check the task need power saving or not
		osal_stop_timerEx( sensorTag_TaskID, ST_DELAY_EVT );
		/*if (SystemBuyDelay < SystemBuyDelaySleep)
		{
		SystemBuyDelay++;
		osal_start_reload_timer( sensorTag_TaskID, ST_DELAY_EVT, SBP_DELAY_EVT_PERIOD );

		}*/
		return (events ^ ST_DELAY_EVT);
	}

#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      sensorTag_test
 *
 * @brief   Run a self-test of the sensor TAG
 *
 * @param   none
 *
 * @return  bitmask of error flags
 */
uint16 sensorTag_test(void)
{
  selfTestResult = HalSensorTest();
  HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
#if defined FEATURE_TEST
  // Write the self-test result to the test service
  Test_SetParameter( TEST_DATA_ATTR, TEST_DATA_LEN, &selfTestResult);
#endif
  return selfTestResult;
}

/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      sensorTag_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
/*
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      sensorTag_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    default:
      // do nothing
      break;
  }
}
*/
/*********************************************************************
 * @fn      sensorTag_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
/*
static void sensorTag_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  VOID shift;  // Intentionally unreferenced parameter

  if (keys & HAL_KEY_SW_1)
  {
    // Reset the system if side key is pressed for more than 3 seconds
    sysResetRequest = TRUE;
    osal_start_timerEx( sensorTag_TaskID, ST_SYS_RESET_EVT, ST_SYS_RESET_DELAY );

    if (!testMode ) // Side key
    {
      // If device is not in a connection, pressing the side key should toggle
      //  advertising on and off
      if ( gapProfileState != GAPROLE_CONNECTED )
      {
        uint8 current_adv_enabled_status;
        uint8 new_adv_enabled_status;

        // Find the current GAP advertising status
        GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

        if( current_adv_enabled_status == FALSE )
        {
          new_adv_enabled_status = TRUE;
        }
        else
        {
          new_adv_enabled_status = FALSE;
        }

        // Change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      }

      if ( gapProfileState == GAPROLE_CONNECTED )
      {
        uint8 adv_enabled = TRUE;

        // Disconnect
        GAPRole_TerminateConnection();
        // Start advertising
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enabled );
      }
    }
    else
    {
      // Test mode
      if ( keys & HAL_KEY_SW_1 ) // Side key
      {
        SK_Keys |= SK_KEY_SIDE;
      }
    }
  }

  if ( keys & HAL_KEY_SW_2 )   // Carbon S2
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_3 )   // Carbon S3
  {
    SK_Keys |= SK_KEY_RIGHT;
  }

  if (!(keys & HAL_KEY_SW_1))
  {
    // Cancel system reset request
    sysResetRequest = FALSE;
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
*/

/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
/*
static void resetSensorSetup (void)
{
  if (HalIRTempStatus()!=TMP006_OFF || irTempEnabled)
  {
    HalIRTempTurnOff();
    irTempEnabled = FALSE;
  }

  if (accConfig != ST_CFG_SENSOR_DISABLE)
  {
    accConfig = ST_CFG_SENSOR_DISABLE;
  }

  if (HalMagStatus()!=MAG3110_OFF || magEnabled)
  {
    HalMagTurnOff();
    magEnabled = FALSE;
  }

  if (gyroEnabled)
  {
    HalGyroTurnOff();
    gyroEnabled = FALSE;
  }

  if (barEnabled)
  {
    HalBarInit();
    barEnabled = FALSE;
  }

  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }

  // Reset internal states
  sensorGyroAxes = 0;
  sensorGyroUpdateAxes = FALSE;
  testMode = FALSE;

  // Reset all characteristics values
  resetCharacteristicValues();
}
*/

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
extern bool advConStatus;
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];
        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;
        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;
      
    case GAPROLE_ADVERTISING: //ATUS turn it on for BLE_State
      //HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );
      if (BLE_State != BLE_STATE_ADVERTISING)
      {
        BLE_State = BLE_STATE_ADVERTISING;
        sendStatus();
      }
      break;
  
    case GAPROLE_CONNECTED:
    case GAPROLE_CONNECTED_ADV:
      //HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );
      ccUpdate();
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
      //setCONN_INTERVAL(0x00);
      if (BLE_State != BLE_STATE_CONNECTED)
      {
        BLE_State = BLE_STATE_CONNECTED;
        sendStatus();
      }
#if defined( PLUS_BROADCASTER )     //atus add
      osal_start_timerEx(sensorTag_TaskID, ST_ADV_IN_CONNECTION_EVT, ADV_IN_CONN_WAIT);
#endif
      break;
/*  
    case GAPROLE_WAITING:
      // Link terminated intentionally: reset all sensors
      //  resetSensorSetup();
      if (BLE_State != BLE_STATE_IDLE)
      {
        BLE_State = BLE_STATE_IDLE;
        sendStatus();
      }
      break;
 */     
/*
    default:
      if (BLE_State != BLE_STATE_IDLE)
      {
        if(advConStatus == true)
        {
          advConStatus = false;
          uint8 new_adv_enabled_status = true ;
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
        }
        //SentAlready = 0;
        BLE_State = BLE_STATE_ADVERTISING; //BLE_STATE_IDLE;
        sendStatus();
      }
      break;
*/
  }
  if(gapProfileState != newState)
  {
    gapProfileState = newState;
    sendStatus();
  }
}

/*********************************************************************
 * @fn      readAccData
 *
 * @brief   Read accelerometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readAccData(void)
{
  uint8 aData[ACCELEROMETER_DATA_LEN];

  if (HalAccRead(aData))
  {
    Accel_SetParameter( SENSOR_DATA, ACCELEROMETER_DATA_LEN, aData);
  }
}

/*********************************************************************
 * @fn      readMagData
 *
 * @brief   Read magnetometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readMagData( void )
{
  uint8 mData[MAGNETOMETER_DATA_LEN];

  if (HalMagRead(mData))
  {
    Magnetometer_SetParameter(SENSOR_DATA, MAGNETOMETER_DATA_LEN, mData);
  }
}

/*********************************************************************
 * @fn      readHumData
 *
 * @brief   Read humidity data
 *
 * @param   none
 *
 * @return  none
 */
/*
static void readHumData(void)
{
  uint8 hData[HUMIDITY_DATA_LEN];

  if (HalHumiReadMeasurement(hData))
  {
    Humidity_SetParameter( SENSOR_DATA, HUMIDITY_DATA_LEN, hData);
  }
}
*/

/*********************************************************************
 * @fn      readBarData
 *
 * @brief   Read barometer data
 *
 * @param   none
 *
 * @return  none
 */
/*
static void readBarData( void )
{
  uint8 bData[BAROMETER_DATA_LEN];

  if (HalBarReadMeasurement(bData))
  {
    Barometer_SetParameter( SENSOR_DATA, BAROMETER_DATA_LEN, bData);
  }
}
*/
/*********************************************************************
 * @fn      readBarCalibration
 *
 * @brief   Read barometer calibration
 *
 * @param   none
 *
 * @return  none
 */
/*
static void readBarCalibration( void )
{
  uint8* cData = osal_mem_alloc(BAROMETER_CALI_LEN);

  if (cData != NULL )
  {
    HalBarReadCalibration(cData);
    Barometer_SetParameter( SENSOR_CALB, BAROMETER_CALI_LEN, cData);
    osal_mem_free(cData);
  }
}
*/
/*********************************************************************
 * @fn      readIrTempData
 *
 * @brief   Read IR temperature data
 *
 * @param   none
 *
 * @return  none
 */
/*
static void readIrTempData( void )
{
  uint8 tData[IRTEMPERATURE_DATA_LEN];

  if (HalIRTempRead(tData))
  {
    IRTemp_SetParameter( SENSOR_DATA, IRTEMPERATURE_DATA_LEN, tData);
  }
}
*/

/*********************************************************************
 * @fn      readGyroData
 *
 * @brief   Read gyroscope data
 *
 * @param   none
 *
 * @return  none
 */
static void readGyroData( void )
{
  uint8 gData[GYROSCOPE_DATA_LEN];

  if (HalGyroRead(gData))
  {
    Gyro_SetParameter( SENSOR_DATA, GYROSCOPE_DATA_LEN, gData);
  }
}

/*********************************************************************
 * @fn      barometerChangeCB
 *
 * @brief   Callback from Barometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
/*
static void barometerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case SENSOR_CONF:
      Barometer_GetParameter( SENSOR_CONF, &newValue );

      switch ( newValue)
      {
      case ST_CFG_SENSOR_DISABLE:
        if (barEnabled)
        {
          barEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT);
        }
        break;

      case ST_CFG_SENSOR_ENABLE:
        if(!barEnabled)
        {
          barEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT);
        }
        break;

      case ST_CFG_CALIBRATE:
        readBarCalibration();
        break;

      default:
        break;
      }
      break;

  case SENSOR_PERI:
      Barometer_GetParameter( SENSOR_PERI, &newValue );
      sensorBarPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // should not get here!
      break;
  }
}
*/
/*********************************************************************
 * @fn      irTempChangeCB
 *
 * @brief   Callback from IR Temperature Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */

static void irTempChangeCB( uint8 paramID )
{
#if (AA00_PATCH==1)
  switch (paramID)
  {
    case SENSOR_CONF:
		BLEBufferSlave[UART_ID_LEN_POS] = (UART_LEN_FOU + irTempLenWrite);
		BLEBufferSlave[UART_ID_CMD_POS] = UART_CMD_APPDATA;
		osal_memcpy(&BLEBufferSlave[UART_ID_DATA_POS], irTempDataWrite, irTempLenWrite);
		MessageType = STATE_COM_DATA;
#if (MODEL_TYPE==1)	//only HEALTHCARE_TYPE apply filter.
		if((BLEBufferSlave[3] != 0x07) && (BLEBufferSlave[3] != 0x06)) //?¨½«Ì½ª±¼??©Mµu«H¡]?¦Û¤_APPºÝ¡^. Atus: check when add.
#endif
			txEncodeSend(BLEBufferSlave, BLEBufferSlave[UART_ID_LEN_POS]);
      break;
    case SENSOR_PERI:
      break;
    default:
      // Should not get here
      break;

  }  
#else
  uint8 newValue;
 
  switch (paramID)
  {
    case SENSOR_CONF:
      IRTemp_GetParameter( SENSOR_CONF, &newValue );
    
      if ( newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (irTempEnabled)
        {
          irTempEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT);
        }
      }
      else if (newValue == ST_CFG_SENSOR_ENABLE)
      {
        if (!irTempEnabled)
        {
          irTempEnabled = TRUE;
          osal_set_event( sensorTag_TaskID,ST_IRTEMPERATURE_READ_EVT);
        }
      }
      break;
    
    case SENSOR_PERI:
      IRTemp_GetParameter( SENSOR_PERI, &newValue );
      sensorTmpPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;
    
    default:
      // Should not get here
      break;
  }
#endif
}

/*********************************************************************
 * @fn      accelChangeCB
 *
 * @brief   Callback from Accelerometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void accelChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Accel_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (accConfig != ST_CFG_SENSOR_DISABLE)
        {
          accConfig = ST_CFG_SENSOR_DISABLE;
          osal_set_event( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT);
        }
      }
      else
      {
        if (accConfig == ST_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
          osal_set_event( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT);
        }
        // Scheduled already, so just change range
        accConfig = newValue;
        HalAccSetRange(accConfig);
      }
      break;

    case SENSOR_PERI:
      Accel_GetParameter( SENSOR_PERI, &newValue );
      sensorAccPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      magnetometerChangeCB
 *
 * @brief   Callback from Magnetometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void magnetometerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Magnetometer_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(magEnabled)
        {
          magEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!magEnabled)
        {
          magEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Magnetometer_GetParameter( SENSOR_PERI, &newValue );
      sensorMagPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      humidityChangeCB
 *
 * @brief   Callback from Humidity Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
/*
static void humidityChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch ( paramID)
  {
  case  SENSOR_CONF:
    Humidity_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == ST_CFG_SENSOR_DISABLE)
    {
      if (humiEnabled)
      {
        humiEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
      }
    }
    
    if ( newValue == ST_CFG_SENSOR_ENABLE )
    {
      if (!humiEnabled)
      {
        humiEnabled = TRUE;
        humiState = 0;
        osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
      }
    }
    break;
    
  case SENSOR_PERI:
    Humidity_GetParameter( SENSOR_PERI, &newValue );
    sensorHumPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
*/
/*********************************************************************
 * @fn      gyroChangeCB
 *
 * @brief   Callback from GyroProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void gyroChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch (paramID) {
  case SENSOR_CONF:
    Gyro_GetParameter( SENSOR_CONF, &newValue );
    
    if (newValue == 0)
    {
      // All three axes off, put sensor to sleep
      if (gyroEnabled)
      {
        gyroEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT);
      }
    }
    else
    {
      // Bitmap tells which axis to enable (bit 0: X, but 1: Y, but 2: Z)
      gyroEnabled = TRUE;
      sensorGyroAxes = newValue & 0x07;
      sensorGyroUpdateAxes = TRUE;
      osal_set_event( sensorTag_TaskID,  ST_GYROSCOPE_SENSOR_EVT);
    }
    break;
    
  case SENSOR_PERI:
    Gyro_GetParameter( SENSOR_PERI, &newValue );
    sensorGyrPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}

#if defined FEATURE_TEST
/*********************************************************************
 * @fn      testChangeCB
 *
 * @brief   Callback from Test indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void testChangeCB( uint8 paramID )
{
  if( paramID == TEST_CONF_ATTR )
  {
    uint8 newValue;

    Test_GetParameter( TEST_CONF_ATTR, &newValue );

    if (newValue & TEST_MODE_ENABLE)
    {
      testMode = TRUE;
    }
    else
    {
      testMode = FALSE;
    }

    if (testMode)
    {
      // Test mode: possible to operate LEDs. Key hits will cause notifications,
      // side key does not influence connection state
      if (newValue & 0x01)
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      }

      if (newValue & 0x02)
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      }
    }
    else
    {
      // Normal mode; make sure LEDs are reset and attribute cleared
      HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      newValue = 0x00;
      Test_SetParameter( TEST_CONF_ATTR, 1, &newValue );
    }
  }
}
#endif

/**********************************************************************
 * @fn      ccUpdate
 *
 * @brief   Update the Connection Control service with the current connection
 *          control settings
 *
 */
static void ccUpdate( void )
{
  uint8 buf[CCSERVICE_CHAR1_LEN];
  uint16 connInterval;
  uint16 connSlaveLatency;
  uint16 connTimeout; 
  
  // Get the connection control data
  GAPRole_GetParameter(GAPROLE_CONN_INTERVAL, &connInterval);
  GAPRole_GetParameter(GAPROLE_SLAVE_LATENCY, &connSlaveLatency);
  GAPRole_GetParameter(GAPROLE_CONN_TIMEOUT, &connTimeout);

  buf[0] = LO_UINT16(connInterval);
  buf[1] = HI_UINT16(connInterval);
  buf[2] = LO_UINT16(connSlaveLatency);
  buf[3] = HI_UINT16(connSlaveLatency);
  buf[4] = LO_UINT16(connTimeout);
  buf[5] = HI_UINT16(connTimeout);

  CcService_SetParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}


/*********************************************************************
 * @fn      ccChangeCB
 *
 * @brief   Callback from Connection Control indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void ccChangeCB( uint8 paramID )
{
  // CCSERVICE_CHAR1: read & notify only

  // CCSERVICE_CHAR: requested connection parameters
  if( paramID == CCSERVICE_CHAR2 )
  {
    uint8 buf[CCSERVICE_CHAR2_LEN];
    uint16 minConnInterval;
    uint16 maxConnInterval;
    uint16 slaveLatency;
    uint16 timeoutMultiplier;

    CcService_GetParameter( CCSERVICE_CHAR2, buf );

    minConnInterval = BUILD_UINT16(buf[0],buf[1]);
    maxConnInterval = BUILD_UINT16(buf[2],buf[3]);
    slaveLatency = BUILD_UINT16(buf[4],buf[5]);
    timeoutMultiplier = BUILD_UINT16(buf[6],buf[7]);

    // Update connection parameters
    GAPRole_SendUpdateParam( minConnInterval, maxConnInterval, slaveLatency, 
                            timeoutMultiplier, GAPROLE_NO_ACTION);
  }

  // CCSERVICE_CHAR3: Disconnect request
  if( paramID == CCSERVICE_CHAR3 )
  {
    // Any change in the value will terminate the connection
    GAPRole_TerminateConnection();
  }
}


/*********************************************************************
 * @fn      gapRolesParamUpdateCB
 *
 * @brief   Called when connection parameters are updates
 *
 * @param   connInterval - new connection interval
 *
 * @param   connSlaveLatency - new slave latency
 *
 * @param   connTimeout - new connection timeout
 *
 * @return  none
*/
static void gapRolesParamUpdateCB( uint16 connInterval, uint16 connSlaveLatency,
                                  uint16 connTimeout )
{
  uint8 buf[CCSERVICE_CHAR1_LEN];

  buf[0] = LO_UINT16(connInterval);
  buf[1] = HI_UINT16(connInterval);
  buf[2] = LO_UINT16(connSlaveLatency);
  buf[3] = HI_UINT16(connSlaveLatency);
  buf[4] = LO_UINT16(connTimeout);
  buf[5] = HI_UINT16(connTimeout);
  CcService_SetParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}


/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, 
                                     uint8 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

  switch(servUuid)
  {
    /*
    case IRTEMPERATURE_SERV_UUID:
      IRTemp_SetParameter( paramID, paramLen, pData);
      break;
    */

    case ACCELEROMETER_SERV_UUID:
      Accel_SetParameter( paramID, paramLen, pData);
      break;

    case MAGNETOMETER_SERV_UUID:
      Magnetometer_SetParameter( paramID, paramLen, pData);
      break;
    /*
    case HUMIDITY_SERV_UUID:
      Humidity_SetParameter( paramID, paramLen, pData);
      break;
    */

    //case BAROMETER_SERV_UUID:
    //  Barometer_SetParameter( paramID, paramLen, pData);
    //  break;

    case GYROSCOPE_SERV_UUID:
      Gyro_SetParameter( paramID, paramLen, pData);
      break;

    default:
      // Should not get here
      break;
  }

  osal_mem_free(pData);
}

/*********************************************************************
 * @fn      resetCharacteristicValues
 *
 * @brief   Initialize all the characteristic values
 *
 * @return  none
 */
static void resetCharacteristicValues( void )
{
  //resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_DATA, 0, IRTEMPERATURE_DATA_LEN);
  //resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  //resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_PERI, TEMP_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_DATA, 0, ACCELEROMETER_DATA_LEN );
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_PERI, ACC_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  //resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_DATA, 0, HUMIDITY_DATA_LEN);
  //resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  //resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_PERI, HUM_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_PERI, MAG_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  //resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_DATA, 0, BAROMETER_DATA_LEN);
  //resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  //resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_PERI, BAR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA, 0, GYROSCOPE_DATA_LEN);
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_PERI, GYRO_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
}

#if 0 //ATUS: remark it because not use and conflict define sendData()
/*********************************************************************
 * @fn      sendData
 *
 * @brief   parse available serial data and send max possible amount over the air
 *
 * @param   diff - how many bytes can be sent
 *
 * @return  none
 */

static uint8 sendData( uint16 diff )
{
  //can send max 4 packets per connection interval
  uint8 packets_sent = 0;
  //ensure queue of notification is successful
  bool send_error = FALSE;
  //return value to update tail and send ack to msp
  uint8 bytes_sent = 0;
  
  attHandleValueNoti_t noti;
  uint16 len;
  //dummy handle
  noti.handle = 0x2E;  
  
  //counter
  uint8 i;
  
  while ((packets_sent < 4) &&  (diff >= 20) && (send_error == FALSE))
  {  
    //send 20 bytes
    noti.len = 20;
    noti.pValue = (uint8 *)GATT_bm_alloc( 0, ATT_HANDLE_VALUE_NOTI,
                                        GATT_MAX_MTU, &len );
   if ( noti.pValue != NULL )
   {
      for (i = 0; i < 20; i++)
      {
        noti.pValue[i] = serialBuffer[circular_add(buffer_tail , bytes_sent+i)];
      }
      //connection handle currently hardcoded
      if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
      {
        bytes_sent += 20;
        diff -= 20;
        packets_sent++;
      }
      else
      {
        send_error = TRUE;
        GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
      }
    }
  }
  //send remaining bytes  
  if ((packets_sent < 4) && (diff > 0) && (send_error == FALSE))
  {
    noti.len = diff;
    noti.pValue = (uint8 *)GATT_bm_alloc( 0, ATT_HANDLE_VALUE_NOTI,
                                        GATT_MAX_MTU, &len );
    if ( noti.pValue != NULL )
    {
      for (i = 0; i < diff; i++)
      {
      noti.pValue[i] = serialBuffer[circular_add(buffer_tail, bytes_sent + i)];
      }
        //connection handle currently hardcoded
      if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
      {
        bytes_sent += i;
        diff -= i;//amount of data sent
      }
      else
      {
        send_error = TRUE;
        GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
      }
    }
  }
  return bytes_sent;
}
#endif

void sendStatus(void)
{
  uint8 status[7] = {UART_DATA_START, 7, UART_CMD_INFOR, UART_TYPE_INFOR, 0x0, 0x0, UART_DATA_STOP};
  status[4] = BLE_State; //gapProfileState; //
  status[5] = gapProfileState;
  MessageType = STATE_CMD_INFOR;
  txEncodeSend(status, 7); //HalUARTWrite(HAL_UART_PORT_0, status, 7);
}

void delayUS(uint16 us)
{
	while(us--)
	{
		/* 32 NOPs == 1 usecs */
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}
}

#if (NOTI_PATCH==1)
/*******************************************************
***	Desc¡GSend the notification Data to Host. (Ex, Gather Data, response notification).
***	Params:
***	Return: none
*******************************************************/
static void sendData(void )
{
#define CHANNEL1_LOCAL  2
#define CHANNEL2_LOCAL  2
#define CHANNEL3_LOCAL  2
#define CHANNEL5_LOCAL  2
#define CHANNEL6_LOCAL  2
#define CHANNEL7_LOCAL  2
    extern gattAttribute_t accelAttrTbl[];  //AA10:
	extern gattAttribute_t irTempAttrTbl[];  //AA00:
	extern gattAttribute_t heartRateAttrTbl[]; //108D:
	extern gattAttribute_t sensorGyroscopeAttrTbl[]; //AA50:
	extern gattAttribute_t thermometerAttrTbl[]; //1809:
	//  extern gattAttribute_t runningAttrTbl[];
	uint16 handle_tab;



	uint8 i = 0;
	uint8 id = 0;
	//static uint8 sendDataBuf_tab = 0;

	for (i = 0; i < 3; i++)
	{
		if ((id = rxDtatCheck()) != 0)
		{
			id--;

			switch(sendDataBuf[id][sendDataId])
			{
				case 1:
					handle_tab = irTempAttrTbl[CHANNEL1_LOCAL].handle;
					break;

				case 2:
					handle_tab = accelAttrTbl[CHANNEL2_LOCAL].handle;
					break;

				case 3:
					//handle_tab = heartRateAttrTbl[CHANNEL3_LOCAL].handle;
                    handle_tab = 0;
					break;

				case 4:
					handle_tab = 0;
					break;

				case 5:
					handle_tab = sensorGyroscopeAttrTbl[CHANNEL5_LOCAL].handle;
					break;

				case 6:
					//handle_tab = thermometerAttrTbl[CHANNEL6_LOCAL].handle;
                    handle_tab = 0;
					break;

				case 7:
					handle_tab = 0; //runningAttrTbl[CHANNEL7_LOCAL].handle;
					break;
			}

			attHandleValueNoti_t nData;//handle notification structure, the att.h is changed value[] to *pValue.
			nData.len = sendDataBuf[id][sendDataLen];
			nData.handle = handle_tab;
			//osal_memcpy( &nData.value, &sendDataBuf[id][sendDataData], sendDataBuf[id][sendDataLen]);
#if (NOTI2_PATCH==1)
            uint16 len;
            nData.pValue = (uint8 *)GATT_bm_alloc( 0, ATT_HANDLE_VALUE_NOTI, GATT_MAX_MTU, &len );  //The NOTI GATT_MAX_MTU len=20.
            if (nData.pValue!=NULL)
              osal_memcpy(nData.pValue, &sendDataBuf[id][sendDataData], nData.len);
#else
            nData.pValue = &sendDataBuf[id][sendDataData];
#endif
			if (GATT_Notification( 0, &nData, FALSE ) == SUCCESS) //send the notification to BLE.
			{
				sendDataBuf[id][sendDataLen] = 0;
				BLEDtatMOV();
			}
			else
			{
#if (NOTI2_PATCH==1)
               GATT_bm_free( (gattMsg_t *)&nData, ATT_HANDLE_VALUE_NOTI );
#endif
				return;
			}
		}
		else  //rxDtatCheck()) == 0
		{
#if (NOTI2_PATCH==1)
            send_flage = 0;
#endif
			return;
		}
	}
}
/*******************************************************
***	Desc¡GCheck the sendDataBuff[][] available data want to send
***	Params: None
***	Return: 0: none; not 0 the array index+1
*******************************************************/
uint8 rxDtatCheck(void)
{
	uint8 i = 0;

	for (i = 0; i < sendDataBufSiz; i++)
	{
		if (sendDataBuf[i][sendDataLen] != 0)
		{
			return i + 1;
		}
	}

	return 0;
}

/*******************************************************
***	Desc: Scroll one line buffer.
***	Params: none
***	Return: none
*******************************************************/
void BLEDtatMOV(void)
{
	uint8 i = 0;

	for (i = 0; i < (sendDataBufSiz - 1); i++)
	{
		memcpy(&sendDataBuf[i][0], &sendDataBuf[i + 1][0], 24);
		sendDataBuf[i + 1][sendDataLen] = 0;
	}
}

/*******************************************************
***	Desc: data move 24 bytes.
***	Params: target data address
***	Return: None
*******************************************************/
void rxDtatMov(uint8* data)
{
	memcpy(data, &data[24], 24);
}
#endif
/*********************************************************************
*********************************************************************/
#if (BOND_PATCH==1)
/*********************************************************************
* @fn      pairStateCB
*
* @brief   Pairing state callback.
*
* @return  none
*/
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
	if ( state == GAPBOND_PAIRING_STATE_STARTED )
	{
		timeAppPairingStarted = TRUE;
	}
	else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
	{
		timeAppPairingStarted = FALSE;

		DEBUG_PRINT("Paired\r\n", 0);

		if ( status == SUCCESS )
		{
			linkDBItem_t*  pItem;

			if ( (pItem = linkDB_Find( gapConnHandle )) != NULL )
			{
				// Store bonding state of pairing
				timeAppBonded = ( (pItem->stateFlags & LINK_BOUND) == LINK_BOUND );

				if ( timeAppBonded )
				{
					osal_memcpy( timeAppBondedAddr, pItem->addr, B_ADDR_LEN );
				}
			}

			// If discovery was postponed start discovery
			//if ( timeAppDiscPostponed && timeAppDiscoveryCmpl == FALSE )
			if ( timeAppDiscPostponed )  //ATUSDBG: try fix hardly connect by nRF 
			{
				timeAppDiscPostponed = FALSE;
				DEBUG_PRINT("Discovery!!!\r\n", 0) ;
				osal_set_event( sensorTag_TaskID, BP_START_DISCOVERY_EVT );
			}
		}
	}
}

/*********************************************************************
* @fn      timeAppPasscodeCB
*
* @brief   Passcode callback.
*
* @return  none
*/
static void timeAppPasscodeCB( uint8* deviceAddr, uint16 connectionHandle,
                               uint8 uiInputs, uint8 uiOutputs )
{
	// Send passcode response
	GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, 0 );
}

/*********************************************************************
* @fn      timeAppDisconnected
*
* @brief   Handle disconnect.
*
* @return  none
*/
static void timeAppDisconnected( void )
{
	// Initialize state variables
	timeAppDiscState = DISC_IDLE;
	timeAppPairingStarted = FALSE;
	timeAppDiscPostponed = FALSE;
}
#endif
