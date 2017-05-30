/**************************************************************************************************
Filename:       SensorTag.c
Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
Revision:       $Revision: 33575 $

Description:    This file contains the Sensor Tag sample application
for use with the TI Bluetooth Low Energy Protocol Stack.

Copyright 2012-2013  Texas Instruments Incorporated. All rights reserved.

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
PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "OSAL_PwrMgr.h"
#include "string.h"
#include "SensorTagUser.h"
#include "hal_flash.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_mcu.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "CC2540_bitdef.h"
#include "CC2540.h"     //[BG036-1] add.

#if defined ( PLUS_BROADCASTER )
#include "peripheralBroadcaster.h"
#else
#include "peripheral.h"
#endif

#include "gapbondmgr.h"

#if defined FEATURE_OAD
#include "oad.h"
#include "oad_target.h"
#endif

// Services
#include "devinfoservice-st.h"
#include "irtempservice.h"
#include "accelerometerservice.h"
#include "humidityservice.h"
#include "magnetometerservice.h"
#include "barometerservice.h"
#include "gyroservice.h"
#include "thermometerservice.h"
#include "heartrateservice.h"
#include "runningservice.h"
//#include "testservice.h"
#include "immediateAlert.h"
#include "testAlertNotification.h" //2015年9月2日09:04:24新加的

#include "simplekeys.h"

#include "battservice.h"
#include "heartrateservice.h"
#include "serial2Service.h"

// Sensor drivers
#include "SensorTag.h"

#include "SerialApp.h"
#include "serial2service.h"
#include "gatt_profile_uuid.h"
/*
#include "hal_irtemp.h"
#include "hal_acc.h"
#include "hal_humi.h"
#include "hal_mag.h"
#include "hal_bar.h"
#include "hal_gyro.h"
*/
#include "linkdb.h"
#include "timeapp.h"

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
#define SBP_DELAY_EVT_PERIOD                  200
#define DEFAULT_DISCOVERY_DELAY               1000
// Constants for two-stage reading
#define TEMP_MEAS_DELAY                       275   // Conversion time 250 ms
#define BAR_FSM_PERIOD                        80
#define ACC_FSM_PERIOD                        20
#define HUM_FSM_PERIOD                        20

#if	1
#define SBP_BURST_EVT_PERIOD				20
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          (160*20)//2014-03-17 

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// General discoverable mode advertises indefinitely
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED//受限制
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL//受限制

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8//80 

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     16//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          600


#else
#define SBP_BURST_EVT_PERIOD				1000
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160*20

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// General discoverable mode advertises indefinitely
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED//受限制
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL//受限制

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     300

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     400

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          0x0C79
#endif

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
#define sendDataBufSiz		6
uint8 sendDataBuf[sendDataBufSiz][24]; //[i][0]:ID, [i][1]:len, ..[i][21] data. 
//第一字节放ID，第二个字节放长度，后面20个字节放数据
//uint8 sendDataBuf_tab = 0;
#define sendDataLen	1
#define sendDataId	0
#define sendDataData	2
void rxDtatSave(uint8* data, uint8 len, uint8 id);
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
__no_init XDATA uint8 mac_buf[6]@0x780E;
__no_init XDATA uint8 Dev_Infor_ChipID@0x624A;
__no_init XDATA uint8 Dev_Infor_Cap@0x6276;
/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
typedef struct
{
	bool advConStatus;//广播连接间隔是否正在改变。因为改变广播间隔，要先关广播，修改，再打开广播（在关广播完成才能操作）。
} stBleStateFlag_Type; //BLE状态标志位

stBleStateFlag_Type g_stBleStateFlag =
{
	.advConStatus = false,
};


uint8 g_ucANCSFlag = 0;
uint8 sensorTag_TaskID;   // Task ID for internal task/event processing
uint16 gapConnHandle;

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

static bool connectedToLastAddress = false;


static gaprole_States_t gapProfileState = GAPROLE_INIT;
static void gyroChangeCB( uint8 paramID );
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
	// complete name
#if (VENDOR_TYPE==1)
	0x0e,   // length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	'A',   // i   accuLife
	'c',   // h
	'c',   // e
	'u',   // a
	'l',   // l
	'i',   // t
	'f',   // h
	'e',   // T
	'_',
	'x',
	'x',
	'x',
	'x',
	//  'r',   // r
	//  'a',   // a
	//  'c',   // c
	//  'k',   // k
#elif (VENDOR_TYPE==2)
	0x0c,   // length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	'A',   // i   aicare accuLife
	'i',   // h
	'C',   // e
	'a',   // a
	'r',   // l
	'e',   // t
	'_',
	'x',
	'x',
	'x',
	'x',
#else
	0x0e,   // length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	'A',   // i   accuLife
	'c',   // h
	'c',   // e
	'u',   // a
	'l',   // l
	'i',   // t
	'f',   // h
	'e',   // T
	'_',
	'x',
	'x',
	'x',
	'x',
#endif
	// connection interval range
	0x05,   // length of this data
	GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
	LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100 ms
	HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
	LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
	HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

	// Tx power level
	0x02,   // length of this data
	GAP_ADTYPE_POWER_LEVEL,
	4       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
#if 0
static uint8 advertData[] =
{
	// Flags; this sets the device to use limited discoverable
	// mode (advertises for 30 seconds at a time) instead of general
	// discoverable mode (advertises indefinitely)
	0x02,   // length of this data
	GAP_ADTYPE_FLAGS,
	DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};
#else

//注意：下面的数据长度不能设置错误，如果设置的数据长度多余实际的数据个数，那么广播出去的包的内容就会随意变动。
uint8  advertData[] =
{
	//这个广播数据，会被其他BLE设备收到吗？？？广播数据可以再添加吗
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

#if (MODEL_TYPE==1)
	0x08,
	GAP_ADTYPE_MANUFACTURER_SPECIFIC,
	0x00,
	0x0D,
	0xFF,
	0xAA,
	0xFF,
	0xFF,
	0xFF,
#elif (MODEL_TYPE==2)
	0x06,
	GAP_ADTYPE_MANUFACTURER_SPECIFIC,
	0x00,
	0x0D,
	0xAA,
	0xFF,
	0xFF,
#endif
};
#endif

// GAP GATT Attributes
#if (VENDOR_TYPE==1)
static uint8 attDeviceName[] = "Acculife_xxxx";
#elif (VENDOR_TYPE==2)
static uint8 attDeviceName[] = "AiCare_xxxx";
#else
static uint8 attDeviceName[] = "Acculife_xxxx";
#endif

// Sensor State Variables
/*
static bool   irTempEnabled = FALSE;
static bool   magEnabled = FALSE;
static bool   accEnabled = FALSE;
static bool   barEnabled = FALSE;
static bool   humiEnabled = FALSE;
static bool   gyroEnabled = FALSE;
*/
/*
static bool   barBusy = FALSE;
static uint8  humiState = 0;
*/
static bool   sysResetRequest = FALSE;
/*
static uint16 sensorMagPeriod = MAG_DEFAULT_PERIOD;
static uint16 sensorAccPeriod = ACC_DEFAULT_PERIOD;
static uint8  sensorGyroAxes = 0;
static bool   sensorGyroUpdateAxes = FALSE;*/
//static uint16 selfTestResult = 0;


/*********************************************************************
* LOCAL FUNCTIONS
*/
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void sensorTag_ProcessGATTMsg( gattMsgEvent_t* pMsg );

static void timeAppPasscodeCB( uint8* deviceAddr, uint16 connectionHandle,
                               uint8 uiInputs, uint8 uiOutputs );
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );


void readIrTempData( void );

void readAccData( void );

void readGyroData( void );
void readHeartRateData(void);
void readBATTData(void);

//static void barometerChangeCB( uint8 paramID );
static void irTempChangeCB( uint8 paramID );
static void accelChangeCB( uint8 paramID );

void SEND_BLE_STA(void);
void INT_HOST(void);
static void sendData(void );

//static void resetSensorSetup( void );
//void sensorTag_HandleKeys( uint8 shift, uint8 keys );
//static void resetCharacteristicValue(uint16 servID, uint8 paramID, uint8 value, uint8 paramLen);
//static void resetCharacteristicValues();

void SPI_Analyze(void);
uint8 rxDtatCheck(void);
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
#if 1
	timeAppPasscodeCB,
	timeAppPairStateCB
#else //test PC Dongle pair must or not.
    NULL,
    NULL
#endif
};

// Simple GATT Profile Callbacks

static irTempCBs_t sensorTag_IrTempCBs =
{
	irTempChangeCB,           // Characteristic value change callback. This is download firmware channgel.
};

static accelCBs_t sensorTag_AccelCBs =
{
	accelChangeCB,            // Characteristic value change callback
};



//#define IMBUSY P0 &= ~BIT5
//#define IMFREE P0 |= BIT5

#define INT_RES P0 &= ~BIT4
#define INT_SET P0 |= BIT4
/***********************************************************************************
* CONSTANTS
*/



// Define size of buffer
//#define BUFFER_SIZE 128
//#define BUFFER_SIZE SBP_UART_RX_BUF_SIZE

#define UART_DATA_LEN  128
#define BUFFER_SIZE UART_DATA_LEN
/***********************************************************************************
* LOCAL VARIABLES
*/

// Slave's recieve buffer
static uint8 rxBufferUart[BUFFER_SIZE];

static uint8 rxBufferSlave[BUFFER_SIZE];

//static uint8 TxBufferLen = 0;
static uint8 rxBufferLen = 0; //index the length of rxBufferUart[] used.

static uint8 BLEBufferSlave[BUFFER_SIZE];
static uint8 BLEBufferLen = 0;

static uint8 DeviceInfo[UART_LEN_DEV];

static uint8 BLE_STATUS[UART_LEN_STARTED];

static gyroCBs_t sensorTag_GyroCBs =
{
	gyroChangeCB,             // Characteristic value change callback
};

//static uint8 RECEIVE_MESSAGE[24];


/*
static uint8 RxLen=0;
static uint8 TxLen=0;
static uint8 TxFlage=0;
static uint8 TxLenSend=0;
static uint8 RxFlage = 0;*/
uint8 SPI_Delay = 0;

uint8 BLE_State = BLE_STATE_IDLE;
uint8 CharValuData[20];
uint8 CharValuLen;
uint8 CharValuID = 0;

uint8 MessageType = STATE_CMD_INFOR;
uint16 delaySleep = 0;
uint8 timFlag = 0;
uint8 SystemBuyDelay = 0;
uint8 send_flage = 0x00;

static uint16 BLE_DELAY = 0;

//#define SystemBuyDelaySleep	10
/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      SensorTag_Init
*
* @brief   Initialization function for the Simple BLE Peripheral App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notificaiton ... ).
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  none
*/

extern void DevInfoInit(void);
void SensorTag_Init( uint8 task_id )
{

	sensorTag_TaskID = task_id;
	devDataInit(); //Initialize the serial protocol.
	SerialApp_Init();//打开串口
	uint8 ucTemp;
	DevInfoInit();

	uint8 ucAncsTemp[2];
	HalFlashRead(ANCS_ENABLE_PAGE, 0, ucAncsTemp, 2);

	if(ucAncsTemp[0] == ANCS_DISABLE_FLAG && ucAncsTemp[1] == ANCS_DISABLE_FLAG) //关
	{
		g_ucANCSFlag = ANCS_DISABLE_FLAG;
	}
	else
	{
		g_ucANCSFlag = ANCS_ENABLE_FLAG;
	}

	//总是打开ANCS
	//  g_ucANCSFlag=ANCS_ENABLE_FLAG;
	//	g_ucANCSFlag = ANCS_DISABLE_FLAG;


#if (VENDOR_TYPE==1)
	ucTemp = mac_buf[1];
	scanRspData[0x0b] = attDeviceName[9] = HEX_TO_ASCII_H(ucTemp);
	scanRspData[0x0c] = attDeviceName[10] = HEX_TO_ASCII_L(ucTemp);
	ucTemp = mac_buf[0];
	scanRspData[0x0d] = attDeviceName[11] = HEX_TO_ASCII_H(ucTemp);
	scanRspData[0x0e] = attDeviceName[12] = HEX_TO_ASCII_L(ucTemp);
#elif (VENDOR_TYPE==2)
	ucTemp = mac_buf[1];
	scanRspData[0x09] = attDeviceName[7] = HEX_TO_ASCII_H(ucTemp);
	scanRspData[0x0a] = attDeviceName[8] = HEX_TO_ASCII_L(ucTemp);
	ucTemp = mac_buf[0];
	scanRspData[0x0b] = attDeviceName[9] = HEX_TO_ASCII_H(ucTemp);
	scanRspData[0x0c] = attDeviceName[10] = HEX_TO_ASCII_L(ucTemp);
#else
	ucTemp = mac_buf[1];
	scanRspData[0x0b] = attDeviceName[9] = HEX_TO_ASCII_H(ucTemp);
	scanRspData[0x0c] = attDeviceName[10] = HEX_TO_ASCII_L(ucTemp);
	ucTemp = mac_buf[0];
	scanRspData[0x0d] = attDeviceName[11] = HEX_TO_ASCII_H(ucTemp);
	scanRspData[0x0e] = attDeviceName[12] = HEX_TO_ASCII_L(ucTemp);
#endif
	// Setup the GAP Peripheral Role Profile
	{
		// Device starts advertising upon initialization
		uint8 initial_advertising_enable = TRUE;//FALSE;

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
		GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );//广播数据

		GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
		GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
		GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
		GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
		GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
		BLE_DELAY = 20;
	}
	//GATT_InitClient
	GATT_InitClient();
	GATT_RegisterForInd(sensorTag_TaskID);//注册，实现接收属性值的通知。

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

	//g_ucANCSFlag=ANCS_ENABLE_FLAG;

	if(g_ucANCSFlag == ANCS_DISABLE_FLAG)
	{
		uint32 passkey = 0; // passkey "000000"
		uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
		uint8 mitm = TRUE;
		uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
		uint8 bonding = FALSE;
		GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
		GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
		GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
		GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
		GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
	}
	else
		// Setup the GAP Bond Manager
	{
		uint32 passkey = 0; // passkey "000000"
		uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
		uint8 mitm = FALSE;
		uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
		uint8 bonding = TRUE;

		GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
		GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
		GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
		GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
		GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
	}

	// Add services
	GGS_AddService( GATT_ALL_SERVICES );            // GAP
	GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
	   GATT_RegisterForInd(task_id);
	DevInfo_AddService();                           // Device Information Service

	//手机APP端的各种指令都是走AA00service(IR Temperature Service)的AA02的characteristic发送命令的。对于watch来说就是接收，该蓝牙程序运行与watch上，这里就是接收。
	IRTemp_AddService (GATT_ALL_SERVICES );         // IR Temperature Service
	//改服务0xAA10的characteristic 0xAA10,被手机端用作接收响应，那么对于watch来说，就是发送端，可以作为对APP的各种命令的响应，也可以单独向APP发送数据。
	Accel_AddService (GATT_ALL_SERVICES );          // Accelerometer Service
	HeartRate_AddService(GATT_ALL_SERVICES);
	//Humidity_AddService (GATT_ALL_SERVICES );       // Humidity Service
	//Magnetometer_AddService( GATT_ALL_SERVICES );   // Magnetometer Service
	Thermometer_AddService( GATT_ALL_SERVICES );
	//  Running_AddService( GATT_ALL_SERVICES );


	Gyro_AddService( GATT_ALL_SERVICES );
	Batt_AddService();
	ImmediateAlert_AddService();

	//2015年9月2日08:51:44 新添加的
	MyAlert_AddService(GATT_ALL_SERVICES);


//	NotificationSignal_AddService();       //新加的的2015年7月8日11:11:56
	//Barometer_AddService( GATT_ALL_SERVICES );      // Barometer Service
	// Gyro Service
	//  SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
	//  Test_AddService( GATT_ALL_SERVICES );           // Test Profile

	// Setup the Seensor Profile Characteristic Values
	//resetCharacteristicValues();

	// Register for all key events - This app will handle all key events
	//  RegisterForKeys( sensorTag_TaskID );
	// Register callbacks with profile

	//-----------------------------上面是添加服务，下面是注册各个服务---------------

	VOID IRTemp_RegisterAppCBs( &sensorTag_IrTempCBs );//当APP或者dongle给手表发送下载firmware的时候，就会调用这里的回调函数sensorTag_IrTempCBs，它把数据发送给MCU。
	//  VOID Magnetometer_RegisterAppCBs( &sensorTag_MagnetometerCBs );
	VOID Accel_RegisterAppCBs( &sensorTag_AccelCBs );
	Thermometer_Register(Thermometer_CBs);
	HeartRate_Register(heartRateCB);
#if 0
	//为了检测使用升级工具不能下载数据的问题
	//为了查错，先屏蔽 2015年8月12日11:43:18
	Acceler_Register(HeartRateAlert_CBs);
#endif
	//  Running_Register(Running_CBs);
	//VOID Humidity_RegisterAppCBs( &sensorTag_HumidCBs );
	//VOID Barometer_RegisterAppCBs( &sensorTag_BarometerCBs );
	VOID Gyro_RegisterAppCBs( &sensorTag_GyroCBs );
	//  VOID Test_RegisterAppCBs( &sensorTag_TestCBs );

	//2015年9月2日10:18:37添加的
	MyAlert_Register(myAlertCB); //ATUSDBG

	// Enable clock divide on halt
	// This reduces active current while radio is active and CC254x MCU
	// is halted
	HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );// 2013-11-17
	HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);// 2013-11-17

	//turn on overlapped processing
	HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
	HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
	//HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_4_DBM);
#if 1
	uint8 EXT_TX_POWER_4_DBM = TRUE;
#if defined(EXT_MAX_TX_POWER)
	LL_EXT_SetMaxDtmTxPower(EXT_MAX_TX_POWER);//Set the maximum transmit power
	LL_EXT_SetTxPower(EXT_MAX_TX_POWER, &EXT_TX_POWER_4_DBM); //[BG036] LL_EXT_TX_POWER_4_DBM, LL_EXT_TX_POWER_MINUS_23_DBM
#else
	LL_EXT_SetMaxDtmTxPower(LL_EXT_MAX_TX_POWER);//Set the maximum transmit power
	LL_EXT_SetTxPower(LL_EXT_TX_POWER_4_DBM, &EXT_TX_POWER_4_DBM); //[BG036] LL_EXT_TX_POWER_4_DBM, LL_EXT_TX_POWER_MINUS_23_DBM
#endif
	LL_EXT_SetRxGain(HCI_EXT_RX_GAIN_HIGH, &EXT_TX_POWER_4_DBM);
#else
	HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_4_DBM);
#endif
#if defined(RSSI_DIFF) //[BG036-1]

    uint8 cur_RFRAMCFG=0, set_RSSI_DIFF=0;
    int i=0;
    do
    {
      if (SEMAPHORE1==1)
      {
        cur_RFRAMCFG = RFRAMCFG; //store the current page.
        RFRAMCFG = 0x05; //switch to page2. [BG036-2] 0x02 >> 0x05
        PRFX_RSSI_DIFF = RSSI_DIFF; //force high/low RSSI gain to 0x0.
        RFRAMCFG = cur_RFRAMCFG; //restore current page.
        SEMAPHORE1 = 0x01; //release SEMAPHORE1.
        set_RSSI_DIFF = 1;
      }
      else
      {
        Delay_uS(1000);
        i++;
      }
    } while (set_RSSI_DIFF==0 && i<5);
#endif
	// Setup a delayed profile startup
	osal_set_event( sensorTag_TaskID, ST_START_DEVICE_EVT );
	// osal_start_reload_timer(sensorTag_TaskID, PERIOD_EVT, 1000);
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

uint16 g_uiAddr, g_uiLen;
uint16 SensorTag_ProcessEvent( uint8 task_id, uint16 events )
{
	//   static uint8 IR_Tempe = 0;
	VOID task_id; // OSAL required parameter that isn't used in this function

	if ( events & SYS_EVENT_MSG )
	{
		uint8* pMsg;

		if ( (pMsg = osal_msg_receive( sensorTag_TaskID )) != NULL )
		{
			sensorTag_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );   //该函数中仅仅处理消息事件。

			// Release the OSAL message
			VOID osal_msg_deallocate( pMsg ); //当一个任务处理完它接收的message时就调用该函数，它用来释放接收消息的buffer。
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
		VOID GAPRole_StartDevice( &sensorTag_PeripheralCBs );//主要做外设初始化，

		// Start Bond Manager
		VOID GAPBondMgr_Register( &sensorTag_BondMgrCBs );

		txDtatSend(DeviceInfo, DeviceInfo[UART_ID_LEN_POS]);
		Delay_uS(1000);
		SEND_BLE_STA();  //这里吧BLE的连接状态发送给MCU。
		//osal_start_timerEx( sensorTag_TaskID, ST_DELAY_EVT, SBP_BURST_EVT_PERIOD );

		return ( events ^ ST_START_DEVICE_EVT );
	}

	if ( events & ST_SPI_BLE_EVT )//ST应该表示Sensor Tag.这里应该和cc2540的串口操作有关吧。
	{
		sendData();//这里是发送的notification.cc2540把notification发送出去，手机端蓝牙就可以接收到该

		//if ((BLE_State == BLE_STATE_CONNECTED)/* && (rxDtatCheck())*/)
		if (send_flage)
		{

			osal_start_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT, BLE_DELAY );//2013/9/9 14:10
		}
		else
		{
			timFlag = 0;

		}

		return (events ^ ST_SPI_BLE_EVT);
	}

	if ( events & ST_UART_READ_EVT )
	{
		UART_Analyze(); //串口数据分析，当把串口中的数据收完整后，触发ST_UART_READ_EVT，达到这里
		return (events ^ ST_UART_READ_EVT);
	}

	if ( events & ST_DELAY_EVT )
	{
		send_flage = 0x00;
		osal_stop_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT );//2013/9/9 14:10
		(void)osal_pwrmgr_task_state(sensorTag_TaskID, PWRMGR_CONSERVE);//判断该任务是否需要节省电源。
		osal_stop_timerEx( sensorTag_TaskID, ST_DELAY_EVT );
		/*if (SystemBuyDelay < SystemBuyDelaySleep)
		{
		SystemBuyDelay++;
		osal_start_reload_timer( sensorTag_TaskID, ST_DELAY_EVT, SBP_DELAY_EVT_PERIOD );

		}*/
		return (events ^ ST_DELAY_EVT);
	}

	if ( events & USER_UART_RX_EVT )
	{
		//串口收到数据后触发USER_UART_RX_EVT
		osal_stop_timerEx( sensorTag_TaskID, USER_UART_RX_EVT );
		rxDataRead();//当CC2540接收到串口的数据就执行该函数。如果没有错误的话，在最后启动ST_UART_READ_EVT事件。
		return (events ^ USER_UART_RX_EVT);
	}

	if ( events & BP_START_DISCOVERY_EVT )
	{
		if(g_ucANCSFlag != ANCS_DISABLE_FLAG)
		{
			if ( timeAppPairingStarted )
			{
				// Postpone discovery until pairing completes
				timeAppDiscPostponed = TRUE;
			}
			else
			{
				uint8 ucDataArry[6] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_ANCS, 0, 0, UART_DATA_STOP};
				txDtatSend(ucDataArry, sizeof(ucDataArry));
				timeAppDiscState = timeAppDiscStart();
			}
		}

		return ( events ^ BP_START_DISCOVERY_EVT );
	}

	// Read ANCS Data
	if ( events & BP_READ_ANCS_DATA_EVT )
	{
		// Read ANCS Data
		ReadAncsAppNameTask();//这个函数将ANCS的characteristic写入到server中。
		//启动超时计时器,也就是要求数据必须要一定时间内全部接收完成

		return (events ^ BP_READ_ANCS_DATA_EVT);
	}

//ANCS怎么可能在串口读事件中了？
	// Read ANCS Data Complete
	if ( events & BP_READ_ANCS_CML_EVT )
	{
		// Read ANCS Data Complete
		//    Ancs_Date_Complete();
		extern uint8 g_ucFlag;

		g_ucFlag = 0;
		uint8 ucSendDataTemp[128] = {0x3c};
#define ANCS_UART_MAX_NUM       50
		extern uint8 Ancs_Data_Buff[];
		extern  uint16 Ancs_Data_Len;
		//    Ancs_Data_Len +=1;
		//    Ancs_Data_Buff[1]=Ancs_Data_Len;
		//    Ancs_Data_Buff[2]=0X02;
		//    Ancs_Data_Buff[Ancs_Data_Len-1]=0x3e;
		//    g_uiAddr=0;
		//
		//    g_uiLen=Ancs_Data_Len;

		if(g_uiAddr == 0)
		{
			//首个数据发送。
			if(Ancs_Data_Len <= ANCS_UART_MAX_NUM) //一个包的数据就可以发送完。
			{
				Ancs_Data_Len += 5; //3c uLen 09 02 xx xx 3e
				ucSendDataTemp[1] = Ancs_Data_Len;
				ucSendDataTemp[2] = UART_CMD_ANCS; //0x09;
				ucSendDataTemp[3] = 0x02;
				ucSendDataTemp[Ancs_Data_Len - 1] = 0x3e;
				osal_memcpy( &ucSendDataTemp[4], Ancs_Data_Buff, (Ancs_Data_Len - 4) );
				txDtatSend(ucSendDataTemp, Ancs_Data_Len);
				Ancs_Data_Len = ANCS_DATA_LOCATION;
				osal_stop_timerEx( sensorTag_TaskID, BP_READ_ANCS_CML_EVT);
			}
			else
			{
				//          Ancs_Data_Len+=4;//3c uLen 09 02 xx xx 3e
				ucSendDataTemp[1] = (ANCS_UART_MAX_NUM + 5);
				ucSendDataTemp[2] = UART_CMD_ANCS; //0x09;
				ucSendDataTemp[3] = 0x03;
				ucSendDataTemp[ANCS_UART_MAX_NUM + 3] = 0x3e;
				osal_memcpy( &ucSendDataTemp[4], Ancs_Data_Buff, (ANCS_UART_MAX_NUM) );
				txDtatSend(ucSendDataTemp, ucSendDataTemp[1] );
				g_uiAddr += ANCS_UART_MAX_NUM;
				g_uiLen -= ANCS_UART_MAX_NUM;
				osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_CML_EVT, 5 );
			}
		}
		else
		{
			if(g_uiLen > ANCS_UART_MAX_NUM) //分包
			{
				ucSendDataTemp[1] = (ANCS_UART_MAX_NUM + 5);
				ucSendDataTemp[2] = UART_CMD_ANCS; //0x09;
				ucSendDataTemp[3] = 0x04;
				ucSendDataTemp[ANCS_UART_MAX_NUM + 3] = 0x3e;
				osal_memcpy( &ucSendDataTemp[4], &Ancs_Data_Buff[g_uiAddr], (ANCS_UART_MAX_NUM) );
				txDtatSend(ucSendDataTemp, ucSendDataTemp[1] );
				g_uiAddr += ANCS_UART_MAX_NUM;
				g_uiLen -= ANCS_UART_MAX_NUM;
				osal_start_timerEx( sensorTag_TaskID, BP_READ_ANCS_CML_EVT, 5 );

			}
			else
			{
				ucSendDataTemp[1] = (g_uiLen + 5);
				ucSendDataTemp[2] =UART_CMD_ANCS;// 0x09;
				ucSendDataTemp[3] = 0x05;
				ucSendDataTemp[g_uiLen + 3] = 0x3e;
				osal_memcpy( &ucSendDataTemp[4], &Ancs_Data_Buff[g_uiAddr], (g_uiLen) );
				txDtatSend(ucSendDataTemp, ucSendDataTemp[1]);
				g_uiAddr += ANCS_UART_MAX_NUM;
				g_uiLen -= ANCS_UART_MAX_NUM;
				Ancs_Data_Len = ANCS_DATA_LOCATION;
				osal_stop_timerEx( sensorTag_TaskID, BP_READ_ANCS_CML_EVT);
			}
		}

		//    txDtatSend(Ancs_Data_Buff,Ancs_Data_Len);


		return (events ^ BP_READ_ANCS_CML_EVT);
	}

	// Discard unknown events
	return 0;
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
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
	switch ( pMsg->event )
	{
		case KEY_CHANGE:
			//sensorTag_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
			break;

		case GATT_MSG_EVENT:
			sensorTag_ProcessGATTMsg( (gattMsgEvent_t*) pMsg );
			break;

		default:
			// do nothing
			break;
	}
}
/*********************************************************************
* @fn      simpleBLECentralProcessGATTMsg
*
* @brief   Process GATT messages
*
* @return  none
*/
static void sensorTag_ProcessGATTMsg( gattMsgEvent_t* pMsg )
{
	//  if ( simpleBLEState != BLE_STATE_CONNECTED )
	//  {
	//    // In case a GATT message came after a connection has dropped,
	//    // ignore the message
	//    return;
	//  }
#define NOTIFICATION_HANDLE 0X0099
	if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ||
	        pMsg->method == ATT_HANDLE_VALUE_IND )     //只有当消息的类型是notification或者是indication时，获取消息内容。
	{
		if(NOTIFICATION_HANDLE == pMsg->msg.handleValueNoti.handle)
		{

			//      osal_start_reload_timer( sensorTag_TaskID, ST_DELAY_EVT, SBP_DELAY_EVT_PERIOD );
			//      (void)osal_pwrmgr_task_state(sensorTag_TaskID, PWRMGR_HOLD);

			//      uint8 iLen,ucTempBuf[27]={0,0,0,'<',0,4};

			uint8 iLen, ucTempBuf[23] = {0, 0, 4}; //这里的4是什么意思？

			for( iLen = 0; iLen < (pMsg->msg.handleValueNoti.len); iLen++)
			{
				ucTempBuf[iLen + 3] = pMsg->msg.handleValueNoti.value[iLen];
			}

//这个地方屏蔽的是：通过PC Tool发送的短信或电话信息。但是手机端的短信没有屏蔽掉???
#if (MODEL_TYPE==1)	//only HEALTHCARE_TYPE apply filter.
			if((ucTempBuf[3] != 0x07) && (ucTempBuf[3] != 0x06))  //Atus: check when add.
#endif
				txDtatSend(ucTempBuf, (pMsg->msg.handleValueNoti.len + 4));//这里有将收到的部分notification的数据返回给MCU。

#if 0
			ucTempBuf[4] = iLen + 4;

			sbpSerialAppWrite(ucTempBuf, sizeof(ucTempBuf));

			sbpSerialAppWrite(pMsg->msg.handleValueNoti.value, iLen);

			ucTempBuf[0] = '>';
			sbpSerialAppWrite(ucTempBuf, 1);
#endif

#if 0
			bool hasError = false;

			for(uint8 i = 0; i < pMsg->msg.handleValueNoti.len; i++)
			{
				if(i != pMsg->msg.handleValueNoti.value[i])
				{
					hasError = true;
					break;
				}
			}

			if (hasError)
			{
				pMsg->msg.handleValueNoti.value[0] = 0xf0;
			}

			sbpSerialAppWrite(pMsg->msg.handleValueNoti.value, iLen);
#endif
		}
		else
		{
			timeAppIndGattMsg( pMsg );

		}
	}
	else if ( pMsg->method == ATT_READ_RSP ||
	          pMsg->method == ATT_WRITE_RSP )
	{
		timeAppConfigState = timeAppConfigGattMsg ( timeAppConfigState, pMsg );

		if ( timeAppConfigState == TIMEAPP_CONFIG_CMPL )
		{
			timeAppDiscoveryCmpl = TRUE;
		}
	}
	else
	{
		timeAppDiscState = timeAppDiscGattMsg( timeAppDiscState, pMsg );

		if ( timeAppDiscState == DISC_IDLE )
		{
			// Start characteristic configuration
			timeAppConfigState = timeAppConfigNext( TIMEAPP_CONFIG_START );
			DEBUG_PRINT("@@@@@@@@@@@@@#\r\n", 0);
		}
	}

}

/*********************************************************************
* @fn      peripheralStateNotificationCB
*
* @brief   Notification from the profile of a state change.
*
* @param   newState - new state
*
* @return  none
*/

uint8 SentAlready = 0; //make sure just one time to send "adver"

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

		case GAPROLE_ADVERTISING:
			if (BLE_State != BLE_STATE_ADVERTISING)
			{
				BLE_State = BLE_STATE_ADVERTISING;
//#if (BG036)
//				if(SentAlready == 0)
//				{
//					SentAlready = 1;
//				}
//endif
				SEND_BLE_STA();
			}

			break;

		case GAPROLE_CONNECTED:
			DEBUG_PRINT("connect\r\n", 0);
			// Get connection handle
			GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
			linkDBItem_t*  pItem;

			// Get peer bd address
			if ( (pItem = linkDB_Find( gapConnHandle )) != NULL)
			{
				// If connected to device without bond do service discovery
				if ( !osal_memcmp( pItem->addr, timeAppBondedAddr, B_ADDR_LEN ) )
				{
					timeAppDiscoveryCmpl = FALSE;
				}
				else
				{
					timeAppDiscoveryCmpl = FALSE; //[BG036] TRUE >> FALSE
				}

				// if this was last connection address don't do discovery
				if(osal_memcmp( pItem->addr, lastConnAddr, B_ADDR_LEN ))
				{
					timeAppDiscoveryCmpl = FALSE; //[BG036] TRUE >> FALSE
					connectedToLastAddress = true;
				}
				else
				{
					//save the last connected address
					osal_memcpy(lastConnAddr, pItem->addr, B_ADDR_LEN );
                    connectedToLastAddress = false; //Atus: add for future used.
				}

				if(g_ucANCSFlag != ANCS_DISABLE_FLAG)
				{
					// Initiate service discovery if necessary
                  if ( timeAppDiscoveryCmpl == FALSE ) //ATUSDBG: 
					{
						osal_start_timerEx( sensorTag_TaskID, BP_START_DISCOVERY_EVT, DEFAULT_DISCOVERY_DELAY );
                        //[BG036] ATUSDBG: should add timeAppDiscoveryCmpl = TRUE;
					}
				}

			}

			//		osal_start_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT, SBP_BURST_EVT_PERIOD );//2013/9/9 14:10
			if (BLE_State != BLE_STATE_CONNECTED)
			{
				BLE_State = BLE_STATE_CONNECTED;
				SEND_BLE_STA();
			}

			setCONN_INTERVAL(0x00);
			break;

        case GAPROLE_WAITING:
#if 1
          // Link terminated intentionally: reset all sensors
            if (BLE_State != BLE_STATE_IDLE)
            {
                if(g_stBleStateFlag.advConStatus == true)
                {
                    g_stBleStateFlag.advConStatus = false;
                    uint8 new_adv_enabled_status =true ;//
                    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
                }
                DEBUG_PRINT("Disconnect!!!\r\n",0);
                SentAlready=0;
                BLE_State = BLE_STATE_IDLE;
                SEND_BLE_STA();
            }
#endif
            break;

		default:
			if (BLE_State != BLE_STATE_IDLE)
			{
				if(g_stBleStateFlag.advConStatus == true)
				{
					g_stBleStateFlag.advConStatus = false;
					uint8 new_adv_enabled_status = true ; //
					GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
				}

				DEBUG_PRINT("Disconnect!!!\r\n", 0);
				SentAlready = 0;
				BLE_State = BLE_STATE_ADVERTISING; //BLE_STATE_IDLE;
				SEND_BLE_STA();
			}

			break;
	}

    if (gapProfileState!=newState)    //ATUSDBG: check gapProfileState change.
    {
	gapProfileState = newState;
    SEND_BLE_STA();
    }
}
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
	if ( paramID == IRTEMPERATURE_CONF)
	{
		//    BLEBufferSlave[UART_ID_START] = UART_DATA_START; //2014-03-15
		BLEBufferSlave[UART_ID_LEN_POS] = (UART_LEN_FOU + irTempLenWrite);
		BLEBufferSlave[UART_ID_CMD_POS] = UART_CMD_APPDATA;
		memcpy(&BLEBufferSlave[UART_ID_DATA_POS], irTempDataWrite, irTempLenWrite);
		MessageType = STATE_COM_DATA;

		//    BLEBufferSlave[BLEBufferSlave[UART_ID_LEN]-1] = UART_DATA_STOP; //2014-03-15
#if (MODEL_TYPE==1)	//only HEALTHCARE_TYPE apply filter.
		if((BLEBufferSlave[3] != 0x07) && (BLEBufferSlave[3] != 0x06)) //这里屏蔽掉电话和短信（来自于APP端）. Atus: check when add.
#endif
			txDtatSend(BLEBufferSlave, BLEBufferSlave[UART_ID_LEN_POS]);

		//这里把从???端来的数据，打包通过串口发送给watch的MCU。

	}
}


//在这里添加一个上传递心率级别的  2015年7月8日17:23:36
static void accelChangeCB( uint8 paramID )
{

}


/*******************************************************
***	Desc：Send the notification Data to Host. (Ex, Gather Data, response notification).
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
  extern gattAttribute_t accelAttrTbl[]; //AA10:
	extern gattAttribute_t irTempAttrTbl[]; //AA00
	extern gattAttribute_t heartRateAttrTbl[] ;
	extern gattAttribute_t sensorGyroscopeAttrTbl[];
	extern gattAttribute_t thermometerAttrTbl[];
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
					handle_tab = heartRateAttrTbl[CHANNEL3_LOCAL].handle;
					break;

				case 4:
					handle_tab = 0;
					break;

				case 5://来确定来的数据是哪个服务下的那个handle要处理的，准备好之后，交给下面GATT_Notification（）发送出去的。
					handle_tab = sensorGyroscopeAttrTbl[CHANNEL5_LOCAL].handle;
					break;

				case 6:
					handle_tab = thermometerAttrTbl[CHANNEL6_LOCAL].handle;
					break;

				case 7:
					handle_tab = 0; //runningAttrTbl[CHANNEL7_LOCAL].handle;
					break;
			}

			attHandleValueNoti_t nData;//notification数据结构。下面为notification准备数据
			nData.len = sendDataBuf[id][sendDataLen];
			nData.handle = handle_tab;
			osal_memcpy( &nData.value, &sendDataBuf[id][sendDataData], sendDataBuf[id][sendDataLen]);

			//nData.value[19] = sendDataBuf_tab;//加尾巴
			if (GATT_Notification( 0, &nData, FALSE ) == SUCCESS) //这里就把准备好的notification发送出去（发送应该是cc254自动完成的。这里的数据的来源是MCU通过串口发送给cc2540）
			{
				sendDataBuf[id][sendDataLen] = 0;
				//sendDataBuf_tab++;
				BLEDtatMOV();
			}
			else
			{
				return;
			}
		}
		else
		{
			return;
		}
	}
}
/*******************************************************
***	功能描述：设备信息数据初始化
***	入口参数：无
***	返回值：  无
***	调用方法：devDataInit();
*******************************************************/
/* SPI settings */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20
#define SPI_BAUD_E  16
#define SPI_BAUD_M 0



__no_init XDATA uint8 mac_buf[6]@0x780E;
__no_init XDATA uint8 Dev_Infor_ChipID@0x624A;
__no_init XDATA uint8 Dev_Infor_Cap@0x6276;

void devDataInit(void)
{
	//  DeviceInfo[UART_ID_START] = UART_DATA_START;
	DeviceInfo[UART_ID_LEN_POS] = UART_LEN_DEV;
	DeviceInfo[UART_ID_CMD_POS] = UART_CMD_INFOR;
	DeviceInfo[UART_ID_DATA_POS + 0] = UART_TYPE_DEV;
	DeviceInfo[UART_ID_DATA_POS + 1] = Dev_Infor_ChipID;
	DeviceInfo[UART_ID_DATA_POS + 2] = Dev_Infor_Cap;
	DeviceInfo[UART_ID_DATA_POS + 3] = Dev_FW_V1;
	DeviceInfo[UART_ID_DATA_POS + 4] = Dev_FW_V2;
	memcpy(&DeviceInfo[UART_ID_DATA_POS + 5], mac_buf, 6);
	DeviceInfo[UART_ID_DATA_POS + 11] = BLE_APP;
	//  DeviceInfo[UART_ID_DATA+12] = UART_DATA_STOP; //2014-03-15
}

/*******************************************************
***	功能描述：串口的RX线配置成外部中断
***	入口参数：无
***	返回值：  无
***	调用方法：uartRxIsrInit();
*******************************************************/
void uartRxIsrInit(void)
{
#if (HAL_UART_DMA == 2)
	IEN2 |= BIT7;
	P1IFG &= ~BV(7);
	P1IF = 0;
	P1SEL &= ~BIT7;
	P1DIR &= ~BIT7;
	P1IEN |= BIT7;
	PICTL |= 0x07;
	P1 = 0xff;

#else
	//P0.2
	U0CSR &= ~BIT6;//接收器禁止。
	IEN1 |= BIT5; //允许P0口中断。
	P0IFG &= ~BV(2);//初始化中断标志位
	P0IF = 0; //清除中断标志位。
	P0SEL = BIT2;//0：普通IO 1：第二功能
	P0DIR = BIT2;// 0：输入，1：输出
	P0IEN = BIT2; //P0_2中断使能。
	PICTL |= 0x01; //下降沿触发
	P0 = 0xff;
#endif
}
/*******************************************************
***	功能描述：串口的RX线配置RX
***	入口参数：无
***	返回值：  无
***	调用方法：uartRxInit();
*******************************************************/
void uartRxInit(void)
{
#if 1
	static uint8 a = 0;
#if (HAL_UART_DMA == 2)
	a = U1DBUF;
	P1IFG = 0x00;
	P1IF = 0x00;
	P1SEL |= (BIT7 | BIT6);
	P1IEN &= ~BIT7;
	rxBufferLen = 0;
	P1 = 0xff;
#else
	a = U0DBUF;

	if (P0IFG & BIT2)
	{
		extern volatile uint8 dmaRdyIsr;
		dmaRdyIsr = 1;
		CLEAR_SLEEP_MODE();
		delaySleep = DELAYSLEEP_INIT;
		osal_start_reload_timer( sensorTag_TaskID, ST_DELAY_EVT, SBP_DELAY_EVT_PERIOD );
		(void)osal_pwrmgr_task_state(sensorTag_TaskID, PWRMGR_HOLD);
		//		SystemBuyDelay = 0x00;
		//		osal_start_timerEx( sensorTag_TaskID, ST_DELAY_EVT, SBP_BURST_EVT_PERIOD );
	}

	P0IEN &= ~BIT2;
	P0IFG &= ~BIT2;
	P0IF &= ~BIT2;
	P0SEL |= (BIT2 | BIT3);

	P0 = 0xff;
	U0CSR |= BIT6;
	/*extern void HalUARTInit(void);
	HalUARTInit();
	SerialApp_Init();*/
#endif


	a++;
	//	SerialApp_Init();
#endif
}
/*******************************************************
*	Desc: SPI get the data then decode the procotol.
*	Params: Protocol ID
*	return: None
*	Useage: The BLE input via UART, trigger it by UART interrupt.
*******************************************************/

//当有串口读事件发生时，就调用改函数。
uint8 g_ucFlashRead[2];
uint8 Uart_Data_ACK[UART_LEN_FOU] = {UART_DATA_START, UART_LEN_FOU, UART_CMD_DATA, UART_DATA_STOP};
const uint16 g_ucANCSWritePage = (uint16)(ANCS_ENABLE_PAGE << 9); //0x7A<<9=0xF400. [BG029-1] add type convert.
void UART_Analyze(void)  //URAT_Analyze >> UART_Analyze
{
	if (rxBufferSlave[UART_ID_CMD_POS] == UART_CMD_DATA)
	{
		uint8 group = 0;
		uint8 rem = 0;
		uint8 add = 0;
		uint8 i = 0;
		uint8 id = 0;
		uint8 temp = 0;
		Uart_Data_ACK[UART_ID_CMD_POS] = UART_CMD_DATA;
		//
		uint8 ucCmd = rxBufferSlave[UART_ID_DATA_POS];//这里是数据的位置
		uint8 ucLen = rxBufferSlave[UART_ID_LEN_POS] - UART_DATA_EXTRA_LEN;  //pure noti data length only.

		if(ucCmd == DEVICE_INFO_CHANNEL)
		{
			/* device information. get the device information and stored. */
			fnSerial2Service[DEVICE_INFO_CHANNEL](&rxBufferSlave[UART_ID_DATA_NOTI], ucLen);
		}
#if 0	//Atus: Why remark it both HEALTHCARE_TYPE and CONSUMER_TYPE ???
//		else if(ucCmd == HEART_ALERT_CHANNEL)
//		{//新添加的 作为心率级别报警
//			fnSerial2Service[HEART_ALERT_CHANNEL](&rxBufferSlave[UART_ID_DATA_NOTI], ucLen);
//		}
#endif
		else if (BLE_State == BLE_STATE_CONNECTED)
		{
			if(ucCmd <= S2S_FN_MAX)
			{
				//命令有效。根据不同的命令执行不同的函数
				temp = fnSerial2Service[ucCmd](&rxBufferSlave[UART_ID_DATA_NOTI], ucLen);
			}
			else
			{

			}

			if(temp == 0xfe)
			{
				//串口到service的空函数返回0xfe
				id = rxBufferSlave[UART_ID_DATA_POS]; //channel
				group = rxBufferSlave[UART_ID_LEN_POS]; //the whole msg length
				group = group - UART_ID_DATA_NOTI; //0x04: <,len,cmd,ch... the length from noti to end (include stop).

				if (group > 0)
				{
					group--; //minus the last stop byte.
				}
				else
				{
					txDtatSend(Uart_Data_ACK, UART_LEN_FOU);
					return;
				}

				rem = group % UART_DATA_COM_LEN;
				group = group / UART_DATA_COM_LEN;

				for (i = 0; i < group; i++)
				{
					if (i == 0)
					{
						add =  0x04;

					}
					else
					{
						add = i;
						add = add * UART_DATA_COM_LEN;
						add +=  0x04;
					}

					rxDtatSave(&rxBufferSlave[add], UART_DATA_COM_LEN, id);//cut by UART_DATA_COM_LEN length store to sendDataBuf[][]
				}

				if (rem > 0)
				{
					add = i;
					add = add * UART_DATA_COM_LEN;
					add +=  0x04;
					rxDtatSave(&rxBufferSlave[add], rem, id);
				}

//这里把一些不需要处理的数据，先保存下来，然后，启动定时器，在ST_SPI_BLE_EVT中根据不同的情况调用不同的服务handle来发送出去
				if (send_flage == 0x00)
				{
					send_flage = 0x01;
					osal_start_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT, SBP_BURST_EVT_PERIOD );//2013/9/9 14:10
					//经过SBP_BURST_EVT_PERIOD毫秒，sensorTag_TaskID将会收到ST_SPI_BLE_EVT事件
				}
			}
		}

		txDtatSend(Uart_Data_ACK, UART_LEN_FOU);//这里把应答发送出去，是发送给MCU的
	}
	else if (rxBufferSlave[UART_ID_CMD_POS] == UART_CMD_INFOR)//设置命令 BLE收到来自MCU的设置命令后，执行相关操作，并给MCU以响应。
	{
		Uart_Data_ACK[UART_ID_CMD_POS] = UART_CMD_INFOR;

		switch (rxBufferSlave[UART_ID_DATA_POS + 0])
		{
			case UART_TYPE_ANCS:
			{
				uint8 Uart_ANCS_Data_ACK[6] = {UART_DATA_START, 6, UART_CMD_ANCS, 0x11, 0x0, UART_DATA_STOP};

				if(0x01 == rxBufferSlave[UART_ID_DATA_POS + 1]) //查ANCS状态
				{
					Uart_ANCS_Data_ACK[3] = 0x11;

					if(g_ucANCSFlag == ANCS_DISABLE_FLAG)
					{
						Uart_ANCS_Data_ACK[4] = 0;
					}
					else
					{
						Uart_ANCS_Data_ACK[4] = 1;
					}
				}
				else if(0x02 == rxBufferSlave[UART_ID_DATA_POS + 1]) //设置ANCS状态
				{
					Uart_ANCS_Data_ACK[3] = 0x12;
					uint8 temp[2] = {ANCS_DISABLE_FLAG, ANCS_DISABLE_FLAG};
					HalFlashErase(ANCS_ENABLE_PAGE);

					while (FCTL & 0x80);


					if(0 == rxBufferSlave[UART_ID_DATA_POS + 2]) //关
					{
						Uart_ANCS_Data_ACK[4] = 0;
						HalFlashWrite(g_ucANCSWritePage, temp, 2); //这是写到CC2540内部flash的
					}
					else
					{
						Uart_ANCS_Data_ACK[4] = 1;
						temp[0] = temp[1] = ANCS_ENABLE_FLAG;
						HalFlashWrite(g_ucANCSWritePage, temp, 2);
					}

					//发送连接应答。
				}

				txDtatSend(Uart_ANCS_Data_ACK, 6);
			}
			break;

			case UART_TYPE_ADVER:
			{
				//广播
				uint8 new_adv_enabled_status;

				// Find the current GAP advertising status

				if ( rxBufferSlave[UART_ID_DATA_POS + 1] == ADVER_EN )
				{
					//change the advertising interval, disable, then set interval, then enable.
					uint16 davInt2 = rxBufferSlave[UART_ID_DATA_POS + 2];
					new_adv_enabled_status = false ; //
					GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );//设置广播参数
					g_stBleStateFlag.advConStatus = true;


					davInt2 *= 160;
					GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, davInt2 );
					GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, davInt2 );
					GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, davInt2 );
					GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, davInt2 );
                    //the advertising will turn on in peripheralStateNotificationCB()
				}
				else
				{
					GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &new_adv_enabled_status ); //这里获取广播参数

					if (new_adv_enabled_status != FALSE)
					{
						new_adv_enabled_status = false;
						GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );//
					}
				}

				txDtatSend(Uart_Data_ACK, UART_LEN_FOU);//发送应答
				break;
			}
#if 0
			//这个主要配合I4B
			case UART_TYPE_SEND_DEV:
#endif
			case UART_TYPE_DEV:
			{
				//设备，实际上是查询BLE设备信息的命令，发往MCU。
				txDtatSend(DeviceInfo, DeviceInfo[UART_ID_LEN_POS]);
				break;
			}

			case UART_TYPE_INFOR:
			case UART_TYPE_COM:
			{
				if (MessageType == STATE_CMD_INFOR)
				{
					txDtatSend(BLE_STATUS, BLE_STATUS[UART_ID_LEN_POS]);
				}
				else
				{
					BLEBufferSlave[UART_ID_LEN_POS] = BLEBufferLen + UART_LEN_FOU;
					//          BLEBufferSlave[BLEBufferSlave[UART_ID_LEN]-1] = UART_DATA_STOP; //2014-03-15
					txDtatSend(BLEBufferSlave, BLEBufferLen);
					BLEBufferLen = 0;
				}

				break;
			}

            //MACU issue the change BLE interval, set to CC2540.
			case UART_TYPE_CONN:
			{
				setCONN_INTERVAL(rxBufferSlave[UART_ID_DATA_POS + 1]);//set the BLE connecgtion interval
				break;
			}

			case UART_TYPE_BROADCAST:
			{
				//update broadcast data
				extern uint8  advertData[];

				advertData[15] = rxBufferSlave[UART_ID_DATA_POS + 1];
				advertData[16] = rxBufferSlave[UART_ID_DATA_POS + 2];
				advertData[17] = rxBufferSlave[UART_ID_DATA_POS + 3];
#if (MODEL_TYPE==1)
				advertData[18] = rxBufferSlave[UART_ID_DATA_POS + 4];
                advertData[19] = rxBufferSlave[UART_ID_DATA_POS + 5];

				GAPRole_SetParameter( GAPROLE_ADVERT_DATA, 20, advertData );//advertising data for HEALTHCARE_TYPE.
#elif (MODEL_TYPE==2)
				GAPRole_SetParameter( GAPROLE_ADVERT_DATA, 18, advertData );//advertising data
#else
#error "Check again MODEL_TYPE defined!!!"
				GAPRole_SetParameter( GAPROLE_ADVERT_DATA, 18, advertData );//广播数据. Atus: check I4_C/I4_H.
#endif
				break;
			}
		}
	}
	else if (rxBufferSlave[UART_ID_CMD_POS] == UART_CMD_UPDATA)
	{
		/* upgrade data, the BLE self upgrade. (bootloader write back) */
		Uart_Data_ACK[UART_ID_CMD_POS] = UART_CMD_UPDATA;

		if (rxBufferSlave[UART_ID_DATA_POS] == 0x00) //0x00: start, 0x01:stop //refer usbhhidkb- BLE_LEUART_DMA.c MyBLE_Update_Start();
		{
			txDtatSend(Uart_Data_ACK, UART_LEN_FOU);
			HalFlashErase(crcCheck_Page);

			while (FCTL & 0x80); //wait until erase complete, then reset CC2540.

			HAL_SYSTEM_RESET();
		}
	}
}
/*******************************************************
***	功能描述：SPI发送即时消息  //report the BLE status to MCU cycling???
***	入口参数：无
***	返回值：  无
***	调用方法：SEND_BLE_STA();
*******************************************************/
void SEND_BLE_STA(void)
{

	  BLE_STATUS[UART_ID_START_POS] = UART_DATA_START;
	BLE_STATUS[UART_ID_LEN_POS] = UART_LEN_STARTED;
	BLE_STATUS[UART_ID_CMD_POS] = UART_CMD_INFOR;
	BLE_STATUS[UART_ID_DATA_POS] = UART_TYPE_INFOR;
	BLE_STATUS[UART_ID_DATA_POS + 1] = BLE_State;
	BLE_STATUS[UART_ID_DATA_POS + 2] = gapProfileState;
    
	//  BLE_STATUS[UART_ID_DATA_POS+2] = UART_DATA_STOP;
	  BLE_STATUS[UART_ID_DATA_POS+3] = UART_DATA_STOP;
	MessageType = STATE_CMD_INFOR;
	txDtatSend(BLE_STATUS, BLE_STATUS[UART_ID_LEN_POS]);
}
/*******************************************************
***	功能描述：SPI从机发送数据的请求函数
***	入口参数：无
***	返回值：  无
***	调用方法：SPI_SendApply();
*******************************************************/
void INT_HOST(void)
{
	INT_RES;
	Delay_uS(1);
	INT_SET;
}
/*******************************************************
***	功能描述：延时函数
***	入口参数：需要延时的时间
***	返回值：  无
***	调用方法：Delay_uS();
*******************************************************/
void Delay_uS(uint16 microSecs)
{
	while(microSecs--)
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
/*******************************************************
***	desc: save data to sendDataBuf[][]
***	params:
***   data: the address(pointer) of data.
***   len: the data length
***	  id: the channel tag
*******************************************************/
void rxDtatSave(uint8* data, uint8 len, uint8 id)
{
	uint8 i = 0;

	for (i = 0; i < sendDataBufSiz; i++)
	{
		if (sendDataBuf[i][sendDataLen] == 0)
		{
			sendDataBuf[i][sendDataId] = id;
			memcpy(&sendDataBuf[i][sendDataData], data, len);
			sendDataBuf[i][sendDataLen] = len;
			return;
		}
	}
}
/*******************************************************
***	Desc：Check the sendDataBuff[][] available data want to send
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
***	Desc: data move 24 bytes.
***	Params: target data address
***	Return: None
*******************************************************/
void rxDtatMov(uint8* data)
{
	memcpy(data, &data[24], 24);
}
/*******************************************************
***	功能描述：串口数据读取
***	入口参数：无
***	返回值：  无
***	调用方法：rxDtatRead();
*******************************************************/
void rxDataRead(void)
{
	uint8 start_add = 0;
	uint8 error = 1;
	uint8 len = 0;

	//P1 ^= 0x01;//仿真板绿灯
	//    SystemBuyDelay = 0;
	delaySleep = DELAYSLEEP_INIT;

	for (uint8 i = 0; i < rxBufferLen; i++)
	{
		if (rxBufferUart[i] == UART_DATA_START)
		{
			start_add = i;
			len = rxBufferUart[i + 1]; //取总长度
#if 0   //not test.
            /* Atus: we should validate the len to prevent system crash */
            if (len > BUFFER_SIZE || (start_add+len) > BUFFER_SIZE)
                break;
#endif
			if (rxBufferUart[start_add + len - 1] == UART_DATA_STOP)
			{
				error = 0;  //we got the full msg now. can extract it.
				break;
			}
		}
	}

	if (error == 0)
	{
		P1 ^= 0x10;//仿真板黄灯
		memcpy(rxBufferSlave, &rxBufferUart[start_add], len); //取数据
		rxBufferSlave[len - 1] = UART_DATA_STOP;
		rxBufferSlave[UART_ID_LEN_POS] -= 0x02;//remove the tail padding 2 0x00
        //rxBufferSlave[rxBufferSlave[UART_ID_LEN_POS]-1] = UART_DATA_STOP; //FIXME: ATUS ADD.
        //clean the rxBufferUart[],rxBufferLen for reuse next time.
		memset(rxBufferUart, 0, BUFFER_SIZE);
		rxBufferLen = 0;  //We get the whole msg packet, reset to store new msg.
		osal_set_event( sensorTag_TaskID, ST_UART_READ_EVT);//该为任务设置事件标志
	}
}
/*******************************************************
***	功能描述：串口数据发送
***	入口参数：无
***	返回值：  无
***	调用方法：rxDtatRead();
*******************************************************/

#define Head_S       '<'         // Start of packet byte
#define Head_E       '>'         // End of packet byte

#define SOH_ESCAPE_CHAR          0x1b
#define SOH_ESCAPE_CHAR_MASK     0x33

#if 1
#define ADD_ZERO_NUMBER  8

void txDtatSend(uint8* data, uint8 len)
{
	uint8 TxBufferSlave[128];
	uint8 len_temp, i, char_temp, index;

	delaySleep = DELAYSLEEP_INIT; //It seems would like make POWER_SAVING delay to sleep while count is not zero.

	len_temp = len - 3; //skip [0]START,[1] LEN, [LAST]STOP. All the remain will encode.


	TxBufferSlave[0] = 0;
	TxBufferSlave[1] = 0;
	TxBufferSlave[2] = 0;
	TxBufferSlave[3] = 0;
	TxBufferSlave[4] = 0;
	TxBufferSlave[5] = 0;
	TxBufferSlave[6] = 0;
	TxBufferSlave[7] = 0;
	TxBufferSlave[ADD_ZERO_NUMBER] = UART_DATA_START; //'<';
	index = ADD_ZERO_NUMBER + 2;

	for(i = 0; i < len_temp; i++)
	{
		char_temp = data[2 + i];

		switch(char_temp)
		{
			case Head_S:
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR;
				TxBufferSlave[index++] = Head_S ^ SOH_ESCAPE_CHAR_MASK;
				break;

			case Head_E:
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR;
				TxBufferSlave[index++] = Head_E ^ SOH_ESCAPE_CHAR_MASK;
				break;

			case SOH_ESCAPE_CHAR:
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR;
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR ^ SOH_ESCAPE_CHAR_MASK;
				break;

			default:
				TxBufferSlave[index++] = char_temp;
				break;

		}

	}

	TxBufferSlave[index++] = UART_DATA_STOP; //'>'
	TxBufferSlave[(UART_ID_LEN_POS + ADD_ZERO_NUMBER)] = index - ADD_ZERO_NUMBER; //overwrite len after encode.


#ifndef _DEBUG_PRINT
	sbpSerialAppWrite(TxBufferSlave, index);//这里就把数据写到串口中。发送给MCU。
#endif

}

#else

void txDtatSend(uint8* data, uint8 len)
{
	uint8 TxBufferSlave[BUFFER_SIZE];
	//	SystemBuyDelay = 0;
	delaySleep = DELAYSLEEP_INIT;
	memset(TxBufferSlave, 0, BUFFER_SIZE);
	memcpy(&TxBufferSlave[3], data, (len - 1)); //起始数据填充2个0x00
	TxBufferSlave[(UART_ID_LEN + 3)] += 0x02; //整个数据长度加2
	TxBufferSlave[(TxBufferSlave[(UART_ID_LEN + 3)] + 2)] = UART_DATA_STOP;
	sbpSerialAppWrite(TxBufferSlave, (TxBufferSlave[(UART_ID_LEN + 3)] + 3));
}
#endif
/*******************************************************
***	功能描述：数据转移
***	入口参数：需要保存的起始位子，长度
***	返回值：  无
***	调用方法：BLEDtatMOV();
*******************************************************/
void BLEDtatMOV(void)
{
	uint8 i = 0;

	for (i = 0; i < (sendDataBufSiz - 1); i++)
	{
		memcpy(&sendDataBuf[i][0], &sendDataBuf[i + 1][0], 24); // 2013-11-17
		sendDataBuf[i + 1][sendDataLen] = 0;
	}
}
/*******************************************************
***	Desc：UART data read to global rxBufferUart[]
***	params:
***     data: the pointer of uart buffer pointer.
***     len:  the uart data length.
***	return: none.
*** output: store the data to global rxBufferUart[]
*******************************************************/
void getRxDataRead(uint8* data, uint8 len)
{
	osal_start_reload_timer( sensorTag_TaskID, ST_DELAY_EVT, SBP_DELAY_EVT_PERIOD );
	(void)osal_pwrmgr_task_state(sensorTag_TaskID, PWRMGR_HOLD);

	if ((rxBufferLen + len) > SBP_UART_RX_BUF_SIZE)
	{
		rxBufferLen = 0;
	}

	memcpy(&rxBufferUart[rxBufferLen], data, len);
	rxBufferLen += len;
	startEvt_UartRX();
}
/*******************************************************
***	功能描述：触发串口读函数
***	入口参数：无
***	返回值：  无
***	调用方法：startEvt_UartRX();
*******************************************************/
void startEvt_UartRX(void)
{
	osal_start_reload_timer(sensorTag_TaskID, USER_UART_RX_EVT, 3);
}
/*******************************************************
***	功能描述：关闭串口读
***	入口参数：无
***	返回值：  无
***	调用方法：stopEvt_UartRX();
*******************************************************/
void stopEvt_UartRX(void)
{
	osal_stop_timerEx( sensorTag_TaskID, USER_UART_RX_EVT );
}

/*******************************************************
***	功能描述：设置连接间隔
***	入口参数：无
***	返回值：  无
***	调用方法：setCONN_INTERVAL();
*******************************************************/
uint8 g_ucConnInterval = 0;
void setCONN_INTERVAL(uint8 conn_interval)
{
	if(conn_interval > 7)conn_interval = 7;

	g_ucConnInterval = conn_interval;
	uint16 intervalMin_tab[10] = {(8 * 1), (8 * 2), (8 * 5), (8 * 10), (8 * 15), (8 * 20), (8 * 25), (8 * 50), (8 * 75), (8 * 100),
	                             };
	uint16 intervalMax_tab[10] = {(8 * 2), (8 * 5), (8 * 10), (8 * 20), (8 * 30), (8 * 40), (8 * 50), (8 * 100), (8 * 150), (8 * 200),
	                             };
	uint16 desired_interval;
	uint16 conn_timeout = 0;

	if (conn_interval < 10)
	{
		desired_interval = intervalMax_tab[conn_interval];
		GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_interval );

		BLE_DELAY = desired_interval;
		BLE_DELAY = BLE_DELAY / 8;
		BLE_DELAY = BLE_DELAY * 10;

		conn_timeout = desired_interval / 8;
		conn_timeout = conn_timeout * 6;

		// conn_timeout = 500;
		GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &conn_timeout );

		desired_interval = intervalMin_tab[conn_interval];
		GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_interval );

		uint8 update_request = TRUE; // And send update request to make iPad accept the conn intervals defined in the device
		GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &update_request );
	}
}

/*******************************************************
***	功能描述：设置连接间隔更新状态回复
***	入口参数：无
***	返回值：  无
***	调用方法：CONN_INTERVAL_UPDATA_RES();
*******************************************************/
void CONN_INTERVAL_UPDATA_RES(void)
{

	BLE_STATUS[UART_ID_START_POS] = UART_DATA_START;
	BLE_STATUS[UART_ID_LEN_POS] = 0x06;
	BLE_STATUS[UART_ID_CMD_POS] = UART_CMD_INFOR;
	BLE_STATUS[UART_ID_DATA_POS] = UART_TYPE_CONN;
	BLE_STATUS[UART_ID_DATA_POS + 1] = g_ucConnInterval;
	BLE_STATUS[UART_ID_DATA_POS + 2] = UART_DATA_STOP; //ATUSDBG: add to change BLE_STATUS[] size   
	txDtatSend(BLE_STATUS, BLE_STATUS[UART_ID_LEN_POS]);
}
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
	if ( paramID == GYROSCOPE_DATA)
	{

	}
}



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
