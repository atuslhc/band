#ifndef _ble_H
#define _ble_H


#define BLE_ON  1
#define BLE_OFF 0

typedef enum _BLE_STATE
{
  BLE_STATE_IDLE = 0x00,
  BLE_STATE_ADVERTISING = 0x01,
  BLE_STATE_CONNECTED = 0x02,
} BLE_STATE_t;

//==================
typedef enum _BLE_CONNECT_INTERVAL
{
  BLE_CONNECT_20ms = 0,
  BLE_CONNECT_50ms,     //1
  BLE_CONNECT_100ms,    //2
  BLE_CONNECT_200ms,    //3
  BLE_CONNECT_300ms,    //4
  BLE_CONNECT_400ms,    //5
  BLE_CONNECT_500ms,    //6
  BLE_CONNECT_1000ms,   //7
} BLE_CONNECT_INTERVAL_t;

//定义心率报警级别
#define HEART_ALERT_LEVEL0  0x1    //100<heart rate < 110
#define HEART_ALERT_LEVEL1  0x2    //110<heart rate < 120
#define HEART_ALERT_LEVEL2  0x3	 //120<heart rate < 130
#define HEART_ALERT_LEVEL3  0x4	 //130<heart rate < 140
#define HEART_ALERT_LEVEL4  0x5	 //140<heart rate < 150
#define HEART_ALERT_LEVEL5  0x6    //150<heart rate 



//=================
typedef enum _BLE_ADVEN_INTERVAL
{
  BLE_ADVEN_100mS = 1,
  BLE_ADVEN_500mS = 5,

  BLE_ADVEN_1S = 10,
  BLE_ADVEN_2S = 20,
} BLE_ADVEN_INTERVAL_t;

/* The structure of ble packet in the com port */
#define UART_DATA_START			0x3c    //'<' ble packet start delimiter
#define UART_DATA_STOP			0x3e    //'>' ble packet end delimiter
#define UART_ID_START_POS		0x00
#define UART_ID_LEN_POS			0x01
#define UART_ID_CMD_POS			0x02
#define UART_ID_DATA_POS		0x03
#define SOH_ESCAPE_CHAR         0x1b    // ble packet data encode ESCAPE (like modem at command format).
#define SOH_ESCAPE_CHAR_MASK    0x33    // ble packet data encode mask. (x^0x33)
/* Sender side:
 * if data[i]==UART_DATA_START/UART_DATA_STOP/SOH_ESCAPE_CHAR
 *    data[i] encode into [0x1b, data[i]^SOH_ESCAPE_CHAR_MASK] two bytes.
 * Receiver side:
 * if data[i]==SOH_ESCAPE_CHAR
 *    data[i], data[i+1] decode into data[i+1]^SOH_ESCAPE_CHAR_MASK one byte.
 * Note: the data encode/decode, but delimiter without encode/decode.
 *    So, scan the delimiter first, then scan SO_ESCAPE_CHAR and decode.
 */

//这里的HOST是指BLE。这是命令类型
typedef enum _cmd_type
{
	UART_CMD_2HOST=0x00,
	UART_CMD_INFOR,			//0x01
	UART_CMD_UPDATA,		//0x02
	UART_CMD_UPGRADE_DATA,	//0x03
	UART_CMD_DATA_FROM_HOST, //0x04
	UART_CMD_ACK,			//0x05 reserved.
	UART_CMD_TEST,			//0x06 reserved.
	UART_CMD_ASK_DevInfo,	//0x07
	UART_CMD_WRITE,			//0x08 apply to FindMe
	UART_CMD_ANCS			//0x09
} CMD_TYPE_t;

/****************这些均未实现********************************/
//#define UART_DATA_COM_LEN		(20+1)
//#define UART_DATA_IRTEMP		0x01
//#define UART_DATA_ACCEL			0x02
//#define UART_DATA_HEARTRATE		0x03
//#define UART_DATA_BATT			0x04
//#define UART_DATA_GYRO			0x05
/****************这些均未实现********************************/



/* query between system(mcu) and ble subcommand */
typedef enum _BLE_SUBCMD
{
  SBLE_ADVER_SET=0x00,      //0x00 set advertise on/off and interval. UART_TYPE_ADVER
  SBLE_DEVINFO_QUERY,       //0x01 get ble deviceinfo. UART_TYPE_DEV
  SBLE_STATE_UPDATE,        //0x02 ble update ble state. UART_TYPE_INFOR (BLE_STATE)
  SBLE_HR_NOTIFY_SET,       //0x03 ble update HR notify state.(should be from client) UART_TYPE_NOTIFY
  SBLE_CONNECT_INTERVAL,    //0x04 ble update the ble_connect_interval value.UART_CONNECT_NOTIFY
  SBLE_ANCS_CONTROL,        //0x05 ble ANCS message control. UART_TYPE_ANCS
  SBLE_BROADCAST_UPDATE,    //0x06 ble broadcast user data update. UART_BROADCAST_UPDATE
} BLE_SUBCMD_t;

#define UART_LEN_FOU			0x04
#define UART_LEN_DEV			(UART_LEN_FOU+11)

//#define Dev_FW_V1				0x00
//#define Dev_FW_V2				0x0e

#define BOOTLOAD				0x00
#define BLE_APP					0x01

#define UART_LEN_STARTED		 (UART_LEN_FOU+2)
#define UART_INFOR_IDLE          0x00
#define UART_INFOR_ADVERTISING   0x01
#define UART_INFOR_CONNECTED     0x02



#define ADVER_EN            0x01
#define ADVER_DIS           0x00

#define crcCheck_Page       123

#define STATE_CMD_INFOR		0x00
#define STATE_COM_DATA		0x01

typedef enum _CH_Type
{
	BLE_CH1=1,  //SendRealDataOverBLE() [UUID:AA00,Char:AA01]
	BLE_CH2=2,	//response the APP request. [UUID:AA10,Char:AA11]
	BLE_CH3=3,	//use in HR data.Send_1HZ_PacketOverBLE() [UUID:180D,Char:2A37]
	BLE_CH4=4,	//reserved for battery, not use yet.
	BLE_CH5=5,  //use in Upload Data.(just data only.) [UUID:AA50,Char:AA51]
	BLE_CH9=9,  //get device information. UART_CMD_ASK_DevInfo
	BLE_CHA=10  //HR send and tunnel. SendNotificationAlert() [UUID:AA60,Char:AA61]
} CH_Type_t;

// BLE HOST Command
typedef enum _BLE_CMD
{
	getRealData=0x01,
	NotifySettings=0x02,
	ClockSynch=0x03,
	AlarmSynch=0x04,
	UserProfileSynch=0x05,
	PhoneComing=0x06,   	//All client (android ,winApp use except the iOS. it use ANCS
	notifyFeature=0x07,     //sms notify. vibra/ring
	FindMeSettings=0x08,
	
	CalibrateDevice=0x11,
	SensorSettings=0x12,

	GET_DEV_INFO=0x13,
	
	FW_Update_COMM=0x14,
	FW_CRC_GET=0x15,
	FW_Update_DATA=0x16,
	ERASE_SECTOR_COMM=0x17,
	WR_SECTOR_COMM=0x18,
	
	get_FactoryTime=0x19,
	Set_SystemMode=0x1a,
	ActivityGoals=0x1b,
	Change_Connect_Interval=0x1c,
	GET_BATTERY_LEVEL=0x1d,
	READ_SERIAL_NUMBER=0x1e,
	WRITE_SERIAL_NUMBER=0x1f,
	
	DeviceModel=0x30,
	
	//upload flash data cmd
	Flash_Upload_COMM=0x50,
	SEC_Upload_START=0x51,
	SEC_Upload_END=0x52,
	Flash_Upload_END=0x53,
    //extend upload flash by index
	Flash_Upload_ByIndex=0x54,
	Flash_Upload_NextSector=0x55,
	Get_Max_Index=0x56,
	
	ResetAccumulativeData=0x5e,     //ERASE_INDEX_DATA.  Reset the running accumulative data(isteps,idistance,icalories)
	EraseDataGather=0x5f,           //ResetDataGather  Erase the DataGather flash. (sector 512~20147)
	
	Get_Menus_Info=0x80,            //Get Menu info list
	Enable_Menus=0x81,              //Enable/disable menu(user options type)
//#if (MODEL_TYPE==2) //CONSUMER_TYPE  //can ignore in enum define.
	PPG_MODE=0xa0,
//#endif
	ResetDevice=0xf0,
	RunOutBatQuick=0xf1,
	
	SetDeviceColor=0xf4,        //i4 only
	
	StatusReport=0xf8,
	
	ResetRebootCount=0xfd,
	ResetSettings=0xfe,
	
} BLE_CMD;

union _BLE_CHIP
{
	struct _BLE_DEVICE
	{
		uint8_t  MCUTYPE;  //  0x41
		uint8_t  MEMCAP;   // 0x08 = 256K
		uint8_t  FW_VER1; // 1
		uint8_t  FW_VER2; // 0
		uint32_t unique0;  // MAC address , the first 4bytes
		uint16_t unique1;  // the second 2 bytes , 6 bytes in total
		uint8_t  WORKSTA; // 0= ble bootload ,1 = ble app
		uint8_t  Rev;
	} BLE_Device;
	uint8_t BLE_DeviceInfo[12];
};

//新添加的
typedef struct _SECTOR_NODE
{
	int sectorIndex;
	int sectorNumber;
} SECTOR_NODE;

extern volatile bool  BLE_ONLINE;
extern uint8_t BLE_STATE;
extern union _BLE_CHIP BLE_DevChip;

#define BLE_UART_Closing 0
#define BLE_UART_Opening 1

extern volatile bool BleLeUartSta;

void BLE_INIT(void);
void BLE_RESET(void);
void LeuartConfig (void);
void BLE_Update_Start(void);
void BLE_Update_End(uint16_t checksum);
void BLE_ADVEN_CON(uint8_t onoff, uint8_t inteval);

void BLE_SET_CONNECT_INTERVAL(uint8_t inteval);

void SendData2Host(uint8_t* p, uint8_t len);
void getBleDeviceInfo(void);

void SendRealDataOverBLE();

void ParseBleUartPak(void);
void BLE_Close(void);
void BLE_Open(void);
void Send_1HZ_PacketOverBLE(void);

void BLE_DATA_UPLOADING_PROC();

void isBLEError(void);
void CHECK_BLE_STUFF(void);

void SimuSendRealOverBLE(int32_t* ppg, int16_t* acc);
void SimuSendReslutOverBLE(void);

void StepsDuringOneHour();
void SendNotificationAlert();
void PermitUpdateBroadcast(void);
void BLE_BROADCAST_UPDATE(void);
#if FALL_DETECT_SUPPORT
void CheckFall(void);
#endif
#if SOS_HIT_SUPPORT
void CheckSOS(void);
#endif
#endif


