#ifndef common_vars_h
#define common_vars_h

#include <stdbool.h>
#include <stdint.h>
#include "typedefs.h"

#include "time.h"

//#include "cmsis_os.h"


//#define Idle_mode 0  // all of the sensors is off or the battery is almost run out
//#define Monitor_mode 1
//#define Running_mode 2


//// ====================================================
//// working mode
//#define WalkingMode  3
//#define RunningMode  2
//#define SleepMode    4
//#define StandByMode  1
//// ----------------------------------------------------

#define SYSTEM_CLOCK	(1) //1M 


// ====================================================
// key codes
#define KEY_LEFT           0x04
#define KEY_RIGHT          0x06
#define KEY_SELECT         0x08
#define KEY_UP             0x0a
#define KEY_DOWN           0x0c

#define KEY_SWITCH_MENU		0xE0 // 虚拟按键，用于在菜单之间切换
#define KEY_EXIT			0xE1 // 旧版本为虚拟按键，从KEY_RIGHT产生，现在直接对应为KEY_LEFT

#define KEY_DETAIL			KEY_SELECT
#define KEY_ENTER			KEY_DETAIL

// ====================================================
typedef enum
{
	NOTIFY_SERVICE_Other = 0,
	NOTIFY_SERVICE_IncomingCall	= 1,
	NOTIFY_SERVICE_MissedCall = 2,
	NOTIFY_SERVICE_Voicemail = 3,
	NOTIFY_SERVICE_Social = 4,
	NOTIFY_SERVICE_Schedule = 5,
	NOTIFY_SERVICE_Email = 6,
	NOTIFY_SERVICE_News = 7,
	NOTIFY_SERVICE_HealthAndFitness = 8,
	NOTIFY_SERVICE_BusinessAndFinance = 9,
	NOTIFY_SERVICE_Location = 10,
	NOTIFY_SERVICE_Entertainment = 11,
} NOTIFY_SERVICE;
// ----------------------------------------------------
// ----------------------------------------------------


// =================================================================
// 系统运行模式
typedef enum _SYSTEM_RUNNING_MODE
{
	SYSTEM_RUNNING_MODE_NORMAL = 0,
	SYSTEM_RUNNING_MODE_WORKOUT = 1,
	SYSTEM_RUNNING_MODE_LOWPOWER = 2,
} SYSTEM_RUNNING_MODE;


// =================================================================
// Beep 音量大小
typedef enum
{
	BEEP_MUTE = 0,
	BEEP_SMALL_VOLUME,
	BEEP_MEDIUM_VOLUME,
	BEEP_LARGE_VOLUME,
} BEEP_VOLUMES;


// =================================================================
// flash状态
typedef enum _FLASH_STATUS
{
	FLASH_STATUS_IDLE = 0,			// 空闲，可进行任意操纵
	FLASH_STATUS_INITIALIZING,		// 正在初始化，包括 chip erase 和 索引重建
//	FLASH_STATUS_INITIALIZING,		// 正在初始化，包括 chip erase 和 索引重建
	FLASH_STATUS_READING,			// 
	FLASH_STATUS_WRITING,			// 正在写，包括 page pragram, sector erase, block erase等
//	ReadytoWrite = 0,
//	BulkErasing = 1,
//	SectorErasing = 2,
} FLASH_STATUS;


//=================================
//message definition
// message发送至 deviceTask

typedef enum _MESSAGES
{
	MESSAGE_SYSTEM_FIRST_BOOTUP = 1,	// 系统第一次启动，通过检查systemsetting是否初始化来确定
	MESSAGE_SYSTEM_POWER_UP, // 2		// 系统上电，在MESSAGE_SYSTEM_FIRST_BOOTUP消息之后。等待用户按下硬件按键，然后开始启动
	MESSAGE_SYSTEM_STARTUP, // 2		// 系统正式启动，在MESSAGE_SYSTEM_POWER_UP消息之后。每次上电后，用户按下硬件按键都会产生
	
	BLE_RX_MSG, // =  3,
	Simu_BLE_Data,// = 4,
	TEMP_Message,// = 5,
	AFE_Message,// = 6,
	TOUCH_Message,// = 7,
	HardKey_Message,// = 8,
	SKINTOUCH_Message,// = 9,
	MESSAGE_SENSOR_ACTIVATED,// = a
	MESSAGE_SENSOR_DEACTIVATED,// = b
	MESSAGE_FLASH_OPERATION_DONE,  // c	
	TICK_Message,//  d 
	ClockSync_Message,// e
	LCD_DMA_Message,// f
	Battery_Message,// 10	
	MESSAGE_USB_CONNECTED, //  11
	MESSAGE_USB_DISCONNECTED, //12	
	MEMS_Message,  // 13
//	ThisisthefirstInit, // 14	// 被MESSAGE_SYSTEM_STARTUP取代
	TouchSensorMsg,
	MESSAGE_Usberror,
} MESSAGES;


// ====================================================
// event types
// 事件类型，根据此类型，进行不同的操作
// event 发送至 displayTask
// ----------------------------------------------------
typedef enum _EVENT_TYPE
{
	EVT_TYPE_NONE = 0,

	EVT_TYPE_BTN_HARD = (1 << 0),
	EVT_TYPE_TIMERB_EVENT,// = (1 << 1),
	EVT_TYPE_BTN_TOUCH,// = (1 << 2), //4
	EVT_TYPE_KEY,// = (1 << 3), //以上3个为原始事件，均会产生此虚拟事件

	EVT_TYPE_RTC,// = (1 << 4), //8
	EVT_TYPE_CLOCK_SYNC,// = (1 << 5),
	EVT_TYPE_AUTO_LOCKED,// = (1 << 6),
	EVT_TYPE_KEY_UNLOCKED,// = (1 << 6),
	EVT_TYPE_BLUETOOTH,// = (1 << 7),
	                              
	EVT_TYPE_USB,// = (1 << 8),
	EVT_TYPE_SUBMCU,// = (1 << 9),
	EVT_TYPE_USB_RX_DATA,// = (1 << 10),

	EVT_TYPE_BATTERY,// = (1 << 11),

	EVT_TYPE_SENSOR,// = (1 << 12),
	
	EVT_TYPE_USER = 0xFF
} EVENT_TYPE;
// ----------------------------------------------------


// =================================================================
// 电池电量
typedef enum _ENUM_BATTERY_REMAINING
{
	VCC0TO5	= (0),
	VCC1TO5	= (1),
	VCC2TO5	= (2),
	VCC3TO5	= (3),
	VCC4TO5	= (4),
	VCC5TO5	= (5)
} ENUM_BATTERY_REMAINING;

////dev_flash.h:
//#define WriteBlockSize 512
//#define ReadytoWrite 0
//#define BulkErasing 1
//#define SectorErasing 2


typedef enum
{
	GOAL_SLEEP_HOURS = 0,
	GOAL_STEPS,
	GOAL_DISTANCE,
	GOAL_CALORIES
} ENUM_USER_GOALS;

// ====================================================
// macros
//#define delay_ms(ms) __delay_cycles((CPU_CLOCK_HZ/1000) * (ms))
// ----------------------------------------------------


// ----------------------------------------------------------------------
// 结构定义

// ----------------------------------------------------------------------

//typedef struct _TOUCH_KEY
//{
//  uint8_t keycnt;
//  uint8_t flag;
//  uint8_t curkeyval;
//  uint8_t lastkeyval;
//} TOUCH_KEY;
//该结构体在globaldata.h中定义过

typedef struct _EVENT_DATA {
	unsigned short sEventType;
	void* pEventData;
} EVENT_DATA;


typedef struct _USER_EVENT
{
	int type;
	void* data;
} USER_EVENT;


typedef struct _DATE_TIME
{
//	bool bl24HourMode;
	UINT16 Year;
	signed char Month, Day, DayOfWeek, Hour, Minute, Second; 
} DATE_TIME;


typedef struct _FLASH_INFO
{
	union _FLASH_CHIP_ID
	{
		BYTE rawData[4];
		struct _FLASH_ID
		{                           //MX25U6435FZ
			BYTE manufacturerID;    //0xc2
			BYTE memoryType;        //0x25
			BYTE memoryDensity;     //0x37
			BYTE reserve;           //0x00
		} flashID;
	} chipID;
	
	int flashSize;	// 总的flash大小，byte
	int sectorSize;	// sector大小，byte
	int blockSize;	// block大小，byte
	
	int sectors;	// flash共有多少个sector
	int blocks;		// flash共有多少个block
	
	int sectorsPerBlock;	// 一个block有多少个sector
} FLASH_INFO;


#pragma pack(push, 1)

typedef struct _ALARM_SETTING
{
	bool enabled;
	
	BYTE mode;	// 0 = ALARM_MODE_ONETIME; 1 = ALARM_MODE_DAILY; 2 = ALARM_MODE_WEEKDAY
	bool weekday[7]; 	// 用于 ALARM_MODE_WEEKDAY
	
	DATE_TIME alarmTime;
} ALARM_SETTING;


typedef struct _USER_PROFILE
{
	BYTE height;	// 公制
	BYTE weight;	// 公制
	BYTE age;
	BYTE gender; // 0-woman 1-man
	BYTE unit;	// 0-英制；1-公制
} USER_PROFILE;

#pragma pack(pop)

// =================================================================
// flash 存储指示，
typedef struct _FLASH_STORAGE_INDICATOR
{
	long index;		// 存储索引号，从1开始
	int sector;		// 对应的sector号，1~(pFlashInfo->sectors-1)，保留第一个sector用于扩展
	int offset;		// 存储的偏移量，从此处可以存储新数据
	int capacity;	// sector剩余容量
	time_t startTimestamp;
	time_t endTimestamp;
	bool nextSectorIsPrepared;
} FLASH_STORAGE_INDICATOR;



// =========================================================================
// 系统设置，需保存在flash中

#define SYSTEM_SETTING_FLAG_NOT_INIT	(0xFFFFFFFF)
#define SYSTEM_SETTING_FLAG_NORMAL		(0x12345678)

// goals
#define MAX_USER_GOALS	(8)

#pragma pack(push, 4)

typedef struct _SYSTEM_SETTING
{
	// 此标志用于检查flash中的数据有效性
	// 若为 SYSTEM_SETTING_FLAG_NOT_INIT，则数据为未初始化数据
	UINT32 checkTag;
	
	// 日期、时间
	bool bl24HourMode;
	//	DATE_TIME sysDateTime;
	
	
	// ----------------------------------------------------------------------
	// 功能设置
	ALARM_SETTING alarmSetting;
	
	bool blTonesEnabled;
	
	bool blBacklightEnabled;

	UINT8 BacklightOn_Delay;

	
	// ----------------------------------------------------------------------
	// 传感器
	bool blHRSensorEnabled;
	bool blECGSensorEnabled;
	bool blAccelSensorEnabled;
	bool blAmbTempSensorEnabled;
	bool blSkinTempSensorEnabled;
	
	//
	bool blBluetoothEnabled;
	
	// ----------------------------------------------------------------------
	// 用户信息
	USER_PROFILE userProfile;
	INT32 userGoals[MAX_USER_GOALS];	// ENUM_USER_GOALS 0-sleep;1-steps;2-distance;3-calories;其余的用于后续扩展
	
	// ----------------------------------------------------------------------
	// 短信/电话等提醒
	UINT16 notifiedServices;	// 2bytes, bitmask, NOTIFY_SERVICE 参考Apple Notification Center Service中CategoryID Values
	BYTE notificationMode;		// 0-none, 1-vibration, 2-ring, 3-vibration+ring

	
	// -----------------------------------------------
	// 界面、菜单
	BYTE iMainMode;
	
} SYSTEM_SETTING;

#pragma pack(pop)


// =================================================================
//#pragma pack(push, 1)

// =================================================================
// 系统状态。运行时的动态信息，无需保存
typedef struct _SYSTEM_STATUS
{
	//
//	unsigned char FlashID[4];
	bool blFlashOnline;
	FLASH_INFO flashInfo;

	bool blFlashInitialized; // flash是否已经初始化。此状态仅在系统第一次启动时更新（做完chip erase之后更新为true）

	bool blFlashPowerOn;
	BYTE flashStatus;
	FLASH_STORAGE_INDICATOR flashStorageIndicator;
	
	bool blDataGatheringInitialized;	// 此标志用于上电后第一个采集数据到达时，写入时间戳
	
	//
	struct tm systemTime;

//	//
//	UINT32 energyMode;
	
	//
	bool blHRSensorOnline;
	bool blECGSensorOnline;
	bool blAccelSensorOnline;
	bool blAmbTempSensorOnline;
	bool blSkinTempSensorOnline;
	bool blAllSensorsOff;
	
	//
	bool blUsbOnline;
	
	bool blBluetoothConnected;
	
	//
//	bool systemStatus.blFlashOnline;
//	bool blSubBoardOnline;
//	bool Enable_BatVcc_Monitor;
//	bool Watch_On_Wrist;
//	
//	
//	volatile BYTE MemoryStatus;
//	bool blMemoryFull;
	
	//
	bool blLowBatteryFlag;
	bool blBatteryCharging;
	
	//UINT32 dwBatteryLevel;
	ENUM_BATTERY_REMAINING bBatteryRemaining;

	// 
	BYTE bAlarmStatus;	// 闹钟状态，0=未触发；1=触发，响铃；2=被用户停止
	BYTE bAlarmDuration;	// 闹钟持续时间，按秒计
	
	//
	bool blKeyAutoLocked;	// 按键锁定标志
	
	//
	bool blHeartBeatLock;	// 检测到有效心跳数据
} SYSTEM_STATUS;

//#pragma pack(pop)


// ----------------------------------------------------------------------
// 以下为变量定义
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
// 系统
//extern unsigned char Flash_ID[4];

extern SYSTEM_SETTING systemSetting;
extern SYSTEM_STATUS systemStatus;
extern float BMR_PER_SECOND ;

//extern bool systemStatus.blFlashOnline;
extern bool blSubBoardOnline;
extern bool Enable_BatVcc_Monitor;
extern bool Watch_On_Wrist;



//extern volatile FLASH_STATUS flashStatus;
//extern bool blMemoryFull;

extern FLASH_INFO* pFlashInfo;
extern volatile FLASH_STORAGE_INDICATOR* pFlashStorageIndicator;

//UINT16 A_Temp_float_2bytes;
//UINT16 S_Temp_float_2bytes;

//extern bool blHeartBeatLock;

// ----------------------------------------------------------------------
// 用户信息
extern USER_PROFILE* pUserProfile;


// ----------------------------------------------------------------------
// 实时指标、数据
extern volatile UINT	iHeartRate;
extern volatile UINT  	iCalories;
extern volatile UINT  	iSteps;
extern volatile UINT  	iDistance;

//extern volatile float fAmbientTemperature;
extern volatile float bSkinTemperature;


// ----------------------------------------------------------------------
// 日期、时间
//static const unsigned char month_days[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/* The current time reference. Number of seconds since midnight
 * January 1, 1970.  */
extern struct tm* pSystemTime;


// ----------------------------------------------------------------------
// 功能设置
typedef enum _ALARM_MODE
{
	ALARM_MODE_ONETIME = 0,
	ALARM_MODE_DAILY = 1,
	ALARM_MODE_WEEKDAY = 2
} ALARM_MODE;

extern ALARM_SETTING* pAlarmSetting;



extern unsigned char Battery_Flash_Cout;



// -----------------------------------------------
// 界面、菜单
extern bool blDisplayModeNormal;
//BYTE bBackgroundColor = 0x00;

//extern unsigned char iMainMode;


// -----------------------------------------------
// 按键
//#define system_tick_hz  64
//#define REPEAT_INTERVAL         (system_tick_hz / 4)
//#define INITIAL_REPEAT_DELAY    (system_tick_hz / 2)

#define APP_HARD_KEY		1
#define APP_TOUCH_UP		2
#define APP_TOUCH_DOWN		3
#define APP_TOUCH_LEFT		4
#define APP_TOUCH_RIGHT		5

//#define ButtonLockDelay  12
//#define ButtonUnLockDelay  3

//extern TOUCH_KEY touchkey;
//extern bool GulTouchKeyMode;
//extern unsigned char mButtons;
//extern unsigned char mTOUCH;
//extern unsigned char mRepeatButton;
//extern unsigned short mRepeatCount;
//extern BYTE bKeyRepeatDelay;
//
//
////extern bool blButtonLocked;
//
//
//// -----------------------------------------------
////
//extern osMailQId hDispEventQueue;


#endif

