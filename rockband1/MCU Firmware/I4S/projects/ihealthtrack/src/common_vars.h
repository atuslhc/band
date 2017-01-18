#ifndef common_vars_h
#define common_vars_h

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>

#include "typedefs.h"

#include "time.h"

/* The section defines are reference only. We porting to IAR
 * configuration options - CCPreprocessor define for multiple
 * configurations in one source code tree for development.
* The symbol xxx in CCdefine equivalent as #define xxx   1
 */
#if defined(MODEL_TYPE) //1 //The macro we set in the options of CC Preprocessor
#if (MODEL_TYPE==1) //HEALTHCARE_TYPE
#if !defined(VENDOR_TYPE)
#define VENDOR_TYPE                     2
#endif
#if !defined(BOARD_TYPE)
#error Stop!, please specify the BOARD_TYPE first!
#define BOARD_TYPE                      2 //rockband_dev      
#endif
#define FALL_DETECT_SUPPORT             1
#define SOS_HIT_SUPPORT                 1
#define BATTERY_LIFE_OPTIMIZATION       0       //just use in i4_H code, will skip write flash.
#define GESTURE_DISP_SUPPORT            0
#define PPG_WORK_MODE_SET		        0
#define ENABLE_PAUSE_STOPWATCH		    0
#define BATTERY_LIFE_OPTIMIZATION2	    1	//turn off some notify.
//#define ENABLE_DATA_GATHER_DOUBLE_BUFFER 0	//keep define in data_gather.c L16
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
#define AFE44x0_SUPPORT                 1
#define OLED_SUPPORT                    1
#define TEMPERATURE_SUPPORT             1
#define VIBRATION_SUPPORT               1
#define CHARGER_SUPPORT                 1
#define ACCELEROMETER_SUPPORT           1
#define UVSENSOR_SUPPORT                1
#define BAROMETER_SUPPORT               0   //[BG037] add the pressure sensor-LPS22HB,LPS35HW driver.
#define GYRO_SUPPORT                    0 //add the gyro sensor-L3GD20H driver.
#define MAGNETIC_SUPPORT                0 //add the magnetic sensor-BMM150 driver.
#define CAP_SUPPORT                     1   //add the AD7156 cap sensor.
#define BLE_SUPPORT                     1
#define BATTERY_SUPPORT                 1
#elif (BOARD_TYPE==2)
#define AFE44x0_SUPPORT                 0
#define OLED_SUPPORT                    0
#define TEMPERATURE_SUPPORT             0
#define VIBRATION_SUPPORT               0
#define CHARGER_SUPPORT                 0
#define ACCELEROMETER_SUPPORT           1
#define UVSENSOR_SUPPORT                1
#define BAROMETER_SUPPORT               1   //[BG037] add the pressure sensor-LPS22HB,LPS35HW driver.
#define GYRO_SUPPORT                    1 //add the gyro sensor-L3GD20H driver.
#define MAGNETIC_SUPPORT                1 //add the magnetic sensor-BMM150 driver.
#define CAP_SUPPORT                     2   //add the AD7156 cap sensor.
#define BLE_SUPPORT                     1
#define BATTERY_SUPPORT                 2
#define BAT_ADC_CONFIG                  2  //1 //1:R9=10M and reference internal 2.5V. 2:R9=5.6M and reference internal 1.25V.
#endif

#elif (MODEL_TYPE==2) //CONSUMER_TYPE
#if !defined(VENDOR_TYPE)
#define VENDOR_TYPE                     2
#endif
#define FALL_DETECT_SUPPORT             0
#define SOS_HIT_SUPPORT                 0
#define BATTERY_LIFE_OPTIMIZATION       0
#define GESTURE_DISP_SUPPORT            1
#define PPG_WORK_MODE_SET		        1
#define ENABLE_PAUSE_STOPWATCH		    0
#define BATTERY_LIFE_OPTIMIZATION2	    0
//#define ENABLE_DATA_GATHER_DOUBLE_BUFFER 0
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
#define AFE44x0_SUPPORT                 1
#define OLED_SUPPORT                    1
#define TEMPERATURE_SUPPORT             1
#define VIBRATION_SUPPORT               1
#define CHARGER_SUPPORT                 1
#define ACCELEROMETER_SUPPORT           1
#define UVSENSOR_SUPPORT                1
#define BAROMETER_SUPPORT               0   //[BG037] add the pressure sensor-LPS22HB,LPS35HW driver.
#define GYRO_SUPPORT                    0 //add the gyro sensor-L3GD20H driver.
#define MAGNETIC_SUPPORT                0 //add the magnetic sensor-BMM150 driver.
#define CAP_SUPPORT                     1   //add the AD7156 cap sensor.
#define BLE_SUPPORT                     1
#define BATTERY_SUPPORT                 1
#elif (BOARD_TYPE==2)
#define AFE44x0_SUPPORT                 0
#define OLED_SUPPORT                    0
#define TEMPERATURE_SUPPORT             0
#define VIBRATION_SUPPORT               0
#define CHARGER_SUPPORT                 0
#define ACCELEROMETER_SUPPORT           1
#define UVSENSOR_SUPPORT                1
#define BAROMETER_SUPPORT               1   //[BG037] add the pressure sensor-LPS22HB,LPS35HW driver.
#define GYRO_SUPPORT                    1 //add the gyro sensor-L3GD20H driver.
#define MAGNETIC_SUPPORT                1 //add the magnetic sensor-BMM150 driver.
#define CAP_SUPPORT                     2   //add the AD7156 cap sensor.
#define BLE_SUPPORT                     1
#define BATTERY_SUPPORT                 2
#endif
#else
#error "Not specify product type MODEL_TYPE!!!"
#endif
#endif

#if (BATTERY_SUPPROT==1)
#define BATTERY_REMAINING_OUT_OF_BATTERY	13
#define BATTERY_REMAINING_LOW_BATTERY		23
#define BATTERY_LEVEL_TOLERANCE 			3
#elif (BATTERY_SUPPORT==2)
#define BATTERY_REMAINING_OUT_OF_BATTERY	5
#define BATTERY_REMAINING_LOW_BATTERY		10
#define BATTERY_LEVEL_TOLERANCE 			3
#else
#define BATTERY_REMAINING_OUT_OF_BATTERY	13
#define BATTERY_REMAINING_LOW_BATTERY		23
#define BATTERY_LEVEL_TOLERANCE 			3
#endif


//#include "cmsis_os.h"
#define BG039   1       //turn on the WDOG setting with MODEL_TYPE 1

#define BG009   0       //for debug active_level show value.
#define BG013   0 //1       //debug show the raw data value range.
#define BG013MS 0 //1       //manual set value. LED_INTENSITY,AMB_uA
#define BG013_LED_ADJ_METHOD 0 //1
#if (BG013_LED_ADJ_METHOD==0)
#define LED_INTENSITY_MS_MAX    200
#define LED_INTENSITY_MS_MIN    5
#define AMB_UA_MS_MAX   5
#define AMB_UA_MS_MIN   0
#elif (BG013_LED_ADJ_METHOD==1)
#define LED_INTENSITY_MS_MAX    200
#define LED_INTENSITY_MS_MIN    5
#define AMB_UA_MS_MAX   5
#define AMB_UA_MS_MIN   0
#else
#stop not define BG013_LED_ADJ_METHOD!
#endif
#define BG013RAW        0 //1
#define DEBUGLOG	1 //[BG034] create.
//#define BGXXX   9 //1       //for try debug
//#define BG030_FIX	1	//verify done, force change to 1
//#define BG033_FIX	1	//verify done, force change to 1 
/*
1: for AFE4400 IR, //BG013
2: for GESTURE count show
3: for SOS_HIT debug
4: for UVIndex and UVgather value debug //menu
5: for BLE read/send buffer debug
6: for Upload_Sector_Num in Download process debug
7: for BLE parsing cc2560 status debug.
8: for BLE upgrade stop debug.
9: for charging battery debug.
*/



//// ====================================================
//// working mode
//#define WalkingMode  3
//#define RunningMode  2
//#define SleepMode    4
//#define StandByMode  1
//// ----------------------------------------------------

#define SYSTEM_CLOCK	(1) //1 MHz
#define RESET_TIME_TO	112 //The time mapping to 2012. struct tm->year base is 1900. 2012-1900=112

// ====================================================
//typedef enum
//{
//	DEVICE_MODEL_I2 = 2,
//	DEVICE_MODEL_I4 = 4,
//} DEVICE_MODEL;
//
//#define DEVICE_SUB_MODEL_STANDARD (0)
//
//#define MAKE_DEVICE_MODEL(m, sm) 	((BYTE) (((m & 0x0F) << 4) + (sm & 0x0F)))
//#define GET_DEVICE_MODEL(m) 		((DEVICE_MODEL) ((m >> 4) & 0x0F))
//#define GET_DEVICE_SUB_MODEL(m) 	((BYTE) (m & 0x0F))
//
//extern BYTE DeviceModel;

/* key codes */
#define KEY_LEFT	0x04
#define KEY_RIGHT	0x06
#if 0  //Not used in this version.
#define KEY_SELECT	0x08
#define KEY_UP		0x0a
#define KEY_DOWN	0x0c

#define KEY_SWITCH_MENU	0xE0 // virtual button, switch the menu.
#define KEY_EXIT	0xE1 // old version is virtual button, come from KEY_RIGHT. mapping to KEY_LEFT now.

#define KEY_DETAIL	KEY_SELECT
#define KEY_ENTER	KEY_DETAIL
#endif

// ====================================================
#define MAX_NOTIFY_EVENTS	(32)
/* The first 12 type service are IOS ANCS deifned CategoryID. In the Android's APP, also define some
   notifications. In Android's APP, we use NOTIFY_SERVICE_Email as brief news inform. */
typedef enum
{
	NOTIFY_SERVICE_Other = 0,
	NOTIFY_SERVICE_IncomingCall = 1,
	NOTIFY_SERVICE_MissedCall = 2,
	NOTIFY_SERVICE_Voicemail = 3,
	NOTIFY_SERVICE_Social = 4,	//IOS brief news in the type.
	NOTIFY_SERVICE_Schedule = 5,
	NOTIFY_SERVICE_Email = 6, //android define this as message flag.
	NOTIFY_SERVICE_News = 7,
	NOTIFY_SERVICE_HealthAndFitness = 8,
	NOTIFY_SERVICE_BusinessAndFinance = 9,
	NOTIFY_SERVICE_Location = 10,
	NOTIFY_SERVICE_Entertainment = 11,
	NOTIFY_SERVICE_Step_Accomplish = 12,
	NOTIFY_SERVICE_Calorie_Accomplish = 13,
	NOTIFY_SERVICE_Distance_Accomplish = 14,

	NOTIFY_SERVICE_Alarm = 29,
	NOTIFY_SERVICE_Intense_UV = 30,
	NOTIFY_SERVICE_Battery = 31,
} NOTIFY_SERVICE;

/* system mode */
typedef enum
{
	SYSTEM_MODE_MANUFACTORING = 0,		// manufactory test mode.
	SYSTEM_MODE_RELEASED = 0x34,		// release(shipping) mode. (original tag 0x1234)
	SYSTEM_MODE_ACTIVATED = 0x78,		// user activated mode. (original tag 0x5678)
} SYSTEM_MODE;

#if 0  //Not used in this version.
/* system running mode */
typedef enum _SYSTEM_RUNNING_MODE
{
	SYSTEM_RUNNING_MODE_NORMAL = 0,
	SYSTEM_RUNNING_MODE_WORKOUT = 1,
	SYSTEM_RUNNING_MODE_LOWPOWER = 2,
} SYSTEM_RUNNING_MODE;

/* Beep volume */
typedef enum
{
	BEEP_MUTE = 0,
	BEEP_SMALL_VOLUME,
	BEEP_MEDIUM_VOLUME,
	BEEP_LARGE_VOLUME,
} BEEP_VOLUMES;
#endif

/* flash status */
typedef enum _FLASH_STATUS
{
	FLASH_STATUS_IDLE = 0,			// idle, can do any operation.
	FLASH_STATUS_INITIALIZING,		// in initializing, include chip erase, and rebuilt index
	FLASH_STATUS_READING,			// read action.
	FLASH_STATUS_WRITING,			// In writing, include page program, ssector erase, block erase, etc.
//	ReadytoWrite = 0,
//	BulkErasing = 1,
//	SectorErasing = 2,
} FLASH_STATUS;

/* message definition, send to deviceTask. */
typedef enum _MESSAGES
{
	MESSAGE_SYSTEM_FIRST_BOOTUP = 1,	//1, system first time bootup, by check the systemsetting.checkTag initialized or not.
	MESSAGE_SYSTEM_POWER_UP,     //2, system power on,after MESSAGE_SYSTEM_FIRST_BOOTUP, wait the user press button to activated.
	MESSAGE_SYSTEM_STARTUP,     //3, system startup, after MESSAGE_SYSTEM_POWER_UP, the user press button after power on.
	MESSAGE_SYSTEM_MODE_CHANGED, //4, system mode changed. Ex. change from manufactoring to release, or release to activated.

	BLE_RX_MSG,                 //5,
	Simu_BLE_Data,              //6,
//	TEMP_Message,// = 7,
	AFE_Message,                //7,
	TOUCH_Message,              //8,
	HardKey_Message,            //9,
	//SKINTOUCH_Message,// = 10,

	MESSAGE_SENSOR_ACTIVATED,	//10,sensor activated
	MESSAGE_SENSOR_DEACTIVATED,	//11, sensor deactivated

	MESSAGE_FLASH_OPERATION_DONE,  //12
	TICK_Message,               //13
	MESSAGE_TIMER,              //14
	MESSAGE_CLOCK_SYNC,         //15
	MESSAGE_ALARM_SYNC,         //16
	LCD_DMA_Message,            //17
	MESSAGE_BATTERY_CHARGING,	//18,battery charging status(charging/not charging(normal))
	MESSAGE_BATTERY_LEVEL,		//19,battery power changed(power level)
	MESSAGE_USB_CONNECTED,      //20
	MESSAGE_USB_DISCONNECTED,   //21
	MEMS_INT1_Message,          // 22
	TouchSensorMsg,             //23
	MESSAGE_BLE_TIMING_Event,   //24
	MEMS_INT2_Message,          //25

	MESSAGE_FIND_ME,            //26
	NOTIFICATION,				//27, one of notification event

	MESSAGE_ENTER_MENU,			//28, swtich to a menu, parameter is the menu type.
	MESSAGE_MENU_ACTION,		// 29,a long press button in a menu, the parameter is the menu type.

	MESSAGE_TASK_HEARTBEAT,		//30, task of heart beat inform, the displayTask/flashTask send to deviceTask per second, to issue a keepalive.
#if (GYRO_SUPPORT)
    MESSAGE_L3GD20H_INT1,       //31
    MESSAGE_L3GD20H_INT2,       //32
#endif
#if (BOARD_TYPE==2)
    KEY1_INT2_Message,          //33, for test button1
    KEY2_INT5_Message,          //34, for test button2
#endif
} MESSAGE_TYPES;

typedef union _MESSAGE
{
	struct
	{
		uint16_t type;			// the value defin in MESSAGE_TYPES
		uint16_t param;
	} params;

	uint32_t id;
} MESSAGE;

/* sensor types */
typedef enum _SENSOR_TYPE
{
	SENSOR_TYPE_PPG = 1,
	SENSOR_TYPE_ECG,
	SENSOR_TYPE_ACCELEROMETER,
	SENSOR_TYPE_AMBIENT_THERMOMETER,
	SENSOR_TYPE_SKIN_THERMOMETER,
	SENSOR_TYPE_UV,
	SENSOR_TYPE_SKIN_TOUCH,
} SENSOR_TYPE;

/* event types: event send to displayTask, base on the type do action. */
typedef enum _EVENT_TYPE
{
	EVT_TYPE_NONE = 0,

	// keys
	EVT_TYPE_BTN_HARD = (1 << 0),
	EVT_TYPE_TIMERB_EVENT,
	EVT_TYPE_BTN_TOUCH,
	//above 3 event is physical event, and will make the next EVT_TYPE_KEYvvirtual event.
	EVT_TYPE_KEY,

	EVT_TYPE_AUTO_LOCKED,
	EVT_TYPE_KEY_UNLOCKED,

	// rtc
	EVT_TYPE_RTC,
	EVT_TYPE_CLOCK_SYNC,

	// alarm
	EVT_TYPE_ALARM_SYNC,

	// communication
	EVT_TYPE_BLUETOOTH,
//#if 0		//I4C define, we make them same enum unmark.
	//As the I4H, did not have these events
	EVT_TYPE_USB,
	EVT_TYPE_SUBMCU,
	EVT_TYPE_USB_RX_DATA,
//#endif
	// battery
	EVT_TYPE_BATTERY,

	// sensor
	EVT_TYPE_SENSOR_STATUS_CHANGED,

	// notification
	EVT_TYPE_NOTIFICATION,

	//download firmware
	EVT_TYPE_DOWNLOAD,

	//
	EVT_TYPE_USER = 0xFF
} EVENT_TYPE;
// ----------------------------------------------------


/* battery power level */
typedef enum _ENUM_BATTERY_LEVEL
{
	OUT_OF_BATTERY = 0,	// will be run out.
	LOW_BATTERY,		// low power level
	BATTERY_NORMAL,		// normal power level
} ENUM_BATTERY_LEVEL;



/* display/graphic mode */
#define DRAW_MODE_NORMAL	(0)
#define DRAW_MODE_OVERWRITE	DRAW_MODE_NORMAL
#define DRAW_MODE_TRANSPARENT	(1 << 1)
#define DRAW_MODE_XOR		(1 << 2)
#define DRAW_MODE_INVERT	(1 << 3)	// invert soruce



/* Temperature Positive/Negative flag,true indicate negative temperature. */
extern bool ambTempFlag;
extern bool skinTempFlag;

/* ----------------------------------------------------------------------
 * structure definition
 * ----------------------------------------------------------------------*/
#if 0
typedef struct _TOUCH_KEY
{
	uint8_t keycnt;
	uint8_t flag;
	uint8_t curkeyval;
	uint8_t lastkeyval;
} TOUCH_KEY;
#endif

typedef struct _EVENT_DATA
{
	unsigned short sEventType;
	void* pEventData;
} EVENT_DATA;


typedef struct _USER_EVENT
{
	int type;
	union
	{
		void* p;
		uint32_t v;
	} data;
} USER_EVENT;


typedef struct _DATE_TIME
{
	UINT16 Year;
	unsigned char Month, Day, DayOfWeek, Hour, Minute, Second;
} DATE_TIME;


typedef struct _FLASH_INFO
{
	union _FLASH_CHIP_ID
	{
		BYTE rawData[4];
		struct _FLASH_ID
		{
			BYTE manufacturerID;
			BYTE memoryType;
			BYTE memoryDensity;
			BYTE reserve;
		} flashID;
	} chipID;

	int flashSize;		// total flash size (bytes).
	int sectorSize;		// a sector size (bytes)
	int blockSize;		// a block size (bytes)

	int sectors;		// how many sectors in the flash.
	int blocks;		// how many blocks in the flash.

	int sectorsPerBlock;	// how many sectors in a block.
} FLASH_INFO;


#pragma pack(push, 1)

// Heart Rate
typedef union
{
	UINT16 data;

	struct
	{
		BYTE heart;
		BYTE reliability;
	} component;
} HEART_RATE;

typedef struct _ALARM_SETTING
{
	bool enabled;
	BYTE mode;		// 0 = ALARM_MODE_ONETIME; 1 = ALARM_MODE_DAILY; 2 = ALARM_MODE_WEEKDAY
	bool weekday[7]; 	// apply while mode=ALARM_MODE_WEEKDAY,[0]=sunday, [1]=monday...
	DATE_TIME alarmTime;
} ALARM_SETTING;

typedef enum
{
	FEMALE = 0,
	MALE = 1,
} ENUM_GENDER;

typedef enum
{
	UNIT_ENGLISH = 0,
	UNIT_METRIC = 1,
} ENUM_UNIT_SYSTEM;

typedef struct _USER_PROFILE
{
	BYTE height;		// cm
	BYTE weight;		// kg
//	BYTE age;
	UINT16 birthYear;
	ENUM_GENDER gender; 	// 0:woman 1:man
	ENUM_UNIT_SYSTEM unit;	// 0:UNIT_ENGLISH, 1:UNIT_METRIC
} USER_PROFILE;

typedef enum
{
	AUTOLOCK_FLAG_CHARGING = 1 << 0,
	AUTOLOCK_FLAG_TESTING_MENU = 1 << 1,
	AUTOLOCK_FLAG_FIND_ME = 1 << 2,
	AUTOLOCK_FLAG_DEMO = 1 << 3,
	AUTOLOCK_FLAG_NOTIFICATION = 1 << 4,	// use notify/inform message. Ex, telephone coming.
	AUTOLOCK_FLAG_BATTERY_DRAIN = 1 << 5,	// use while want to drain out the battery.
} ENUM_DISABLE_AUTOLOCK_FLAGS;

#pragma pack(pop)

/* flash storage layout schema v2:
 *  a flash sector will divide to 8 chunks, a chunk size is 508 bytes.
 *  A historical data write will use a chunk. So, we can skip the remain size,
 * just check how many of chunk in current sector. */

#define FLASH_STORAGE_SCHEMA_V2

#ifdef FLASH_STORAGE_SCHEMA_V2
#define INDEX_DATA_CHUNK_SIZE	508
#define INDEX_DATA_BUFFER_SIZE	INDEX_DATA_CHUNK_SIZE
#else
#endif

/* flash storage indicator */
typedef struct _FLASH_STORAGE_INDICATOR
{
	long index;		// index number, start from 1, and increament 1.
	uint16_t sector;	// mapping to sector number, from 1~(pFlashInfo->sectors-1). reserved the first sector(0) as extension.
	uint16_t offset;	// the stored address offset. The new stored data writtern to here; In FLASH_STORAGE_SCHEMA_V2, did not use in stored data(write), just use in read(upload) data.
	uint16_t capacity;	// sector remain size; [BG030] change uint8_t to uint16_t.
				//in FLASH_STORAGE_SCHEMA_V2, the variable is chunk number, initialize as 8, indicate 8 chunks available.
				// Atus: if not SCHEMA_V2, how can record the sector size with uint8_t? 
	time_t startTimestamp;
	time_t endTimestamp;
	bool nextSectorIsPrepared;
} FLASH_STORAGE_INDICATOR;

#if DEBUGLOG
/* flash debug log storage indicator */
typedef struct _FLASH_DEBUGLOG_INDICATOR
{
	long index;		// index number, start from 1, and increament 1.
	uint16_t sector;	// mapping to sector number, from 1~(pFlashInfo->sectors-1). reserved the first sector(0) as extension.
	uint16_t offset;	// the stored address offset. The new stored data writtern to here; In FLASH_STORAGE_SCHEMA_V2, did not use in stored data(write), just use in read(upload) data.
	uint16_t capacity;	// sector remain size; [BG030] change uint8_t to uint16_t.
				//in FLASH_STORAGE_SCHEMA_V2, the variable is chunk number, initialize as 8, indicate 8 chunks available.
				// Atus: if not SCHEMA_V2, how can record the sector size with uint8_t? 
	time_t startTimestamp;
	time_t endTimestamp;
	bool nextSectorIsPrepared;
} FLASH_DEBUGLOG_INDICATOR;
#endif

// system settings, stored in flash.
#define SYSTEM_SETTING_FLAG_NOT_INIT	(0xFFFFFFFF)
#define SYSTEM_SETTING_FLAG_NORMAL	(0x12345678)

// goals
#define MAX_USER_GOALS	(8)

typedef enum
{
  GOAL_SLEEP = 0,		// 0: set the sleep hours.
	GOAL_STEPS,		// 1: set the walking steps.
	GOAL_DISTANCE,		// 2: set the walking distance (km).
	GOAL_CALORIES,		// 3: set the execise kilocalories.
	GOAL_UVexpT,		// 4 [BG023-1]. set the UV exposion time (minutes).
	GOAL_RESV5,		// 5 reserved for future
	GOAL_RESV6,		// 6 reserved for future
	GOAL_RESV7		// 7 reserved for future
} ENUM_USER_GOALS;

#define DEFAULT_GOAL_SLEEP	(8)	//hours
#define DEFAULT_GOAL_STEPS	(10000)	//steps
#define DEFAULT_GOAL_DISTANCE	(1200)	//km
#define DEFAULT_GOAL_CALORIES	(3000)	//kilocalories
#define DEFAULT_GOAL_UVexp	(10)	//minutes [BG023-1]


#define DEVICE_COLOR_BLACK     0x20
#define DEVICE_COLOR_RED       0x40

#pragma pack(push, 4)

typedef struct _SYSTEM_SETTING
{
	UINT32 checkTag;		//The flag check the flash data validation. If SYSTEM_SETTING_FLAG_NOT_INIT is not initialized yet.

	bool bl24HourMode;		//hour(24/12) mode, not used now.
	//	DATE_TIME sysDateTime;

	/* Function Setting */
	ALARM_SETTING alarmSetting;
	bool blTonesEnabled;		//not bind function yet.
	bool blBacklightEnabled;	//not bind function yet.
	UINT8 BacklightOn_Delay;	//not bind function yet. ATUSOZ:should be OLED display function.

	/* Sensors */
	bool blHRSensorEnabled;
	bool blUVSensorEnabled;
	bool blAccelSensorEnabled;
	bool blAmbTempSensorEnabled;
	bool blSkinTempSensorEnabled;
	//
	bool blBluetoothEnabled;

	/* User information (profile, goals) */
	USER_PROFILE userProfile;
	INT32 userGoals[MAX_USER_GOALS];// ENUM_USER_GOALS 0-sleep;1-steps;2-distance;3-calories; the others as extension.

	/* SMS notification service */
	UINT16 notifiedServices;	// 2bytes, bitmask, NOTIFY_SERVICE ?傝�傾pple Notification Center Service涓瑿ategoryID Values
	BYTE notificationMode;		// 0-none, 1-vibration, 2-ring, 3-vibration+ring

	BYTE iMainMode;			// not bind function yet.

	UINT32 iMenuEnableFlags;	// menu enabled flag.

	UINT16 SKIN_CAPVAL;		//skin/bottom capture value.
	UINT16 TOUCH_CAPVAL;		//touch button capture value.

	/* system running data */
	UINT16 SystemRstTimes;		//system(device) reset times(counter)
	UINT32 FactoryTime;		//store timestamp while change to SYSTEM_MODE_RELEASED
	UINT8 SystemMode; 		// SYSTEM_MODE_RELEASED = released, SYSTEM_MODE_ACTIVATED = device activated, others = factory mode
	UINT8 reserved;

	/* device findMe setting */
	BYTE findMeMode;		// 0-none, 1-vibration, 2-ring, 3-vibration+ring
	BYTE findMeDuration;		//  unit is second.

	bool blTouchSensorEnabled;	// bottom metal touch sensor.

	UINT32 ActivateTime;		//store timestamp while change to SYSTEM_MODE_ACTIVATED

	int16_t timezoneOffset;		// timezone from Utc/GMT, unit is minute.
	//
	BYTE bDeviceModel;		// device model number.i4
	BYTE ppgRunMode;	//I4C use, we define both.
	BYTE deviceColor;	//I4S/H use, we defin both.
	BYTE res[5];		//6 >> 5 for combine deviceColor and ppgRunMode.
} SYSTEM_SETTING;

#pragma pack(pop)

//=====================The backup area stored somes accumulated type data================================
#define I_CALORIES                  BURTC->RET[127].REG
#define I_CALORIES_LASTSAVING       BURTC->RET[126].REG
#define I_STEPS                     BURTC->RET[125].REG
#define I_STEPS_LASTSAVING          BURTC->RET[124].REG
#define I_DISTANCE                  BURTC->RET[123].REG
#define I_DISTANCE_LASTSAVING       BURTC->RET[122].REG
#define ACTIVE_LEVEL                BURTC->RET[121].REG
#define ACTIVE_LEVEL_LASTSAVING     BURTC->RET[120].REG


#define max_SkinTouchVal  systemSetting.SKIN_CAPVAL
#define TOUCHCAPVAL systemSetting.TOUCH_CAPVAL

#define NOTIFY_SENDER_BUFFER_SIZE 20

// =================================================================
//#pragma pack(push, 1)

// =================================================================
// system status. The running information, just save in ram, not necessary store to flash.
typedef struct _SYSTEM_STATUS
{
//	unsigned char FlashID[4];
	bool blFlashOnline;
	FLASH_INFO flashInfo;

	bool blFlashInitialized; //flash initialized is means been scanned, and ready for access.
				  // The system start run will update it. (FLASH_CMD_INIT done, change to true) 
	bool blFlashPowerOn; // flash power on?, flash in low power mode normally, in the read/write need power on.

	bool blDataGatheringInitialized; // The flag use in first time startup or restart data gathering.
	bool blEnableDataGathering;

	BYTE flashStatus;
	FLASH_STORAGE_INDICATOR flashStorageIndicator;

	struct tm systemTime;

//	UINT32 energyMode;

	bool blHRSensorOnline;
	bool blECGSensorOnline;
	bool blAccelSensorOnline;
	bool blAmbTempSensorOnline;
	bool blSkinTempSensorOnline;
	bool blUVSensorOnline;
#if (BAROMETER_SUPPORT==1)
    bool blPressureSensorOnline; //[BG037] BAROMETER add.
#endif
#if (GYRO_SUPPORT==1)
    bool blGyroSensorOnline; //[BG040] GYRO add.
#endif
#if (MAGNETIC_SUPPORT)
    bool blGeoMSensorOnline; //[BG041] magnetic add.
#endif
	bool blAllSensorsOff;

	bool blHeartBeatLock; // Get the validated HR data.
	bool blHRSensorOn; // ppg sensor turn on or not, the sensor status.
	int8_t iAutoHRWarning; // In the AUTO ppg state, prompt user switch ppg operation invalid.
	bool blHRSensorTempEnabled; //Use to turn on/off ppg sensor temporary.

	bool blUsbOnline;

	bool blBleOnline;
	int8_t iBleHeartBeatCounter;
	bool blBluetoothConnected;

	bool blVibratorOnline; // vibration motor

	bool blStopwatchEnabled; // enable the stopwatch mode, in the time menu long press.
				 // In the mode, time menu dispaly the second, steps, distance, calories will reset to 0.
	bool blStopwatchPaused;	//stopwatch pause.

//	bool systemStatus.blFlashOnline;
//	bool blSubBoardOnline;
//	bool Enable_BatVcc_Monitor;
//	bool Watch_On_Wrist;
//
//	volatile BYTE MemoryStatus;
//	bool blMemoryFull;

//	bool blOutOfBatteryFlag; // the flag can skip the bBatteryRemaining check?
//	bool blLowBatteryFlag; // The flag maybe not necessary, because we have the low battery, out of battery mode, we can check the bBatteryRemaining.
	ENUM_BATTERY_LEVEL bBatteryLevel; // The battery capacity.
	bool blBatteryCharging;		// Charging status.

	uint8_t bBatteryRemaining;	// The  percentage of battery remain.

	bool blBatteryDraining;		// [state] Drain battery mode. (Turn on vibration continuous).

	BYTE bAlarmStatus; // alarm status.0=not trigger; 1=trigger, ring; 2=disabled
	BYTE bAlarmDuration;	// alarm keep duration time (seconds)

	BYTE bDisableAutoLockFlags;	// [setting] If the event inhibit the auto lock screen, set the relative bit in this flag. if the bit 0, indicate allow auto lock/press button. initialize 0.
	bool blKeyAutoLocked;	// [state] press key lock.

	// event notifications
	bool blNotificationsReaded;
	BYTE notifyEvents[MAX_NOTIFY_EVENTS];	// notify event list. refer NOTIFY_SERVICE

	char incomingCallNumber[NOTIFY_SENDER_BUFFER_SIZE];		// The last coming call phone number.
	char latestSmsNumber[NOTIFY_SENDER_BUFFER_SIZE];		// The last brief news phone number.
//	bool blIncomingCall;	// The flag of get coming call phone number.
//	BYTE bMissingCalls;		// missing calls counts
//	BYTE bUnreadMessages;	// The unread message counts

	// find me
	bool blFindingMe;
	int iFindingMeCount;


	// uv alarm
	bool blUVAlarmDisabled;
	time_t iUVAlarmDisabledTime;

	bool blSkinTouched;	// [state] skin touched (watch weared or not)

	bool blSystemSettingsCrcOk;	// [state] the crc check of systemsettings result.
#if DEBUGLOG
	FLASH_DEBUGLOG_INDICATOR flashDebugLogIndicator;
#endif
} SYSTEM_STATUS;

//#pragma pack(pop)


// ----------------------------------------------------------------------
// The variables declare below
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
// system
//extern unsigned char Flash_ID[4];

extern SYSTEM_SETTING systemSetting;
extern SYSTEM_STATUS systemStatus;
extern float BMR_PER_SECOND, BMR_RATE ;

//extern bool systemStatus.blFlashOnline;
//extern bool blSubBoardOnline;
//extern bool Enable_BatVcc_Monitor;
//extern bool Watch_On_Wrist;

//extern volatile FLASH_STATUS flashStatus;
//extern bool blMemoryFull;

extern FLASH_INFO* pFlashInfo;
extern volatile FLASH_STORAGE_INDICATOR* pFlashStorageIndicator;

//UINT16 A_Temp_float_2bytes;
//UINT16 S_Temp_float_2bytes;

//extern bool blHeartBeatLock;

// user profile
extern USER_PROFILE* pUserProfile;


// realtime data
extern volatile HEART_RATE	iHeartRate;
extern volatile UINT  	iCalories, iCalories_lastSaving;
extern volatile UINT  	iSteps, iSteps_lastSaving;
extern volatile UINT  	iDistance, iDistance_lastSaving;
extern volatile UINT  active_level, active_level_lastSaving, active_level_delta;
#if BG009
extern volatile UINT  active_level_max, active_level_min; //[BG009] debug
#endif
extern volatile BYTE  	bUltraViolet;
extern volatile BYTE    bUltraVioletGather; //[BG023] add
extern volatile UINT  	iUVexp, iUVexp_lastSaving; //[BG023-1]
extern volatile int16_t  	AmbientLight;

extern volatile float fAmbientTemperature;
extern volatile float fSkinTemperature;
extern volatile uint32_t  XYZ_WINDOW_ENG;

// 以下值为启动（包括暂停后继续）秒表时的基础值（当前值）
// 用于计算某次启动时的累计值
extern time_t	lTimeBase;
extern UINT  	iCaloriesBase;
extern UINT  	iStepsBase;
extern UINT  	iDistanceBase;

// 以下为启动秒表后的累计值，在每次暂停时保存当前累计值，
// 用于实现暂停后继续时，在已有的数据上累计
// 秒表停止后，清除这些值extern time_t	lTimeAccumulated;
extern UINT  	iCaloriesAccumulated;
extern UINT  	iStepsAccumulated;
extern UINT  	iDistanceAccumulated;

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



//extern unsigned char Battery_Flash_Cout;



//
//notification parameters
#if 0
typedef struct _NOTIFICATION_PARAMETERS
{

	uint8_t heartRateLevel;

	uint8_t callsCounterStatus;
	bool stepsCounterStatus;
	bool wearStatus;


	uint8_t lowBatteryLevel;
} NOTIFICATION_PARAMETER;

extern NOTIFICATION_PARAMETER notificationAlertParameters;

#endif
// -----------------------------------------------
// 界面、菜单
//extern bool blDisplayModeNormal;
//BYTE bBackgroundColor = 0x00;

//extern unsigned char iMainMode;


// -----------------------------------------------
// 按键
//#define system_tick_hz  64
//#define REPEAT_INTERVAL         (system_tick_hz / 4)
//#define INITIAL_REPEAT_DELAY    (system_tick_hz / 2)

#define KEY_FLAG_RELEASED 		0x40	// 指示此事件为按键释放事件
#define KEY_FLAG_LONG_PRESS 	0x80	// 指示此事件为长按键事件（释放）

#if 0
// 2015年10月6日11:51:31 去掉
#define APP_HARD_KEY		1
#define APP_TOUCH_UP		2
#define APP_TOUCH_DOWN		3
#define APP_TOUCH_LEFT		4
#define APP_TOUCH_RIGHT		5

//#define ButtonLockDelay  12
//#define ButtonUnLockDelay  3

extern TOUCH_KEY touchkey;
extern bool GulTouchKeyMode;
extern unsigned char mButtons;
extern unsigned char mTOUCH;
extern unsigned char mRepeatButton;
extern unsigned short mRepeatCount;
extern BYTE bKeyRepeatDelay;
#endif

extern bool startCharging;

//extern bool blButtonLocked;


// -----------------------------------------------
//
//extern osMailQId hDispEventQueue;

// 操作I2C的信号量，所有读写操作都使用此信号量
extern SemaphoreHandle_t hI2CSemaphore;


#define Device_Off()          iHeartRate.component.reliability=iHeartRate.component.reliability&0x7f
#define Device_Wearing()      iHeartRate.component.reliability=iHeartRate.component.reliability|0x80

#define IsMotionHr()     iHeartRate.component.reliability=iHeartRate.component.reliability&0xFE
#define IsStillHr()      iHeartRate.component.reliability=iHeartRate.component.reliability|0x01

#define ONE_HOUR_SECONDS  60
#define FIVE_MINUTES_SECONDS  300

typedef struct _SP_VALUE
{
	uint32_t mspTop;
	uint32_t mspBottom;
	uint32_t pspTop;
	uint32_t pspBottom;
	uint32_t controlRegisterValue;
} SP_VALUE;

extern SP_VALUE errlocated;

extern bool blDistanceAccomplish;
extern bool blCalorieAccomplish;
extern bool blStepAccomplish ;
extern bool blAccomplishGoalShine;

extern bool blDuringDownload;
extern int downloadPercentage;


extern volatile bool waitValidHeartrate;
extern volatile bool scanWholeFlash;
#if FALL_DETECT_SUPPORT
extern bool isDetectedFall;
extern uint8_t SendAlertNotification;
extern bool blAlertMenu;
#endif
#if SOS_HIT_SUPPORT
extern uint8_t sosNotification;
extern bool isSOSDetected;
#endif
//#if PPG_WORK_MODE_SET
typedef enum _PPG_RUN_MODE
{
	CONTINUE24_7 = 1,
	WORK_1_DURING_5_MIN ,
	WORK_1_DURING_10_MIN ,
	WORK_1_DURING_15_MIN,
	WORK_1_DURING_30_MIN,
	WORK_1_DURING_60_MIN,
} PPG_RUN_MODES;
//#endif
#if PPG_WORK_MODE_SET
extern bool isWear;
extern uint16_t ppgWorkSpanCount;
extern uint16_t ppgWorkTimespan;
extern bool blPPGLongpress;
#endif
extern uint8_t isChargeStatusChange;
extern uint8_t blTimeReset;
extern uint8_t lowBatteryLevelAlert;

#if BGXXX
extern int itest; //subMenu.c L2171
extern uint32_t utest; //subMenu.c L2172
extern float ftest; //subMenu.c L2173

typedef union _UNION_VAR {
  int typeint;
  uint32_t typeuint32;
  float typefloat;
  uint16_t typeuint16[2];
  int16_t typeint16[2];
  char typechar[4];
  uint8_t typeuint8[4];
  int8_t  typeint8[4];
} UNION_VAR;

extern UNION_VAR test1, test2, test3;
#endif

#endif
