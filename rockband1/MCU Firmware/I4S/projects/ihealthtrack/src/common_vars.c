#include "common_vars.h"

// ----------------------------------------------------------------------
// System
//unsigned char Flash_ID[4];

//const char DeviceType[]={'i','4'};
//
//#define DEVICE_SUB_MODEL_I4_R1 (1)
//#define DEVICE_SUB_MODEL_I4_R2 (2)
//
//BYTE DeviceModel = MAKE_DEVICE_MODEL(DEVICE_MODEL_I4, DEVICE_SUB_MODEL_STANDARD);


/* Temperature Positive/Negative flag,true indicate negative temperature. */
bool ambTempFlag   = false;
bool skinTempFlag  = false;



//bool systemStatus.blFlashOnline = false;
//bool blSubBoardOnline = true;
//bool Enable_BatVcc_Monitor = true;   //2015/10/6 11:34:37 removed.
//bool Watch_On_Wrist= false;

//unsigned char Test_Mode;

//UINT8 flash_saving_count=0;
//volatile UINT32 inactivity_counter = 0;

//volatile BYTE MemoryStatus = ReadytoWrite;
//bool blMemoryFull = false;

//UINT16 A_Temp_float_2bytes;
//UINT16 S_Temp_float_2bytes;

//bool blHeartBeatLock = true;

//DATE_TIME* pSystemTime = &(systemSetting.sysDateTime);

SYSTEM_SETTING systemSetting;
ALARM_SETTING* pAlarmSetting = &(systemSetting.alarmSetting);
USER_PROFILE* pUserProfile = &(systemSetting.userProfile);

SP_VALUE errlocated;


//Basal Metabolic Rate
float BMR_PER_SECOND, BMR_RATE ;

// ----------------------------------------------------------------------
// global status
// ----------------------------------------------------------------------

SYSTEM_STATUS systemStatus;
struct tm* pSystemTime = &(systemStatus.systemTime);

FLASH_INFO* pFlashInfo = &(systemStatus.flashInfo);
volatile FLASH_STORAGE_INDICATOR* pFlashStorageIndicator = &(systemStatus.flashStorageIndicator);

// ----------------------------------------------------------------------
// realtime data
volatile HEART_RATE	iHeartRate = {0}; // low byte is Heart Rate, high byte is reliability.
volatile UINT		iCalories = 0, iCalories_lastSaving = 0;
volatile UINT		iSteps = 0, iSteps_lastSaving = 0;
volatile UINT 		iDistance = 0, iDistance_lastSaving = 0;
volatile UINT		active_level = 0, active_level_lastSaving = 0, active_level_delta = 0;
#if BG009
volatile UINT		active_level_max = 0, active_level_min = 99999999;
#endif
volatile BYTE  	bUltraViolet = 0;
volatile BYTE          bUltraVioletGather = 0; //[BG023] add for gathering
volatile int16_t  	AmbientLight = 0;

volatile float fAmbientTemperature = 25.6f;
volatile float fSkinTemperature = 0.0f;
volatile UINT  	iUVexp = 0, iUVexp_lastSaving = 0; //[BG023-1]

#if ENABLE_PAUSE_STOPWATCH
// The second counter from system startup.
time_t	lTimeBase;
time_t	lTimeAccumulated;
#endif

UINT  	iCaloriesBase;
UINT  	iStepsBase;
UINT  	iDistanceBase;

UINT  	iCaloriesAccumulated;
UINT  	iStepsAccumulated;
UINT  	iDistanceAccumulated;


// ----------------------------------------------------------------------
// 日期、时间
/* The current time reference. Number of seconds since midnight
 * January 1, 1970.  */
//time_t curTime = 1347273358;

//bool bl24HourMode = true;

//UINT16 wYear = 2012;
//BYTE bYearLow = 12;
//BYTE bYearHigh = 20;
//BYTE bMonth = 9;
//BYTE bDayOfWeek = 7;
//BYTE bDay = 11;
//BYTE bHour = 23;
//BYTE bMinute = 59;
//BYTE bSecond = 50;

// ----------------------------------------------------------------------
// 功能设置
//bool blAlarmEnabled = true;
//BYTE bAlarmMode = ALARM_MODE_WEEKDAY;
//BYTE bAlarmHour = 8;
//BYTE bAlarmMinute = 30;

//BYTE bAlarmStatus = 0;	// 闹钟状态，0=未触发；1=触发，响铃；2=被用户停止
//BYTE bAlarmDuration = 60;	// 闹钟持续时间，按秒计

//unsigned char alarm_loop_times=0;

//bool blTonesEnabled = true;
//
//bool blBacklightEnabled = true;


// ----------------------------------------------------------------------
// 传感器
//bool blHrSensorEnabled = true;
//bool blTouchSensorOnline = false;
//bool blAccelSensorOnline = false;
//bool blTempSensorOnline = true;

//bool No_Sensor_On= false;


// ----------------------------------------------------------------------
// 设备

// 电池
//bool LowBatFlag=false;
//bool blChargingOnline = true;
//uint32_t batterylevel;

//unsigned char Battery_Flash_Cout = 0; //2015年10月6日11:41:27 去掉



// -----------------------------------------------
// 界面、菜单
//bool blDisplayModeNormal = true;
//BYTE bBackgroundColor = 0x00;

//unsigned char iMainMode = 0;


// -----------------------------------------------
#if 0
// 按键
TOUCH_KEY touchkey;
bool GulTouchKeyMode = false;
unsigned char mButtons = 0;
unsigned char mTOUCH = 0;
unsigned char mRepeatButton = 0;
unsigned short mRepeatCount = 0;
BYTE bKeyRepeatDelay = 0;
bool blButtonLocked = true;
#endif

bool startCharging  = false;

//目标完成
bool blDistanceAccomplish = false;
bool blCalorieAccomplish = false;
bool blStepAccomplish = false;
bool blAccomplishGoalShine = false;

//download process
bool blDuringDownload = false;
int downloadPercentage = 0;


#if 0
//The I4 did not use the parameters, 2015/10/6 11:48:33
NOTIFICATION_PARAMETER notificationAlertParameters;
#endif

volatile bool waitValidHeartrate = false;

volatile bool scanWholeFlash = true;
#if PPG_WORK_MODE_SET
bool isWear = false;
uint16_t ppgWorkSpanCount;
uint16_t ppgWorkTimespan = 0;
bool blPPGLongpress = false; //Atus: actually just assign in Device_taks.c, not use condition.
#endif

#if FALL_DETECT_SUPPORT
bool isDetectedFall = false;
uint8_t SendAlertNotification = 0;  //Fall Detect alert
bool blAlertMenu = false;
#endif
#if SOS_HIT_SUPPORT
bool isSOSDetected = false;
uint8_t sosNotification = 0;
#endif
uint8_t isChargeStatusChange = 0;
uint8_t lowBatteryLevelAlert = 0;  //I4C lowPowerNotification
uint8_t blTimeReset = 0xFF;

#if BGXXX
UNION_VAR test1={0};	//for submenu debug used.
UNION_VAR test2={0};
UNION_VAR test3={0};
#endif

uint8_t realdata_count = 0;
uint16_t readbuf16[5];