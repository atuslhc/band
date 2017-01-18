#include "common_vars.h"

// ----------------------------------------------------------------------
// 系统
//unsigned char Flash_ID[4];



//bool systemStatus.blFlashOnline = false;
bool blSubBoardOnline = true;
bool Enable_BatVcc_Monitor = true;
bool Watch_On_Wrist= false;

//unsigned char Test_Mode;

//BYTE bWorkingMode = Monitor_mode;
//bool blModeSwitched = false;

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

//Basal Metabolic Rate
float BMR_PER_SECOND ;

// ----------------------------------------------------------------------
// global status
// ----------------------------------------------------------------------

SYSTEM_STATUS systemStatus;
struct tm* pSystemTime = &(systemStatus.systemTime);

FLASH_INFO* pFlashInfo = &(systemStatus.flashInfo);
volatile FLASH_STORAGE_INDICATOR* pFlashStorageIndicator = &(systemStatus.flashStorageIndicator);

// ----------------------------------------------------------------------
// 实时指标、数据
volatile UINT	iHeartRate = 0;
volatile UINT  	iCalories = 0;
volatile UINT  	iSteps = 0;
volatile UINT  	iDistance = 0;

//volatile float fAmbientTemperature = 25.6f;
volatile float bSkinTemperature = 0.0f;

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
unsigned char Battery_Flash_Cout=0;


#if 0
// -----------------------------------------------
// 界面、菜单
bool blDisplayModeNormal = true;
//BYTE bBackgroundColor = 0x00;

//unsigned char iMainMode = 0;


// -----2015年10月5日08:34:38调整代码去掉的
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

