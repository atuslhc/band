/*
 * clockTask.c
 *
 *  Created on: 2013-6-15
 *      Author: Administrator
 */
#include <stdlib.h>

#include "freertos.h"
#include "task.h"

//#include "cmsis_os.h"

#include "sleep.h"

#include "common_vars.h"
#include "main.h"
#include "device_task.h"
#include "display_task.h"
#include "flash_task.h"

#include "menu.h"
#include "notification.h"

#include "m00930.h"
#include "mcp9804.h"
#include "Si14x.h"
#include "ble.h"
#if (ACCELEROMETER_SUPPORT==1)
#include "mems.h"
#endif
#if (BAROMETER_SUPPORT==1)
#include "LPS22HB.h"
#elif (BAROMETER_SUPPORT==2)
#include "LPS35HW.h"
#endif
#if (MAGNETIC_SUPPORT==1)
#include "BMM150.h"
#endif
#if (GYRO_SUPPORT==1)
#include "L3GD20H.h"
#endif
#if (CAP_SUPPORT==2)
#include "AD7156.h"
#endif
#if (SOS_HIT_SUPPORT)
#include "mems_tracking.h"
#endif

#define ALARM_DURATION	(30)

bool volatile isFactoryReleased = true;

//osMessageQDef(MsgInterrupt, 32, uint32_t);
//osMessageQId hMsgInterrupt;

/********************新加进来的*******************************/

#define DEVICE_TASK_NAME			"Device"	// 堆栈级别
#define DEVICE_TASK_STACK_SIZE		(768)		// task reserved stack size
#define DEVICE_TASK_STACK_DEPTH		(DEVICE_TASK_STACK_SIZE / sizeof(portSTACK_TYPE))	// stack depth(number)
#define DEVICE_TASK_PRIORITY		(2)

#define DEVICE_TASK_QUEUE_LEN			32
#define DEVICE_TASK_QUEUE_ITME_SIZE		sizeof(uint32_t)

QueueHandle_t hEvtQueueDevice = 0;


#if 0 //move to device_task.h
typedef struct _BACKUP_DATA
{
	uint32_t headFlag;
	int backupDistance;
	int backupSteps;
	int backupCalories;
	uint32_t trailFlag;
} BACKUP_DATA;
#endif

BACKUP_DATA* backupDCS;

BACKUP_DATA* backupDCS = (BACKUP_DATA*)BACKUP_ADDRESS; //0x20007fe0;
/********************新加进来的*******************************/

//uint16_t HardKeyActivedStatus = 0;

//extern void TempReadAndStop(void);


// The I2C semaphore. All of the read/write operation must throught the semaphore control to avoid reentrace.
//osSemaphoreId hI2CSemaphore;

SemaphoreHandle_t hI2CSemaphore = 0;

void triggerAlarm()
{
	// ============================================================
	// 此函数用于检查是否触发alarm

	if (pAlarmSetting->enabled && systemStatus.bBatteryLevel > OUT_OF_BATTERY)
	{
		// 首先检查systemStatus.bAlarmStatus
		// systemStatus.bAlarmStatus == 2，意味着闹钟被触发，然后被用户停止，则不响铃；同时 bAlarmDuration 递减
		// systemStatus.bAlarmStatus == 1，意味着闹钟被触发，继续响铃；同时 bAlarmDuration 递减
		// systemStatus.bAlarmStatus == 0，则检查当前时间是否能触发闹钟，根据bAlarmMode按不同条件检查
		// 若闹钟被触发，则置 systemStatus.bAlarmStatus = 1，bAlarmDuration = 60
		// bAlarmDuration递减至0时，置 systemStatus.bAlarmStatus = 0

		switch (systemStatus.bAlarmStatus)
		{
			case 0:
			{
				// 检查当前时间是否能触发闹钟，根据bAlarmMode按不同条件检查

				switch (pAlarmSetting->mode)
				{
					case ALARM_MODE_ONETIME:
					{
						if (pAlarmSetting->alarmTime.Year == pSystemTime->tm_year + 1900
						        && pAlarmSetting->alarmTime.Month == pSystemTime->tm_mon + 1
						        && pAlarmSetting->alarmTime.Day == pSystemTime->tm_mday
						        && pAlarmSetting->alarmTime.Hour == pSystemTime->tm_hour
						        && pAlarmSetting->alarmTime.Minute == pSystemTime->tm_min
						        && pSystemTime->tm_sec == 0)
						{
							// 触发闹钟
							RaiseNotification(NOTIFY_SERVICE_Alarm);

							systemStatus.bAlarmDuration = ALARM_DURATION;
							systemStatus.bAlarmStatus = 1;
						}

						break;
					}

					case ALARM_MODE_DAILY:
					{
						if (pAlarmSetting->alarmTime.Hour == pSystemTime->tm_hour
						        && pAlarmSetting->alarmTime.Minute == pSystemTime->tm_min
						        && pSystemTime->tm_sec == 0)
						{
							// 触发闹钟
							RaiseNotification(NOTIFY_SERVICE_Alarm);

							systemStatus.bAlarmDuration = ALARM_DURATION;
							systemStatus.bAlarmStatus = 1;
						}

						break;
					}

					case ALARM_MODE_WEEKDAY:
					{
						if (pAlarmSetting->weekday[pSystemTime->tm_wday] == true
						        && pAlarmSetting->alarmTime.Hour == pSystemTime->tm_hour
						        && pAlarmSetting->alarmTime.Minute == pSystemTime->tm_min
						        && pSystemTime->tm_sec == 0)
						{
							// 触发闹钟
							RaiseNotification(NOTIFY_SERVICE_Alarm);

							systemStatus.bAlarmDuration = ALARM_DURATION;
							systemStatus.bAlarmStatus = 1;
						}

						break;
					}
				}

				break;
			}

			case 1:
			{
				static uint8_t alart_Count = 0, alart_delay = 0;;
				// 闹钟被触发，响铃；同时 bAlarmDuration 递减
				// bAlarmDuration递减至0时，置 systemStatus.bAlarmStatus = 0
				systemStatus.bAlarmDuration--;

				if (systemStatus.bAlarmDuration == 0)
				{
					// 停止闹钟
					RemoveNotification(NOTIFY_SERVICE_Alarm);

					systemStatus.bAlarmStatus = 0;
					alart_Count = 0;
					alart_delay = 0;
				}
				else
				{
					alart_delay++;
					alart_delay %= 2;

					if(alart_delay == 1)
					{
						alart_Count++;

#if (VIBRATION_SUPPORT==1)
						if(alart_Count < 3)
							VibrateCon(SoftFuzz_60, 2, 1);

						else
							VibrateCon(BuzzAlert1000ms, 2, 1);
#endif

					}
				}

				break;
			}

			case 2:
			{
				// 闹钟被触发，然后被用户停止，则不响铃；同时 bAlarmDuration 递减
				// bAlarmDuration递减至0时，置 systemStatus.bAlarmStatus = 0
				systemStatus.bAlarmDuration--;

				if (systemStatus.bAlarmDuration == 0)
					systemStatus.bAlarmStatus = 0;

				break;
			}
		}
	}
}


void enumDataGathering()
{
	/*
	模拟数据采集
	*/

	time_t ts = time(NULL);
//	ts /= 1000;

	INDEX_DATA_DEF* pDef = NULL;
//	INDEX_DATA_TYPE type = DATATYPE_UNKNOWN;
	BYTE sample[2] = {0};

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];
//		type = pDef->type;

#ifdef DEBUG

		if (ts % (pDef->sampleInterval / 2) == 0)
#else
		if (ts % pDef->sampleInterval == 0)
#endif
		{
			if (!systemStatus.blDataGatheringInitialized)
			{
				systemStatus.blDataGatheringInitialized = true;

				// 第一次翻转数据采集缓冲区，用于写入时间戳
				flipAndInitIndexDataBuffer();
			}
			else if (checkIndexDataBuffer(pDef->type))
			{
				// initiate a saving request here
				BYTE* buff = getActivatedIndexDataBuffer();

//					FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hEvtQueueFlash, 0);
//					fcmd->cmd = FLASH_CMD_WRITE;
//					fcmd->data.p = buff;
//
//					osMailPut(hEvtQueueFlash, fcmd);
				FLASH_COMMAND fcmd;
				fcmd.cmd = FLASH_CMD_WRITE;
				fcmd.data.p = buff;

				xQueueSend(hEvtQueueFlash, &fcmd, 0);

				//
				flipAndInitIndexDataBuffer();
			}

			for (int j = 0; j < pDef->sampleSize; j++)
			{
				sample[j] = rand() % 0x100;
			}

			putIndexData(i, pDef->sampleSize, sample);
		}
	}
}

//void EnumDataGathering2()
//{
//	/*
//	模拟数据采集，每次采集若干个字节
//	每次采集数据后，检查剩余容量，若不足一半，则启动flash写入
//
//	flash每次最多写入256字节，因此需要分批操作
//	flash操作比较耗时，所以将某些IO操作交给flashtask处理，需增加相应的状态信息
//	目前将sectorerase,chiperase交由flashtask处理。可行的话，可以考虑将pageprogram交由flashtask处理
//
//	flash写入完成后，需调整 dataBufferTail 指针，以释放旧数据
//	*/
//
//	int bytes = (rand() % 40) + 10;
////	CCTRACE("bytes: %d\n", bytes);
//
//	int i = 0;
////	if (dataBufferHead >= dataBufferTail)
////	{
////		cap = dataBufferTail + (FLASH_BUFFER_SIZE - dataBufferHead)
////	}
////	else
////	{
////		cap = dataBufferTail - dataBufferHead;
////	}
//
//	if (dataBufferCapability < bytes)
//	{
////		dataBufferTail += bytes;
////		if (dataBufferTail >= FLASH_BUFFER_SIZE)
////			dataBufferTail -= FLASH_BUFFER_SIZE;
//	}
//
//	// 数据写入缓冲区，注意缓冲区为环形使用
//	int left = FLASH_BUFFER_SIZE - dataBufferHead;
//	if (left >= bytes)
//	{
//		for (i = 0; i < bytes; i++)
//		{
//			dataGatherBuffer[dataBufferHead] = rand() % 0x100;
//			dataBufferHead++;
//		}
//	}
//	else
//	{
////		CCTRACE("no space, left: %d, size: %d\n", left, bytes);
//
//		for (i = 0; i < left; i++)
//		{
//			dataGatherBuffer[dataBufferHead] = rand() % 0x100;
//			dataBufferHead++;
//		}
//
//		dataBufferHead = 0;
//		for (i = left; i < bytes; i++)
//		{
//			dataGatherBuffer[dataBufferHead] = rand() % 0x100;
//			dataBufferHead++;
//		}
//	}
//
//	// 调整剩余容量
//	dataBufferCapability -= bytes;
//
////	CCTRACE("bytes: %d, tail: %d, head: %d, cap: %d\n", bytes, dataBufferTail, dataBufferHead, dataBufferCapability);
//
//
//	//==============================================================
//	// 根据情况写入flash
//	SaveToFlash();
//}

//==============================================================
// 系统初次启动，也会发送系统启动的消息
// 在这里应该只处理于初次启动相关的事务
void onSystemFirstTimeStartup()
{
//	// 发送消息，对flash进行chip erase
//	FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	fcmd->cmd = FLASH_CMD_PREPARE_CHIP;
//
//	osMailPut(hFlashCommandQueue, fcmd);

//	// 发送命令初始化flash（重建索引）
//	FLASH_COMMAND* cmdInit = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmdInit->cmd = FLASH_CMD_INIT;
//
//	osMailPut(hFlashCommandQueue, cmdInit);
}

// 为调试方便定义此符号 INIT_FLASH，启用时将擦除flash chip，并且不会启动模拟数据采集
#define INIT_FLASH1

// 为调试方便定义此符号 ENABLE_ENUM_GATHER，启用时将会启动模拟数据采集
#define ENABLE_ENUM_GATHER1

// 进入release模式的动作，
// 包括 通过蓝牙指令 切换到发行模式，以及在发行模式下系统重新上电
void DoActionsInReleaseMode()
{
	ForceShowMenu(2, MENU_TYPE_Device_activate, MENU_TYPE_Battery);
#if (AFE44x0_SUPPORT==1)
	AFE44xx_Shutoff();
#endif
}

//bool xxxFlag = false;
void LoadBackupData()
{
    if ((backupDCS->headFlag == BACKUP_HEADFLAG) && (backupDCS->tailFlag == BACKUP_TAILFLAG))
	{
		iCalories = backupDCS->backupCalories;
		iDistance = backupDCS->backupDistance;
		iSteps    = backupDCS->backupSteps;
//		xxxFlag   = true;
	}
}
//==============================================================
// 系统启动
void onSystemStartup()
{
#if (OLED_SUPPORT==1)
	OLEDInit();
#endif

#if (UVSENSOR_SUPPORT==1)
	UV_Init(systemSetting.blUVSensorEnabled);
#endif

//#if (BAROMETER_SUPPORT==1)  //move to DeviceTask()
//    LPS22HB_Init(systemSetting.blPressureSensorEnabled);
//#elif (BAROMETER_SUPPORT==2)
//    LPS35HW_Init(systemSetting.blPressureSensorEnabled);
//#endif
    
//#if (CAP_SUPPORT==2)  //move to DeviceTask()
//    AD7156_Init();
//#endif
    
	
#if BATTERY_LIFE_OPTIMIZATION
	Si114xPauseAll(); //initialize then turn off UV.
#endif
    
	// initialize data gather buffer
	initIndexDataBuffer();


	LoadBackupData();

	// 系统启动，如果flash已经初始化（chip erase）
	// 则扫描flash，确定当前sector（使用其保存数据）
#ifdef INIT_FLASH
//	FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hEvtQueueFlash, 0);
//	//	fcmd->cmd = FLASH_CMD_PREPARE_CHIP;
//	fcmd->cmd = FLASH_CMD_PREPARE_DATA_STORAGE;
//	osMailPut(hEvtQueueFlash, fcmd);
	FLASH_COMMAND fcmd;
	fcmd.cmd = FLASH_CMD_PREPARE_DATA_STORAGE;

	xQueueSend(hEvtQueueFlash, &fcmd, 0);
#else
	//	if (systemStatus.blFlashInitialized)
	//	{
//	FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hEvtQueueFlash, 0);
//	fcmd->cmd = FLASH_CMD_INIT;
//#if 0
//	fcmd->cmd = FLASH_CMD_PREPARE_SECTOR_FOR_TESTING;
//#endif
//	osMailPut(hEvtQueueFlash, fcmd);
	FLASH_COMMAND fcmd;
	fcmd.cmd = FLASH_CMD_INIT;

	xQueueSend(hEvtQueueFlash, &fcmd, 0);
	//	}
#endif

#if (CAP_SUPPORT==1)
	capSenseSwitchModel(KEYsSCAN); //为了一开始按键灵敏，2015年5月18日11:15:24
#endif
    //FIXME: We need a control set start.
	// =====================================================================
	if(systemSetting.SystemMode == SYSTEM_MODE_RELEASED)
	{
		DoActionsInReleaseMode();
		return;
	}
}

// 设备模式发生变化
void onSystemModeChanged(SYSTEM_MODE oldMode, SYSTEM_MODE newMode)
{
	switch (newMode)
	{
//		case SYSTEM_MODE_MANUFACTORING:
//		{
//			UnforceShowMenu();
//			MENU_AppLeft();
//
//			systemSetting.alarmSetting.enabled = true;
//			systemSetting.blHRSensorEnabled = true;
//			systemSetting.blTouchSensorEnabled = true;
//	        systemSetting.blAmbTempSensorEnabled = true;
//	        systemSetting.blSkinTempSensorEnabled = true;
//	        systemSetting.blUVSensorEnabled= true;
//
//			//
//			StartDataGathering();
//
//			break;
//		}

		case SYSTEM_MODE_RELEASED:
		{
			// 发送消息，重置采集数据
//			FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//			fcmd->cmd = FLASH_CMD_RESET_DATA_GATHER;
//			fcmd->data.v = false;
//			osMailPut(hFlashCommandQueue, fcmd);

			FLASH_COMMAND fcmd;
			fcmd.cmd = FLASH_CMD_RESET_DATA_GATHER;
			fcmd.data.v = false;
			xQueueSend(hEvtQueueFlash, &fcmd, 0);


			//
			DoActionsInReleaseMode();

			//
			systemSetting.blHRSensorEnabled = false;
			systemSetting.alarmSetting.enabled = false;
			systemSetting.blTouchSensorEnabled = false;
			systemSetting.blAmbTempSensorEnabled = false;
			systemSetting.blSkinTempSensorEnabled = false;
			systemSetting.blUVSensorEnabled = false;
			systemSetting.blAccelSensorEnabled = false;
			systemSetting.blGyroSensorEnabled = false;
			systemSetting.blGeoMSensorEnabled = false;
			systemSetting.blPressureSensorEnabled = false;
			systemSetting.blCAPSensorEnabled = false;
			//systemSetting.blBluetoothEnabled = false;

			break;
		}

		case SYSTEM_MODE_MANUFACTORING:
		case SYSTEM_MODE_ACTIVATED:
		{
			UnforceShowMenu();
			MENU_AppLeft();

			//
			SensorOffDelay = 0;
			systemStatus.blSkinTouched = false;

			systemSetting.alarmSetting.enabled = true;
			systemSetting.blHRSensorEnabled = true;
			systemSetting.blTouchSensorEnabled = true;
			systemSetting.blAmbTempSensorEnabled = true;
			systemSetting.blSkinTempSensorEnabled = true;
			systemSetting.blUVSensorEnabled = true;
			systemSetting.blAccelSensorEnabled = true;
			systemSetting.blGyroSensorEnabled = true;
			systemSetting.blGeoMSensorEnabled = true;
			systemSetting.blPressureSensorEnabled = true;
			systemSetting.blBluetoothEnabled = true;
			systemSetting.blCAPSensorEnabled = true;
			

			//
			StartDataGathering();

			//在这里应该把系统设置保存一下  2015年7月27日10:55:38
			SaveSystemSettings();

			break;
		}
	}

	SaveSystemSettings();
}


// 进入某个菜单
void onEnterMenu(MENU_TYPE menuType)
{
	// 重置测试菜单自动锁屏选项
	systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_TESTING_MENU; //

	switch (menuType)
	{
		case MENU_TYPE_UltraViolet:

#if BATTERY_LIFE_OPTIMIZATION

//			Start_Cap_Temp();
//			Si114xAlsForce();
#else
			Start_Cap_Temp();
			Si114xAlsForce();

#endif
			break;

		case MENU_TYPE_Testing:
		{
//			 // 进入此菜单时，禁用自动锁屏，即lcd常亮
//			 systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_TESTING_MENU;

			extern char test_menu_loopcount;
			test_menu_loopcount = 0;
			break;
		}

		default:

			break;
	}
}


#define ENABLE_PAUSE_STOPWATCH1

bool blLastLongPressedisBLE = false;
int8_t lastLongPressedisBLE  = 0;
//bool isFirstTimeRunning=true;
#if BGXXX>0
int8_t lastLongPressedisTest = 0;       //control menuTest long press.
#endif


void onManualRestMCU(void* data)
{
	blLastLongPressedisBLE = false;
}

// 用户在某个菜单长按按键
void onMenuAction(MENU_TYPE menuType)
{

#if (VIBRATION_SUPPORT==1)
	//VibrateCon(BuzzAlert1000ms,1,5);
#endif
  
	switch (menuType)
	{
#if BGXXX>0
                case MENU_TYPE_Testing:
                    lastLongPressedisTest = 4;  //1
                break;
#endif
#if 1
		//通过手机APP来激活设备，所以把这个屏蔽
		case MENU_TYPE_Device_activate:
		{
			uint8_t oldMode = systemSetting.SystemMode;
			systemSetting.SystemMode = SYSTEM_MODE_ACTIVATED;

			//
			MESSAGE msg;
			msg.params.type = MESSAGE_SYSTEM_MODE_CHANGED;
			msg.params.param = oldMode + ((systemSetting.SystemMode) << 8);

//			osMessagePut(hMsgInterrupt, msg.id, 0);

			xQueueSend(hEvtQueueDevice, &msg, 0);
			//
			break;
		}

#endif

		case MENU_TYPE_Time:
		{
			DisableLongTimer(LONG_TIMER_MANUAL_RESET_MCU);

			if(blLastLongPressedisBLE == true)
			{
				RESET_MCU(); // reset the device

				while(1);
			}

//			else
//			{
//				// 启动/停止秒表
//				systemStatus.blStopwatchEnabled = !systemStatus.blStopwatchEnabled;
//
//				if (systemStatus.blStopwatchEnabled)
//				{
//					lTimeBase= time(NULL);
//
//					iCaloriesBase = iCalories;
//					iStepsBase = iSteps;
//					iDistanceBase = iDistance;
//
//					// 清除累计值
//					lTimeAccumulated = 0;
//					iCaloriesAccumulated = 0;
//					iStepsAccumulated = 0;
//					iDistanceAccumulated = 0;
//				}
//
//				// 发送一个rtc消息以立即刷新界面
//				fireDisplayEvent(EVT_TYPE_RTC, NULL);
//			}

			break;
		}

		case MENU_TYPE_HeartRate:
			if(systemSetting.blTouchSensorEnabled == false)
			{
				//manual mode
#if (VIBRATION_SUPPORT==1)
				VibrateCon(StrongClick1_100, 1, 1);
#endif
#if PPG_WORK_MODE_SET
				//The count will reset while manual pressing.
				blPPGLongpress = true;
				ppgWorkSpanCount = 0;
#endif
				if (systemStatus.blHRSensorOn)
				{
#if (AFE44x0_SUPPORT==1)
					AFE44xx_Shutoff();
#endif
				}
				else
				{
					NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY; //延迟自动锁屏时间
#if BATTERY_LIFE_OPTIMIZATION
					systemStatus.blHRSensorTempEnabled = false;
#endif
#if (AFE44x0_SUPPORT==1)
					AFE44xx_PowerOn_Init();
#endif
					SensorOffDelay = default_Key_SensorOffDelay * current_touchsensor_feq;
				}
			}
			else
			{
				// 在自动开关ppg的设置下，开关ppg的操作无效，应提示用户
				// 提示的方式是在心率菜单显示 AUTO
				systemStatus.iAutoHRWarning = 3;

				// 发送一个rtc消息以立即刷新界面
				fireDisplayEvent(EVT_TYPE_RTC, NULL);
			}

			blLastLongPressedisBLE = false;

			break;

		case MENU_TYPE_Step:

//			if (!systemStatus.blStopwatchEnabled)
//			{
//				lastLongPressedisStep = true;
//				break;
//			}

			break;

		case MENU_TYPE_Calories:
#if BG013MS
                {
			blLastLongPressedisBLE = false;
    extern uint8_t LED_INTENSITY,AMB_uA;
                        if (LED_INTENSITY<LED_INTENSITY_MS_MAX)
                            LED_INTENSITY ++;
                        else
                            LED_INTENSITY = LED_INTENSITY_MS_MIN;
                        systemSetting.res[0] = LED_INTENSITY;
                        SaveSystemSettings();
                        LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
#if (VIBRATION_SUPPORT==1)
                        VibrateCon(StrongClick1_100, 1, 1);
#endif
                        break;
                }
#endif
		case MENU_TYPE_Distance:
		{
			blLastLongPressedisBLE = false;
#if BG013MS
    extern uint8_t LED_INTENSITY,AMB_uA;
                        if (AMB_uA<AMB_UA_MS_MAX)
                            AMB_uA ++;
                        else
                            AMB_uA = AMB_UA_MS_MIN;
                        systemSetting.res[1] = AMB_uA;
                        SaveSystemSettings();
                        LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
#if (VIBRATION_SUPPORT==1)
                       VibrateCon(StrongClick1_100, 1, 1);
#endif
#endif
#ifdef DEBUG_MODE

			if (systemStatus.blBatteryDraining)
			{
				StopBatteryDrain();
			}
			else
			{
				StartBatteryDrain();
			}

#endif

			if (systemStatus.blStopwatchEnabled)
				systemStatus.blStopwatchPaused = !systemStatus.blStopwatchPaused;

#if ENABLE_PAUSE_STOPWATCH

			if (systemStatus.blStopwatchPaused)
			{
				// 暂停时保存累计值
				lTimeAccumulated += time(NULL) - lTimeBase;
				iCaloriesAccumulated += iCalories - iCaloriesBase;
				iStepsAccumulated += iSteps - iStepsBase;
				iDistanceAccumulated += iDistance - iDistanceBase;
			}
			else
			{
				// 重新保存起始值
				lTimeBase = time(NULL);

				iCaloriesBase = iCalories;
				iStepsBase = iSteps;
				iDistanceBase = iDistance;
			}

#endif

			//
			break;
		}

                case MENU_TYPE_Battery:
                {
			blLastLongPressedisBLE = false;
#if BG013MS
    extern uint8_t LED_INTENSITY,AMB_uA;
                        LED_INTENSITY = LED_INTENSITY_MS_MIN;
                        AMB_uA = AMB_UA_MS_MIN;
                        systemSetting.res[0] = LED_INTENSITY;
                        systemSetting.res[1] = AMB_uA;
                        SaveSystemSettings();
                        LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
#if (VIBRATION_SUPPORT==1)
                        VibrateCon(StrongClick1_100, 1, 1);
#endif
#endif
                        break;
                }
                
		case MENU_TYPE_UltraViolet:
		{
			systemStatus.blUVAlarmDisabled = true;
			systemStatus.iUVAlarmDisabledTime = time(NULL);

			// 取消震动
			RemoveNotification(NOTIFY_SERVICE_Intense_UV);
#if (VIBRATION_SUPPORT==1)
			stopVibrate();
#endif
			break;
		}



		case MENU_TYPE_IncomingCall:
		{
			blLastLongPressedisBLE = false;

#if (VIBRATION_SUPPORT==1)
			VibrateCon(BuzzAlert1000ms, 1, 5);
#endif
			memset(systemStatus.incomingCallNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);

			RemoveNotification(NOTIFY_SERVICE_IncomingCall);
			RemoveNotification(NOTIFY_SERVICE_MissedCall);

			// 发送一个rtc消息以立即刷新界面
			fireDisplayEvent(EVT_TYPE_RTC, NULL);

			break;
		}

		case MENU_TYPE_Notifications:
		{
			blLastLongPressedisBLE = false;

#if (VIBRATION_SUPPORT==1)
			VibrateCon(BuzzAlert1000ms, 1, 5);
#endif
			memset(systemStatus.latestSmsNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);

			RemoveNotification(NOTIFY_SERVICE_Email);
			// 发送一个rtc消息以立即刷新界面
			fireDisplayEvent(EVT_TYPE_RTC, NULL);

			break;
		}

		case MENU_TYPE_BleMAC:
		{
          
#if (VIBRATION_SUPPORT==1)
			VibrateCon(StrongClick1_100, 1, 1);
#endif
			lastLongPressedisBLE = 4;
			blLastLongPressedisBLE = true;
			EnableLongTimer(LONG_TIMER_MANUAL_RESET_MCU, false, 6, onManualRestMCU, NULL);//再次长按6秒后检测有没有在时间菜单下按，没有就不能复位mcu
			// 发送一个rtc消息以立即刷新界面
			fireDisplayEvent(EVT_TYPE_RTC, NULL);
			break;
		}

		default:
			blLastLongPressedisBLE = false;
			break;
	}
}


extern void LongTimerFunc();
extern uint8_t heart_display;
extern uint8_t heart_buffer[3];
extern uint8_t heart_timecount;
void onRealtimeClockTick()
{
	pSystemTime = GetRTCCalendar();

	// -------------------------------------------------------------
	// Maintain the long timer counter function.
	LongTimerFunc();

	// -------------------------------------------------------------
	// Send the EVT_TYPE_RTC event to hEvtQueueDisplay base on Devicetask with TICK_Message driven.
//	USER_EVENT* tickEvent = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//	tickEvent->type = EVT_TYPE_RTC;
//
//	osMailPut(hDispEventQueue, tickEvent);
	USER_EVENT tickEvent;
	tickEvent.type = EVT_TYPE_RTC;

	xQueueSend(hEvtQueueDisplay, &tickEvent, 0);

	// -------------------------------------------------------------
	// 发送到flash任务
//	FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	fcmd->cmd = FLASH_CMD_DATA_GATHER_TICK;
//	fcmd->data.p = buff;
//
//	osMailPut(hFlashCommandQueue, fcmd);

	FLASH_COMMAND fcmd;
	fcmd.cmd = FLASH_CMD_DATA_GATHER_TICK;

	xQueueSend(hEvtQueueFlash, &fcmd, 0);
	// -------------------------------------------------------------
	// 检查是否触发alarm
	triggerAlarm();


	// -------------------------------------------------------------
	// Check Find me expire
	if (systemStatus.blFindingMe)
	{
		systemStatus.iFindingMeCount--;

		if (systemStatus.iFindingMeCount <= 0)
		{
			// find me 超时
			systemStatus.blFindingMe = false;
			systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_FIND_ME;
		}
	}


//说明皮肤已经接触了ppg，但是还没有心率数据.这里还需要判断（是否在自动模式下，如果不在，则不需要这样处理）
	extern uint16_t SkinTouchVal;

#if PPG_WORK_MODE_SET
		if((waitValidHeartrate == true) && (systemStatus.blHeartBeatLock == false)
		        && (systemSetting.blTouchSensorEnabled == true)
		        && (SkinTouchVal < max_SkinTouchVal)
		        && (ppgWorkSpanCount <= 60)  )
#else
	if((waitValidHeartrate == true) && (systemStatus.blHeartBeatLock == false)
	        && (systemSetting.blTouchSensorEnabled == true)
	        && (SkinTouchVal < max_SkinTouchVal))
#endif
		{
			NoTOUCHPressCount = LONGER_LOCK_SCREEN_DELAY;//延迟心率显示界面
		}

	if(systemStatus.blHeartBeatLock == true)
	{
		heart_buffer[heart_timecount] = iHeartRate.component.heart;
		heart_timecount++;

		if(heart_timecount == 3)
		{
			heart_timecount = 0;
			heart_display = (heart_buffer[0] + heart_buffer[1] + heart_buffer[2]) / 3;
		}
	}

	// -------------------------------------------------------------
	//
	AutoLockScreen();
	/*
	每秒中都会把NoTOUCHPressCount减1，当减到0时会锁屏，在锁屏函数中发送一个虚拟事件自动关屏，在OLEDOff（）中
	把blOLEDPowerOn置位false。那么以后再RTC事件来时，就会退出，不会在刷屏。
	*/


	// -------------------------------------------------------------
	//
	CHECK_BATTERY();

	CHECK_PER_Xsecond();

	CheckGoalsAccomplish();
#if 0
	//去掉原来的测试notification
	StepsDuringOneHour();
#endif
#if SOS_HIT_SUPPORT
	CheckSOS();
#endif
#if FALL_DETECT_SUPPORT  //note: base on stop action similar SOS trigger action, need put after CheckSOS.
	CheckFall();
#endif

	SendNotificationAlert(); //Atus: i4_C put after next command, it is wrong sequence.
//
	PermitUpdateBroadcast();


	// -------------------------------------------------------------
	// 重置 UV alarm
	if (systemStatus.blUVAlarmDisabled)
	{
		time_t now = time(NULL);

		if (now - systemStatus.iUVAlarmDisabledTime > 30 * 60) // 30 mins
			systemStatus.blUVAlarmDisabled = false;
	}

//	// -------------------------------------------------------------
//	// 跨天时，重置数据采集
//	if (pSystemTime->tm_hour == 0
//		&& pSystemTime->tm_min == 0
//		&& pSystemTime->tm_sec == 0)
//	{
//		// 先停止采集
//		StopDataGathering(true);
//
//		// 重置指标
//		ResetIndexData();
//
//		// 先停止采集
//		StartDataGathering();
//	}
}

void initDeviceTask()
{
//	hMsgInterrupt = osMessageCreate(osMessageQ(MsgInterrupt), osThreadGetId());
//	osMsgQAddToRegistry(hMsgInterrupt, "DEV");


	hEvtQueueDevice = xQueueCreate(DEVICE_TASK_QUEUE_LEN, DEVICE_TASK_QUEUE_ITME_SIZE);
	vQueueAddToRegistry(hEvtQueueDevice, "MsgQ_Device");

}

/* Defining the watchdog initialization data */
WDOG_Init_TypeDef wathdoginit =
{
	.enable	  = true,				/* Start watchdog when init done */
	.debugRun   = false,				/* WDOG not counting during debug halt */
	.em2Run	  = true,				/* WDOG counting when in EM2 */
#if (BG039==0 && MODEL_TYPE==1) //[BG039] enable WDOG EM3 for all model.//HEALTHCARE_TYPE
	.em3Run	  = false,				/* WDOG counting when in EM3 */
#else
	.em3Run	  = true,				/* WDOG counting when in EM3 */
#endif
	.em4Block   = false,				/* EM4 can be entered */
	.swoscBlock = false,				/* Do not block disabling LFRCO/LFXO in CMU */
	.lock 	  = false,				/* Do not lock WDOG configuration (if locked, reset needed to unlock) */
	.clkSel	  = wdogClkSelULFRCO,	/* Select 1kHZ WDOG oscillator */
	.perSel	  = wdogPeriod_4k,	/* Set the watchdog period to 4seconds*/

};

void onBleHeartBeatCheck(void* data)
{
	if (systemSetting.blBluetoothEnabled)
	{
		if (systemStatus.iBleHeartBeatCounter >= 3)
		{
			// ble no response after reset ble
			// reset device
			RESET_MCU();

			while(1);
		}
		else if (systemStatus.iBleHeartBeatCounter >= 2)
		{
			// ble no response
			// reset ble
			BLE_RESET();
		}
		else if (systemStatus.iBleHeartBeatCounter >= 1)
		{
			// systemStatus.iBleHeartBeatCounter try to getBleDeviceInfo()
			// if still can not get response will reset BLE as above.
			getBleDeviceInfo();
		}

		//
		systemStatus.iBleHeartBeatCounter++;
	}
}

#define MAX_HEART_BEAT_LIMIT	10
int displayTaskHeartBeat = 0;
int flashTaskHeartBeat = 0;
time_t lastKeyTouchTime = 0;
uint8_t KEY1_count=0, KEY2_count=0;
#if (SOS_2S==1)
time_t KEY1_LastPressTime=0, KEY1_LastReleaseTime=0;
time_t KEY2_LastPressTime=0, KEY2_LastReleaseTime=0;
#endif

void DeviceTask(void* argument)
{
//    WDOG_Feed(); //20140428    这个不应该要，只有初始化看门狗之后，再喂狗，这里没有初始化
//就喂狗了。


	// 初始化I2C信号量
//	osSemaphoreDef_t semDef;
//	hI2CSemaphore = osSemaphoreCreate(&semDef, 1);
//	osSemaphoreAddToRegistry(hI2CSemaphore, "I2CSemap");


	hI2CSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(hI2CSemaphore);
	vQueueAddToRegistry(hI2CSemaphore, "I2CSemap");

	//
	initLETIMER();
#if (ACCELEROMETER_SUPPORT==1)
	MEMS_Init(systemSetting.blAccelSensorEnabled);
	MEMS_CLOSE();
#endif

	M25Pxx_INIT();  //ext flash
	FLASH_POWER_DOWN();
    
	Pre_INIT_I2C1();
#if (TEMPERATURE_SUPPORT==1)
	MCP9804_Init(); //temperature skin, ambient
#endif
#if (VIBRATION_SUPPORT==1)
	DRV2605_Init(); //vibrating moter.
#endif

    /* baro/pressure sensor */
#if (BAROMETER_SUPPORT==1)
    LPS22HB_Init(systemSetting.blPressureSensorEnabled);
#elif (BAROMETER_SUPPORT==2)
    LPS35HW_Init(systemSetting.blPressureSensorEnabled);
#endif    

    /* magnetic sensor */
#if (MAGNETIC_SUPPORT==1)
    BMM150_Init(systemSetting.blGeoMSensorEnabled);
#endif
    
    /* gyro sensor */
#if (GYRO_SUPPORT==1)
    L3GD20H_Init(systemSetting.blGyroSensorEnabled);
#endif
    
	Battery_ADC_Init();
#if (CHARGER_SUPPORT==1)
	CHARGER_STA_INIT();
#endif

#if (AFE44x0_SUPPORT==1)
	AFE44xx_PowerOn_Init();
	AFE44xx_Shutoff();
#endif

#if (BLE_SUPPORT==1)
	BLE_INIT();
#endif

#if (CAP_SUPPORT==1)
	CAPLESENSE_Init();
#elif (CAP_SUPPORT==2)
    AD7156_Init(systemSetting.blCAPSensorEnabled);
#endif
 	//vTaskDelay(50);
#if (BLE_SUPPORT==1)
	BLE_Open();
	//BLE_Close();
	EnableLongTimer(LONG_TIMER_FLAG_BLE_HEART_BEAT, true, 60, onBleHeartBeatCheck, NULL);
#endif

	//MEMS_MOTION_MONITOR();
#if (ACCELEROMETER_SUPPORT==1)
	MEMS_FIFO_INIT();
#endif

	//
	lastKeyTouchTime = time(NULL);


	//========================
#ifdef WatchDogON

	wathdoginit.enable = true;

#else

	wathdoginit.enable = false;

#endif

	WDOG_Init(&wathdoginit);

	//========================
//	osEvent event;
//	uint32_t value;

	MESSAGE msg;
	uint16_t msgType = 0;

	while (1)
	{
//		event = osMessageGet(hMsgInterrupt, osWaitForever);
//		value = event.value.v;
//
//		msg.id = value;
//		msgType = (uint16_t) msg.params.type;
        //debug it,
		xQueueReceive(hEvtQueueDevice, &msg, portMAX_DELAY);
		msgType = (uint16_t) msg.params.type;


		// =========================================
		// mcu fault handle
		errlocated.pspBottom = __get_PSP();
		errlocated.controlRegisterValue = __get_CONTROL();

#ifdef DEBUG_MODE
		TEST_PORT2_SET();
#endif

#ifdef WatchDogON
//		if (displayTaskHeartBeat < MAX_HEART_BEAT_LIMIT)
//			&& flashTaskHeartBeat < MAX_HEART_BEAT_LIMIT)
//		if (flashTaskHeartBeat < MAX_HEART_BEAT_LIMIT)
//		{
		WDOG_Feed();
//		}
#endif
        

		switch (msgType)
		{

			case MESSAGE_SYSTEM_FIRST_BOOTUP:

				onSystemFirstTimeStartup();
				break;


			case MESSAGE_SYSTEM_STARTUP:

				onSystemStartup();

				break;

			case MESSAGE_SYSTEM_MODE_CHANGED:
			{
				SYSTEM_MODE oldMode = (SYSTEM_MODE) (msg.params.param & 0xFF);
				SYSTEM_MODE newMode = (SYSTEM_MODE) ((msg.params.param >> 8) & 0xFF);

				onSystemModeChanged(oldMode, newMode);

				break;
			}

			case TICK_Message:
                        {
				//
				displayTaskHeartBeat++;
				flashTaskHeartBeat++;

				//这里防止BLE发送给MCu的第一条命令（包含mac），MCU没有收到。
				uint8_t k = 0;
				static bool once  = true;

				if((systemStatus.blBleOnline == true) && (once == true))
				{
					for(k = 0; k < MAC_SIZE; k++)
					{
						if(BLE_DevChip.BLE_DeviceInfo[4 + k] != 0)
						{
						  	once = false; //[BG024] add.
							break;
						}
					}

					if(k == MAC_SIZE)
					{
						//说明mac地址没有获得，发送命令重新获取一次。
						getBleDeviceInfo();
                        //FIXME: should be consider if always fail(BLE fail), reset BLE.
					}

					//once = false;  //[BG024] remark. keep re-read until get MAC.
				}


				// 检测时间有没有回到从前
				struct tm* timecount = 0;

				if (blTimeReset != 0x01)
				{
					timecount = GetRTCCalendar();

					if(timecount->tm_year == RESET_TIME_TO)
						blTimeReset = 0x01;
				}

				//
				onRealtimeClockTick();

				//Backup the accumulative data which we want here.
//				iCalories,iSteps,iCalories,
				backupDCS->headFlag = BACKUP_HEADFLAG; //0x12345678;
				backupDCS->tailFlag = BACKUP_TAILFLAG; //0xabcdef00;

				backupDCS->backupDistance = iDistance;
				backupDCS->backupSteps    = iSteps;
				backupDCS->backupCalories = iCalories;
				/*****
								在这里可以控制PPGsensor的开关。在这里设置计数器，小于60，检测PPGsensor状态，如果是开，则保持开，如果是关，则打开；
								当在60到300之间时，关闭PPGsensor。
								检测变量systemStatus.blHRSensorOn。
								以上的检测过程需要在戴上手表时进行。
				*****/
#if PPG_WORK_MODE_SET

				if((isWear == false) && (ppgWorkSpanCount != 0))
				{
					ppgWorkSpanCount = 0;
				}

				if(isWear == true && ppgWorkTimespan != 0)
				{
					ppgWorkSpanCount++;
					ppgWorkSpanCount %= ppgWorkTimespan; //5分钟周期 300

					if((ppgWorkSpanCount == 1))
					{
						//用于下一个5分种开始时，显示心率
						UnLockScreen(false);
#if (OLED_SUPPORT==1)
						OLEDON();
#endif

						JumpToMenu(MENU_TYPE_HeartRate);
					}

					if(ppgWorkSpanCount  <= PPG_WORK_SECONDS) //61 
					{
						if((systemStatus.blHRSensorOn == false)&& (systemSetting.blTouchSensorEnabled == true))
						{
							//在这1分钟之内没有打开PPGSensor的话，要打开
#if (AFE44x0_SUPPORT==1)
							AFE44xx_PowerOn_Init();
#endif
						}
					}
					else
					{
						//在这4分钟之内如果打开PPGSensor的话，就要关闭
						if(systemStatus.blHRSensorOn == true)
						{
#if (AFE44x0_SUPPORT==1)
							AFE44xx_Shutoff();
#endif
						}
					}
				}

#endif
				break;
                        }
			case MESSAGE_TASK_HEARTBEAT:
			{
				if (msg.params.param == 0x02)
					displayTaskHeartBeat = 0;
				else if (msg.params.param == 0x04)
					flashTaskHeartBeat = 0;

				break;
			}

			case MESSAGE_CLOCK_SYNC:
			{
				pSystemTime = GetRTCCalendar();

//				USER_EVENT* event = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//				event->type = EVT_TYPE_CLOCK_SYNC;
//				event->data.v = 0;
//				osMailPut(hDispEventQueue, event);


				USER_EVENT event;
				event.type = EVT_TYPE_CLOCK_SYNC;


				xQueueSend(hEvtQueueDisplay, &event, 0);
				break;
			}

			case MESSAGE_ALARM_SYNC:
			{
//				USER_EVENT* event = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//				event->type = EVT_TYPE_ALARM_SYNC;
//				event->data.v = 0;
//				osMailPut(hDispEventQueue, event);

				USER_EVENT event;
				event.type = EVT_TYPE_ALARM_SYNC;

				xQueueSend(hEvtQueueDisplay, &event, 0);
				break;
			}

			case MESSAGE_BATTERY_CHARGING:
			{
				// send out charging report
//				if (systemStatus.blBatteryCharging == false)
//				{
				uint8_t u8buff[6];
				u8buff[0] = BLE_CH2;
				u8buff[1] = (BYTE) StatusReport;

				u8buff[2] = 0xff;
				u8buff[3] = 0xff;
				u8buff[4] = 0xff;

				u8buff[5] = systemStatus.blBatteryCharging;

				SendData2Host(u8buff, 6);
				vTaskDelay(150); //该延时确保这里发送出去的数据有时间被送到BLE。
//				}

				//
				USER_EVENT batteryEvent;
				batteryEvent.type = EVT_TYPE_BATTERY;
				batteryEvent.data.v = systemStatus.blBatteryCharging;

				xQueueSend(hEvtQueueDisplay, &batteryEvent, 0);

				//
				break;
			}

			case MESSAGE_BATTERY_LEVEL:
			{
				ENUM_BATTERY_LEVEL bPreviousBatteryLevel = (ENUM_BATTERY_LEVEL) msg.params.param;

				// 如果电量降到 OUT_OF_BATTERY 或 LOW_BATTERY，都震动提示
				// 正在充电时例外
				if (systemStatus.blBatteryCharging == false)
				{
					if (systemStatus.bBatteryLevel == OUT_OF_BATTERY
					        || (systemStatus.bBatteryLevel == LOW_BATTERY && bPreviousBatteryLevel == BATTERY_NORMAL))
					{
#if (VIBRATION_SUPPORT==1)
						VibrateCon(StrongBuzz_100, 2, 2);
#else
                        LED_ON();
#endif
                        
					}
				}

//				if (systemStatus.bBatteryLevel == BATTERY_NORMAL)
//					RemoveNotification(NOTIFY_SERVICE_Battery);
//				else
				RaiseNotification(NOTIFY_SERVICE_Battery);

				break;
			}

			case BLE_RX_MSG: //5

				ParseBleUartPak();
				break;

			case MESSAGE_USB_CONNECTED:

				break;


			case MESSAGE_USB_DISCONNECTED:

				break;


			case MESSAGE_FLASH_OPERATION_DONE:
			{
				// flash操作完成
				BYTE flashCMD = (BYTE) msg.params.param;//(value >> 8);

#ifndef INIT_FLASH
				FlashOpeartionCallback(flashCMD);
#endif
				break;
			}

			case MESSAGE_TIMER:
              // break;  //Atus: debug not process TIMER action.
			{
				TIMER_FLAG tf = (TIMER_FLAG) msg.params.param;

				switch (tf)
				{
					case TIMER_FLAG_TEMP:
					{
						TempReadAndStop();

						if(AMB_TMP[0] & 0x80)
						{
//							fAmbientTemperature = -(AMB_TMP[0] - 0x80 + (float)AMB_TMP[1] / 10);
							ambTempFlag = true;
							fAmbientTemperature = (uint8_t)(~(AMB_TMP[0]) + 1) + (float)AMB_TMP[1] / 10;
						}
						else
						{
							ambTempFlag = false;
							fAmbientTemperature = AMB_TMP[0] + (float)AMB_TMP[1] / 10;
						}

						if(SKIN_TMP[0] & 0x80)
						{
//						  	fSkinTemperature = -(SKIN_TMP[0] - 0x80 + (float)SKIN_TMP[1] / 10);
							skinTempFlag = true;
							fSkinTemperature = (uint8_t)(~SKIN_TMP[0] + 1) + (float)SKIN_TMP[1] / 10;
						}
						else
						{
							skinTempFlag = false;
							fSkinTemperature = SKIN_TMP[0] + (float)SKIN_TMP[1] / 10;
						}

						break;
					}
                    case TIMER_FLAG_pressure:  //FIXME: should be divide two event for one by one.
#if (BAROMETER_SUPPORT==1)
						LPS22HB_Read_converter();
#endif
#if (MAGNETIC_SUPPORT==1)
						////BMM150_read_mag_data_XYZ(); //move to interrupt
#endif
#if (CAP_SUPPORT==2)
                        if(systemSetting.blCAPSensorEnabled == true)
                        {
#if (CAP_CH==1)
#if (BGXXX==11)
                          test1.typeuint32 = GetAD7156_Ch1();
#endif
                          if(GetAD7156_OUT(AD7156_CHANNEL1))
#else
#if (BGXXX==11)
                          test1.typeuint32 = GetAD7156_Ch2();
#endif
                          if(GetAD7156_OUT(AD7156_CHANNEL2))
#endif
                          {
#if (LED_TEST==0)
                            LEDB_ON();
#endif
                          }else{
#if (LED_TEST==0)
                            LEDB_OFF();
#endif
                          }
                        }
#endif
						break;
					

//当上位机发送一个有效的时间戳，mcu计算出那个sector，具体的读地址后启动定时器，然后时间到即会到这里
					case TIMER_FLAG_BLE:
						BLE_DATA_UPLOADING_PROC();
						break;
#if (REALDATA_TIMER==1)
                    case TIMER_FLAG_REALDATA:
                        if (RealDataOnPhone)
                        {
#if (CAP_CH==1)
                          readbuf16[realdata_count]=GetAD7156_Ch1();
#else
                          readbuf16[realdata_count]=GetAD7156_Ch2(); //GetAD7156_Ch1();
#endif
                          SendRealDataOverBLE((uint8_t *)&readbuf16[realdata_count],2,0, 0, 0);
                          realdata_count++;
                          if (realdata_count==5)
                          {
                            realdata_count=0;
                            SendRealDataOverBLE(NULL, 0, 0, 1, 1);
                          }
                        }
                        break;
#endif
				}

				break;
			}

			case MEMS_INT1_Message: //22
#if (BGXXX==10)
                if (systemSetting.blAccelSensorEnabled==0)
                    test1.typeuint32++;
#endif
				if (systemStatus.blHRSensorOn == false)
				{
					Mems_Proc();
				}
				break;
			case MEMS_INT2_Message:
#if (BGXXX==10)
                if (systemSetting.blAccelSensorEnabled==0)
                    test1.typeuint32++;
#endif
				Mems_WakeUp();

				break;
				
#if (BOARD_TYPE==2)
			case KEY1_INT2_Message:	//push button1 event
                if (GetKEY1()==0x00) //low active, pressing
                  KEY1_count++;
#if (MECH_TEST==1)
                if (GetKEY1()==0x01)
                  LEDR_OFF();
                else
                  LEDR_ON();
#endif
#if 0   //for BLE driver debug.
				LEDR_ON();
                getBleDeviceInfo();	
                LEDR_OFF();
#endif
#if (LED_TEST==1)
                uint8_t keymode;
                keymode = KEY1_count%3;
                if (keymode==0x01)
                {
                    LEDR_ON();
                    LEDG_OFF();
                    LEDB_OFF();
                }else if (keymode==2)
                {
                    LEDR_OFF();
                    LEDG_ON();
                    LEDB_OFF();
                }else 
                {
                    LEDR_OFF();
                    LEDG_OFF();
                    LEDB_ON();
                }
#else 
#if (SOS_HIT_SUPPORT && BOARD_TYPE==2 && MECH_TEST==0x00)
#if (SOS_2S==1)
                if (GetKEY1()==0x01) //low active, key release
                    KEY1_LastReleaseTime = time(NULL);
                else //low active, key press
                {
                    KEY1_LastPressTime = time(NULL);
                    if (systemSetting.blSOSEnabled==0x01)
                      SOS_result++;
                }
#else
                if (systemSetting.blSOSEnabled==0x01)
                    SOS_result++;
#endif
#endif
#endif
				break;
				
			case KEY2_INT5_Message: //push button2 event.
              if (GetKEY2()==0x00)
              {
                KEY2_count++;
#if (P180F_PATCH==1 && MECH_TEST==0 && 0) //for BLE driver test
                sendBleBatteryInfo(systemStatus.bBatteryRemaining, systemStatus.fBatteryVolt);
                //sendBleBatteryInfo(80, (float) 2.75);
#endif
              }
#if (MECH_TEST==1)
              if (GetKEY2()==0x01)
                LEDG_OFF();
              else
                LEDG_ON();
#endif
#if (SOS_2S==1)
              if (GetKEY2()==0x01) //low active, key release
                  KEY2_LastReleaseTime = time(NULL);
              else //low active, key press
                  KEY2_LastPressTime = time(NULL);
#endif
#if (0)
              {
                //uint8_t ubuff[4] = {BLE_CH2, GET_BATTERY_LEVEL, 0,0};
                //ubuff[2] = 50; //systemStatus.bBatteryRemaining;
                uint8_t ubuff[4] = {BLE_CH8, GET_BATTERY_LEVEL, 0,0};
                ubuff[1] = 50; //systemStatus.bBatteryRemaining;
                
                LEUARTSentByDma(UART_CMD_2HOST, &ubuff[0], 3);

              }
#endif
#if (0)   //for BLE driver debug.
                getBleDeviceInfo();
#endif
#if (0)  //for develope debug for all sensor control.
                    if (KEY2_count&0x01)
                    {
                      systemSetting.blUVSensorEnabled = false;
                      systemSetting.blPressureSensorEnabled = false;
                      systemSetting.blGeoMSensorEnabled = false;
                      systemSetting.blGyroSensorEnabled = false;
                      systemSetting.blAccelSensorEnabled = false;
                      systemSetting.blSOSEnabled = false;
                      systemSetting.blFDEnabled = false;
                      systemSetting.blCAPSensorEnabled = false;
                    }
                    else
                    {
                      systemSetting.blUVSensorEnabled = true;
                      systemSetting.blPressureSensorEnabled = true;
                      systemSetting.blGeoMSensorEnabled = true;
                      systemSetting.blGyroSensorEnabled = true;
                      systemSetting.blAccelSensorEnabled = true;
                      systemSetting.blSOSEnabled = true;
                      systemSetting.blFDEnabled = true;
                      systemSetting.blCAPSensorEnabled = true;
                    }
#endif
#if (0)  //for develope debug for UV sensor control.
                    if (systemSetting.blUVSensorEnabled==0x01)
                    {
                      systemSetting.blUVSensorEnabled = false;
                      UV_Init(systemSetting.blUVSensorEnabled);
                    }
                    else
                    {
                      systemSetting.blUVSensorEnabled = true;
                      UV_Init(systemSetting.blUVSensorEnabled);
                    }
                    SaveSystemSettings();
#endif
#if (0)  //for develope debug for Pressure sensor control
                    if (systemSetting.blPressureSensorEnabled==0x01)
                    {
                      systemSetting.blPressureSensorEnabled = false;
                      LPS22HB_Init(systemSetting.blPressureSensorEnabled);
                    }
                    else
                    {
                      systemSetting.blPressureSensorEnabled = true;
                      LPS22HB_Init(systemSetting.blPressureSensorEnabled);
                    }
                    SaveSystemSettings();
#endif

#if (0)  //for develope debug for GeoMagnetic sensor control
                    if (systemSetting.blGeoMSensorEnabled==0x01)
                    {
                      systemSetting.blGeoMSensorEnabled = false;
                      BMM150_Init(systemSetting.blGeoMSensorEnabled);
                    }
                    else
                    {
                      systemSetting.blGeoMSensorEnabled = true;
                      BMM150_Init(systemSetting.blGeoMSensorEnabled);
                    }
                    SaveSystemSettings();
#endif
#if (0)  //for develope debug for Gyro sensor control
                    if (systemSetting.blGyroSensorEnabled==0x01)
                    {
                      systemSetting.blGyroSensorEnabled = false;
                      L3GD20H_Init(systemSetting.blGyroSensorEnabled);
                    }
                    else
                    {
                      systemSetting.blGyroSensorEnabled = true;
                      L3GD20H_Init(systemSetting.blGyroSensorEnabled);
                    }
                    SaveSystemSettings();
#endif
#if (0)  //for develope debug for Accel sensor control
                    if (systemSetting.blAccelSensorEnabled==0x01)
                    {
                      systemSetting.blAccelSensorEnabled = false;
                      MEMS_Init(systemSetting.blAccelSensorEnabled);
                    }
                    else
                    {
                      systemSetting.blAccelSensorEnabled = true;
                      MEMS_Init(systemSetting.blAccelSensorEnabled);
                    }
                    SaveSystemSettings();
#endif
#if (0)  //for develope debug for SOS alert control
                    if (systemSetting.blSOSEnabled==0x01)
                    {
                      systemSetting.blSOSEnabled = false;
                    }
                    else
                    {
                      systemSetting.blSOSEnabled = true;
                    }
                    SaveSystemSettings();
#endif
#if (0)  //for develope debug for FD alert control
                    if (systemSetting.blFDEnabled==0x01)
                    {
                      systemSetting.blFDEnabled = false;
                    }
                    else
                    {
                      systemSetting.blFDEnabled = true;
                    }
                    SaveSystemSettings();
#endif
#if (0)  //for develope debug for Cap Sensor control
                    if (systemSetting.blCAPSensorEnabled==0x01)
                    {
                      systemSetting.blCAPSensorEnabled = false;
                      AD7156_Init(systemSetting.blCAPSensorEnabled);
                    }
                    else
                    {
                      systemSetting.blCAPSensorEnabled = true;
                      AD7156_Init(systemSetting.blCAPSensorEnabled);
                    }
                    SaveSystemSettings();
#endif
#if (0) //for develope debug and QA check Cap data.
#if (REALDATA_TIMER==1)   //capative sensor get realdata.
                    if (RealDataOnPhone==0)
                    {
                      RealDataOnPhone = 1;
                      //set the BLE connection interval to fast.
                      vTaskDelay(20);
                      Change_BLE_CONNECT_INTERVAL(BLE_CONNECT_20ms);
                      vTaskDelay(20);
                      //clean the RealDataBuf.
                      SendRealDataOverBLE(NULL,0,0, 0, 1);
                      realdata_count = 0;
                      //set timer2 event trigger flag
                      EnableDelayTimer(TIMER_FLAG_REALDATA, true, 200, NULL, NULL);
#if BATTERY_LIFE_OPTIMIZATION
                      systemStatus.blHRSensorTempEnabled = false;
#endif
#if (AFE44x0_SUPPORT==1)
                      AFE44xx_PowerOn_Init();
#endif
                      SensorOffDelay = default_Ble_SensorOffDelay * current_touchsensor_feq;
                    }
                    else
                    {
                      RealDataOnPhone = 0;
                      //clean timer2 event trigger flag
                      DisableDelayTimer(TIMER_FLAG_REALDATA);
                    }
#endif
#endif
				break;
#endif

#if (GYRO_SUPPORT==1)
			case MESSAGE_GYRO_DRDY:
			{
              if (systemStatus.blGyroSensorOnline)
                ReadL3GD20HFIFO((uint8_t*)&L3GD20H_BUFF[0][0],
	             (uint8_t*)&L3GD20H_BUFF[1][0],
	             (uint8_t*)&L3GD20H_BUFF[2][0],
	             L3GD20H_FIFO_SIZE);
				break;
			}

            case MESSAGE_GYRO_INT:
			{
              if (systemStatus.blGyroSensorOnline)
                ReadL3GD20HFIFO((uint8_t*)&L3GD20H_BUFF[0][0],
	             (uint8_t*)&L3GD20H_BUFF[1][0],
	             (uint8_t*)&L3GD20H_BUFF[2][0],
	             L3GD20H_FIFO_SIZE);
				break;
			}
#endif
            
			case AFE_Message:
			{
				//TEST_L();
#if (AFE44x0_SUPPORT==1)
				extern void AFE_DATA_PROC(void);
				AFE_DATA_PROC();
#endif
				//TEST_H();
				break;
			}

			case TouchSensorMsg:
				//TouchEventACK=false;
				SkinTouchDetect();
				KeyTouchDetect();
				break;

			case TOUCH_Message:
			{
				NoTOUCHPressCount = LOCK_SCREEN_DELAY;

				extern uint8_t  KeyCode;
				BYTE key = KeyCode;

				if (key != 0)
				{
					// 如果闹钟正在响铃，则停止响铃，并立即返回
					if (systemStatus.bAlarmStatus == 1)
					{
						// 停止闹钟
						RemoveNotification(NOTIFY_SERVICE_Alarm);

						systemStatus.bAlarmStatus = 2;
						break;
					}

					// 如果正在 find me，则取消此状态
					if (systemStatus.blFindingMe)
					{
						systemStatus.blFindingMe = false;
						systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_FIND_ME;

						// 取消震动
#if (VIBRATION_SUPPORT==1)
						stopVibrate();
#endif
					}


					static bool ingore_wakeup_key = false;

					//
					if (systemStatus.blKeyAutoLocked)
					{
						if((time(NULL) - lastKeyTouchTime) > 60) // 超过一分钟没有触摸，则跳转至缺省菜单
						{
							if (systemStatus.bBatteryLevel <= LOW_BATTERY)
								JumpToMenu(MENU_TYPE_Battery);
							else
								JumpToMenu(MENU_TYPE_Time);
						}

						lastKeyTouchTime = time(NULL);
//					   capSenseSwitchModel(KEYsSCAN); //

						UnLockScreen(true);
//					   fireDisplayEvent(EVT_TYPE_KEY_UNLOCKED, 0);

//					   systemStatus.blKeyAutoLocked = false;
						ingore_wakeup_key = true;
						break;
					}

					bool blKeyReleased = ((key & KEY_FLAG_RELEASED) == KEY_FLAG_RELEASED);

					if (blKeyReleased)
					{
						if(ingore_wakeup_key == true)
							ingore_wakeup_key = false;
						else
						{
//							USER_EVENT* keyEvent = (USER_EVENT*)osMailCAlloc(hDispEventQueue, 0);
//							keyEvent->type = EVT_TYPE_KEY;
//							keyEvent->data.p = &key;
//							osMailPut(hDispEventQueue, keyEvent);


							USER_EVENT event;
							event.type = EVT_TYPE_KEY;
							event.data.v = key;    //[BG025] p>>v and remove (void *)key

							xQueueSend(hEvtQueueDisplay, &event, 0);
						}
					}
				}

				break;
			}

			case MESSAGE_ENTER_MENU:
			{
				// 进入某个菜单
				MENU_TYPE menuType = (MENU_TYPE) msg.params.param;// (value >> 16);

				onEnterMenu(menuType);

				break;
			}

			case MESSAGE_MENU_ACTION:
			{
				// 用户在某个菜单长按按键
				MENU_TYPE menuType = (MENU_TYPE) msg.params.param;//(value >> 16);

				onMenuAction(menuType);

				break;
			}

			case NOTIFICATION:
			{
				NOTIFY_SERVICE ns = (NOTIFY_SERVICE) msg.params.param;//(value >> 16);

//				if (checkNotifications(false) > 0)
//				{
//					UnLockScreen();
//
//					// 震动提醒
//				}

				// 特别处理incoming call事件
				// 如延长解锁、震动时间
//				if (systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] > 0)
				if (ns == NOTIFY_SERVICE_IncomingCall)
				{
					if (checkNotification(NOTIFY_SERVICE_IncomingCall) > 0)
					{
//						systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_NOTIFICATION;
						RaiseNotification(NOTIFY_SERVICE_IncomingCall);

						if ((systemSetting.notifiedServices & (1 << NOTIFY_SERVICE_IncomingCall)) != 0)
						{
							if ((systemSetting.notificationMode & 0x01) == 0x01)
							{
#if (VIBRATION_SUPPORT==1)
								VibrateCon(BuzzAlert750ms, 1, 10);
#endif
							}
						}
					}
					else
					{
						systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_NOTIFICATION;
						RemoveNotification(NOTIFY_SERVICE_IncomingCall);

#if (VIBRATION_SUPPORT==1)
						stopVibrate();
#endif
					}
				}
				else if (ns == NOTIFY_SERVICE_MissedCall)
				{
					if (checkNotification(NOTIFY_SERVICE_MissedCall) > 0)
					{
//						systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_NOTIFICATION;
						RaiseNotification(NOTIFY_SERVICE_MissedCall);

						if ((systemSetting.notifiedServices & (1 << NOTIFY_SERVICE_MissedCall)) != 0)
						{
							if ((systemSetting.notificationMode & 0x01) == 0x01)
							{
#if (VIBRATION_SUPPORT==1)
								VibrateCon(BuzzAlert750ms, 1, 5);
#endif
							}
						}
					}

//					else
//					{
//						systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_NOTIFICATION;
//						RemoveNotification(NOTIFY_SERVICE_MissedCall);
//
//#if (VIBRATION_SUPPORT==1)
//						stopVibrate();
//#endif
//					}
				}
				else
				{
					//设置某个notification
					RaiseNotification(ns);

					if ((systemSetting.notifiedServices & (1 << ns)) != 0)
					{
						if ((systemSetting.notificationMode & 0x01) == 0x01)
						{
#if (VIBRATION_SUPPORT==1)
							VibrateCon(BuzzAlert750ms, 1, 3);
#endif
						}
					}
				}

				break;
			}

			case MESSAGE_SENSOR_ACTIVATED:
			case MESSAGE_SENSOR_DEACTIVATED:
			{
				SENSOR_TYPE sensorType = (SENSOR_TYPE) msg.params.param;

				// 目前仅通知ppg的开关（通过ble）
				if (sensorType == SENSOR_TYPE_PPG)
				{
					// 发送ble通知 (通过StatusReport 0xf8 指令)

					uint8_t u8buff[10];
					u8buff[0] = BLE_CH2;
					u8buff[1] = (BYTE) StatusReport;

					u8buff[2] = (BYTE) systemSetting.blHRSensorEnabled;
					u8buff[3] = (BYTE) systemStatus.blHRSensorOn;
					u8buff[4] = (BYTE) systemSetting.blTouchSensorEnabled;

					u8buff[5] = 0xFF;//(BYTE) GetCurrentMenuIndex();
//					u8buff[6] = (BYTE) GetCurrentMenuType();

					SendData2Host(u8buff, 21);
				}
				else if (sensorType == SENSOR_TYPE_SKIN_TOUCH)
				{
					if (msgType == MESSAGE_SENSOR_ACTIVATED)
					{
						// 用户带上手表，跳到缺省菜单，并点亮屏幕
						if (systemStatus.bBatteryLevel <= LOW_BATTERY)
							JumpToMenu(MENU_TYPE_Battery);
						else
						{
							JumpToMenu(MENU_TYPE_HeartRate);//这里要改成跳转到心率菜单。。。2015年7月15日13:56:26
#if !BATTERY_LIFE_OPTIMIZATION
							waitValidHeartrate = true;//这个变量的作用就是（正常情况下）如果没有获取心率数据，就一直保持屏亮。
#endif
						}

						UnLockScreen(false); // 解锁屏幕
#if (OLED_SUPPORT==1)
						OLEDON();			// self turn on OLED
#endif
					}
				}

				break;
			}

			case MESSAGE_FIND_ME:
			{
				BYTE level = (BYTE) msg.params.param;

				if (level == 0)
				{
					systemStatus.blFindingMe = false;
					systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_FIND_ME;

					// stop alert
#if (VIBRATION_SUPPORT==1)
					stopVibrate();
#endif
				}
				else
				{
					systemStatus.blFindingMe = true;
					systemStatus.iFindingMeCount = systemSetting.findMeDuration;
					systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_FIND_ME;

					//
					UnLockScreen(false); // 解锁屏幕
#if (OLED_SUPPORT==1)
					OLEDON();			// self turn on OLED
#endif

					// start alert
#if (VIBRATION_SUPPORT==1)
					if(level == 1)
						VibrateCon(SoftFuzz_60, 1, systemSetting.findMeDuration);
					else if(level == 2)
						VibrateCon(BuzzAlert750ms, 1,  systemSetting.findMeDuration);
#else
                    startFlashLed(true, 1, 4, 1, 1, 4, 1);
#endif
				}

				break;
			}
#if (MAGNETIC_SUPPORT==1)
        case MESSAGE_BMM150_DRDY:  //FIXME: patch while need.
            {
				if (systemSetting.blGeoMSensorEnabled)	
				BMM150_read_mag_data_XYZ();
                //LEDR_OFF();
                //LEDG_OFF();
                //LEDB_OFF();
                break;
            }
#endif
			default:
				break;
		}

#ifdef DEBUG_MODE
		TEST_PORT2_CLEAR();
#endif
	}
}


BaseType_t createDeviceTask()
{
	return xTaskCreate(DeviceTask, DEVICE_TASK_NAME, DEVICE_TASK_STACK_DEPTH, 0, DEVICE_TASK_PRIORITY, NULL);
}
