/****************************************Copyright (c)**************************************************
**                               Honestar Technology Co.,LTD.
**
**                                 http://www.honestar.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:           GlobalData.c
** Last modified Date:  2013.06.26
** Last Version:        V1.0
** Description:
**
**------------------------------------------------------------------------------------------------------
** Created By:          Alan Lan
** Created date:        2013/06/26
** Version:             V1.0
** Descriptions:
**
**------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
********************************************************************************************************/
#include "GlobalData.h"
#include "sleep.h"
#include "em_msc.h"
#include "m00930.h"

#include "config.h"

#include "main.h"
#include "crc.h"
#include "common_vars.h"
#include "device_task.h"
#include "display_task.h"
#include "DelayUsingTimerInt.h"




bool isMemsDeepSleeping = false;
bool isMemsSleeping = false;
uint32_t MemsSleepCount = 0;

uint32_t SystemEnergyMode = 0;

bool I2C0_used = false;

//bool TouchEventACK=false;

union _CHIP DevChip;

#define USERPAGE    0x0FE00000 /**< Address of the user page, 1k space */


uint8_t  SYSCLOCK = 14; //sysclk


__STATIC_INLINE BYTE getUserAge()
{
	return pSystemTime->tm_year + 1900 - pUserProfile->birthYear;
	//return (BYTE) pUserProfile->birthYear;
}

#if PPG_WORK_MODE_SET
void GetPPGMode(void)
{
	switch (systemSetting.ppgRunMode)
	{
		case 1:
			ppgWorkTimespan = 0;
			break;

		case 2:
			ppgWorkTimespan = 300;
			break;

		case 3:
			ppgWorkTimespan = 600;
			break;

		case 4:
			ppgWorkTimespan = 900;
			break;

		case 5:
			ppgWorkTimespan = 1800;
			break;

		case 6:
			ppgWorkTimespan = 3600;
			break;

		default:
			break;
	}
}
#endif

/* Reset the system settings to default. */
void ResetParaSettings(void)
{
  	/* clean the buffer before programming */
  	memset(&systemSetting, 0x00, sizeof(systemSetting));
  
	systemSetting.checkTag = SYSTEM_SETTING_FLAG_NORMAL;
        systemSetting.bl24HourMode = false; //[BG026] not used now.

	/* Function Setting */
	// alarm
	systemSetting.alarmSetting.enabled = false;
	systemSetting.alarmSetting.mode = ALARM_MODE_WEEKDAY;
	memset(systemSetting.alarmSetting.weekday, 1, 7);
	//systemSetting.alarmSetting.alarmTime.Year = 0; //Atus add.
	//systemSetting.alarmSetting.alarmTime.Month = 0; //Atus add.
	//systemSetting.alarmSetting.alarmTime.Day = 0; //Atus add.
	//systemSetting.alarmSetting.alarmTime.DayOfWeek = 0; //Atus add.
	systemSetting.alarmSetting.alarmTime.Hour = 8;
	systemSetting.alarmSetting.alarmTime.Minute = 0;
	systemSetting.alarmSetting.alarmTime.Second = 0;

	systemSetting.blTonesEnabled = true;
	systemSetting.blBacklightEnabled = true;
	systemSetting.BacklightOn_Delay = 5;

	/* Sensors */
	systemSetting.blHRSensorEnabled = true;
	systemSetting.blUVSensorEnabled = true;
	systemSetting.blAccelSensorEnabled = true;
	systemSetting.blAmbTempSensorEnabled = true;
	systemSetting.blSkinTempSensorEnabled = true;

	systemSetting.blBluetoothEnabled = true;

	/* User information (profile, goals) */
	systemSetting.userProfile.height = 175;
	systemSetting.userProfile.weight = 70;
	//systemSetting.userProfile.age = 36;
	systemSetting.userProfile.birthYear = 1980;
	systemSetting.userProfile.gender = MALE;
	systemSetting.userProfile.unit = UNIT_METRIC;

	/* Goals */
	systemSetting.userGoals[GOAL_SLEEP] = DEFAULT_GOAL_SLEEP;
	systemSetting.userGoals[GOAL_STEPS] = DEFAULT_GOAL_STEPS;
	systemSetting.userGoals[GOAL_DISTANCE] = DEFAULT_GOAL_DISTANCE;
	systemSetting.userGoals[GOAL_CALORIES] = DEFAULT_GOAL_CALORIES;
	systemSetting.userGoals[GOAL_UVexpT] = DEFAULT_GOAL_UVexp; //[BG023-1]
	//systemSetting.userGoals[GOAL_RESV5] = 0;

	/* SMS notification service */
	// 2bytes, bitmask, NOTIFY_SERVICE
	systemSetting.notifiedServices = (1 << NOTIFY_SERVICE_IncomingCall)
	                                 | (1 << NOTIFY_SERVICE_MissedCall)
	                                 | (1 << NOTIFY_SERVICE_Email
	                                    | (1 << NOTIFY_SERVICE_Social));	// add NOTIFY_SERVICE_Social for IOS brief news.
	// refer the CategoryID Values of Apple Notification Center Service
	systemSetting.notificationMode = 0x03;

	systemSetting.iMainMode = 0;
	
	systemSetting.iMenuEnableFlags = 0xFFFFFFFF; //bitmask enable all menu

	systemSetting.SKIN_CAPVAL = SKIN_CAPVAL_Default;
	systemSetting.TOUCH_CAPVAL = TOUCH_CAPVAL_Default;

	/* system running data */
	systemSetting.SystemRstTimes = 0; //Atus add.
	//systemSetting.FactoryTime = time(NULL); //Atus: remark to keep original factory time before release command;
	//
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	systemSetting.SystemMode = SYSTEM_MODE_MANUFACTORING;
#else
	systemSetting.SystemMode = SYSTEM_MODE_ACTIVATED;
#endif
	systemSetting.reserved = 0; //Atus add.

	/* device findMe setting */
	systemSetting.findMeMode = 3;		// 0-none, 1-vibration, 2-ring, 3-vibration+ring
	systemSetting.findMeDuration = 30;	// 30 seconds.

	systemSetting.blTouchSensorEnabled = true;

	systemSetting.ActivateTime = time(NULL); //Atus add: remark to keep original activate time before activate command;

	systemSetting.timezoneOffset = 0; //Atus add.
#if (MODEL_TYPE==1) //HEALTHCARE_TYPE
	systemSetting.bDeviceModel = 0x41;//I4S
#else
	systemSetting.bDeviceModel = 0x40;//default is i4.
#endif
	systemSetting.ppgRunMode = CONTINUE24_7;//I4C used, default PPGsensor always turn on.
	systemSetting.deviceColor = DEVICE_COLOR_BLACK;//I4H used, default is black.
        //systemSetting.res[0] = 0; //Atus add. but remark.
}

void LoadSystemSettings(void)
{
	/* Enable access to BURTC registers */
	RMU_ResetControl(rmuResetBU, false);

	if((uResetCause.data & RMU_RSTCAUSE_EM4WURST) && ( GPIO->EM4WUCAUSE & GPIO_EM4WUCAUSE_EM4WUCAUSE_C9 )
	        && ((uResetCause.data & RMU_RSTCAUSE_PORST) == 0x00))
	{
		/* Enable access to BURTC registers */
		RMU_ResetControl(rmuResetBU, false);

		//如果从EM4中复位，则恢复保存的数据
		iCalories = I_CALORIES;
		iCalories_lastSaving = I_CALORIES_LASTSAVING;
		iSteps = I_STEPS ;
		iSteps_lastSaving = I_STEPS_LASTSAVING;
		iDistance = I_DISTANCE ;
		iDistance_lastSaving = I_DISTANCE_LASTSAVING;
		active_level = ACTIVE_LEVEL;
		active_level_lastSaving = ACTIVE_LEVEL_LASTSAVING;
	}
	else
	{
		I_CALORIES = 0;
		I_CALORIES_LASTSAVING = 0;
		I_STEPS = 0;
		I_STEPS_LASTSAVING = 0;
		I_DISTANCE = 0;
		I_DISTANCE_LASTSAVING = 0;
		ACTIVE_LEVEL = 0;
		ACTIVE_LEVEL_LASTSAVING = 0;
	}


	for (int i = 0; i < 16; i++)
	{
		DevChip.EFM32DeviceInfo[i] = *((unsigned char*)(EFM32_ADDRESS + i));
	}


	/* Copy contents of the userpage (flash) into the systemSetting struct */
	memcpy(&systemSetting, (void*) USERPAGE, sizeof(SYSTEM_SETTING));

	uint32_t savedCrc = *((uint32_t*) (USERPAGE + sizeof(SYSTEM_SETTING)));
	uint16_t crc = calcCRC((uint8_t*)&systemSetting, sizeof(SYSTEM_SETTING));


	// =====================================================================
	// Check the system config initialize.
	if (systemSetting.checkTag != SYSTEM_SETTING_FLAG_NORMAL)
	{
		/* The first time startup system */
		systemStatus.blSystemSettingsCrcOk = false;

		//
		ResetParaSettings();

		//
		systemSetting.SystemRstTimes = 0;
		systemStatus.blFlashInitialized = false;

		//
//		osMessagePut(hMsgInterrupt, MESSAGE_SYSTEM_FIRST_BOOTUP, 0);
		int32_t msg = MESSAGE_SYSTEM_FIRST_BOOTUP;
		xQueueSend(hEvtQueueDevice, &msg, 0);
	}
	else if (savedCrc != crc)
	{
        /* The crc mismatch, system setting error or new firmware change the system settting. */
		// 标记正确，但crc错误
		systemStatus.blSystemSettingsCrcOk = false;

		//
		ResetParaSettings();
	}
	else
	{
        /* System setting ok, just validate some parameters while bootup. */
		systemStatus.blSystemSettingsCrcOk = true;

		// =================================================================
		// calibrate parameters
		systemSetting.alarmSetting.enabled = (systemSetting.alarmSetting.enabled != 0);
		systemSetting.alarmSetting.mode = ALARM_MODE_WEEKDAY;
		systemSetting.alarmSetting.alarmTime.Hour %= 24;
		systemSetting.alarmSetting.alarmTime.Minute %= 60;
		systemSetting.alarmSetting.alarmTime.Second %= 60;

		//
		if (systemSetting.notificationMode > 0x03)
			systemSetting.notificationMode = 0x03;

#if PPG_WORK_MODE_SET
		if(systemSetting.ppgRunMode > 6 || systemSetting.ppgRunMode < 1)
			systemSetting.ppgRunMode = CONTINUE24_7;//当该数据不正常时，给其赋默认值。
#endif

#if (MODEL_TYPE==1) //HEALTHCARE_TYPE
		if((systemSetting.bDeviceModel & 0xFF) != 0x41)
			systemSetting.bDeviceModel = 0x41; //I4S
#else
		if((systemSetting.bDeviceModel & 0xFF) != 0x40)
			systemSetting.bDeviceModel = 0x40; //I4		
#endif
		//
		if (systemSetting.SystemMode != SYSTEM_MODE_ACTIVATED
		        && systemSetting.SystemMode != SYSTEM_MODE_RELEASED)
			systemSetting.SystemMode = SYSTEM_MODE_MANUFACTORING;

		//主要是为了解决ANCS中来短信不震动的问题
		if(systemSetting.notifiedServices != 0x56)
			systemSetting.notifiedServices = 0x56;

		//这里为了解决在升级后加载系统参数时出现异常导致PPG sensor打不开，触摸出现问题等。
		if(systemSetting.SKIN_CAPVAL == 0)
			systemSetting.SKIN_CAPVAL = SKIN_CAPVAL_Default;

		if(systemSetting.TOUCH_CAPVAL == 0)
			systemSetting.TOUCH_CAPVAL = TOUCH_CAPVAL_Default;

		if((systemSetting.deviceColor != DEVICE_COLOR_BLACK) && (systemSetting.deviceColor != DEVICE_COLOR_RED))
			systemSetting.deviceColor = DEVICE_COLOR_BLACK; //当颜色配置出错时，也指定一种默认颜色。
#if BG013MS
    extern uint8_t LED_INTENSITY,AMB_uA;
                                    LED_INTENSITY = systemSetting.res[0];
                                    AMB_uA = systemSetting.res[1];
#endif
	}

#if PPG_WORK_MODE_SET
	GetPPGMode();
#endif
	//
	pSystemTime = GetRTCCalendar(); // need the age to calculate BMR
	UpdateBaseLine();

	//
	systemSetting.SystemRstTimes++; //set the system reboot counter.

	//
//	systemSetting.SystemMode = SYSTEM_MODE_ACTIVATED;//20150103
	SaveSystemSettings();


	// =================================================================
	// 系统启动
//	osMessagePut(hMsgInterrupt, MESSAGE_SYSTEM_STARTUP, 0);
	int32_t msg = MESSAGE_SYSTEM_STARTUP;
	xQueueSend(hEvtQueueDevice, &msg, 0);
}


void UpdateBaseLine(void)
{
	// =================================================================
	//This Calorie Calculator is based on the Mifflin - St Jeor equation. With this equation,
	//the Basal Metabolic Rate (BMR) is calculated by using the following formula:
	//BMR = 10 * weight(kg) + 6.25 * height(cm) - 5 * age(y) + 5		   (man)
	//BMR = 10 * weight(kg) + 6.25 * height(cm) - 5 * age(y) - 161	 (woman)

	if(systemSetting.userProfile.gender == MALE)
	{
		BMR_PER_SECOND = 10 * systemSetting.userProfile.weight
		                 + 6.25 * systemSetting.userProfile.height
		                 - 5 * getUserAge() //systemSetting.userProfile.age
		                 + 5;
	}
	else
	{
		BMR_PER_SECOND = 10 * systemSetting.userProfile.weight
		                 + 6.25 * systemSetting.userProfile.height
		                 - 5 * getUserAge() //systemSetting.userProfile.age
		                 - 161;
	}

	BMR_PER_SECOND = BMR_PER_SECOND / (24 * 3600);	// 1second
	BMR_RATE = (float)systemSetting.userProfile.weight / 3600;


}


int SaveSystemSettings()
{
	msc_Return_TypeDef ret;

	/* Initialize the MSC for writing */
	MSC_Init();

	/* Erase the page */
	ret = MSC_ErasePage((uint32_t*) USERPAGE);

	/* Check for errors. If there are errors, set the global error variable and
	  * deinitialize the MSC */
	if (ret != mscReturnOk)
	{
		//currentError = ret;
		MSC_Deinit();
		return -1;
	}

	/* Write data to the userpage */
	ret = MSC_WriteWord((uint32_t*) USERPAGE, &systemSetting,
	                    sizeof(SYSTEM_SETTING));

	/* Check for errors. If there are errors, set the global error variable and
	  * deinitialize the MSC */
	if (ret != mscReturnOk)
	{
		//currentError = ret;
		MSC_Deinit();
		return -2;
	}


	// wirte crc
	uint32_t crc = calcCRC((uint8_t*) &systemSetting, sizeof(SYSTEM_SETTING));
	ret = MSC_WriteWord((uint32_t*) (USERPAGE + sizeof(SYSTEM_SETTING)), &crc, sizeof(crc));

	/* Check for errors. If there are errors, set the global error variable and
	  * deinitialize the MSC */
	if (ret != mscReturnOk)
	{
		//currentError = ret;
		MSC_Deinit();
		return -3;
	}


	/* Deinitialize the MSC. This disables writing and locks the MSC */
	MSC_Deinit();


	//
	return 0;
}


__weak void LeuartConfig (void)
{

}

__weak void DisableLeUart (void)
{

}

void SysSleepCallBack(SLEEP_EnergyMode_t eMode)
{
// #ifdef WatchDogON
//	WDOG_Feed();
// #endif

#ifdef DEBUG_MODE
	TEST_PORT15_SET();
#endif
}

void SysWakeupCallBack(SLEEP_EnergyMode_t eMode)
{
//#ifdef WatchDogON
//	WDOG_Feed();
//#endif

#ifdef DEBUG_MODE
	TEST_PORT15_CLEAR();
#endif
}

//***  don't  use it at int code
void SetSysEnergyModeFlag(uint32_t flag)
{
	SystemEnergyMode |= flag;
	EMU_EM2Block();
}


//***  don't  use it at int code
void ClearSysEnergyModeFlag(uint32_t flag)
{
	SystemEnergyMode &= ~(flag);
	EMU_EM2UnBlock();
}

SLEEP_EnergyMode_t  GetEnergyMode(void)
{
	if ((SystemEnergyMode & (uint32_t)ForceEnterEM0)!=0) //[BG025] add (uint32_t),!=0
		return sleepEM0;
	else if(SystemEnergyMode)
		return sleepEM1;
	else  return sleepEM2;

}

#if 0
//该函数没有调用，直接把他屏蔽 2015年10月6日14:30:05
void MyIdleWork(void)
{
	SLEEP_Sleep();
}
#endif

void BatteryDrainVibratorControlFunc(void* pData)
{
#if (VIBRATION_SUPPORT==1)
	VibrateCon(BuzzAlert1000ms, 2, 1);
#endif
}

void StartBatteryDrain()
{
	systemStatus.blBatteryDraining = true;

#if BATTERY_LIFE_OPTIMIZATION
	systemStatus.blHRSensorTempEnabled = false;
#endif
    
#if (AFE44x0_SUPPORT==1)
	// 打开ppg sensor
	AFE44xx_PowerOn_Init();
#endif

	// 打开led
	systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_BATTERY_DRAIN; //
	UnLockScreen(false);

	// 打开马达
	EnableLongTimer(LONG_TIMER_FLAG_VIBRATE, true, 1, BatteryDrainVibratorControlFunc, NULL);
}

void StopBatteryDrain()
{
	systemStatus.blBatteryDraining = false;

	//
#if (AFE44x0_SUPPORT==1)
	AFE44xx_Shutoff();
#endif
	//
	systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_BATTERY_DRAIN; //
	LockScreen();

//#if (VIBRATION_SUPPORT==1)
//	stopVibrate();
//#endif
	DisableLongTimer(LONG_TIMER_FLAG_VIBRATE);
}