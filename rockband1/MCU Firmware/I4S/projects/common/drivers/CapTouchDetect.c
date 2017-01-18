#include <stdlib.h>

/* EM header files */
#include "em_device.h"

/* Drivers */
#include "caplesense.h"
#include "em_emu.h"
#include "em_acmp.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_lesense.h"

#include "common_vars.h"
#include "main.h"
#include "device_task.h"
#include "display_task.h"
#include "menu.h"
#include "Si14x.h"

/*
按键的处理:
   1. 常态 四个触摸和底部感应都是关闭的
   2. 按KEY 激活 5个按键
   3. 按KEY 三秒，使能底部感应
   4. 连续5秒没有按键，同时也没有使能底部感应，关闭所有的触摸，降低功耗
   5. 如果 底部感应已经使能，连续5秒没有四个触摸键的动作，关闭四个
   6. 底部感应 脱离时 自动关闭所有的触摸
*/

/* Capacitive sense configuration */
#include "caplesenseconfig.h"
#include "CapTouchDetect.h"


extern  uint16_t touchValues;



uint8_t  SensorOffDelay = 0;


//bool  SkinTouched = false;
bool allow_sensor = false;
//extern osMessageQId hMsgInterrupt;

extern uint16_t SkinTouchVal;//, max_SkinTouchVal;
bool firstWearOff = true;
time_t wearoffOldTime = 0;
time_t wearoffCurrentTime = 0;
time_t wearonTime = 0;
/******************************************************************************
	检测没戴表的思路：
	1）以1小时为时间段，记录下起始时间wearoffOldTime,结束时间wearoffCurrentTime;
	2) 记录戴表的时间 wearonTime
	3) 在1)的时间段中，如果wearonTime等于0，说明在这一个小时内没有戴表。如果wearonTime
	   非零，并且大于最后没戴表的时间wearoffcurrentTime,那也说明在过去的1小时内没有戴（这是一种临界情况）
	4)在每个时间段结束时，把戴表时间清零，并且把当前没戴表的时间作为新的起点
*******************************************************************************/

void SkinTouchDetect(void)
{
	if(SkinTouchVal < max_SkinTouchVal)
	{
#if PPG_WORK_MODE_SET
		isWear = true;
#endif

		if(systemStatus.blSkinTouched == false)
		{
			if(systemSetting.blTouchSensorEnabled == true)
			{
#if BATTERY_LIFE_OPTIMIZATION
			  	systemStatus.blHRSensorTempEnabled = false;
#endif
#if (AFE44x0_SUPPORT==1)
				AFE44xx_PowerOn_Init();// used the osdelay ,need to run after freeRTOS
#endif
			}

			systemStatus.blSkinTouched = true;
			Device_Wearing();

			wearonTime = time(NULL);

			//
			MESSAGE msg;
			msg.params.type = (UINT16) MESSAGE_SENSOR_ACTIVATED;
			msg.params.param = (UINT16) SENSOR_TYPE_SKIN_TOUCH;
//			osMessagePut(hMsgInterrupt, msg.id, 0);
			xQueueSend(hEvtQueueDevice, &msg.id, 0);
		}

		SensorOffDelay = default_SensorOffDelay * current_touchsensor_feq;
	}
	else // released
	{
		Device_Off();
#if PPG_WORK_MODE_SET
		isWear = false;
#endif
#if 1

//去掉原来测试notification。
		if(firstWearOff)
		{
			wearoffOldTime = time(NULL);
			firstWearOff = false;
		}

		wearoffCurrentTime = time(NULL);
#endif
		//if(systemSetting.SystemMode!=SYSTEM_MODE_MANUFACTORING)
		{
			if(SensorOffDelay)
			{
				SensorOffDelay--;

				if(SensorOffDelay == 0)
				{
					systemStatus.blSkinTouched = false;
					// 皮护感应器关闭优先,注释下一行
					//if(systemSetting.blTouchSensorEnabled == true)
					{
//#ifndef  PPG2Dongle  // for phone
						if(systemSetting.blTouchSensorEnabled == true)
						{
#if (AFE44x0_SUPPORT==1)
							AFE44xx_Shutoff(); //仅仅当时自动模式时，到这里才关闭；如果是手动模式，长按HRmenu关闭
#endif
						}

//#endif
						//
						MESSAGE msg;
						msg.params.type = (UINT16) MESSAGE_SENSOR_DEACTIVATED;
						msg.params.param = (UINT16) SENSOR_TYPE_SKIN_TOUCH;
//						osMessagePut(hMsgInterrupt, msg.id, 0);
						xQueueSend(hEvtQueueDevice, &msg.id, 0);
					}
				}
			}
		}


//		else
//		{
//			SkinTouched=false;
//		}
	}

#if 0

//去掉原来测试notification的
	if(wearoffCurrentTime - wearoffOldTime >= ONE_HOUR_SECONDS)
	{
		if(wearonTime == 0)
		{
			notificationAlertParameters.wearStatus = 0x01;
		}
		else if(wearonTime > wearoffCurrentTime)
		{
			notificationAlertParameters.wearStatus = 0x01;
		}

		//这里已经过了一个小时，需要重新计数
		wearonTime = 0;
		wearoffOldTime = wearoffCurrentTime;
	}

#endif
}


const uint8_t KeyTable[2] = {KEY_LEFT, KEY_RIGHT};
uint8_t  KeyCode;


bool key_released = true;
bool long_pressed_happened = false;


void AdjustCapVal(void)
{

	float float_temp = TouchRange * touchValues;
	TOUCHCAPVAL = (uint16_t)float_temp;

	float_temp = SkinTouchRange * SkinTouchVal;
	max_SkinTouchVal = (uint16_t)float_temp;

	SaveSystemSettings();
}

void KeyTouchDetect(void)
{

	static  uint16_t hold_Count = 0;

	if(ModelSwitched == true)
	{
		ModelSwitched = false;

		return;
	}

	if(touchValues < TOUCHCAPVAL)
	{
		hold_Count++;
		KeyCode = KeyTable[0];

		if(key_released == true)
		{
			key_released = false;

//			osMessagePut(hMsgInterrupt, TOUCH_Message, 0);

			uint32_t msg = TOUCH_Message;
			xQueueSend(hEvtQueueDevice, &msg, 0);
		}
		else
		{
			if(hold_Count > (2 * current_touchsensor_feq))
			{
				hold_Count = 0;

				if(long_pressed_happened == false)
				{
					long_pressed_happened = true;
					KeyCode |= KEY_FLAG_LONG_PRESS | KEY_FLAG_RELEASED;

//					osMessagePut(hMsgInterrupt, TOUCH_Message, 0);

					uint32_t msg = TOUCH_Message;
					xQueueSend(hEvtQueueDevice, &msg, 0);
				}
			}
		}

	}
	else
	{
		hold_Count = 0;
		KeyCode = KeyTable[0];
		KeyCode |= KEY_FLAG_RELEASED;

		if(key_released == false)
		{
			key_released = true;

			if(long_pressed_happened == false)
			{

//				osMessagePut(hMsgInterrupt, TOUCH_Message, 0);

				uint32_t msg = TOUCH_Message;
				xQueueSend(hEvtQueueDevice, &msg, 0);
			}

			long_pressed_happened = false;
		}

	}

}

// ===============================================================
// 自动锁屏/解锁

uint8_t NoKeyPressCount = 0;
uint8_t NoTOUCHPressCount = LOCK_SCREEN_DELAY;
uint8_t KeyHoldCount = 0;

void LockScreen(void)
{
	systemStatus.blKeyAutoLocked = true;

    if (current_touchsensor_feq>0) //protect divide 0.
      SensorOffDelay /= current_touchsensor_feq;
      

	capSenseSwitchModel(SKINSCANONLY); //

	fireDisplayEvent(EVT_TYPE_AUTO_LOCKED, 0); //这里面有关闭屏幕的

	LED_OFF();
}

void AutoLockScreen(void)
{
	if (NoTOUCHPressCount && (systemStatus.bDisableAutoLockFlags == 0))
	{
		NoTOUCHPressCount--;

		if (NoTOUCHPressCount == 0)
		{
			LockScreen();
		}
	}
}

// raiseEvent = false 可用于自行刷新显示的场合，比如事件通知
void UnLockScreen(bool raiseEvent)
{
	if(systemStatus.blKeyAutoLocked)
	{
		NoTOUCHPressCount = LOCK_SCREEN_DELAY;
		systemStatus.blKeyAutoLocked = false;
		capSenseSwitchModel(KEYsSCAN); //

		if (raiseEvent)
		{
			fireDisplayEvent(EVT_TYPE_KEY_UNLOCKED, 0);
		}
	}
}
