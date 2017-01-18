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
�����Ĵ���:
   1. ��̬ �ĸ������͵ײ���Ӧ���ǹرյ�
   2. ��KEY ���� 5������
   3. ��KEY ���룬ʹ�ܵײ���Ӧ
   4. ����5��û�а�����ͬʱҲû��ʹ�ܵײ���Ӧ���ر����еĴ��������͹���
   5. ��� �ײ���Ӧ�Ѿ�ʹ�ܣ�����5��û���ĸ��������Ķ������ر��ĸ�
   6. �ײ���Ӧ ����ʱ �Զ��ر����еĴ���
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
	���û�����˼·��
	1����1СʱΪʱ��Σ���¼����ʼʱ��wearoffOldTime,����ʱ��wearoffCurrentTime;
	2) ��¼�����ʱ�� wearonTime
	3) ��1)��ʱ����У����wearonTime����0��˵������һ��Сʱ��û�д������wearonTime
	   ���㣬���Ҵ������û�����ʱ��wearoffcurrentTime,��Ҳ˵���ڹ�ȥ��1Сʱ��û�д�������һ���ٽ������
	4)��ÿ��ʱ��ν���ʱ���Ѵ���ʱ�����㣬���Ұѵ�ǰû�����ʱ����Ϊ�µ����
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

//ȥ��ԭ������notification��
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
					// Ƥ����Ӧ���ر�����,ע����һ��
					//if(systemSetting.blTouchSensorEnabled == true)
					{
//#ifndef  PPG2Dongle  // for phone
						if(systemSetting.blTouchSensorEnabled == true)
						{
#if (AFE44x0_SUPPORT==1)
							AFE44xx_Shutoff(); //������ʱ�Զ�ģʽʱ��������Źرգ�������ֶ�ģʽ������HRmenu�ر�
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

//ȥ��ԭ������notification��
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

		//�����Ѿ�����һ��Сʱ����Ҫ���¼���
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
// �Զ�����/����

uint8_t NoKeyPressCount = 0;
uint8_t NoTOUCHPressCount = LOCK_SCREEN_DELAY;
uint8_t KeyHoldCount = 0;

void LockScreen(void)
{
	systemStatus.blKeyAutoLocked = true;

    if (current_touchsensor_feq>0) //protect divide 0.
      SensorOffDelay /= current_touchsensor_feq;
      

	capSenseSwitchModel(SKINSCANONLY); //

	fireDisplayEvent(EVT_TYPE_AUTO_LOCKED, 0); //�������йر���Ļ��

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

// raiseEvent = false ����������ˢ����ʾ�ĳ��ϣ������¼�֪ͨ
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
