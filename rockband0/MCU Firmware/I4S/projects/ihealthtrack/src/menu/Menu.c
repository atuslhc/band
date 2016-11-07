#include "freertos.h"
#include "task.h"
#include "stdarg.h"

#include "m00930.h"
#include "em_gpio.h"

#include "main.h"
#include "device_task.h"

#include "menu.h"
#include "subMenu.h"

#include "notification.h"

//�����ϵͳ�� MEMU_NO_SYS����Ϊ 0
#define MEMU_NO_SYS         0

/****************************************************
  CPU��������++�ǰ��OLED��
  AREA0           AREA1            AREA2
  \---------------+++++++++++++++++-----------------\
  \               +++++++++++++++++                 \
  \               +++++++++++++++++                 \
  \---------------+++++++++++++++++-----------------\

******************************************************/




/*------------------------------------------------------------------------------
 *ȫ�����ݵĶ���
 *-----------------------------------------------------------------------------*/

//�˵���Ϣ
tMemuInfoTypeDef g_tMenuInfo = { 0, UPDATA_AREA_FOREGROUND, MENU_ACT_CURRENT};

//�˵��ص��������顢ͬʱȷ���˲˵�����ʾ˳��
typedef void (* MENU_FUNC)(void);
typedef bool (* MENU_FILTER_FUNC)(void);

typedef struct
{
	MENU_TYPE type;
//	char* name;
	bool optional;
	BYTE enabled; // 0=false;1=true;0xff=undefined(defined by user data)
	MENU_FUNC func;
	MENU_FILTER_FUNC filterFunc;
} MENU_INFO;


// menu filter funcs
bool MenuFilterAlarm();
bool MenuFilterIncomingCall();
bool MenuFilterBattery();
//bool MenuFilterIntenseUV();
bool MenuFilterNotifications();
bool MenuFilterTesting();
bool MenuFilterActivate();
bool MenuFilterDataGatherMonitor();
bool MenuFilterDownlaod();
#if FALL_DETECT_SUPPORT
bool MenuFilterFallAlert();
#endif

/*

optional�����ȼ�����enable����optionalΪfalseʱ�������ò˵���ǿ����ʾ.
optionalΪtrueʱ������enable���жϡ�
	enableΪ0xFFʱ������ϵͳԭ����������������
	���enable����0xFF, ��������ĵ��������������������0����ô�ò˵��Ͳ�����ʾ��

*/
MENU_INFO pFuncUpDataMenu[] =
{

	{
		.type = MENU_TYPE_Testing,
//		.name = "Battery",
		.func = CallBackSubMenuTesting,
		.filterFunc = MenuFilterTesting,
		.optional = true,
		.enabled = 0xff,
	},//�Ҹ�λԭ��ʱ���Ƚ�ֹ�ò˵���
	{
		.type = MENU_TYPE_Device_activate,
//		.name = "DateTime",
		.func = CallBackSubMenuActivate,
		.filterFunc = MenuFilterActivate,
		.optional = true,
		.enabled = 0xff,
	},
	{
		.type = MENU_TYPE_Time,
//		.name = "DateTime",
		.func = CallBackSubMenuTime,
		.optional = false,
		.enabled = 0xff,
	},
	{
		.type = MENU_TYPE_HeartRate,
//		.name = "HeartRate",
		.func = CallBackSubMenuHeartRate,
		.optional = true,
		.enabled = 0xff,
	},

	{
		.type = MENU_TYPE_Step,
//		.name = "Steps",
		.func = CallBackSubMenuStep,
		.optional = true,
		.enabled = 0xff,
	},

	{
		.type = MENU_TYPE_Calories,
//		.name = "Calories",
		.func = CallBackSubMenuCalories,
		.optional = true,
		.enabled = 0xff,
	},

	{
		.type = MENU_TYPE_Distance,
//		.name = "Distance",
		.func = CallBackSubMenuDistance,
		.optional = true,
		.enabled = 0xff,
	},
	{
		.type = MENU_TYPE_UltraViolet,
//		.name = "UV",
		.func = CallBackSubMenuUltraViolet,
		.optional = true,
		.enabled = 0xff,
	},
	{
		.type = MENU_TYPE_BodyTemper,
//		.name = "BodyTempeture",
		.func = CallBackSubMenuBodyTemper,
		.optional = true,
		.enabled = 0xff,
	},
	{
		.type = MENU_TYPE_AmbientTemper,
//		.name = "AmbientTempeture",
		.func = CallBackSubMenuAmbientTemper,
		.optional = true,
		.enabled = 0,
	},
	{
		.type = MENU_TYPE_Alarm,
//		.name = "Battery",
		.func = CallBackSubMenuAlarm,
//		.filterFunc = MenuFilterAlarm,
		.optional = true,//��false---true
		.enabled = 0xff,
	},

#if !BATTERY_LIFE_OPTIMIZATION2
	{
		.type = MENU_TYPE_Notifications,
//		.name = "Battery",
		.func = CallBackSubMenuSMS, //CallBackSubMenuNotifications,
//		.filterFunc = MenuFilterNotifications,
		.optional = true,
		.enabled = 0xff,
	},
	{
		.type = MENU_TYPE_IncomingCall,
//		.name = "Battery",
		.func = CallBackSubMenuIncomingCall,
//		.filterFunc = enuFilterIncomingCall,
		.optional = true,
		.enabled = 0xff,
	},
#endif
	{
		.type = MENU_TYPE_Battery,
//		.name = "Battery",
		.func = CallBackSubMenuBattery,
//		.filterFunc = MenuFilterBattery,
		.optional = false,
		.enabled = 0xff,
	},

	{
		.type = MENU_TYPE_BleMAC,
		.func = CallBackSubMenuBleMAC,
//		.filterFunc = NULL,
		.optional = false,
		.enabled  = 0xff,
	},

	{
		.type = MENU_TYPE_DOWNLOAD,
		.func = CallBackSubMenuDownload,
		.filterFunc = MenuFilterDownlaod,
		.optional = true,
		.enabled  = 0xff,
	},
#if FALL_DETECT_SUPPORT
	{
	  .type = MENU_TYPE_FALL_ALERT,
	  .func = CallBackSubMenuFallAlert,
	  .filterFunc = MenuFilterFallAlert,
	  .optional = true,
	  .enabled = 0xff,
	}
#endif
//#if defined(DEBUG) || defined(DEBUG_MODE)
//	{
//		.type = MENU_TYPE_Ble_Testing,
//		.func = CallBackSubMenuBLE,
//		.optional = true,
//		.enabled = true,
//	},
//#endif
//
//#if defined(DEBUG) || defined(DEBUG_MODE)
//	{
//		.type = MENU_TYPE_Data_gather_monitor,
//		.func = CallBackSubMenuDataGather,
////		.filterFunc = MenuFilterDataGatherMonitor,
//		.optional = true,
//		.enabled = true,
//	},
//#endif
};

//�˵�����Ŀ
#define MENU_NUM   				(sizeof(pFuncUpDataMenu) / sizeof(MENU_INFO))
#define THE_FIST_MENU           0          		//��һ���˵�
#define THE_LAST_MENU           (MENU_NUM - 1)      //���һ���˵�

// ����˵�ʹ��״̬���˹���������ʱǿ����ʾĳ����ĳЩ�����˵��������ص�������ʱ
bool menu_enabled_status_saved = false;
bool menu_enabled_status[MENU_NUM];

//�˵�ramBuf
uint8_t ucMenuRamBuf[4][192];

//
void MENU_delay(void);
void MENU_RequestUpdataAt(MENU_TYPE menu);
//void MENU_UpdataRam(uint8_t x);
//void MENU_ReplyUpdata(int8_t cAct);

//void Menu_FlyIn();
void MENU_FadeOut();

// =================================================================
void MENU_Init(void)
{
	for (int i = 0; i < MENU_NUM; i++)
	{
		MENU_INFO* mi = &(pFuncUpDataMenu[i]);

		if (mi->optional)
		{
			if (mi->enabled == 0xff)
			{
				if ((systemSetting.iMenuEnableFlags & (1 << (BYTE) mi->type)) != 0)
					mi->enabled = true;
				else
					mi->enabled = false;
			}
		}
		else
			mi->enabled = true;
	}

	//ȫ�ֱ�������
	g_tMenuInfo.mtMenu = 0;
//	g_tMenuInfo.mucAction = MENU_ACT_CURRENT;
	g_tMenuInfo.mucUpDataArea = UPDATA_AREA_FOREGROUND;


	// �ر���������Բ˵�Ϊ��һ���˵�����ֱ�ӽ�ֹ����
//	if (pFuncUpDataMenu[0].type == MENU_TYPE_Testing)
//		systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_TESTING_MENU;
}

BYTE GetMenuCount()
{
	return MENU_NUM;
}

// ͨ�����ͻ��ĳ���˵������
int8_t GetMenuIndex(MENU_TYPE menu)
{
	for (int i = 0; i < MENU_NUM; i++)
	{
		if (pFuncUpDataMenu[i].type == menu)
		{
			return i;
		}
	}

	return -1;
}

BYTE GetCurrentMenuIndex()
{
	return g_tMenuInfo.mtMenu;
}

MENU_TYPE GetCurrentMenuType()
{
	return pFuncUpDataMenu[g_tMenuInfo.mtMenu].type;
}

int GetMenuInfo(BYTE** pmi)
{
	for (int i = THE_FIST_MENU; i <= THE_LAST_MENU; i++)
	{
		(*pmi)[3 * i + 0] = pFuncUpDataMenu[i].type;
		(*pmi)[3 * i + 1] = pFuncUpDataMenu[i].optional;
		(*pmi)[3 * i + 2] = pFuncUpDataMenu[i].enabled;
//		mi[2 * i + 0] = pFuncUpDataMenu[i].type;
//		mi[2 * i + 1] = pFuncUpDataMenu[i].optional;
	}

	return MENU_NUM;
}

int GetEnabledMenus()
{
	int em = 0;

	for (int i = 0; i < MENU_NUM; i++)
	{
		if (pFuncUpDataMenu[i].enabled)
			em++;
	}

	return em;
}

void EnableMenu(MENU_TYPE menu, bool enabled)
{
	for (int i = 0; i < MENU_NUM; i++)
	{
		if (pFuncUpDataMenu[i].type == menu
		        && pFuncUpDataMenu[i].optional == true)
			pFuncUpDataMenu[i].enabled = enabled;
	}
}

// ���ָ���˵��Ƿ����
// ����ָ�Ƿ�������ʾ������������ enable �� filter �������
bool IsMenuAvaliable(MENU_TYPE menu)
{
	int idx = -1;

	if (menu == MENU_TYPE_Current)
		idx = g_tMenuInfo.mtMenu;
	else
		idx = GetMenuIndex(menu);

	if (idx < 0)
		return false;

	MENU_INFO* pMenu = &(pFuncUpDataMenu[idx]);

	if (pMenu->enabled == false)
		return false;
	else
	{
		if (pMenu->filterFunc == NULL)
			return true;
		else
			return pMenu->filterFunc();
	}

//	return false;
}

// ֱ����ת��ĳ���˵�
// ����˵������ã�enable=false || filter=false)�����޶���
void JumpToMenu(MENU_TYPE menu)
{
	// ��ȻӦ��filter����
//	int idx = GetMenuIndex(menu);
//	if (idx < 0)
//		return ;
//	if (pFuncUpDataMenu[idx].filterFunc() == false)
//		return;
	if (!IsMenuAvaliable(menu))
		return;

	MENU_RequestUpdataAt(menu);//׼������

	// �л�����
	if (menu != GetCurrentMenuType())
		Menu_FlyIn();

	//
	MENU_ReplyUpdata(MENU_ACT_LIFT_SLIDING); //����������ݰ��ƴ�ram���Դ档
}

//// ȡ����ʾĳ���˵�
//// �˴�ȡ����˵���enable��ͬ
//void CancelMenu(MENU_TYPE menu)
//{
//	AutoLockScreen();
//}

// ǿ����ʾĳ����ĳЩ�����˵��������ص�������ʱ
void ForceShowMenu(int num, ...)
{
	// �ȱ���˵�ʹ��״̬
	// ����Ѿ����棬����Դ˲���
	if (!menu_enabled_status_saved)
	{
		for (int i = 0; i < MENU_NUM; i++)
			menu_enabled_status[i] = pFuncUpDataMenu[i].enabled;

		menu_enabled_status_saved = true;

		// ��ֹ�����˵�
		for (int i = 0; i < MENU_NUM; i++)
		{
			pFuncUpDataMenu[i].enabled = false;
		}

		//
		va_list valist;
		va_start(valist, num);

		MENU_TYPE firstMenu = MENU_TYPE_Current;
		MENU_TYPE mt = MENU_TYPE_Current;

		for ( int x = 0; x < num; x++ )
		{
			mt = va_arg(valist, MENU_TYPE);  //[BG025] not fixed.

			if (firstMenu == MENU_TYPE_Current)
				firstMenu = mt;

			for (int i = 0; i < MENU_NUM; i++)
			{
				if (pFuncUpDataMenu[i].type == mt)
					pFuncUpDataMenu[i].enabled = true;
			}
		}

		va_end(valist);

		//
		JumpToMenu(firstMenu);
	}
}

// ���ǿ�Ʋ˵���ʾ
void UnforceShowMenu()
{
	// �ָ�����Ĳ˵�ʹ��״̬
	if (menu_enabled_status_saved)
	{
		for (int i = 0; i < MENU_NUM; i++)
			pFuncUpDataMenu[i].enabled = menu_enabled_status[i];

		menu_enabled_status_saved = false;
	}
}

bool IsForcedShowMenu()
{
	return menu_enabled_status_saved;
}

bool MenuFilterAlarm()
{
	return pAlarmSetting->enabled;
}

bool MenuFilterIncomingCall()
{
	if (checkNotification(NOTIFY_SERVICE_IncomingCall) > 0
	        || checkNotification(NOTIFY_SERVICE_MissedCall) > 0)
		return true;
	else
		return false;
}

bool MenuFilterBattery()
{
	if (systemStatus.blBatteryCharging)
		return true;

//	if (systemStatus.blLowBatteryFlag || systemStatus.blOutOfBatteryFlag)
	if (systemStatus.bBatteryLevel <= LOW_BATTERY)
//	if (checkNotification(NOTIFY_SERVICE_Battery))
		return true;
	else
		return false;
}

//bool MenuFilterIntenseUV()
//{
//	if (checkNotification(NOTIFY_SERVICE_Intense_UV) > 0)
//		return true;
//	else
//		return false;
//}

bool MenuFilterNotifications()
{
	if (getNotifications(false) > 0)
		return true;
	else
		return false;
}

// return true to enable menu
bool MenuFilterTesting()
{
	if(systemSetting.SystemMode == SYSTEM_MODE_MANUFACTORING) //
		return true;
	else
#if BGXXX>0
		return true; //false; //[BGXXX] turn on for debug show.      
#else
		return false;
#endif
}

#if FALL_DETECT_SUPPORT
bool MenuFilterFallAlert()
{
  return blAlertMenu==true?true:false;
}
#endif

// return true to enable menu
bool MenuFilterActivate()
{
	if(systemSetting.SystemMode == SYSTEM_MODE_RELEASED)
		return true;
	else
		return false;
}

// return true to enable menu
bool MenuFilterDataGatherMonitor()
{
	if(systemSetting.SystemMode == SYSTEM_MODE_ACTIVATED)
		return false;
	else
		return true;
}


bool MenuFilterDownlaod()
{
	if(blDuringDownload)
		return true;
	else
		return false;
}

/*------------------------------------------------------------------------------
 *������static void MENU_delay(void)
 *�вΣ���
 *���أ���
 *����������ʱ����ʱ
 *----------------------------------------------------------------------------*/
void MENU_delay(void)
{
#if (0!= MEMU_NO_SYS)//û��ϵͳ
	uint32_t i, j;

	for(i = 0xff; i > 0; i--)
		for(j = 200; j > 0; j--);

#else
	vTaskDelay(1);
#endif
}

// ֱ�Ӹ���ָ���Ĳ˵�
void MENU_RequestUpdataAt(MENU_TYPE menu)
{
//	if(menu == GetCurrentMenuType())
//		return;
//	static uint8_t count = 0;
	int8_t idx = GetMenuIndex(menu);

	if (idx < 0)
		return;

	g_tMenuInfo.mtMenu = idx;
//	g_tMenuInfo.mucAction       = MENU_ACT_RIGHT_SLIDING;
	g_tMenuInfo.mucUpDataArea   = UPDATA_AREA_BACKGROUND;

	// ��������˵����¼�
	if(menu != GetCurrentMenuType())
	{
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_ENTER_MENU + ((long)menu << 16), 0);
		MESSAGE msg;
		msg.params.type = MESSAGE_ENTER_MENU;
		msg.params.param = menu;
		xQueueSend(hEvtQueueDevice, &msg.id, 0);
	}

	pFuncUpDataMenu[g_tMenuInfo.mtMenu].func();

//��������Ŀ�����ʾ�� ��ô������Ҫ�ر���������ʾЧ����
//	if(blAccomplishGoalShine)
//	{
//		uint32_t volatile i = 0;
//
//		while(i < 120000)
//			i++;
//
//		clearScreen(false);
////		count++;
////
////		if(count > 3)
////			blAccomplishGoal = false;
//
//	}
}


/*------------------------------------------------------------------------------
  ��������void MENU_RequestUpdata(int8_t cAct)
  �вΣ� 1.cAct �˵�����
            MENU_ACT_RIGHT_SLIDING
            MENU_ACT_CURRENT
            MENU_ACT_LIFT_SLIDING
		 2. offset ƫ����
  ���أ���
  ����������������ݣ���Ҫ������� a�˵���ȫ����Ϣ,b����callBack,��MCU��RAM��䡣
        ����Ҫ�ڶ���֮ǰ�����С�
------------------------------------------------------------------------------*/
void MENU_RequestUpdata(int8_t cAct, BYTE offset)
{
	/*1.ȫ�ֱ����ĸ���*/
	if(MENU_ACT_CURRENT == cAct)
	{
//		return;
//		g_tMenuInfo.mucAction       = MENU_ACT_CURRENT;
		g_tMenuInfo.mucUpDataArea   = UPDATA_AREA_FOREGROUND;
	}
//	else if(MENU_ACT_RIGHT_SLIDING == cAct)
//	{
////		if(THE_FIST_MENU == g_tMenuInfo.mtMenu)
////		{
////			g_tMenuInfo.mtMenu = THE_LAST_MENU;
////		}
////		else
////		{
////			g_tMenuInfo.mtMenu          -= 1;
////		}
//		while(true)
//		{
//			// ������һ���˵�ʱ���ȼ�� filterFunc �Ƿ���ڣ�
//			//    ��������ʹ�ô˲˵�
//			//    ��������ú����䷵��ֵ
//			//		 Ϊ true ��ʹ�ô˲˵�
//			//       Ϊ false �������˲˵��������һ��
//			g_tMenuInfo.mtMenu -= offset;
//			if (g_tMenuInfo.mtMenu < 0)
//				g_tMenuInfo.mtMenu += MENU_NUM;
//
//			if (pFuncUpDataMenu[g_tMenuInfo.mtMenu].filterFunc == NULL)
//				break;
//			else
//			{
//				if (pFuncUpDataMenu[g_tMenuInfo.mtMenu].filterFunc() == true)
//					break;
//			}
//		}
//
//		g_tMenuInfo.mucAction       = MENU_ACT_RIGHT_SLIDING;
//		g_tMenuInfo.mucUpDataArea   = MENU_UPDATA_AREA_0;
//
//		// ��������˵����¼�
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_ENTER_MENU + ((long)GetCurrentMenuType() << 16), 0);
//	}
	else
	{
//		if( THE_LAST_MENU == g_tMenuInfo.mtMenu)
//		{
//			g_tMenuInfo.mtMenu = THE_FIST_MENU;
//		}
//		else
//		{
//			g_tMenuInfo.mtMenu          += 1;
//		}
		while(true)
		{
			// ������һ���˵�ʱ���ȼ�� �˵��Ƿ� enabled
			//    Ϊ false �������˲˵��������һ��
			//
			// Ȼ���� filterFunc �Ƿ���ڣ�
			//    ��������ʹ�ô˲˵�
			//    ��������ú����䷵��ֵ
			//		 Ϊ true ��ʹ�ô˲˵�
			//       Ϊ false �������˲˵��������һ��
			g_tMenuInfo.mtMenu += offset;
			g_tMenuInfo.mtMenu %= MENU_NUM;

			if (pFuncUpDataMenu[g_tMenuInfo.mtMenu].enabled == false)
				continue;

//			if (pFuncUpDataMenu[g_tMenuInfo.mtMenu].filterFunc != NULL
//					&& pFuncUpDataMenu[g_tMenuInfo.mtMenu].filterFunc() == false)
//				continue;

			if (pFuncUpDataMenu[g_tMenuInfo.mtMenu].filterFunc == NULL)
				break;
			else
			{
				if (pFuncUpDataMenu[g_tMenuInfo.mtMenu].filterFunc() == true)
					break;
			}
		}

//		g_tMenuInfo.mucAction       = MENU_ACT_RIGHT_SLIDING;
		g_tMenuInfo.mucUpDataArea   = UPDATA_AREA_BACKGROUND;

		// ��������˵����¼�
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_ENTER_MENU + ((long)GetCurrentMenuType() << 16), 0);

		MESSAGE msg;
		msg.params.type = MESSAGE_ENTER_MENU;
		msg.params.param = GetCurrentMenuType();
		xQueueSend(hEvtQueueDevice, &msg.id, 0);

	}

	/*2.��ʾ���ݵĸ���*/
#ifdef DEBUG0
	// ����ģ������
	iHeartRate = (rand() % 150) + 50;
	iCalories = rand() % 20;
	iSteps = rand() % 50;
	iDistance = rand() % 10;

	bUltraViolet = rand() % 50;

	fAmbientTemperature = ((float)(rand() % 200 + 50)) / 10;
	fSkinTemperature = ((float)(rand() % 200 + 50)) / 10;
#endif

	pFuncUpDataMenu[g_tMenuInfo.mtMenu].func();
}

/*------------------------------------------------------------------------------
  ��������void MENU_ReplyUpdata(int8_t cAct)
  �вΣ� 1.cAct �˵�����
            MENU_ACT_RIGHT_SLIDING
            MENU_ACT_CURRENT
            MENU_ACT_LIFT_SLIDING
  ���أ���
  ��������Ӧ�������ݣ���Ҫ������� a�˵����ٴθ���ȫ����Ϣ,b��RAM�����¡�
        ����Ҫ�ڶ���֮�����С�
------------------------------------------------------------------------------*/
void MENU_ReplyUpdata(int8_t cAct)
{
	uint8_t i, j;
	BYTE UPDATA_SOURCE_AREA = 0;

	/*1.ucMenuRamBuf[][]���ݵİ��� ֻ�ðᵱǰ��λ��*/
//	if(MENU_ACT_RIGHT_SLIDING == cAct)
//	{
//		UPDATA_SOURCE_AREA = MENU_UPDATA_AREA_0;
//	}
//	else if(MENU_ACT_LIFT_SLIDING == cAct)
//	{
	UPDATA_SOURCE_AREA = UPDATA_AREA_BACKGROUND;
//	}

	for(j = 0; j < 4; j++)
		for(i = 0; i < SCREEN_WIDTH; i++)
			ucMenuRamBuf[j][i + UPDATA_AREA_FOREGROUND] = ucMenuRamBuf[j][i + UPDATA_SOURCE_AREA];

	//
//	g_tMenuInfo.mucAction = MENU_ACT_CURRENT;
	MENU_UpdataRam(0);
}

/*------------------------------------------------------------------------------
  ��������void MENU_UpdataRam(uint8_t x)
  �вΣ� 1.x MCU��RAM����X�п�ʼˢ��
  ���أ���
  ������MCU��RAM����X�п�ʼˢ����������ظ�ˢ���Ϳ���ʵ�ֹ���
------------------------------------------------------------------------------*/
void MENU_UpdataRam(uint8_t x)
{
	uint8_t i, a[3] = {0xb0, 0x00, 0x12};

	// =============================================================
	// ���i2c�ź���
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return; // ��ȡ�ź���ʧ��

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return ;

	// =============================================================
	for(i = 0; i < 4; i++)
	{
		a[0] = i + 0xb0;
		OLED_Write_CommandNBytes(a, 3);
		OLED_Write_DataNBytes(&ucMenuRamBuf[i][x], SCREEN_WIDTH);
	}


	// =============================================================
	// �ͷ�I2C�ź���
//	osSemaphoreRelease(hI2CSemaphore);

	xSemaphoreGive(hI2CSemaphore);
}

/*------------------------------------------------------------------------------
  ��������void MENU_UpdataRam(uint8_t x)
  �вΣ� 1.x MCU��RAM����X�п�ʼˢ��
  ���أ���
  ������MCU��RAM����X�п�ʼˢ����������ظ�ˢ���Ϳ���ʵ�ֹ���
------------------------------------------------------------------------------*/
void MENU_UpdataRamByRow(uint8_t x, uint8_t row)
{
	uint8_t a[3] = {0xb0, 0x00, 0x12}; //[BG025] delete i.

	// =============================================================
	// ���i2c�ź���
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return; // ��ȡ�ź���ʧ��

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return;


	{
	a[0] = row + 0xb0;
	OLED_Write_CommandNBytes(a, 3);
	OLED_Write_DataNBytes(&ucMenuRamBuf[row][x], SCREEN_WIDTH);
	}


	// =============================================================
	// �ͷ�I2C�ź���
//	osSemaphoreRelease(hI2CSemaphore);

	xSemaphoreGive(hI2CSemaphore);
}


/*------------------------------------------------------------------------------
  �������˵�����Ч������������ڹر���ʾ
------------------------------------------------------------------------------*/
void MENU_FadeOut()
{
	// �л�����
	uint8_t i, b;

	for (b = 0; b < 16; b++)
	{
		for(i = 0; i < SCREEN_WIDTH; i++) // x, pixel
		{
			BYTE b0 = ucMenuRamBuf[0][i + UPDATA_AREA_FOREGROUND];
			BYTE b1 = ucMenuRamBuf[1][i + UPDATA_AREA_FOREGROUND];

			BYTE bit0 = (b0 & 0x80) >> 7;	// b0�����λ����Ҫ�Ƶ�b1�����λ
			BYTE bit1 = (b1 & 0x80);		// b1�����λ��ʼ�ձ�����b1�����λ

			b0 = b0 << 1;
			b1 = b1 << 1;
			b1 |= bit0;
			b1 |= bit1;

			//
			BYTE b2 = ucMenuRamBuf[2][i + UPDATA_AREA_FOREGROUND];
			BYTE b3 = ucMenuRamBuf[3][i + UPDATA_AREA_FOREGROUND];

			BYTE bit2 = (b2 & 0x01) << 7;		// b2�����λ����Ҫ �� ��b1�����λ
			BYTE bit3 = (b3 & 0x01) << 7;	// b3�����λ����Ҫ�Ƶ���b2�����λ

			b2 = b2 >> 1;
			b3 = b3 >> 1;
			b2 |= bit3;

			b1 |= bit2;

			//
			ucMenuRamBuf[0][i + UPDATA_AREA_FOREGROUND] = b0;
			ucMenuRamBuf[1][i + UPDATA_AREA_FOREGROUND] = b1;
			ucMenuRamBuf[2][i + UPDATA_AREA_FOREGROUND] = b2;
			ucMenuRamBuf[3][i + UPDATA_AREA_FOREGROUND] = b3;
		}

		if (b >= 8)
		{
			MENU_UpdataRamByRow(UPDATA_AREA_FOREGROUND, 1);
			MENU_UpdataRamByRow(UPDATA_AREA_FOREGROUND, 2);
		}
		else
			MENU_UpdataRam(UPDATA_AREA_FOREGROUND);

//		MENU_delay();
	}

	//
	clearScreen(false);
}


/*------------------------------------------------------------------------------
  ��������void MENU_AppLeft(void)
  �вΣ� offset ƫ������һ��Ϊ1����ƫ��һ��
  ���أ���
  �������˵�����.APP���Ǹ��ϲ���õġ�
------------------------------------------------------------------------------*/
void MENU_AppLeft()
{
	if (IsForcedShowMenu() && GetEnabledMenus() <= 1)
		return;

	MENU_RequestUpdata(MENU_ACT_LIFT_SLIDING, 1);

	// �л�����
	Menu_FlyIn();

	//
	MENU_ReplyUpdata(MENU_ACT_LIFT_SLIDING);
}

void Menu_FlyIn()
{
	// ���Ҳ����
	uint8_t i;

	for(i = UPDATA_AREA_FOREGROUND; i < UPDATA_AREA_FOREGROUND + 50; i += 10)
	{
		MENU_UpdataRam(i);
//		MENU_delay();
	}

	for(i = UPDATA_AREA_FOREGROUND + 50; i < UPDATA_AREA_FOREGROUND + 62; i += 4)
	{
		MENU_UpdataRam(i);
//		MENU_delay();
	}

	for(i = UPDATA_AREA_FOREGROUND + 62; i <= UPDATA_AREA_FOREGROUND + 64; i += 1)
	{
		MENU_UpdataRam(i);
//		MENU_delay();
	}
}

///*------------------------------------------------------------------------------
//  ��������void MENU_AppRight(void)
//  �вΣ� ��
//  ���أ���
//  �������˵�����.APP���Ǹ��ϲ���õġ�
//------------------------------------------------------------------------------*/
//void MENU_AppRight(void)
//{
//	if (IsForcedShowMenu())
//		return;
//
//	int8_t i;//һ�����������޷�����
//	MENU_RequestUpdata(MENU_ACT_RIGHT_SLIDING, 1);
//
//	// �л�����
////	for(i = 64; i >= 0; i -= 8)
////	{
////		MENU_UpdataRam(i);
////		MENU_delay();
////	}
//	for(i = 64; i > 16; i -= 16)
//	{
//		MENU_UpdataRam(i);
//		MENU_delay();
//	}
//	for(i = 16; i > 4; i -= 6)
//	{
//		MENU_UpdataRam(i);
//		MENU_delay();
//	}
//	for(i = 4; i >= 0; i -= 2)
//	{
//		MENU_UpdataRam(i);
//		MENU_delay();
//	}
//
//	//
//	MENU_ReplyUpdata(MENU_ACT_RIGHT_SLIDING);
//}

//void MENU_showEvents()
//{
//	g_tMenuInfo.mucUpDataArea = UPDATA_AREA_FOREGROUND;
//	CallBackSubMenuNotification();
//
//	//
//	MENU_UpdataRam(UPDATA_AREA_FOREGROUND);
//}

/*------------------------------------------------------------------------------
  ��������void MENU_AppUpdata(void)
  �вΣ� ��
  ���أ� ��
  �������˵����ƣ���Ҫ������ʾ�����ݣ�.APP���Ǹ��ϲ���õġ�
------------------------------------------------------------------------------*/
void MENU_AppUpdata(void)
{
	if (IsMenuAvaliable(MENU_TYPE_Current) == false)
	{
		// ��ǰ�˵������ԣ���Ϊ�˵��л�����
		MENU_AppLeft();
	}
	else
	{
		MENU_RequestUpdata(MENU_ACT_CURRENT, 0);
		MENU_UpdataRam(UPDATA_AREA_FOREGROUND);
	}
}

/*------------------------------------------------------------------------------
  ��������void MENU_AppUpdata(void)
  �вΣ� ��
  ���أ� ��
  �������˵����ƣ���Ҫ������ʾ�����ݣ�.APP���Ǹ��ϲ���õġ�
------------------------------------------------------------------------------*/
void MENU_AppPwrOn(uint8_t enable)
{
	if( true == enable)
	{

		OledPwrEnable(true);

	}
	else
	{
		OledPwrEnable(false);
	}
}

/*------------------------------------------------------------------------------
  ��������void MENU_T_Display(void)
  �вΣ� ��
  ���أ� ��
  �����������õ�
------------------------------------------------------------------------------*/
void MENU_T_Display(void)
{
	while(1)
	{
		MENU_AppLeft();
	}
}

// ���Ͻ�Ϊ 0,0
// page: 0-left page;1-center;2-right page
// drawMode: 0-overwrite, 1-transparent, 2-xor
// ��led���ֱ���Ϊ 32x64,
// �ֽڷ���Ϊ����y������4���ֽ�, x��Χ [0, 63]
// �Դ���ʷ�ʽΪ [YB][x]������YBΪy�����ֽ�
void Oled_DrawPixel(BYTE page, BYTE x, BYTE y, int drawMode)
{
	if (x >= SCREEN_WIDTH)
		return;

	if (y >= SCREEN_HEIGHT)
		return;

	// ��yȷ�����ĸ��ֽڣ����ڼ�λ
	BYTE yb = y / 8;
	BYTE ybit = y % 8;

	BYTE pd = 1 << ybit;

	// ��ȡ��ǰ�ֽ�
	BYTE opd = ucMenuRamBuf[yb][x + page * SCREEN_WIDTH];

	// draw
	switch (drawMode)
	{
		case 2:
			opd ^= pd;
			break;

		default:
			opd |= pd;
			break;
	}

	// update
	ucMenuRamBuf[yb][x + page * SCREEN_WIDTH] = opd;
}

void oled_drawHortLine(BYTE page, BYTE y, BYTE xFrom, BYTE xTo, int mode)
{
	for (BYTE x = xFrom; x <= xTo; x++)
		Oled_DrawPixel(page, x, y, mode);
}

void oled_drawVertLine(BYTE page, BYTE x, BYTE yFrom, BYTE yTo, int mode)
{
	for (BYTE y = yFrom; y <= yTo; y++)
		Oled_DrawPixel(page, x, y, mode);
}

// ��ͼ�����ݣ��ֿ��ͼƬ��д���Դ�򻺴�
// page-�Դ�ҳ��0,1,2
// x,y-��ʾλ��
// pic-ͼ������
// w,h-ͼ��ĳߴ�
// mode-����ģʽ��0=normal/overwrite,1=or/transparent,2=xor,4=not(invert)
void oled_drawPic(BYTE page, short x, short y, BYTE* const pic, short w, short h, int mode)
{
	//       y
	//		 cy0              cy1
	//       [0|1|2|3|4|5|6|7][0|1|2|3|4|5|6|7]
	// [0|1|2|3|4|5|6|7][0|1|2|3|4|5|6|7]
	// s0               s1
	//       l
	short i, j, s, l, srcByte, c1, c2, shift_f, shift_r; //[BG025] keep to review shift_r function.

	// ˼·Ϊ��Դ�������ֽڿ�����Ŀ�������Դ棩
	// �ֽڵ���ʼλ�ÿ���δ�����ֽڣ���yδ�����ֽڣ�

	// �߶ȵ��ֽ�����Ӧ�ö����ֽڣ�
	short hb;
	hb = h / 8;

	unsigned short cy;	// ��ǰ�ֽ�(Դ����)��Ӧ��λ��
	l = y % 8; // Դ�ֽ������Ŀ�����ֽڵ�ƫ��������ƫ������y�̶�

	// ��������λ�����ڿ���ʱ����Ŀ������
	shift_f = 0xff >> (8 - l);	// ��λ����
	shift_r = 0xff << l;		// ��λ����


	// ��Դ�������ֽڿ���
	for(i = 0; i < w; i++)
	{
		for (j = 0; j < hb; j++)
		{
			cy = y + j * 8;	// Դ���ݵ�ǰ�ֽڵ�λ��

			s = cy / 8; // Դ�ֽ���Ŀ�������ֽ�λ��

			// Դ����
			srcByte = pic[i + j * w];

			if (mode & DRAW_MODE_INVERT)
				srcByte = 0xFF - srcByte;

			c1 = srcByte << l;

			if (mode & DRAW_MODE_OVERWRITE)
				c2 = ucMenuRamBuf[s][x + i + page * SCREEN_WIDTH] & shift_f;
			else
				c2 = ucMenuRamBuf[s][x + i + page * SCREEN_WIDTH];

//			if (mode & DRAW_MODE_XOR)
			ucMenuRamBuf[s][x + i + page * SCREEN_WIDTH] = c1 | c2;

			if (l) // ƫ������Ϊ0���追������λ
			{
				c1 = srcByte >> (8 - l);

				if (mode & DRAW_MODE_OVERWRITE)
					c2 = ucMenuRamBuf[s + 1][x + i + page * SCREEN_WIDTH] & shift_f;
				else
					c2 = ucMenuRamBuf[s + 1][x + i + page * SCREEN_WIDTH];

				ucMenuRamBuf[s + 1][x + i + page * SCREEN_WIDTH] = c1 | c2;
			}
		}
	}
}


//// ��ͼ�����ݣ��ֿ⣩д���Դ�򻺴�
//// page-�Դ�ҳ��0,1,2
//// x,y-��ʾλ��
//// char-Ҫ��ʾ���ַ�
//// font-�ֿ�����
//// w,h-����ĳߴ�
//// base-�ֿ�ƫ�������ֿ���Ա�׼ascii��ƫ���������ֿ�Ϊ�����֣���ƫ����Ϊ
//// mode-����ģʽ��0=normal/overwrite,1=or/transparent,2=xor,4=not(invert)
//void oled_drawChar(BYTE page, short x, short y, char c, BYTE* const font, short w, short h, short base, int mode)
//{
//}
