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

//如果有系统把 MEMU_NO_SYS设置为 0
#define MEMU_NO_SYS         0

/****************************************************
  CPU缓冲区：++是搬进OLED的
  AREA0           AREA1            AREA2
  \---------------+++++++++++++++++-----------------\
  \               +++++++++++++++++                 \
  \               +++++++++++++++++                 \
  \---------------+++++++++++++++++-----------------\

******************************************************/




/*------------------------------------------------------------------------------
 *全局数据的定义
 *-----------------------------------------------------------------------------*/

//菜单信息
tMemuInfoTypeDef g_tMenuInfo = { 0, UPDATA_AREA_FOREGROUND, MENU_ACT_CURRENT};

//菜单回调函数数组、同时确定了菜单的显示顺序
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

optional的优先级高于enable，当optional为false时，表明该菜单被强制显示.
optional为true时，根据enable来判断。
	enable为0xFF时，根据系统原来的配置来决定。
	如果enable不是0xFF, 就以下面的的配置来决定。如果他是0，那么该菜单就不会显示。

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
	},//找复位原因时，先禁止该菜单。
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
		.optional = true,//从false---true
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

//菜单的数目
#define MENU_NUM   				(sizeof(pFuncUpDataMenu) / sizeof(MENU_INFO))
#define THE_FIST_MENU           0          		//第一个菜单
#define THE_LAST_MENU           (MENU_NUM - 1)      //最后一个菜单

// 保存菜单使能状态，此功能用于暂时强制显示某个（某些？）菜单，比如电池电量不足时
bool menu_enabled_status_saved = false;
bool menu_enabled_status[MENU_NUM];

//菜单ramBuf
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

	//全局变量定义
	g_tMenuInfo.mtMenu = 0;
//	g_tMenuInfo.mucAction = MENU_ACT_CURRENT;
	g_tMenuInfo.mucUpDataArea = UPDATA_AREA_FOREGROUND;


	// 特别处理：如果测试菜单为第一个菜单，则直接禁止锁屏
//	if (pFuncUpDataMenu[0].type == MENU_TYPE_Testing)
//		systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_TESTING_MENU;
}

BYTE GetMenuCount()
{
	return MENU_NUM;
}

// 通过类型获得某个菜单的序号
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

// 检查指定菜单是否可用
// 可用指是否满足显示的条件，包括 enable 与 filter 两个检查
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

// 直接跳转到某个菜单
// 如果菜单不可用（enable=false || filter=false)，则无动作
void JumpToMenu(MENU_TYPE menu)
{
	// 依然应用filter规则
//	int idx = GetMenuIndex(menu);
//	if (idx < 0)
//		return ;
//	if (pFuncUpDataMenu[idx].filterFunc() == false)
//		return;
	if (!IsMenuAvaliable(menu))
		return;

	MENU_RequestUpdataAt(menu);//准备数据

	// 切换动画
	if (menu != GetCurrentMenuType())
		Menu_FlyIn();

	//
	MENU_ReplyUpdata(MENU_ACT_LIFT_SLIDING); //这里完成数据搬移从ram到显存。
}

//// 取消显示某个菜单
//// 此处取消与菜单的enable不同
//void CancelMenu(MENU_TYPE menu)
//{
//	AutoLockScreen();
//}

// 强制显示某个（某些？）菜单，比如电池电量不足时
void ForceShowMenu(int num, ...)
{
	// 先保存菜单使能状态
	// 如果已经保存，则忽略此步骤
	if (!menu_enabled_status_saved)
	{
		for (int i = 0; i < MENU_NUM; i++)
			menu_enabled_status[i] = pFuncUpDataMenu[i].enabled;

		menu_enabled_status_saved = true;

		// 禁止其他菜单
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

// 解除强制菜单显示
void UnforceShowMenu()
{
	// 恢复保存的菜单使能状态
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
 *函数：static void MENU_delay(void)
 *行参：无
 *返回：无
 *描述：滚动时的延时
 *----------------------------------------------------------------------------*/
void MENU_delay(void)
{
#if (0!= MEMU_NO_SYS)//没有系统
	uint32_t i, j;

	for(i = 0xff; i > 0; i--)
		for(j = 200; j > 0; j--);

#else
	vTaskDelay(1);
#endif
}

// 直接更新指定的菜单
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

	// 产生进入菜单的事件
	if(menu != GetCurrentMenuType())
	{
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_ENTER_MENU + ((long)menu << 16), 0);
		MESSAGE msg;
		msg.params.type = MESSAGE_ENTER_MENU;
		msg.params.param = menu;
		xQueueSend(hEvtQueueDevice, &msg.id, 0);
	}

	pFuncUpDataMenu[g_tMenuInfo.mtMenu].func();

//如果是完成目标的显示， 那么下面需要关闭屏，做显示效果。
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
  函数名：void MENU_RequestUpdata(int8_t cAct)
  行参： 1.cAct 菜单动作
            MENU_ACT_RIGHT_SLIDING
            MENU_ACT_CURRENT
            MENU_ACT_LIFT_SLIDING
		 2. offset 偏移量
  返回：无
  描述：请求更新数据，主要负责更新 a菜单的全局信息,b调用callBack,把MCU的RAM填充。
        它在要在动作之前，运行。
------------------------------------------------------------------------------*/
void MENU_RequestUpdata(int8_t cAct, BYTE offset)
{
	/*1.全局变量的更新*/
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
//			// 跳到下一个菜单时，先检查 filterFunc 是否存在，
//			//    不存在则使用此菜单
//			//    存在则调用后检查其返回值
//			//		 为 true 才使用此菜单
//			//       为 false 则跳过此菜单，检查下一个
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
//		// 产生进入菜单的事件
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
			// 跳到下一个菜单时，先检查 菜单是否 enabled
			//    为 false 则跳过此菜单，检查下一个
			//
			// 然后检查 filterFunc 是否存在，
			//    不存在则使用此菜单
			//    存在则调用后检查其返回值
			//		 为 true 才使用此菜单
			//       为 false 则跳过此菜单，检查下一个
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

		// 产生进入菜单的事件
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_ENTER_MENU + ((long)GetCurrentMenuType() << 16), 0);

		MESSAGE msg;
		msg.params.type = MESSAGE_ENTER_MENU;
		msg.params.param = GetCurrentMenuType();
		xQueueSend(hEvtQueueDevice, &msg.id, 0);

	}

	/*2.显示数据的更新*/
#ifdef DEBUG0
	// 生成模拟数据
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
  函数名：void MENU_ReplyUpdata(int8_t cAct)
  行参： 1.cAct 菜单动作
            MENU_ACT_RIGHT_SLIDING
            MENU_ACT_CURRENT
            MENU_ACT_LIFT_SLIDING
  返回：无
  描述：回应更新数据，主要负责更新 a菜单的再次更新全局信息,b把RAM区更新。
        它在要在动作之后，运行。
------------------------------------------------------------------------------*/
void MENU_ReplyUpdata(int8_t cAct)
{
	uint8_t i, j;
	BYTE UPDATA_SOURCE_AREA = 0;

	/*1.ucMenuRamBuf[][]数据的搬移 只用搬当前的位置*/
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
  函数名：void MENU_UpdataRam(uint8_t x)
  行参： 1.x MCU的RAM区第X列开始刷屏
  返回：无
  描述：MCU的RAM区第X列开始刷屏，如果是重复刷屏就可以实现滚动
------------------------------------------------------------------------------*/
void MENU_UpdataRam(uint8_t x)
{
	uint8_t i, a[3] = {0xb0, 0x00, 0x12};

	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return; // 获取信号量失败

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
	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);

	xSemaphoreGive(hI2CSemaphore);
}

/*------------------------------------------------------------------------------
  函数名：void MENU_UpdataRam(uint8_t x)
  行参： 1.x MCU的RAM区第X列开始刷屏
  返回：无
  描述：MCU的RAM区第X列开始刷屏，如果是重复刷屏就可以实现滚动
------------------------------------------------------------------------------*/
void MENU_UpdataRamByRow(uint8_t x, uint8_t row)
{
	uint8_t a[3] = {0xb0, 0x00, 0x12}; //[BG025] delete i.

	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return;


	{
	a[0] = row + 0xb0;
	OLED_Write_CommandNBytes(a, 3);
	OLED_Write_DataNBytes(&ucMenuRamBuf[row][x], SCREEN_WIDTH);
	}


	// =============================================================
	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);

	xSemaphoreGive(hI2CSemaphore);
}


/*------------------------------------------------------------------------------
  描述：菜单淡出效果，比如可用于关闭显示
------------------------------------------------------------------------------*/
void MENU_FadeOut()
{
	// 切换动画
	uint8_t i, b;

	for (b = 0; b < 16; b++)
	{
		for(i = 0; i < SCREEN_WIDTH; i++) // x, pixel
		{
			BYTE b0 = ucMenuRamBuf[0][i + UPDATA_AREA_FOREGROUND];
			BYTE b1 = ucMenuRamBuf[1][i + UPDATA_AREA_FOREGROUND];

			BYTE bit0 = (b0 & 0x80) >> 7;	// b0的最高位，将要移到b1的最低位
			BYTE bit1 = (b1 & 0x80);		// b1的最高位，始终保持在b1的最高位

			b0 = b0 << 1;
			b1 = b1 << 1;
			b1 |= bit0;
			b1 |= bit1;

			//
			BYTE b2 = ucMenuRamBuf[2][i + UPDATA_AREA_FOREGROUND];
			BYTE b3 = ucMenuRamBuf[3][i + UPDATA_AREA_FOREGROUND];

			BYTE bit2 = (b2 & 0x01) << 7;		// b2的最低位，将要 与 到b1的最高位
			BYTE bit3 = (b3 & 0x01) << 7;	// b3的最低位，将要移到在b2的最高位

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
  函数名：void MENU_AppLeft(void)
  行参： offset 偏移量，一般为1，即偏移一个
  返回：无
  描述：菜单左移.APP就是给上层调用的。
------------------------------------------------------------------------------*/
void MENU_AppLeft()
{
	if (IsForcedShowMenu() && GetEnabledMenus() <= 1)
		return;

	MENU_RequestUpdata(MENU_ACT_LIFT_SLIDING, 1);

	// 切换动画
	Menu_FlyIn();

	//
	MENU_ReplyUpdata(MENU_ACT_LIFT_SLIDING);
}

void Menu_FlyIn()
{
	// 从右侧飞入
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
//  函数名：void MENU_AppRight(void)
//  行参： 无
//  返回：无
//  描述：菜单右移.APP就是给上层调用的。
//------------------------------------------------------------------------------*/
//void MENU_AppRight(void)
//{
//	if (IsForcedShowMenu())
//		return;
//
//	int8_t i;//一定不可以用无符号型
//	MENU_RequestUpdata(MENU_ACT_RIGHT_SLIDING, 1);
//
//	// 切换动画
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
  函数名：void MENU_AppUpdata(void)
  行参： 无
  返回： 无
  描述：菜单不移（主要更新显示的数据）.APP就是给上层调用的。
------------------------------------------------------------------------------*/
void MENU_AppUpdata(void)
{
	if (IsMenuAvaliable(MENU_TYPE_Current) == false)
	{
		// 当前菜单不可以，改为菜单切换动作
		MENU_AppLeft();
	}
	else
	{
		MENU_RequestUpdata(MENU_ACT_CURRENT, 0);
		MENU_UpdataRam(UPDATA_AREA_FOREGROUND);
	}
}

/*------------------------------------------------------------------------------
  函数名：void MENU_AppUpdata(void)
  行参： 无
  返回： 无
  描述：菜单不移（主要更新显示的数据）.APP就是给上层调用的。
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
  函数名：void MENU_T_Display(void)
  行参： 无
  返回： 无
  描述：调试用的
------------------------------------------------------------------------------*/
void MENU_T_Display(void)
{
	while(1)
	{
		MENU_AppLeft();
	}
}

// 左上角为 0,0
// page: 0-left page;1-center;2-right page
// drawMode: 0-overwrite, 1-transparent, 2-xor
// 此led屏分辨率为 32x64,
// 字节方向为列向，y方向有4个字节, x范围 [0, 63]
// 显存访问方式为 [YB][x]，其中YB为y所在字节
void Oled_DrawPixel(BYTE page, BYTE x, BYTE y, int drawMode)
{
	if (x >= SCREEN_WIDTH)
		return;

	if (y >= SCREEN_HEIGHT)
		return;

	// 由y确定在哪个字节，及第几位
	BYTE yb = y / 8;
	BYTE ybit = y % 8;

	BYTE pd = 1 << ybit;

	// 读取当前字节
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

// 将图像数据（字库或图片）写入显存或缓存
// page-显存页，0,1,2
// x,y-显示位置
// pic-图像数据
// w,h-图像的尺寸
// mode-绘制模式；0=normal/overwrite,1=or/transparent,2=xor,4=not(invert)
void oled_drawPic(BYTE page, short x, short y, BYTE* const pic, short w, short h, int mode)
{
	//       y
	//		 cy0              cy1
	//       [0|1|2|3|4|5|6|7][0|1|2|3|4|5|6|7]
	// [0|1|2|3|4|5|6|7][0|1|2|3|4|5|6|7]
	// s0               s1
	//       l
	short i, j, s, l, srcByte, c1, c2, shift_f, shift_r; //[BG025] keep to review shift_r function.

	// 思路为将源数据逐字节拷贝到目标区（显存）
	// 字节的起始位置可能未对其字节（即y未对其字节）

	// 高度的字节数（应该对齐字节）
	short hb;
	hb = h / 8;

	unsigned short cy;	// 当前字节(源数据)对应的位置
	l = y % 8; // 源字节相对于目标区字节的偏移量，此偏移量由y固定

	// 两个屏蔽位，用于拷贝时保护目标数据
	shift_f = 0xff >> (8 - l);	// 低位部分
	shift_r = 0xff << l;		// 高位部分


	// 对源数据逐字节拷贝
	for(i = 0; i < w; i++)
	{
		for (j = 0; j < hb; j++)
		{
			cy = y + j * 8;	// 源数据当前字节的位置

			s = cy / 8; // 源字节在目标区的字节位置

			// 源数据
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

			if (l) // 偏移量不为0则需拷贝其余位
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


//// 将图像数据（字库）写入显存或缓存
//// page-显存页，0,1,2
//// x,y-显示位置
//// char-要显示的字符
//// font-字库数据
//// w,h-字体的尺寸
//// base-字库偏移量，字库相对标准ascii的偏移量，如字库为纯数字，则偏移量为
//// mode-绘制模式；0=normal/overwrite,1=or/transparent,2=xor,4=not(invert)
//void oled_drawChar(BYTE page, short x, short y, char c, BYTE* const font, short w, short h, short base, int mode)
//{
//}
