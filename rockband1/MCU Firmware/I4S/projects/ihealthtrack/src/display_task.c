/*
 * displayTask.c
 *
 *  Created on: 2013-6-24
 */
#include <time.h>

//#include "cmsis_os.h"
#include "em_letimer.h"
//#include "framebufferctrl.h"

#include "main.h"

#include "common_vars.h"
#include "device_task.h"
#include "display_task.h"
#include "task.h"

#include "menu.h"
#include "submenu.h"
#include "led_font.h"
#include "notification.h"


//osMailQDef(DispTaskQueue, 8, USER_EVENT);
//osMailQId hDispEventQueue = NULL;

/******2015Äê5ÔÂ4ÈÕ17:17:10***********************************************/

#define DISPLAY_TASK_NAME				"Display"
#define DISPLAY_TASK_STACK_DEPTH		(256 + 32)		//
#define DISPLAY_TASK_PRIORITY			(1)

#define DISPLAY_TASK_QUEUE_LEN			(10)
#define DISPLAY_TASK_QUEUE_ITME_SIZE	sizeof(USER_EVENT)

QueueHandle_t hEvtQueueDisplay = 0;
/************************************************************************/

//osSemaphoreDef(SemaphoreFramebuffer);
//osSemaphoreId hSemaphoreFramebuffer;

void fireDisplayEvent(int evtType, void* data)
{
//	USER_EVENT* userEvent = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//	userEvent->type = evtType;
//	userEvent->data.p = data;
//
//	osMailPut(hDispEventQueue, userEvent);

	USER_EVENT userEvent;
	userEvent.type = evtType;
	userEvent.data.p = data;

	if (__get_IPSR())
		xQueueSendFromISR(hEvtQueueDisplay, &userEvent, 0);
	else
		xQueueSend(hEvtQueueDisplay, &userEvent, 0);

}

void initDisplayTask()
{
	// =====================================================================
	//
//	hDispEventQueue = osMailCreate(osMailQ(DispTaskQueue), osThreadGetId());
//	osMailQAddToRegistry(hDispEventQueue, "DISP");

	hEvtQueueDisplay = xQueueCreate(DISPLAY_TASK_QUEUE_LEN, DISPLAY_TASK_QUEUE_ITME_SIZE);
	vQueueAddToRegistry(hEvtQueueDisplay, "MsgQ_Display");


//	osSemaphoreDef_t SemaphoreFramebuffer;
//	hSemaphoreFramebuffer = osSemaphoreCreate(&SemaphoreFramebuffer, 1);
//	osSemaphoreAddToRegistry(hSemaphoreFramebuffer, "Semap_DisplayBuffer");
}


void DisplayTask(void* argument)
{

	MESSAGE msg;

	MENU_Init();

	//
//	osEvent osEvt;

	static USER_EVENT evtDisplay;

	while(1)
	{
		//
//		osEvt = osMailGet(hDispEventQueue, osWaitForever);

		xQueueReceive(hEvtQueueDisplay, &evtDisplay, portMAX_DELAY);

#ifdef DEBUG_MODE
		TEST_PORT4_SET();
#endif

		// =========================================
		// mcu fault handle
		errlocated.pspBottom = __get_PSP();
		errlocated.controlRegisterValue = __get_CONTROL();


		//
		switch (evtDisplay.type)
		{
			case EVT_TYPE_AUTO_LOCKED:
			{
				MENU_FadeOut();
#if (OLED_SUPPORT==1)
				OLEDOff();
				clearScreen(false);
#endif
				break;
			}

			case EVT_TYPE_KEY_UNLOCKED:
			{
#if (OLED_SUPPORT==1)
				OLEDON();
#endif

				//发送一个模拟时钟消息，用于立即更新显示
				fireDisplayEvent(EVT_TYPE_RTC, 0);
				JumpToMenu(MENU_TYPE_Time);//解锁了，该显示时间菜单
				break;
			}

			case EVT_TYPE_RTC:
			{
				//

				msg.params.type = MESSAGE_TASK_HEARTBEAT;
				msg.params.param = 0x02;

//				osMessagePut(hMsgInterrupt, msg.id, 0);
				xQueueSend(hEvtQueueDevice, &msg.id, 0);

				//
#if 0
				static char led_cont = 0;
				led_cont++;
				led_cont %= 5;

				if(led_cont == 0)
					LED_ON();
				else
					LED_OFF();

#endif

#if (OLED_SUPPORT==1)
				if (isOLEDOff())
					break;
#endif

#ifdef DEBUG0
// 测试强制菜单显示
				static int bc = 0;
				bc++;

				if ((bc / 20) % 2 == 0)
					UnforceShowMenu();
				else //if (bc % 20 == 0)
					ForceShowMenu(2, MENU_TYPE_Battery, MENU_TYPE_Time);

#endif

//				if (systemStatus.bBatteryRemaining <= VCC0TO5)
//					ForceShowMenu(MENU_TYPE_Battery);
//				else
//				{
//					if (IsForcedShowMenu())
//						UnforceShowMenu();
//
//					// 检查是否有事件需要显示
//					if (checkNotifications(false) > 0)
//					{
//						JumpToMenu(MENU_TYPE_Notifications);
////						MENU_showEvents();
////
////						systemStatus.blNotificationsReaded = true;
//					}
//					else
//						MENU_AppUpdata();
//				}
				MENU_AppUpdata();  //准备显存数据

				break;
			}

//-----------------------------------------------------------------

			case EVT_TYPE_DOWNLOAD:
			{
				UnLockScreen(false); // 解锁屏幕
#if (OLED_SUPPORT==1)
//				clearScreen(false);
				OLEDON();			// 自行打开led
#endif
				NoTOUCHPressCount = 100;//默认升级保持时间

				JumpToMenu(MENU_TYPE_DOWNLOAD);
			}
			break;

//--------------------------------------------------------------------------

			case EVT_TYPE_KEY:
			{
#if (OLED_SUPPORT==1)
				if (isOLEDOff())
					break;
#endif

//				// 若当前正在显示通知，则清除通知
//				if (systemStatus.blNotificationsReaded)
//				{
//					systemStatus.blNotificationsReaded = false;
//
//					// 清除已读通知
//					cleanEventNotifications();
//				}
//
//				// 如果还有其他通知，则显示其他通知
//				if (checkEventNotifications(false) > 0)
//				{
//					MENU_showEvents();
//
//					systemStatus.blNotificationsReaded = true;
//
//					break;
//				}


				//
//				BYTE key = *((BYTE*)event->data.p);

				BYTE key = (BYTE)evtDisplay.data.v; //[BG025] p>>v

				bool blLongPress = ((key & KEY_FLAG_LONG_PRESS) == KEY_FLAG_LONG_PRESS);
				key &= ~KEY_FLAG_LONG_PRESS;
				key &= ~KEY_FLAG_RELEASED;

				if (blLongPress)
				{
					//
//					osMessagePut(hMsgInterrupt, (long)MESSAGE_MENU_ACTION + ((long)GetCurrentMenuType() << 16), 0);
					msg.params.type = MESSAGE_MENU_ACTION;
					msg.params.param = GetCurrentMenuType();
					xQueueSend(hEvtQueueDevice, &msg.id, 0);
				}
				else
				{
					if (key == KEY_LEFT)
					{
						MENU_AppLeft();
					}

//					else if (key == KEY_RIGHT)
//					{
//						MENU_AppRight();
//					}
				}

				break;
			}

			case EVT_TYPE_CLOCK_SYNC:
			{
				UnLockScreen(false); // 解锁屏幕
#if (OLED_SUPPORT==1)
				OLEDON();			// self turn on OLED
#endif

				JumpToMenu(MENU_TYPE_Time);

				break;
			}

			case EVT_TYPE_ALARM_SYNC:
			{
				UnLockScreen(false); // 解锁屏幕
#if (OLED_SUPPORT==1)
				OLEDON();			// self turn on OLED
#endif

				JumpToMenu(MENU_TYPE_Alarm);

				break;
			}

			case EVT_TYPE_NOTIFICATION:
			{
				NOTIFY_SERVICE ns = (NOTIFY_SERVICE)evtDisplay.data.v; //[BG025] add convert (NOTIFY_SERVICE)

				UnLockScreen(false);
#if (OLED_SUPPORT==1)
				OLEDON();			// self turn on OLED
#endif

				if (ns == NOTIFY_SERVICE_Battery)
				{
					NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY; //延迟自动锁屏时间

					if(systemStatus.bBatteryLevel == LOW_BATTERY)
					{
						if(IsForcedShowMenu())
						{
							//这里做的原因是：当从超低电压回到低电压状态时，显示菜单也要进行更改。
							if (systemSetting.SystemMode != SYSTEM_MODE_RELEASED)
								UnforceShowMenu();
						}

						if (systemSetting.SystemMode != SYSTEM_MODE_RELEASED)
							ForceShowMenu(5, MENU_TYPE_Battery, MENU_TYPE_Time, MENU_TYPE_Notifications, MENU_TYPE_IncomingCall, MENU_TYPE_Alarm);
					}
					else if(systemStatus.bBatteryLevel == OUT_OF_BATTERY)
					{
						if(IsForcedShowMenu())
						{
							//先把第low battrery中设置的取消掉，重新设置强制显示的菜单
							if (systemSetting.SystemMode != SYSTEM_MODE_RELEASED)
								UnforceShowMenu();
						}

						if (systemSetting.SystemMode != SYSTEM_MODE_RELEASED)
							ForceShowMenu(3, MENU_TYPE_Battery, MENU_TYPE_Time, MENU_TYPE_Alarm);
					}
					else if(systemStatus.bBatteryLevel == BATTERY_NORMAL)
					{
						if (IsForcedShowMenu())
						{
							if (systemSetting.SystemMode != SYSTEM_MODE_RELEASED)
								UnforceShowMenu();
						}

						JumpToMenu(MENU_TYPE_Battery);
					}
				}
				else if (ns == NOTIFY_SERVICE_IncomingCall)
				{
					NoTOUCHPressCount = LONGER_LOCK_SCREEN_DELAY; //延迟自动锁屏时间
					JumpToMenu(MENU_TYPE_IncomingCall);
				}
				else if (ns == NOTIFY_SERVICE_MissedCall)
				{
					NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY; //延迟自动锁屏时间
					JumpToMenu(MENU_TYPE_IncomingCall);
				}
//				else if (ns == NOTIFY_SERVICE_Intense_UV)
//				{
//					NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY; //延迟自动锁屏时间
//					JumpToMenu(MENU_TYPE_UltraViolet); //不需要UV报警 2015年7月14日13:03:22
//				}
				else if (ns == NOTIFY_SERVICE_Alarm)
				{
					NoTOUCHPressCount = LONGER_LOCK_SCREEN_DELAY; //延迟自动锁屏时间
					JumpToMenu(MENU_TYPE_Alarm);
				}
				else if(ns == NOTIFY_SERVICE_Step_Accomplish)
				{
					//当目标完成后主动显示相应的菜单
//					NoTOUCHPressCount = 1;
					JumpToMenu(MENU_TYPE_Step);
				}
				else if(ns == NOTIFY_SERVICE_Calorie_Accomplish)
				{
//					NoTOUCHPressCount = 1;
					JumpToMenu(MENU_TYPE_Calories);
				}
				else if(ns == NOTIFY_SERVICE_Distance_Accomplish)
				{
//					NoTOUCHPressCount = 1;
					JumpToMenu(MENU_TYPE_Distance);
				}
				else if (getNotifications(true) > 0)
				{
					//如果有其他notification被置位了，那么也跳转到短信显示那里。
					NoTOUCHPressCount = LONGER_LOCK_SCREEN_DELAY;
					JumpToMenu(MENU_TYPE_Notifications);
				}

				break;
			}


			case EVT_TYPE_BATTERY:
			{
				// 目前仅处理 充电/未充电 状态变化
				if (systemStatus.blBatteryCharging)
				{
					// 切换电池菜单
					UnLockScreen(false); // 解锁屏幕
#if (OLED_SUPPORT==1)
                    OLEDON();			// self turn on OLED
#endif

					//延长lcd点亮时间
					NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY;

					JumpToMenu(MENU_TYPE_Battery);
				}

//				if (systemStatus.blBatteryCharging)
//					systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_CHARGING;
//				else
//					systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_CHARGING;

				break;
			}


#if defined(DEBUG) || defined(DEBUG_MODE)

			case EVT_TYPE_BLUETOOTH:
			{
				UnLockScreen(false); //
#if (OLED_SUPPORT==1)
				OLEDON();			// self turn on OLED
#endif

//				blecmd = (char)event->data.v;
				blecmd = (char)evtDisplay.data.v;
				JumpToMenu(MENU_TYPE_Ble_Testing);

				break;
			}

#endif

			default:
				break;
		}

		//
		//		osMailFree(hDispEventQueue, event);    ÕâÀïÐèÒªÊÍ·ÅÂð£¿£¿£¿£¿£¿ 2015Äê5ÔÂ6ÈÕ14:29:16

#ifdef DEBUG_MODE
		TEST_PORT4_CLEAR();
#endif
	}
}

BaseType_t createDisplayTask()
{
	return xTaskCreate(DisplayTask, DISPLAY_TASK_NAME, DISPLAY_TASK_STACK_DEPTH, 0, DISPLAY_TASK_PRIORITY, NULL);
}