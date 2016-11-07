//#include <core_cm4.h>

#include "notification.h"
#include "device_task.h"
#include "display_task.h"
#include "caplesense.h"

// 是否有新通知
// 有新通知时，系统会主动显示通知菜单，并清除此标志
//bool blNewNotification = false;


// 发出一个通知，此处仅处理通知的显示，不处理通知的业务
// 此通知可能会被马上显示
void RaiseNotification(NOTIFY_SERVICE n)
{
//	blNewNotification = true;
	
	switch (n)
	{
		case NOTIFY_SERVICE_IncomingCall:
		case NOTIFY_SERVICE_MissedCall:
//		{
////			if (checkNotification(n) <= 0) // 电话呼叫已停止（接听、挂断、未接）
////			{
////				// 如果当前显示的是呼叫菜单，则取消其显示，并锁屏
////			}
//			
//			break;
//		}
		
		case NOTIFY_SERVICE_Battery:
		{
			systemStatus.notifyEvents[n] = 1;
			break;
		}
		
		default:
			systemStatus.notifyEvents[n]++;
			break;
	}
	
	// 发事件至显示任务，通知有事件产生
	fireDisplayEvent(EVT_TYPE_NOTIFICATION, (void*) n);
}

// 移除 incoming call 通知
void RemoveIncomingCallNotify()
{
	systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 0;
}

// 移除通知，比如 low battery 通知在充电后会被移除
void RemoveNotification(NOTIFY_SERVICE n)
{
	systemStatus.notifyEvents[n] = 0;
	
	// 直接锁屏，显示更新由后续解锁等操作完成
//	LockScreen();
}

inline BYTE checkNotification(NOTIFY_SERVICE n)
{
	return systemStatus.notifyEvents[n];
}

// 检查是否有事件需要显示
// 始终忽略 NOTIFY_SERVICE_Other, IncomingCall MissedCall 和 Battery
// 返回事件种类数量
BYTE getNotifications(bool excludeIncomingCall)
{
	BYTE eventTypes = 0;
	for (int i = 1; i < MAX_NOTIFY_EVENTS; i++)
	{
//		if (excludeIncomingCall && i == NOTIFY_SERVICE_IncomingCall)
		if (i == NOTIFY_SERVICE_IncomingCall 
			|| i == NOTIFY_SERVICE_MissedCall 
				|| i == NOTIFY_SERVICE_Battery)
			continue;
		
		if (systemStatus.notifyEvents[i] > 0)
			eventTypes++;
	}
	
	return eventTypes;
	
//	if (eventTypes > 0)
//		return true;
//	else
//		return false;
}

// 清除已读通知
// 实际上会清除 除  battery 之外的所有通知
// 若有 incomingcall 通知，则仅清除 incomingcall 通知，因为 incomingcall 是排他性 exclusive 显示的
// battery通知 由充电进程清除
void cleanNotifications()
{
	// 
	if (systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] > 0)
	{
		systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_NOTIFICATION;
		RemoveNotification(NOTIFY_SERVICE_IncomingCall);
//		return;
	}
	
	if (systemStatus.notifyEvents[NOTIFY_SERVICE_MissedCall] > 0)
	{
		systemStatus.bDisableAutoLockFlags &= ~AUTOLOCK_FLAG_NOTIFICATION;
		RemoveNotification(NOTIFY_SERVICE_MissedCall);
//		return;
	}
//	else
	{
		for (int i = 0; i < MAX_NOTIFY_EVENTS; i++)
		{
			if (i == NOTIFY_SERVICE_Battery)
				continue;
			
			systemStatus.notifyEvents[i] = 0;
		}
	}
}
