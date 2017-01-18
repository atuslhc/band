//#include <core_cm4.h>

#include "notification.h"
#include "device_task.h"
#include "display_task.h"
#include "caplesense.h"

// �Ƿ�����֪ͨ
// ����֪ͨʱ��ϵͳ��������ʾ֪ͨ�˵���������˱�־
//bool blNewNotification = false;


// ����һ��֪ͨ���˴�������֪ͨ����ʾ��������֪ͨ��ҵ��
// ��֪ͨ���ܻᱻ������ʾ
void RaiseNotification(NOTIFY_SERVICE n)
{
//	blNewNotification = true;
	
	switch (n)
	{
		case NOTIFY_SERVICE_IncomingCall:
		case NOTIFY_SERVICE_MissedCall:
//		{
////			if (checkNotification(n) <= 0) // �绰������ֹͣ���������Ҷϡ�δ�ӣ�
////			{
////				// �����ǰ��ʾ���Ǻ��в˵�����ȡ������ʾ��������
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
	
	// ���¼�����ʾ����֪ͨ���¼�����
	fireDisplayEvent(EVT_TYPE_NOTIFICATION, (void*) n);
}

// �Ƴ� incoming call ֪ͨ
void RemoveIncomingCallNotify()
{
	systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 0;
}

// �Ƴ�֪ͨ������ low battery ֪ͨ�ڳ���ᱻ�Ƴ�
void RemoveNotification(NOTIFY_SERVICE n)
{
	systemStatus.notifyEvents[n] = 0;
	
	// ֱ����������ʾ�����ɺ��������Ȳ������
//	LockScreen();
}

inline BYTE checkNotification(NOTIFY_SERVICE n)
{
	return systemStatus.notifyEvents[n];
}

// ����Ƿ����¼���Ҫ��ʾ
// ʼ�պ��� NOTIFY_SERVICE_Other, IncomingCall MissedCall �� Battery
// �����¼���������
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

// ����Ѷ�֪ͨ
// ʵ���ϻ���� ��  battery ֮�������֪ͨ
// ���� incomingcall ֪ͨ�������� incomingcall ֪ͨ����Ϊ incomingcall �������� exclusive ��ʾ��
// battery֪ͨ �ɳ��������
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
