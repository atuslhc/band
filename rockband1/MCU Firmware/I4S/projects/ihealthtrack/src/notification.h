#ifndef __NOTIFICATION_H
#define __NOTIFICATION_H

#include "common_vars.h"

// ����һ��֪ͨ����֪ͨ���ܻᱻ������ʾ
void RaiseNotification(NOTIFY_SERVICE n);

// �Ƴ�֪ͨ������ low battery ֪ͨ�ڳ���ᱻ�Ƴ�
void RemoveNotification(NOTIFY_SERVICE n);

// �Ƴ� incoming call ֪ͨ
void RemoveIncomingCallNotify();

// �������֪ͨ
// ʵ���ϻ���� ��  battery ֮�������֪ͨ
// ���� incomingcall ֪ͨ�������� incomingcall ֪ͨ����Ϊ incomingcall �������� exclusive ��ʾ��
// battery֪ͨ �ɳ��������
void cleanNotifications();

// ����Ƿ���ĳ��֪ͨ
BYTE checkNotification(NOTIFY_SERVICE n);

// ���֪ͨ������
BYTE getNotifications(bool excludeIncomingCall);

#endif