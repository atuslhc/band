#ifndef __NOTIFICATION_H
#define __NOTIFICATION_H

#include "common_vars.h"

// 发出一个通知，此通知可能会被马上显示
void RaiseNotification(NOTIFY_SERVICE n);

// 移除通知，比如 low battery 通知在充电后会被移除
void RemoveNotification(NOTIFY_SERVICE n);

// 移除 incoming call 通知
void RemoveIncomingCallNotify();

// 清除所有通知
// 实际上会清除 除  battery 之外的所有通知
// 若有 incomingcall 通知，则仅清除 incomingcall 通知，因为 incomingcall 是排他性 exclusive 显示的
// battery通知 由充电进程清除
void cleanNotifications();

// 检查是否有某种通知
BYTE checkNotification(NOTIFY_SERVICE n);

// 检查通知是数量
BYTE getNotifications(bool excludeIncomingCall);

#endif