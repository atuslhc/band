#ifndef DEVICE_TASK_H
#define DEVICE_TASK_H

#include "freertos.h"
#include "queue.h"
#include "semphr.h"

//extern osMessageQId hMsgInterrupt;
extern QueueHandle_t hEvtQueueDevice;
extern int8_t lastLongPressedisBLE;

void initDeviceTask();
BaseType_t createDeviceTask();

#endif