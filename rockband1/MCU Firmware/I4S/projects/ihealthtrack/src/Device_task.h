#ifndef DEVICE_TASK_H
#define DEVICE_TASK_H

#include "freertos.h"
#include "queue.h"
#include "semphr.h"

typedef struct _BACKUP_DATA
{
	uint32_t headFlag;
	int backupDistance;
	int backupSteps;
	int backupCalories;
	uint32_t tailFlag;
} BACKUP_DATA;

#define BACKUP_ADDRESS  (0x20007fe0)    //the sram last 32 bytes.
#define BACKUP_HEADFLAG (0x12345678)
#define BACKUP_TAILFLAG (0xabcdef00)

//extern osMessageQId hMsgInterrupt;
extern QueueHandle_t hEvtQueueDevice;
extern int8_t lastLongPressedisBLE;
#if (SOS_2S==1)
extern time_t KEY1_LastPressTime, KEY1_LastReleaseTime;
extern time_t KEY2_LastPressTime, KEY2_LastReleaseTime;
#endif
void initDeviceTask();
BaseType_t createDeviceTask();

#endif