#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "freertos.h"
#include "semphr.h"

extern QueueHandle_t hEvtQueueDisplay;

void initDisplayTask();
void fireDisplayEvent(int evtType, void* data);
BaseType_t createDisplayTask();

#endif  /* Avoid multiple inclusion */