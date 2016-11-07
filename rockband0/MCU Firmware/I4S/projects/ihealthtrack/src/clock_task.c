/*
 * clockTask.c
 *
 *  Created on: 2013-6-15
 *      Author: Administrator
 */
#include "cmsis_os.h"
#include "main.h"

osMessageQDef(MsgInterrupt,16, uint32_t);
osMessageQId hMsgInterrupt;


void DeviceTask(const void *argument)
{
  (void)argument;
   osEvent event;
   hMsgInterrupt = osMessageCreate(osMessageQ(MsgInterrupt), osThreadGetId());
  
   while (1)
	{
	  uint32_t value;  
	  event = osMessageGet(hMsgInterrupt, osWaitForever);
	  value = event.value.v;
  	  switch(value)
  	  	{
  	  	 case BLE_Message:
		 	  extern void CallBack_From_BLE(void);
		 	  CallBack_From_BLE();
		 	  break;
			
		 default:
		 	
		 	  break;
			
  	  	  }
  
	  }
  
}


