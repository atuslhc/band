/*
 * displayTask.c
 *
 *  Created on: 2013-6-24
 *      Author: Administrator
 */
#include <time.h>
#include "cmsis_os.h"
#include "GUI.h"

#include "bsp.h"

bool SKIN_TOUCHED=false;

osSemaphoreDef(TouchSema);
osSemaphoreId hSemaTouch;





osMessageQDef(MsgBeep, 8, uint32_t);
osMessageQId hMsgBeep;

void BeepTask(const void *argument);
osThreadDef(BeepTask,osPriorityNormal,1,0);

/*********************************************************************************************************** 
* Function Name: TouchKeyHandler()
* Input parameters: void
* Output parameters: void
* Descriptions: -- Touch Event handler and Send Key Msg
* Time: 2013-6-29 
* Creat by: Alan
************************************************************************************************************/ 
uint32_t TouchKeyHandler(void) 
{
  uint32_t Key = 0;
 /*
  * GUI_KEY_LEFT
  */
  if(CAPLESENSE_getSta(0)) {
    GUI_SendKeyMsg(GUI_KEY_LEFT, 1);
    Key = GUI_KEY_LEFT;
    
  } else {
    GUI_SendKeyMsg(GUI_KEY_LEFT, 0);
    
  }
  
 /*
  * GUI_KEY_RIGHT
  */
  if(CAPLESENSE_getSta(1)) {
    GUI_SendKeyMsg(GUI_KEY_RIGHT, 1);
    Key = GUI_KEY_RIGHT;
    
  } else {
    GUI_SendKeyMsg(GUI_KEY_RIGHT, 0);
    
  }
  
 /*
  * GUI_KEY_ENTER
  */
  if(CAPLESENSE_getSta(2)) {
    GUI_SendKeyMsg(GUI_KEY_ENTER, 1);
    Key = GUI_KEY_ENTER;
    
  } else {
    GUI_SendKeyMsg(GUI_KEY_ENTER, 0);
    
  }
  
 /*
  * GUI_KEY_BACKSPACE
  */
  if(CAPLESENSE_getSta(3)) {
    GUI_SendKeyMsg(GUI_KEY_BACKSPACE, 1);
    Key = GUI_KEY_BACKSPACE;
    
  } else{
    GUI_SendKeyMsg(GUI_KEY_BACKSPACE, 0);
    
  }
  return Key;
}

/*********************************************************************************************************** 
* Function Name: TouchTask()
* Input parameters: const void  *argument
* Output parameters: void
* Descriptions: -- Touch Task ,Handler touch key event
* Time: 2013-6-28 
* Creat by: Alan
************************************************************************************************************/ 
void TouchTask(const void *argument)
{  
  static uint32_t lastKey = 0;
  uint32_t key;
     
  /* Initialize capsense */
  CAPLESENSE_Init();
    
  while(1){
      
      osSemaphoreWait(hSemaTouch,osWaitForever);
      
      if(KeyTouchDetect()){
	  	if(CAPLESENSE_getSta(4)==KEY_STATUS_TouchingEvent)
			 {
			  if(SKIN_TOUCHED==false)
			  	{
			  	 SKIN_TOUCHED=true;
			     Sound(4000,1,0,1,SmallVolume);
			  	}
	  		  }
		  else 
		  	if(CAPLESENSE_getSta(4)==KEY_STATUS_Released)
               { 
                SKIN_TOUCHED=false;
		  		}
      	}   
    
    }
}

#if 0
/*********************************************************************************************************** 
* Function Name: TouchMonitorTask()
* Input parameters: const void *argument
* Output parameters: void
* Descriptions: -- Monitor key event and Touch Task
* Time: 2013-6-28 
* Creat by: Alan
************************************************************************************************************/ 
void TouchMonitorTask(const void *argument)
{
//  uint32_t val;
  extern osSemaphoreId hSemaKey;
  
  hSemaTouch = osSemaphoreCreate(osSemaphore(TouchSema),1);
  
  while(1) {

    osSemaphoreWait(hSemaKey,osWaitForever);
    
    if (mButton == true) {
      hTouchThreadId = osThreadCreate(osThread(TouchTask),NULL);
      if (hTouchThreadId == NULL) {
        while(1);
      }
    } else {
   
      osThreadTerminate(hTouchThreadId);
      Close_ALLCAPLESENSE();
    }
  }
}

#endif

/*********************************************************************************************************** 
* Function Name: BeepTask()
* Input parameters: const void *argument
* Output parameters: void
* Descriptions: -- Beep event handler
* Time: 2013-6-28 
* Creat by: Alan
************************************************************************************************************/ 
void BeepTask(const void *argument)
{
  osEvent event;
  uint32_t value;
  
  while(1){
    event = osMessageGet(hMsgBeep, osWaitForever);
    value = event.value.v;
    if (value == 0) {
      BeepStart(2000);
      
    } else {
      osDelay(value);
      BeepStop();
      
    }
  }
}
