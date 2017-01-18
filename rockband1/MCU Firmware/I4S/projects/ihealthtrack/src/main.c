/*
 * main.c
 *
 *  Created on: 2013-6-13
 *      Author: Administrator
 *      verson: V1.0   2013-6-16 Create by liuzhengrong
 *      version:V1.01  2013-6-16 By Alan add USB part
 */
#include "freertos.h"
#include "timers.h"

#include "sleep.h"
#include "em_usb.h"
#include "em_cmu.h"
#include "em_gpio.h"
//#include "cmsis_os.h"
#include "bsp.h"
#include "config.h"
#include "ble.h"
#include "m25pxx.h"
#include "AFE44x0.h"

#include "main.h"
#include "common_vars.h"
#include "device_task.h"
#include "display_task.h"
#include "flash_task.h"
#include "cortex-m_faults.h"
#include "em_msc.h"
//#include "data_gather.h"

//=================================
// delay function, loop asm code. use in usecond unit.
void SysCtlDelay(unsigned long ulCount)
{
	__asm("    subs    r0, #1\n"
	      "    bne.n   SysCtlDelay\n"
	      "    bx      lr");
}


//=================================

//void DeviceTask(const void* argument);
//osThreadDef(DeviceTask, osPriorityRealtime, 1, (1024 + 512));
//
//void DisplayTask(const void* argument);
//osThreadDef(DisplayTask, osPriorityLow, 1, 1024);
//
//void FlashTask(const void* argument);
//osThreadDef(FlashTask, osPriorityLow, 1, 512);

//void UsbCoreTask(const void *arg);
//osThreadDef(UsbCoreTask, osPriorityHigh, 1, 512);

//extern osMessageQId hMsgInterrupt;


#if 0
// =================================================================
// second timer callback
void TickTimerCallback(xTimerHandle xTimer)
{
	//TEST_TOGGLE();
//	osMessagePut(hMsgInterrupt, TICK_Message, 0);
	int32_t msg = TICK_Message;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
}


// =================================================================
// create second timer
xTimerHandle xTickTimer;
osTimerId osTickTimer;

bool createTickTimer()
{
	xTickTimer = xTimerCreate
	             (  /* Just a text name, not used by the RTOS kernel. */
	                 "TickTimer",
	                 /* The timer period in ticks. */
	                 ( 1000 / portTICK_RATE_MS ),
	                 /* The timers will auto-reload themselves when they expire. */
	                 pdTRUE,
	                 /* Assign each timer a unique id equal to its array index. */
	                 0,
	                 /* Each timer calls the same callback when it expires. */
	                 TickTimerCallback
	             );

	if(xTickTimer != NULL)
	{
		return (xTimerStart( xTickTimer, 0 ) == pdPASS);
	}

	return false;
}
#endif

// =================================================================
// initialize system status (set default value)
void initSystemStatus()
{
  	memset(&systemStatus, 0x00, sizeof(systemStatus)); //[BG026-1] reset systemStatus.
//	systemStatus.energyMode = 0;

	//
	systemStatus.blHRSensorTempEnabled = true; //default enable heartrate sensor
	systemStatus.blHRSensorOnline = false;
	systemStatus.blECGSensorOnline = false;
	systemStatus.blAccelSensorOnline = false;
	systemStatus.blAmbTempSensorOnline = false;
	systemStatus.blSkinTempSensorOnline = false;
    systemStatus.blUVSensorOnline = false;
#if (BAROMETER_SUPPORT==1) //[BG037]
    systemStatus.blPressureSensorOnline = false; //[BG037] add.
#endif
	systemStatus.blAllSensorsOff = true;

	systemStatus.blHRSensorOn = false;

	//
	systemStatus.blUsbOnline = false;

	systemStatus.blBluetoothConnected = false;

	//
	systemStatus.blFlashOnline = false;		// ext flash chip
	systemStatus.blFlashInitialized = true; // flash是否已经初始化。此状态仅在系统第一次启动时更新（做完chip erase之后更新为true）
	systemStatus.blFlashPowerOn = false; 		// flash是否上电，平时flash处于低功耗模式，需要读写前，均需上电

	//
	systemStatus.blStopwatchEnabled = false;
	systemStatus.blStopwatchPaused = false;

	//
//	systemStatus.blOutOfBatteryFlag = false;
//	systemStatus.blLowBatteryFlag = false;
	systemStatus.bBatteryLevel = BATTERY_NORMAL;
	systemStatus.blBatteryCharging = false;

	//systemStatus.dwBatteryLevel = 0;
	systemStatus.bBatteryRemaining = 100; //100%

	//
	systemStatus.bAlarmStatus = 0;	// 闹钟状态，0=未触发；1=触发，响铃；2=被用户停止
	systemStatus.bAlarmDuration = 60;	// 闹钟持续时间，按秒计

	//
#if BGXXX>0
	systemStatus.bDisableAutoLockFlags = 0x00;//0xff;  //Atus: 0xff for sreen always on.
#else
	systemStatus.bDisableAutoLockFlags = 0;  //Atus: 0xff for sreen always on.
#endif
	systemStatus.blKeyAutoLocked = false;	// 按键锁定标志

	//
	systemStatus.blHeartBeatLock = false;	// 检测到有效心跳数据
	//
	systemStatus.blNotificationsReaded = false;
	memset(systemStatus.notifyEvents, 0, MAX_NOTIFY_EVENTS);

	//
	systemStatus.blFindingMe = false;

	//
	systemStatus.blUVAlarmDisabled = false;

	//
	systemStatus.blSkinTouched = false;

#ifdef DEBUG0
//	systemStatus.blLowBatteryFlag = true;
//	systemStatus.notifyEvents[NOTIFY_SERVICE_Battery] = 1;

//	systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 1;

	systemStatus.notifyEvents[NOTIFY_SERVICE_MissedCall] = 6;
	systemStatus.notifyEvents[NOTIFY_SERVICE_Email] = 12;
	systemStatus.notifyEvents[NOTIFY_SERVICE_Schedule] = 3;
	systemStatus.notifyEvents[NOTIFY_SERVICE_News] = 15;

	systemStatus.notifyEvents[8] = 8;
	systemStatus.notifyEvents[9] = 8;
	systemStatus.notifyEvents[10] = 8;
	systemStatus.notifyEvents[11] = 8;
	systemStatus.notifyEvents[12] = 8;
	systemStatus.notifyEvents[13] = 8;
#endif
//#if BGXXX	//move initialization to common_vars.c
//	//set debug varible default.
//	test1.typeint = 0;
//	test2.typeint = 0;
//#endif
}

/*
In the default, hardFault is enable, the others is disable. The hardfaultn exception trigger condition:
1）The others none hardfault is disable, and exception occurs. 2）Process a none hardfault exception, another none hardfault occurs.
The hardFault exception with fix priority, but the other three none hardFault have not the fix priority, their priority are configable
via NVIC_SetPriority ( UsageFault_IRQn, 0x10) or via register SCB->SHP[] setup.

 access SCB->SHCSR register can enable the none hardfault exception.
*/

//bool errflag = false;
FOMAT uResetCause = {0};

int main(void)
{
//SCB->VTOR = 0x9800; // for release mode to J-TRACE use.
// =====================================================================
	// Get and store reset bitsmcu fault handle
	errlocated.mspBottom = __get_MSP();
	uResetCause.data = RMU_ResetCauseGet();
	//RMU_ResetCauseClear(); //[BG011] remark, should not clear before used clockAppInit(), SLEEP_Init().

	// mcu fault handle
	//The register valid while error occurs, why?
	SCB->CCR |= 0x10; // enable div-by-0 and unaligned fault
//	SCB->SHCSR |= 0x00070000;   // enable Usage Fault, Bus Fault, and MMU Fault

	// =====================================================================
	/* task initialize to create the event queue before message trigger. */
	initDeviceTask();
	initDisplayTask();
	initFlashTask();
#if (BOARD_TYPE==1 && USB_SUPPORT)
    initUsbTask();
#endif

	// =====================================================================
	/* Initialize SLEEP driver, no calbacks are used */
	SLEEP_Init(SysSleepCallBack, SysWakeupCallBack);
#if 0
#if (configSLEEP_MODE < 3)
	/* do not let to sleep deeper than define */
	SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE + 1));
#endif
#endif

	// =====================================================================
	/* Chip errata */
	CHIP_Init();


	// =====================================================================
	// init system status
	initSystemStatus();


	// =====================================================================
	/* Perform the necessary hardware configuration. */
	BspInit();

#if 0   //for ICE debug test only.
#include "LPS22HB.h"
#include "BMM150.h"
#include "L3GD20H.h"
#include "AD7156.h"
    uint8_t nobuf, databuf;
    uint32_t addbuf;
	///AD7156_IIC_Init();
	LPS22HB_IIC_Init();
	I2CReadNByte(AD7156_I2C, AD7156_IIC_ADDR, 
						I2C_SUBA_ONEBYTE, 
						AD7156_CHIP_ID, ///
						&nobuf, 1);
		databuf = 0xc9;
	I2CWriteNByte(AD7156_I2C, AD7156_IIC_ADDR, 
						I2C_SUBA_ONEBYTE, 
						AD7156_CH1_CAPDAC, ///
						&databuf, 1);
	I2CWriteNByte(AD7156_I2C, AD7156_IIC_ADDR, 
						I2C_SUBA_ONEBYTE, 
						AD7156_CH2_CAPDAC, ///
						&databuf, 1);
	while(1)
		{
		addbuf = AD7156_STATUS;
	I2CReadNByte(AD7156_I2C, AD7156_IIC_ADDR, 
						I2C_SUBA_ONEBYTE, 
						addbuf, ///
						&nobuf, 1);
	
		addbuf = AD7156_STATUS;
		databuf = AD7156_STATUS;
		
	I2CWriteNByte(AD7156_I2C, AD7156_IIC_ADDR, 
						I2C_SUBA_ONEBYTE, 
						addbuf, ///
						&databuf, 1);
		}
	//LPS22HB_Init();
	LPS22HB_IIC_Init();
	I2CReadNByte(LPS22HB_I2C, LPS22HB_IIC_ADDR, I2C_SUBA_ONEBYTE, LPS22HB_WHO_AM_I, &nobuf, 1);
	if(nobuf == 0xB1)
		LEDR_ON();
	else
		LEDR_OFF();
	
	MEMS_Init();
	L3GD20H_Init();
	BMM150_init();
	if(systemStatus.blGeoMSensorOnline)
		LEDG_ON();
	else
		LEDG_OFF();
#endif

//	// =====================================================================
//	osThreadCreate(osThread(DeviceTask), NULL);
//	osThreadCreate(osThread(DisplayTask), NULL);
//	osThreadCreate(osThread(FlashTask), NULL);
	createDeviceTask();
//	createDisplayTask();
	createFlashTask();
#if (BOARD_TYPE==1 && USB_SUPPORT)
    createUSBTask();
#endif


	//
//	osKernelStart();
	vTaskStartScheduler();

	// freertos运行正常时，永远不会到这行代码
	return 0;
}


// =================================================================
// 长时间的定时器（秒级或更高，使用RTC做为驱动，精度较低）
//#define LFRCO_FREQUENCY              32768
//#define WAKEUP_INTERVAL_MS           1000
//#define RTC_COUNT_BETWEEN_WAKEUP    (((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)-1)

volatile uint16_t LongTimerStatus = 0;		// 定时器标志位，每位代表一个定时器请求

// 按位置对应每个定时器的定义，如果为 handler=NULL，则向 device task 发送 timer 消息
TIMER_DEFINE longTimerDefs[MAX_LONG_TIMER_FLAGS] = {NULL};

void DisableLongTimer(uint16_t flag)
{
	LongTimerStatus &= ~(1 << flag);
}

// interval 单位为 s
void EnableLongTimer(uint16_t flag, bool repeat, uint16_t interval, TIMER_HANDLER callback, void* userData)
{
	LongTimerStatus |= (1 << flag);

	longTimerDefs[flag].repeat = repeat;
	longTimerDefs[flag].top = interval;
	longTimerDefs[flag].counter = 0;
	longTimerDefs[flag].handler = callback;
	longTimerDefs[flag].userData = userData;
}

void ResetLongTimer(uint16_t flag)
{
	longTimerDefs[flag].counter = 0;
}


void LongTimerFunc()
{
	MESSAGE msg;

	if(LongTimerStatus == 0)
		return;

	for (int i = 0; i < MAX_LONG_TIMER_FLAGS; i++)
	{
		// 检查标志位是否有效
		if ((LongTimerStatus >> i) & 0x01 == 0x01)
		{
			TIMER_DEFINE* td = &(longTimerDefs[i]);
			td->counter++;

			if (td->counter >= td->top)
			{
				// 自动重复或禁用
				if (td->repeat)
					td->counter = 0;
				else
					DisableLongTimer(i);

				// 到达计数点
				if (td->handler)
					td->handler(td->userData);
				else
				{
//					int32_t msg = MESSAGE_TIMER +  ((long)i << 16);
//					osMessagePut(hMsgInterrupt, MESSAGE_TIMER +  ((long)i << 16), 0);

					msg.params.type = MESSAGE_TIMER;
					msg.params.param = i;

					xQueueSend(hEvtQueueDevice, &msg.id, 0);
				}
			}
		}
	}
}


/* Get the gmt timestamp
 * The RTC/clock/timer run and display use localtime, base on timezoneOffset
 * convert to GMT(UTC) ts */
time_t getGMTTimestamp()
{
	time_t ts = time(NULL) - (systemSetting.timezoneOffset * 60);
	return ts;
}

#if 0
void SendErrorInformation()
{
	uint8_t buffer[56] = {0};
//	FOMAT temp[12] = {0};

//	temp[0].data = *((volatile uint32_t*)(0x20007f00));
//	temp[1].data = *((volatile uint32_t*)(0x20007f00 + 4));
//	temp[2].data = *((volatile uint32_t*)(0x20007f00 + 8));
//	temp[3].data = *((volatile uint32_t*)(0x20007f00 + 12));
//	temp[4].data = *((volatile uint32_t*)(0x20007f00 + 16));
//	temp[5].data = *((volatile uint32_t*)(0x20007f00 + 20));
//	temp[6].data = *((volatile uint32_t*)(0x20007f00 + 24));
//	temp[7].data = *((volatile uint32_t*)(0x20007f00 + 28));
//	temp[8].data = *((volatile uint32_t*)(0x20007f00 + 32));
//	temp[9].data = *((volatile uint32_t*)(0x20007f00 + 36));
//	temp[10].data = *((volatile uint32_t*)(0x20007f00 + 40));
//	temp[11].data = *((volatile uint32_t*)(0x20007f00 + 44));
//
//
////	temp[11].data = *((volatile uint32_t*)(0x20007f00 + 48));
////	temp[3].data = *((volatile uint32_t*)(0x20007f00 + 12));
////	temp[4].data = *((volatile uint32_t*)(0x20007f00 + 16));
//
//
//	memcpy(&buffer[3], temp[0].buffer, 4);
//	memcpy(&buffer[4 + 3], temp[1].buffer, 4);
//	memcpy(&buffer[8 + 3], temp[2].buffer, 4);
//	memcpy(&buffer[12 + 3], temp[3].buffer, 4);
//	memcpy(&buffer[16 + 3], temp[4].buffer, 4);
//	memcpy(&buffer[20 + 3], temp[1].buffer, 4);
//	memcpy(&buffer[24 + 3], temp[2].buffer, 4);
//	memcpy(&buffer[28 + 3], temp[3].buffer, 4);
//	memcpy(&buffer[32 + 3], temp[4].buffer, 4);
//	memcpy(&buffer[36 + 3], temp[2].buffer, 4);
//	memcpy(&buffer[40 + 3], temp[3].buffer, 4);
//	memcpy(&buffer[44 + 3], temp[4].buffer, 4);
//	memcpy(&buffer[48 + 3], uResetCause.buffer,4);

	memcpy(buffer, (void*)0x20007f00, 32);
	extern void SimuUart_9600(unsigned char );
	memcpy(&buffer[32], uResetCause.buffer, 4);

	for(uint8_t j = 0; j < 40; j++)
		SimuUart_9600(buffer[j]);

}
#endif


