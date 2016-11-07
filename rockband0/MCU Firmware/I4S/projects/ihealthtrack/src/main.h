#ifndef _MAIN_H
#define _MAIN_H

#include <stdbool.h>

#include "debug.h"

#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "cortex-m_faults.h"

#include "sleep.h"

#include "vibrate.h"
#include "DelayUsingTimerInt.h"
#include "sys_sharing_source.h"
#include "GlobalData.h"
#include "ble.h"
#include "AFE44x0.h"
#include "ble.h"
#include "Signal_proc.h"

#include "sleep.h"

#include "mems.h"
#include "drv2605.h"
#include "arm_math.h"
#include "em_wdog.h"

#include "mems_tracking.h"

#if (MODEL_TYPE==1) //HEALTHCARE_TYPE
 #if (VENDOR_TYPE==1)
#define Dev_Name 2 //  1=i4  ,2=i4s, 3=i2, 4=i2s, 5=i3
#define APP_FW_VER_M  3
#define APP_FW_VER_S  37
 #elif (VENDOR_TYPE==2)
#define Dev_Name 2
#define APP_FW_VER_M  0
#define APP_FW_VER_S  10
 #else
 #error "Please specify VENDOR_TYPE!"
 #endif
#elif (MODEL_TYPE==2)
 #if (VENDOR_TYPE==1)
#define Dev_Name 1 //  1=i4  ,2=i4s, 3=i2, 4=i2s, 5=i3
#define APP_FW_VER_M  3
#define APP_FW_VER_S  27 //3.24 for debug //3.27
 #elif (VENDOR_TYPE==2)
#define Dev_Name 1
#define APP_FW_VER_M  0
#define APP_FW_VER_S  10
 #else
 #error "Please specify VENDOR_TYPE!"
 #endif
#else
#error "Please set MODEL_TYPE symbol!"
#endif

//const char  abc1 @(0x9800-2) =BOOT_FW_VER_M;
//const char  abc2 @(0x9800-1) =BOOT_FW_VER_S;

#define BOOT_FW_VER_M_Add  (0x9800-2)
#define BOOT_FW_VER_S_Add  (0x9800-1)


//#define PPG_WORK_MODE_SET	true	//move to CCdefine

#define PPG_WORK_SECONDS	121	//2 minutes

// adjust running mode for some debug test
#define DEBUG_MODE1

#define BATTERY_LIFE_OPTIMIZATION_Disable       true
//#define BATTERY_LIFE_OPTIMIZATION2      true    //I4H v3.25 disable SMS,calling, low battery notification. [BG025] Move to CCdefine

#define BURTC_IRQn_LEVEL  255
#define USB_INT_LEVEL   254
#define LEUART0_INT_LEVEL 191
#define GPIO_EVEN_INT_LEVEL  191
#define GPIO_ODD_INT_LEVEL  191
#define LESENSE_INT_LEVEL  191
#define TIMER2_IRQn_LEVEL  191
#define LETIMER0_IRQn_LEVEL  191
#define DMA_IRQn_LEVEL 		191
#define AFE_INT_LEVEL   191
#define  I2C0_IRQn_Level 190// 190 //2014 0429
#define  I2C1_IRQn_Level  190 //190  //2014 0429

#define RESET_MCU()  SCB->AIRCR = 0x05FA0004


//#define PPG2Dongle  1

#define timer_interval  20

#define WatchDogON 1

#ifdef NDEBUG

#define BLE_Sleep_Time 300
#define BLE_Connected_SleepTime  120

#else

#define BLE_Sleep_Time 10
#define BLE_Connected_SleepTime  120

#endif


#define SKIN_CAPVAL_Default  595	//mV
#define TOUCH_CAPVAL_Default 1200 //1400

//#define  PowerTesting   1 // turn off the capacity charging, can measure the power consumption.
#ifdef DEBUG
#define BAT_CHECK_INTERVAL 1  // Check the battery per 1 seconds, add USB will 1 per second.
#else
#define BAT_CHECK_INTERVAL 10  // Check the battery per 10 seconds, add USB will 1 per second.
#endif

//#ifdef DEBUG
#define LOCK_SCREEN_DELAY  8    // 10 >> 8
//#else
//#define LOCK_SCREEN_DELAY  5
//#endif
#define LONG_LOCK_SCREEN_DELAY  10  //用于显示心率以及几个事件时屏幕保持的时长。
#define LONGER_LOCK_SCREEN_DELAY  12 //用于显示心率以及几个事件时屏幕保持的时长。
#define GOAL_SHINE_DELAY          1//当目标达到后闪烁的时长

extern uint8_t NoTOUCHPressCount;

#define CHECK_INTERVAL_At_DeepSleep  300	// check every 5 minutes.
#define CHECK_INTERVAL_At_Sleep  120		//  check every 2 minutes.
#define CHECK_INTERVAL_At_ACT  5		//  check ever 10 seconds.??>
#define CHECK_INTERVAL_AT_BLE 3			//
#define FAST_ADV_TIME 60


#define UV_Warning_Level 6  
#define UVexp_Threshold         2       //[BG023-1] add
//=================================

#define LE_UART0_TX_PORT  (gpioPortF )
#define LE_UART0_TX_PIN    (2)

#define LE_UART0_RX_PORT  (gpioPortA )
#define LE_UART0_RX_PIN    (0)

// ================================
#define Diagnose_GPIOPORT  (gpioPortC)
#define Diagnose_PIN   (9)
#define SKIN_PIN   (8)

#define SKIN_LED_TOGGLE() GPIO_PinOutToggle(Diagnose_GPIOPORT,SKIN_PIN)
#define SKIN_H()      GPIO_PinOutSet(Diagnose_GPIOPORT,SKIN_PIN )
#define SKIN_L()      GPIO_PinOutClear(Diagnose_GPIOPORT,SKIN_PIN )


// ================================
#define KEY_GPIOPORT  (gpioPortA)
#define KEY_PIN   (2)

#define TEST_GPIOPORT  (gpioPortA)
#define TEST_PIN   (15)

#define TEST_H()      GPIO_PinOutSet(TEST_GPIOPORT,TEST_PIN )
#define TEST_L()      GPIO_PinOutClear(TEST_GPIOPORT,TEST_PIN )
#define TEST_TOGGLE() GPIO_PinOutToggle(TEST_GPIOPORT,TEST_PIN)

//
#define TEST_PORTA  (gpioPortA)
#define TEST_PIN4   (4)
#define TEST_PIN15   (15)

#define TEST_PORTB  (gpioPortB)
#define TEST_PIN2   (2)

#define TEST_PORT_SET(type, pin)		GPIO_PinOutSet(type,pin )
#define TEST_PORT_CLEAR(type, pin)		GPIO_PinOutClear(type,pin )
#define TEST_PORT_TOGGLE(type, pin) 	GPIO_PinOutToggle(type,pin)

#define TEST_PORT2_SET()		GPIO_PinOutSet(TEST_PORTB, TEST_PIN2)
#define TEST_PORT2_CLEAR()		GPIO_PinOutClear(TEST_PORTB, TEST_PIN2)
#define TEST_PORT2_TOGGLE() 	GPIO_PinOutToggle(TEST_PORTB, TEST_PIN2)

#define TEST_PORT4_SET()		GPIO_PinOutSet(TEST_PORTA, TEST_PIN4)
#define TEST_PORT4_CLEAR()		GPIO_PinOutClear(TEST_PORTA, TEST_PIN4)
#define TEST_PORT4_TOGGLE() 	GPIO_PinOutToggle(TEST_PORTA, TEST_PIN4)

#define TEST_PORT15_SET()		GPIO_PinOutSet(TEST_PORTA, TEST_PIN15)
#define TEST_PORT15_CLEAR()		GPIO_PinOutClear(TEST_PORTA, TEST_PIN15)
#define TEST_PORT15_TOGGLE() 	GPIO_PinOutToggle(TEST_PORTA, TEST_PIN15)


//=================================
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   (10)

#define LED_ON()      GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
#define LED_OFF()      GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )
#define LED_TOGGLE() GPIO_PinOutToggle(LED_GPIOPORT,LED_PIN)


// ====  TEMP I2C  ==============

#define TMP_I2C_cmuClock_I2C       cmuClock_I2C1
#define TMP_I2C_gpioPort           gpioPortE
#define TMP_I2C_SCL_PIN            1
#define TMP_I2C_SDA_PIN            0
#define TMP_I2C                    I2C1
#define TMP_I2C_LOC                I2C_ROUTE_LOCATION_LOC2


// UV

#define SI14x_I2C_cmuClock_I2C       cmuClock_I2C0
#define SI14x_I2C_gpioPort           gpioPortD
#define SI14x_I2C_SCL_PIN            15
#define SI14x_I2C_SDA_PIN            14
#define SI14x_I2C                    I2C0
#define SI14x_I2C_LOC                I2C_ROUTE_LOCATION_LOC3
#define SI14x_I2C_IRQn               I2C0_IRQn

#define UV_INT_PORT gpioPortD
#define UV_INT_PIN  6

//==================================================================

#define CHARGER_STA_GPIOPORT  (gpioPortF)
#define CHARGER_STA_PIN   (4)

// ==================================
#define MEMS_SPI           (USART1)
#define MEMS_SPI_CMUCLOCK  (cmuClock_USART1)
#define MEMS_SPI_GPIOPORT  (gpioPortD)
#define MEMS_SPI_MOSIPIN   (0)
#define MEMS_SPI_MISOPIN   (1)
#define MEMS_SPI_CLKPIN    (2)
#define MEMS_SPI_LOCATION  (USART_ROUTE_LOCATION_LOC1)

#define MEMS_CS_PORT    (gpioPortD)
#define MEMS_CS_PIN    (13)

#define MEMS_CS_H()    GPIO_PinOutSet(MEMS_CS_PORT,MEMS_CS_PIN )
#define MEMS_CS_L()    GPIO_PinOutClear(MEMS_CS_PORT,MEMS_CS_PIN )

#define MEMS_INT1_PORT gpioPortA
#define MEMS_INT1_PIN  3

#define MEMS_INT2_PORT gpioPortD
#define MEMS_INT2_PIN  11


//==================================


#define AFE_INT_LOC        GPIO_ODD_IRQn

#define AFE_SPI           (USART2)
#define AFE_SPI_CMUCLOCK  (cmuClock_USART2)
#define AFE_SPI_GPIOPORT  (gpioPortC)
#define AFE_SPI_MOSIPIN   (2)
#define AFE_SPI_MISOPIN   (3)
#define AFE_SPI_CLKPIN    (4)
#define AFE_SPI_LOCATION  (USART_ROUTE_LOCATION_LOC0)

#define AFE_CS_PORT    (gpioPortC)
#define AFE_CS_PIN    (5)

#define AFE_CS_H()    GPIO_PinOutSet(AFE_CS_PORT,AFE_CS_PIN )
#define AFE_CS_L()    GPIO_PinOutClear(AFE_CS_PORT,AFE_CS_PIN )


#define AFE_PDNZ_PORT    (gpioPortC)
#define AFE_PDNZ_PIN    (0)
#define AFE_POWER_CON_PORT    (gpioPortC)
#define AFE_POWER_CON_PIN    (0)


#define AFE_RESETZ_PORT    (gpioPortE)
#define AFE_RESETZ_PIN    (5)

#define AFE_RESETZ_H()    GPIO_PinOutSet(AFE_RESETZ_PORT,AFE_RESETZ_PIN )
#define AFE_RESETZ_L()    GPIO_PinOutClear(AFE_RESETZ_PORT,AFE_RESETZ_PIN )

#define AFE_ADC_DRDY_PORT    (gpioPortA)
#define AFE_ADC_DRDY_PIN    (5)



#define BOTTOM_POWER_ON()    GPIO_PinOutSet(AFE_POWER_CON_PORT,AFE_POWER_CON_PIN)
#define BOTTOM_POWER_OFF()    GPIO_PinOutClear(AFE_POWER_CON_PORT,AFE_POWER_CON_PIN)

// ==================================

#define TIMER_DEALY           (TIMER2)
#define cmuClock_TIMER_DEALY  (cmuClock_TIMER2)

#define TIMER_DEALY_IF     (TIMER_IF_OF)
#define TIMER_DEALY_IRQn       TIMER2_IRQn

//=========OLED=====================

#define OLED_I2C_cmuClock_I2C       cmuClock_I2C0
#define OLED_I2C_gpioPort           gpioPortD
#define OLED_I2C_SCL_PIN            15
#define OLED_I2C_SDA_PIN            14
#define OLED_I2C                    I2C0
#define OLED_I2C_LOC                I2C_ROUTE_LOCATION_LOC3

#define OLED_I2C_IRQn    I2C0_IRQn

#define OLED_RST_PORT   (gpioPortD)
#define OLED_RST_PIN    (12)

#define OLED_RST_L()    GPIO_PinOutClear(OLED_RST_PORT,OLED_RST_PIN)
#define OLED_RST_H()   GPIO_PinOutSet(OLED_RST_PORT,OLED_RST_PIN)

//==============================================

#define VIBRATE_CON_PORT   (gpioPortF)
#define VIBRATE_CON_PIN    (5)

#define DisableVIBRATE()  GPIO_PinOutClear(VIBRATE_CON_PORT,VIBRATE_CON_PIN)
#define EnableVIBRATE()   GPIO_PinOutSet(VIBRATE_CON_PORT,VIBRATE_CON_PIN)


//===============================
#define BLE_RX_PORT  (gpioPortA)  // 
#define BLE_RX_PIN   (0)

#define BLE_TX_PORT  (gpioPortF)  // 
#define BLE_TX_PIN   (2)


// UART RX is also as an interrupt input pin
#define BLE_INT_PORT  (gpioPortA)  // 
#define BLE_INT_PIN   (0)

//2014.04.18
#define BLE_INT2_GPIOPORT  (gpioPortB)
#define BLE_INT2_PIN   (0)


#define BLE_RST_PORT  (gpioPortF)  // 
#define BLE_RST_PIN   (7)

#define BLE_32K_PORT  (gpioPortA)
#define BLE_32K_PIN   (1)


#define BLE_RST_L()   GPIO_PinOutClear(BLE_RST_PORT,BLE_RST_PIN)
#define BLE_RST_H()   GPIO_PinOutSet(BLE_RST_PORT,BLE_RST_PIN)


//==========================


#define FLASH_SPI           (USART0)
#define FLASH_SPI_CMUCLOCK  (cmuClock_USART0)
#define FLASH_SPI_GPIOPORT  (gpioPortE)
#define FLASH_SPI_MOSIPIN   (10)
#define FLASH_SPI_MISOPIN   (11)
#define FLASH_SPI_CLKPIN    (12)
#define FLASH_SPI_LOCATION  (0)

#define FLASH_PW_PORT  (gpioPortD)
#define FLASH_PW_PIN    (5)
#define FLASH_PW_PIN2    (3)  //  20141225



#define FLASH_SPI_CS_GPIOPORT  (gpioPortE)
#define FLASH_SPI_CSPIN     (15)


#define FLASH_PW_ON()     GPIO_PinOutSet(FLASH_PW_PORT,FLASH_PW_PIN)
#define FLASH_PW_OFF()    GPIO_PinOutClear(FLASH_PW_PORT,FLASH_PW_PIN)


#define  FLASH_CS_L()     GPIO_PinOutClear(FLASH_SPI_CS_GPIOPORT,FLASH_SPI_CSPIN)
#define  FLASH_CS_H()     GPIO_PinOutSet(FLASH_SPI_CS_GPIOPORT,FLASH_SPI_CSPIN)

//==========================
void SysCtlDelay(unsigned long ulCount);


//==================================================================
// 用于长时间定时的定时器标志（秒级或更高，使用RTC做为驱动，精度较低）

#define MAX_LONG_TIMER_FLAGS	4			// 定时器个数，上限16（对应uint16_t的16位）

// 注意：调整此处定义时，请确保 MAX_LONG_TIMER_FLAGS 能包含所有定义
//       同时，请调整定义使 MAX_LONG_TIMER_FLAGS 尽可能小，以节省空间
//       优先级高的定义请放前面
typedef enum
{
	LONG_TIMER_FLAG_UPLOAD_DATA,		// 用于 数据上传、固件更新 的超时检查
	LONG_TIMER_FLAG_VIBRATE,			// 用于 马达震动控制
	LONG_TIMER_FLAG_BLE_HEART_BEAT,		// 用于 ble心跳检查，1分钟一次
	LONG_TIMER_MANUAL_RESET_MCU,		//手动按键复位MCU
} LONG_TIMER_FLAG;

//
void EnableLongTimer(uint16_t flag, bool repeat, uint16_t interval, TIMER_HANDLER callback, void* userData);
void DisableLongTimer(uint16_t flag);
void ResetLongTimer(uint16_t flag);
//void SendErrorInformation();

//==================================================================
// 获得gmt timestamp
// 时钟运行、显示是本地时间，需要换算为gmt ts
time_t getGMTTimestamp();


extern uint32_t resetCause;
extern FOMAT uResetCause;
#endif

