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
#if (BOARD_TYPE==0)
    #define Dev_Name 2 //  1=i4  ,2=i4s, 3=i2, 4=i2s, 5=i3
    #define APP_FW_VER_M  3
    #define APP_FW_VER_S  37
#elif (BOARD_TYPE==1)
    #define Dev_Name 2 //  1=i4  ,2=i4s, 3=i2, 4=i2s, 5=i3
    #define APP_FW_VER_M  0
    #define APP_FW_VER_S  01
#endif
 #elif (VENDOR_TYPE==2)
  #if (BOARD_TYPE==0 || BOARD_TYPE==1)
    #define Dev_Name 2
    #define APP_FW_VER_M  0
    #define APP_FW_VER_S  11
  #elif (BOARD_TYPE==2)
    #define Dev_Name 2
    #define APP_FW_VER_M  0
    #define APP_FW_VER_S  21
  #endif
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

#define BOOT_FW_VER_M_Add  (0x9800-2)   //The bootloader upgrade will write version in the last 2 bytes of 38k(0x9800).
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
#define  I2C0_IRQn_Level 190 //190 //2014 0429
#define  I2C1_IRQn_Level  190 //190  //2014 0429

#define RESET_MCU()  SCB->AIRCR = 0x05FA0004


//#define PPG2Dongle  1

#define timer_interval  20      //timer interval, unit: ms

//#define WatchDogON 1

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
#define LONG_LOCK_SCREEN_DELAY  10  //The time to keep/lock display the heartrate and some events.
#define LONGER_LOCK_SCREEN_DELAY  12 //The longer keep/lock display the heartrate and some events.
#define GOAL_SHINE_DELAY          1//The goal reach to blink screen.

extern uint8_t NoTOUCHPressCount;

/* events scan/check battery charge interval time */
#define CHECK_INTERVAL_At_DeepSleep 300 // check every 5 minutes at deep sleep mode.
#define CHECK_INTERVAL_At_Sleep     120	// check every 2 minutes at sleep mode.
#define CHECK_INTERVAL_At_ACT       5	// check every 10 seconds at active mode. 5 seconds
#define CHECK_INTERVAL_AT_BLE       3	// check BLE stuff every 3 seconds and battery at BLE connected state.
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

#if (BOARD_TYPE==2)
// ================================
//BUTTON
#define KEY_GPIOPORT  (gpioPortA)
#define KEY1_PIN   (2)      //SW1, GPIO_EVEN_IRQHandler()
#define KEY2_PIN   (5)      //SW2, DONOT Interrupt while reset power. GPIO_ODD_IRQHandler()

#define GetKEY1()		GPIO_PinInGet(KEY_GPIOPORT,KEY1_PIN)
#define GetKEY2()		GPIO_PinInGet(KEY_GPIOPORT,KEY2_PIN)
#endif

//=================================
//LED
#if (BOARD_TYPE==0)
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   (10)              //i4:PC10 high active
#elif (BOARD_TYPE==1)
#define LED_GPIOPORT  (gpioPortB)
#define LED_PIN   (5)               //I4B:PB5 high active
#elif (BOARD_TYPE==2)
#define LED_GPIOPORT  (gpioPortC)
#define LEDC_PIN   (0)
#define LEDR_PIN   (1)              //rb1:PC1 low active
#define LEDG_PIN   (3)
#define LEDB_PIN   (2)
#define LEDAll_PIN   (7)
#define LED_PIN    LEDG_PIN     //redirect mapping, backward compatible.
#endif
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
#define LED_ON()        GPIO_PinOutSet(LED_GPIOPORT, LED_PIN) //high active
#define LED_OFF()       GPIO_PinOutClear(LED_GPIOPORT, LED_PIN)
#define LED_TOGGLE() GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#elif (BOARD_TYPE==2)
// pin PC0 is 3V feedback, input pin.
//#define LEDC_High()      GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
//#define LEDC_Low()      GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )
//#define LEDC_TOGGLE() GPIO_PinOutToggle(LED_GPIOPORT,LED_PIN)
#define GetLEDC() GPIO_PinInGet(LED_GPIOPORT,LEDC_PIN)

#define LEDR_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDR_PIN)  //low active
#define LEDR_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDR_PIN)
#define LEDR_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDR_PIN)
#define LEDR_STAT()     GPIO_PinOutGet(LED_GPIOPORT, LEDR_PIN)

#define LEDG_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDG_PIN) 
#define LEDG_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDG_PIN)
#define LEDG_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDG_PIN)
#define LEDG_STAT()     GPIO_PinOutGet(LED_GPIOPORT, LEDG_PIN)

#define LEDB_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDB_PIN)
#define LEDB_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDB_PIN)
#define LEDB_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDB_PIN)
#define LEDB_STAT()     GPIO_PinOutGet(LED_GPIOPORT, LEDB_PIN)

#define LED_ON()        GPIO_PinOutClear(LED_GPIOPORT, LED_PIN)
#define LED_OFF()       GPIO_PinOutSet(LED_GPIOPORT, LED_PIN)
#define LED_TOGGLE()    GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#define LED_STAT()      GPIO_PinOutGet(LED_GPIOPORT, LED_PIN)

#endif


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

#if (BOARD_TYPE==0 || BOARD_TYPE==2)
#define UV_INT_PORT gpioPortD
#define UV_INT_PIN  6           //ATS_INT:PD6
#elif (BOARD_TYPE==1)
#define UV_INT_PORT gpioPortB
#define UV_INT_PIN  4           //ATS_INT:PB4
#endif

//==================================================================
#if (CHARGER_SUPPORT==1)
#if (BOARD_TYPE==0) 
#define CHARGER_STA_GPIOPORT  (gpioPortF)
#define CHARGER_STA_PIN   (4)       //CHARGER_STA:PF4
#elif (BOARD_TYPE==1)
#define CHARGER_STA_GPIOPORT  (gpioPortF)
#define CHARGER_STA_PIN   (4)       //CHARGER_STA:PF4
//#define CHARGER_STA_GPIOPORT  (gpioPortA)
//#define CHARGER_STA_PIN   (3)       //PA3, also connect PF4
#endif
#endif
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

#if (BOARD_TYPE==0 || BOARD_TYPE==2)
#define MEMS_INT1_PORT gpioPortA
#define MEMS_INT1_PIN  3
#define MEMS_INT2_PORT gpioPortD
#define MEMS_INT2_PIN  11
#elif (BOARD_TYPE==1)
#define MEMS_INT1_PORT gpioPortA
#define MEMS_INT1_PIN  13
#define MEMS_INT2_PORT gpioPortB
#define MEMS_INT2_PIN  11
#endif



//==================================

#if (AFE44x0_SUPPORT==1)
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
#endif

// ==================================

#define TIMER_DEALY           (TIMER2)
#define cmuClock_TIMER_DEALY  (cmuClock_TIMER2)

#define TIMER_DEALY_IF     (TIMER_IF_OF)
#define TIMER_DEALY_IRQn       TIMER2_IRQn

//=========OLED=====================
#if (OLED_SUPPORT==1)
#define OLED_I2C_cmuClock_I2C       cmuClock_I2C0
#define OLED_I2C_gpioPort           gpioPortD
#define OLED_I2C_SCL_PIN            15
#define OLED_I2C_SDA_PIN            14
#define OLED_I2C                    I2C0
#define OLED_I2C_LOC                I2C_ROUTE_LOCATION_LOC3

#define OLED_I2C_IRQn    I2C0_IRQn

#if (BOARD_TYPE==0)
#define OLED_RST_PORT   (gpioPortD)
#define OLED_RST_PIN    (12)
#elif (BOARD_TYPE==1)
#define OLED_RST_PORT   (gpioPortD)
#define OLED_RST_PIN    (12)        //PD12
//#define OLED_RST_PORT   (gpioPortB)
//#define OLED_RST_PIN    (10)      //also in PB10
#endif

#define OLED_RST_L()    GPIO_PinOutClear(OLED_RST_PORT,OLED_RST_PIN)
#define OLED_RST_H()   GPIO_PinOutSet(OLED_RST_PORT,OLED_RST_PIN)

#endif  //OLED_SUPPORT

//==============================================
#if (VIBRATION_SUPPORT==1)
#define VIBRATE_CON_PORT   (gpioPortF)
#define VIBRATE_CON_PIN    (5)

#define DisableVIBRATE()  GPIO_PinOutClear(VIBRATE_CON_PORT,VIBRATE_CON_PIN)
#define EnableVIBRATE()   GPIO_PinOutSet(VIBRATE_CON_PORT,VIBRATE_CON_PIN)
#endif //VIBRATION_SUPPORT

//===============================
#define BLE_RX_PORT  (gpioPortA)  // 
#define BLE_RX_PIN   (0)

#define BLE_TX_PORT  (gpioPortF)  // 
#define BLE_TX_PIN   (2)


// UART RX is also as an interrupt input pin
#define BLE_INT_PORT  (gpioPortA)  // 
#define BLE_INT_PIN   (0)           //BLE_INT

//2014.04.18
#if (BOARD_TYPE==0 || BOARD_TYPE==2)
#define BLE_INT2_GPIOPORT  (gpioPortB)
#define BLE_INT2_PIN   (0)      //PB0
#elif (BOARD_TYPE==1)
#define BLE_INT2_GPIOPORT  (gpioPortD)
#define BLE_INT2_PIN   (6)      //PD6
#endif

#if (BOARD_TYPE==0 || BOARD_TYPE==2)
#define BLE_RST_PORT  (gpioPortF)  // 
#define BLE_RST_PIN   (7)           //PF7
#elif (BOARD_TYPE==1)
#define BLE_EN_PORT  (gpioPortC)  // 
#define BLE_EN_PIN    (15)           //PC15: RF_EN
#define BLE_RST_PORT  (gpioPortC)  // 
#define BLE_RST_PIN   (15)           //PC15. reset is power-on self reset and ccdebug, use EN as reset.
#endif

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

#if (BOARD_TYPE==0 || BOARD_TYPE==2)
#define FLASH_PW_PORT  (gpioPortD)
#define FLASH_PW_PIN    (5)    //PD5
#define FLASH_PW_PORT2  (gpioPortD)
#define FLASH_PW_PIN2    (3)  //  20141225
#elif (BOARD_TYPE==1)
#define FLASH_PW_PORT  (gpioPortE)
#define FLASH_PW_PIN    (13)    //PE13
#define FLASH_PW_PORT2  (gpioPortA)
#define FLASH_PW_PIN2    (15)  //PA15
#endif



#define FLASH_SPI_CS_GPIOPORT  (gpioPortE)
#define FLASH_SPI_CSPIN     (15)


#define FLASH_PW_ON()     GPIO_PinOutSet(FLASH_PW_PORT,FLASH_PW_PIN); \
                          GPIO_PinOutSet(FLASH_PW_PORT2,FLASH_PW_PIN2)   //add turn on both to avoid current lost.
#define FLASH_PW_OFF()    GPIO_PinOutClear(FLASH_PW_PORT,FLASH_PW_PIN); \
                          GPIO_PinOutClear(FLASH_PW_PORT2,FLASH_PW_PIN2) //add turn off both to avoid current lost.


#define  FLASH_CS_L()     GPIO_PinOutClear(FLASH_SPI_CS_GPIOPORT,FLASH_SPI_CSPIN)
#define  FLASH_CS_H()     GPIO_PinOutSet(FLASH_SPI_CS_GPIOPORT,FLASH_SPI_CSPIN)

#if (BOARD_TYPE==1)
#define   BUTTON_PORT_ONE   gpioPortC    //set as touch button
#define   BUTTON_PIN_ONE    9

#define   BUTTON_PORT_TWO   gpioPortE   //set as alert button
#define   BUTTON_PIN_TWO    14

//=================temperature======
#define  TEMPERATURE_POWER_PORT  gpioPortB
#define  TEMPERATURE_Pin      1
#define  ENABLE_TEMPERATURE_POWER()  GPIO_PinOutSet(TEMPERATURE_POWER_PORT, TEMPERATURE_Pin)
#define  DISABLE_TEMPERATURE_POWER() GPIO_PinOutClear(TEMPERATURE_POWER_PORT, TEMPERATURE_Pin)

#define  TEMPERATURE_SAMPLE_PORT gpioPortD
#define  TEMPERATURE_SAMPLE_Pin  5

#endif

#if (BOARD_TYPE==2)
#define DAUIO_1_PIN         (4)  //PD4
#define DAUIO_2_PIN         (4)  //PC4
#define DAUIO_3_PIN         (5)  //PC5
#define DAUIO_4_PIN         (6)  //PC6
#define DAUIO_5_PIN         (4)  //PA4
#define DAUIO_6_PIN         (6)  //PA6
#define DAUIO_7_PIN         (11) //PB11
#define DAUIO_8_PIN         (12) //PB12
#define GetDAUIO_1()		GPIO_PinInGet(gpioPortD,DAUIO_1_PIN)
#define GetDAUIO_2()		GPIO_PinInGet(gpioPortC,DAUIO_2_PIN)
#define GetDAUIO_3()		GPIO_PinInGet(gpioPortC,DAUIO_3_PIN)
#define GetDAUIO_4()		GPIO_PinInGet(gpioPortC,DAUIO_4_PIN)
#define GetDAUIO_5()		GPIO_PinInGet(gpioPortA,DAUIO_5_PIN)
#define GetDAUIO_6()		GPIO_PinInGet(gpioPortA,DAUIO_6_PIN)
#define GetDAUIO_7()		GPIO_PinInGet(gpioPortB,DAUIO_7_PIN)
#define GetDAUIO_8()		GPIO_PinInGet(gpioPortB,DAUIO_8_PIN)
#endif

//==========================
void SysCtlDelay(unsigned long ulCount);


//==================================================================
// Use as the long timer flag (as seconds or longer, use the RTC driver, less precise£©
#define MAX_LONG_TIMER_FLAGS	4			// The timer number, maximum 16 (mapping uint16_t 16 bits)

// Notice: Please make sure MAX_LONG_TIMER_FLAGS is included while change the definition.
//         and keep the MAX_LONG_TIMER_FLAGS as less as possible to saving the space,
//         and priority higher first then lower priority.
typedef enum
{
	LONG_TIMER_FLAG_UPLOAD_DATA,		// use data upload and firmware upgrade time(overtime) check.
	LONG_TIMER_FLAG_VIBRATE,			// use vibration motor control.
	LONG_TIMER_FLAG_BLE_HEART_BEAT,		// use BLE heartbeat check (1 minute=60 seconds)
	LONG_TIMER_MANUAL_RESET_MCU,		// use manual reset check.
} LONG_TIMER_FLAG;

//
void EnableLongTimer(uint16_t flag, bool repeat, uint16_t interval, TIMER_HANDLER callback, void* userData);
void DisableLongTimer(uint16_t flag);
void ResetLongTimer(uint16_t flag);
//void SendErrorInformation();

//==================================================================
// get the GMT timestamp
// RTC use local time, convert to GMT ts.
time_t getGMTTimestamp();


extern uint32_t resetCause;
extern FOMAT uResetCause;
#endif

