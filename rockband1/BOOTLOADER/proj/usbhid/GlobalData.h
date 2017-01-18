/****************************************Copyright (c)**************************************************
**                               Honestar Technology Co.,LTD.
**
**                                 http://www.honestar.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:           GlobalData.h
** Last modified Date:  2013.06.26
** Last Version:        V1.0
** Description:         
**
**------------------------------------------------------------------------------------------------------
** Created By:          Alan Lan
** Created date:        2013/06/26
** Version:             V1.0
** Descriptions:
**
**------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
********************************************************************************************************/
#ifndef __GLOBALDATA_H_
#define __GLOBALDATA_H_

#include <time.h>
#include "efm32.h"
#include "em_dma.h"
#include "em_letimer.h"
#include "em_emu.h"
#include "em_common.h"

//#include "memlcd.h"
//#include "framebufferctrl.h"
//#include "dma.h"
//#include "GUI.h"

//#include "cmsis_os.h"


#ifndef BOARD_TYPE
#error "Please specify BOARD_TYPE value in Options/Preprocessor !"
#endif

//move LED definition from loader.c to here, because both proj:Bootld,usbhhidkbd use GlobalData.h.
//LED definition
#if (BOARD_TYPE==0)
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   	  (10)
#elif (BOARD_TYPE==1)
#define LED_GPIOPORT  (gpioPortB)
#define LED_PIN   	  (5)
#elif (BOARD_TYPE==2)
#define LED_GPIOPORT  (gpioPortC)
#define LEDC_PIN        (0)
#define LEDR_PIN        (1)
#define LEDB_PIN        (2)
#define LEDG_PIN        (3)
#define LED_PIN         LEDR_PIN    //redirect mapping, backward compatible.
#endif
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
#define LED_ON()        GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
#define LED_OFF()       GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )
#define LED_TOGGLE()    GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#elif (BOARD_TYPE==2)
// pin PC0 is 3V feedback, input pin.
//#define LEDC_High()      GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
//#define LEDC_Low()      GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )
//#define LEDC_TOGGLE() GPIO_PinOutToggle(LED_GPIOPORT,LED_PIN)
#define GetLEDC() GPIO_PinInGet(LED_GPIOPORT,LEDC_PIN)

#define LEDR_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDR_PIN)  //low active
#define LEDR_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDR_PIN)
#define LEDR_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDR_PIN)

#define LEDG_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDG_PIN) 
#define LEDG_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDG_PIN)
#define LEDG_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDG_PIN)

#define LEDB_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDB_PIN)
#define LEDB_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDB_PIN)
#define LEDB_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDB_PIN)

#define LED_ON()        GPIO_PinOutClear(LED_GPIOPORT, LED_PIN)
#define LED_OFF()       GPIO_PinOutSet(LED_GPIOPORT, LED_PIN)
#define LED_TOGGLE()    GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
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


#if 0
/*
 * EM1 Running Device
 */
#define USB_RUNNING       (1 << 0)
#define DISPLAY_RUNNING   (1 << 1)
#define BEEP_RUNNING      (1 << 2)
#define Simu_BLE_RUNNING      (1 << 3)
#define DelayTimer_RUNNING   (1 << 4)

#define ForceEnterEM0  (1<<31)



/*
 * EM1 Running Device
 */
#define USB_RUNNING       (1 << 0)
#define DISPLAY_RUNNING   (1 << 1)
#define BEEP_RUNNING      (1 << 2)
#define Simu_BLE_RUNNING      (1 << 3)
#define DelayTimer_RUNNING   (1 << 4)
#define AFE_RUNNING   (1 << 5)


/*
* Status Bar:BlueTooth USB Connect Alarm Battery
*/
#define STATUS_BAR_UPDATA        (1 << 0)
#define STATUS_BLE               (1 << 1)//for bluetooth enable/disable?
#define STATUS_USB_CONNECT       (1 << 2)//for usb connect?
#define STATUS_ALARM             (1 << 3)//for alarm enable/disable?
#define STATUS_BATTERY_UPDATA    (1 << 4)
#define STATUS_MEMORY            (1 << 5)//for memory full?


/*
* display mode :invert color
*/
#define DISPLAY_MODE_INVERT      (1 << 8)

/*
* status:bit9-bit10 for display mode
* /time/pedometer/Hearrate/test
*/
#define DISPLAY_MODE_TIME         (0 << 9)
#define DISPLAY_MODE_PED          (1 << 9)
#define DISPLAY_MODE_HR           (2 << 9)
#define DISPLAY_MODE_TEST         (3 << 9)

#define DISPLAY_MODE_MASK         (3 << 9)
#define DISPLAY_MODE_MASK_BIT     (9)

/*
*status:bit 11 for memory format bit 12 facotry setting
*/
#define FORMAT_MEMORY             (1 << 11)
#define FACOTRY_SETTING           (1 << 12)

/*
*status:bit 13 for beep tone,bit 14 for motor,bit 15 for backlight
*/
#define STATUS_TONE               (1 << 13)
#define STATUS_MOTOR              (1 << 14)
#define STATUS_BACKLIGHT          (1 << 15)

/*
*status:bit 16 for HRM Sensor,bit 17 for GSR Sensor,
*       bit18 for Accel Sensor,bit 19 for Temp Sensor
*/
#define STATUS_SENSOR_HRM         (1 << 16)
#define STATUS_SENSOR_GSR         (1 << 17)
#define STATUS_SENSOR_ACCEL       (1 << 18)
#define STATUS_SENSOR_TEMP        (1 << 19)




/*********************************************************************************************************
** Global variable
*********************************************************************************************************/

/*------------------------------------------------------------------------------
 * System Ctrl:
 * - system level
 * system clk, lcd spi clk, 
 * EnergyMode(EM1 Running) Flag - LCD SPI Running, BeepRunning, USB Running,
 * Current Window handler
 *---------------------------
 * - app level
 * calendar - time_t
 * lockscreen - lock/unlock
 * blue tooth - enable/disable, pair or not?
 * alarm -(number, mode, set time)
 * battery level - percent
 * memory status - full/not
 * sensor - enable/disable(HR, G-Sensor, Temperatrue)
 * display mode - invert/not
 * format memory
 * factory setting
 * 
 *------------------------------------------------------------------------------*/
EFM32_PACK_START(4)
struct _SYSTEMPARAM {
  struct _SYSTEM_CTRL {
     uint32_t systemclk;
     uint32_t lcdspiclk;
     uint32_t energymodeper;
     //uint32_t beeptimer;
     //current window handler
  }lowlevel;
  struct _DISPLAY {
    uint32_t StartLine;
    uint32_t EndLine;
  }display;
  struct _APP_LEVEL {
     uint32_t statusflag;
     //lockscreen;//bluetooth;//memorystatus;//displaymode
     //format memory//factory setting
     uint32_t sensorstatus;//HR,Accelerator,Temperatrue
     //struct ALARM(number, mode, time)
     struct tm calendar;
     struct _ALARM{
       uint32_t hour;
       uint32_t min;
       uint32_t sec;
     }alarm;
     uint32_t batterylevel;
     uint32_t init;
  }app;
};
EFM32_PACK_END()

#define GENDER_MALE     0
#define GENDER_FEMALE   1

/*------------------------------------------------------------------------------
 * Device Function:
 * pedometer - step counter, distance, calorie, step time,
 * user profile - gender/height/weight
 * heartrate - pulse(blood pressure),
 * temperatrue - ambient/skin
 *
 &------------------------------------------------------------------------------*/
EFM32_PACK_START(4)
struct _DEVICE_FUN
{
  struct _PEDOMETER {
     uint32_t StepCounter;//step
     uint32_t Distance;//km
     uint32_t Calorie;//cal
     //uint32_t StepTime;//second cnt
  }pedometer;
  struct _USERPROFILE {
    uint32_t Gender;//male/female
    uint32_t Height;//cm float?
    uint32_t Weight;//kg float?
  }userprofile;
  struct _HEARTRATE {
    uint32_t pulse;
  }heartrate;
  struct _TEMPERATRUE {
    float AmbientTemp;
    float SkinTemp;
  }temperatrue;
};
EFM32_PACK_END()

#define _LONG_KEY   (1 << 0)
#define _SHORT_KEY  (1 << 1)
#define _DOUBLE_KEY (1 << 2)

struct _TOUCH_KEY{
  uint8_t keycnt;
  uint8_t flag;
  uint8_t curkeyval;
  uint8_t lastkeyval;
};





extern bool LowBatFlag;

extern bool GblToggle;
extern bool GulTouchKeyMode;
extern struct _TOUCH_KEY touchkey;

//extern struct tm curTime;
extern void SystemDataInit(void);

extern struct _SYSTEMPARAM *SystemParam;
extern struct _DEVICE_FUN *Device;

extern void SetSysEnergyModeFlag(uint32_t flag);
extern void ClearSysEnergyModeFlag(uint32_t flag);
extern uint32_t GetSysEnergyModeFlag(void);

extern void UpdateDisplayRange(uint8_t startline, uint8_t endline);

extern void UpdateSystemStatus(uint32_t flag, uint32_t ctrl);
extern uint32_t GetSystemStatus(void);
#endif
extern void SysCtlDelay(unsigned long ulCount); //Atus: duplicated in main.c, remark main.c.

#endif /*__GLOBALDATA_H_*/


/*********************************************************************************************************
** End Of File
*********************************************************************************************************/

