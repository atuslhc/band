/****************************************Copyright (c)**************************************************
**                               Honestar Technology Co.,LTD.
**
**                                 http://www.honestar.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:           GlobalData.c
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
#include "GlobalData.h"
#include "em_letimer.h"
#include "config.h"

/*********************************************************************************************************
** Global variable
*********************************************************************************************************/
#if 0
//2015年10月5日08:36:55调整代码去掉的
struct _TOUCH_KEY touchkey;
bool GulTouchKeyMode = false;

bool LowBatFlag=false;//低电压信号

bool GblToggle = 0;
//struct tm curTime;
#endif
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
#if 0
struct _SYSTEMPARAM GstSystemParam;
struct _SYSTEMPARAM *SystemParam = &GstSystemParam;//(struct _SYSTEMPARAM*)(2 * 4 + 0x100 + BURTC_BASE);//(&(BURTC->RET[2].REG));

//EFM32_ALIGN(4)
//struct _SYSTEMPARAM *SystemParam __attribute__ ((aligned(4)))= (struct _SYSTEMPARAM*)(2 * 4 + 0x100 + BURTC_BASE);
/*------------------------------------------------------------------------------
 * Device Function:
 * pedometer - step counter, distance, calorie, step time,
 * user profile - gender/height/weight
 * heartrate - pulse(blood pressure),
 * temperatrue - ambient/skin
 *
 &------------------------------------------------------------------------------*/
struct _DEVICE_FUN GstDevice;
struct _DEVICE_FUN *Device = &GstDevice;//(struct _DEVICE_FUN*)(64 * 4 + 0x100 + BURTC_BASE);//(&BURTC->RET[64].REG);;
union _CHIP DevChip;

extern void SysCtlDelay(unsigned long ulCount);

void SystemDataInit(void)
{
  if (SystemParam->app.init == 0xAA55A5A5) {
    //init ok
    return;
  }
  SystemParam->app.init = 0xAA55A5A5;
  
  //SystemParam->app.statusflag =  STATUS_BLE|STATUS_USB_CONNECT|STATUS_ALARM|STATUS_MEMORY
  //                                |STATUS_BAR_UPDATA|STATUS_BATTERY_UPDATA;
  SystemParam->app.statusflag =STATUS_BAR_UPDATA|STATUS_ALARM
                      |STATUS_BATTERY_UPDATA|STATUS_USB_CONNECT;
  
  /* enable invert color display mode */
  //SystemParam->app.statusflag |= DISPLAY_MODE_INVERT | DISPLAY_MODE_TIME;
  
  /* Disable invert color display mode */
  SystemParam->app.statusflag &= ~DISPLAY_MODE_INVERT;
  
  //initial alarm
  SystemParam->app.alarm.hour = 20;
  SystemParam->app.alarm.min = 0;
  SystemParam->app.alarm.sec = 0;
  
  /* Memory Format/Facotry setting */
  SystemParam->app.statusflag &= ~(FORMAT_MEMORY | FACOTRY_SETTING);
  
  /* Tone/Motor/Backlight */
  SystemParam->app.statusflag |= (STATUS_TONE | STATUS_MOTOR | STATUS_BACKLIGHT);
  
  /* sensor:HRM GSR ACCEL TEMP */
  SystemParam->app.statusflag |= (STATUS_SENSOR_HRM | STATUS_SENSOR_GSR | \
                                 STATUS_SENSOR_ACCEL | STATUS_SENSOR_TEMP);
  
  Device->userprofile.Gender  = GENDER_MALE;//gender
  Device->userprofile.Height  = 170;//cm
  Device->userprofile.Weight  = 60;//kg

  for (int i = 0; i < 16; i++) {
	DevChip.EFM32DeviceInfo[i] = *((unsigned char *)(EFM32_ADDRESS + i));
		  }
	  
}

/*
 * When EnergyMode Set Device Running in EM1,otherwise EM2
 */
void SetSysEnergyModeFlag(uint32_t flag)
{
  SystemParam->lowlevel.energymodeper |= flag;
}

void ClearSysEnergyModeFlag(uint32_t flag)
{
  SystemParam->lowlevel.energymodeper &= ~(flag);
}

uint32_t GetSysEnergyModeFlag(void)
{
  return (SystemParam->lowlevel.energymodeper);
}

void UpdateDisplayRange(uint8_t startline, uint8_t endline)
{
  SystemParam->display.StartLine = (SystemParam->display.StartLine < startline)?SystemParam->display.StartLine:startline;
  SystemParam->display.EndLine   = (SystemParam->display.EndLine > endline)?SystemParam->display.EndLine:endline;;
}

/*********************************************************************************************************** 
* Function Name: void UpdateStatue(uint32_t flag)()
* Input parameters: uint32_t flag -- bluetooth usb alarm batteray
* Output parameters: void
* Descriptions: -- UpdateSystemStatus
* Time: 2013-7-7 
* Creat by: Alan
************************************************************************************************************/ 
void UpdateSystemStatus(uint32_t flag, uint32_t ctrl)
{ 
  if (ctrl == true) {
    SystemParam->app.statusflag |= flag;
  } else {
    SystemParam->app.statusflag &= ~flag;
  }
  SystemParam->app.statusflag |= STATUS_BAR_UPDATA;
}

/*********************************************************************************************************** 
* Function Name: GetSystemStatus
* Input parameters: void
* Output parameters: SystemParam.app.statusflag
* Descriptions: -- Get the sysytem status flag param
* Time: 2013-7-21
* Creat by: Alan
************************************************************************************************************/ 
uint32_t GetSystemStatus(void)
{
  return (SystemParam->app.statusflag);
}


#endif


#if 1   //Atus: duplicate in main.c, but boot usbhhidkbd and Bootld proj refer this, remark main.c.
void SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif

/*********************************************************************************************************
** End Of File
*********************************************************************************************************/


