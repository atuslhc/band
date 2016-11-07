
#ifndef __GLOBALDATA_H_
#define __GLOBALDATA_H_

#include "sleep.h"

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

extern 	uint8_t  SYSCLOCK;

/*
 * EM1 Running Device
 */
//#define USB_RUNNING       (1 << 0)
//#define DISPLAY_RUNNING   (1 << 1)
//#define BEEP_RUNNING      (1 << 2)
//#define Simu_BLE_RUNNING      (1 << 3)
#define DelayTimer_RUNNING   (1 << 4)

#define ForceEnterEM0  (1<<31)

extern bool I2C0_used;
//extern bool TouchEventACK;
extern bool isMemsSleeping,isMemsDeepSleeping;
extern uint32_t MemsSleepCount;

//extern struct tm curTime;
void LoadSystemSettings(void);
int SaveSystemSettings();

extern void SetSysEnergyModeFlag(uint32_t flag);
extern void ClearSysEnergyModeFlag(uint32_t flag);
extern SLEEP_EnergyMode_t GetEnergyMode(void);

extern void USB_ON_CLKTO48MHZ(void);
extern void USB_OFF_CLKTO14MHZ(void);

void SysSleepCallBack(SLEEP_EnergyMode_t eMode);

void SysWakeupCallBack(SLEEP_EnergyMode_t eMode);

//void QuickRunOutBattery(void);
//void SavingBattery(void);
void StartBatteryDrain();
void StopBatteryDrain();

//void MyIdleWork(void);

void UpdateBaseLine(void);
void ResetParaSettings(void);


#endif /*__GLOBALDATA_H_*/


/*********************************************************************************************************
** End Of File
*********************************************************************************************************/

