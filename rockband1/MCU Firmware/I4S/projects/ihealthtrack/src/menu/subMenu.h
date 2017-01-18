#ifndef __SUB_MENU_H
#define __SUB_MENU_H

void clearScreen(bool fillWithWhite);

void CallBackSubMenuActivate(void);

void CallBackSubMenuTime(void);
void CallBackSubMenuHeartRate(void);
void CallBackSubMenuCalories(void);
void CallBackSubMenuStep(void);
void CallBackSubMenuDistance(void);
void CallBackSubMenuUltraViolet(void);
void CallBackSubMenuBodyTemper(void);
void CallBackSubMenuAmbientTemper(void);
void CallBackSubMenuAlarm(void);

void CallBackSubMenuNotifications(void);
void CallBackSubMenuIncomingCall(void);
void CallBackSubMenuSMS(void);
void CallBackSubMenuBattery(void);
void CallBackSubMenuBleMAC(void);
void CallBackSubMenuDownload(void);
void ShowVersion(void);
//void CallBackSubMenuIntenseUV(void);

void CallBackSubMenuTesting(void);
void CallBackSubMenuBLE(void);
void CallBackSubMenuDataGather(void);

#if FALL_DETECT_SUPPORT
void CallBackSubMenuFallAlert(void);
#endif

#if defined(DEBUG) || defined(DEBUG_MODE)
extern uint8_t blecmd;
#endif

#endif

