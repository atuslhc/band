#ifndef _SERIAL2SERVICE_H
#define _SERIAL2SERVICE_H
#include "bcomdef.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "SensorTagUser.h"


#include "thermometerservice.h"
#include "heartrateservice.h"


/*************************************************
***宏定义
*************************************************/
  /*通道定义*/
#define HEART_RATE_CHANNEL      0x03

#define TEMPERATURE_CHANNEL     0X06
#define RUNNING_CHANNEL         0X07
#define READ_ATTRIBUTE_CHANNEL  0X08
#define DEVICE_INFO_CHANNEL     0X09

#define HEART_ALERT_CHANNEL     0x0A //新加 2015-7-9 13:08:53  


typedef struct {
  bool heartRate;
  bool temperature;
  bool running;
  bool heartRateAlert;//新添加
}stNotiFlag_Type;

  /*NOTIF长度定义*/
//#define HEART_RATE_NOTIFY_LEN 11

typedef uint8 (*pFnSerial2Service)(uint8 *pData,uint8 len);


extern const pFnSerial2Service fnSerial2Service[];

extern void Thermometer_CBs(uint8 event);
extern void heartRateCB(uint8 event);
extern bStatus_t Running_CBs(uint8 event);
extern void HeartRateAlert_CBs(uint8 event);
extern void myAlertCB(uint8 event);//2015年9月2日10:25:32新加的 
extern uint8 g_ucTempFlag;
extern uint8 g_ucRunningFlag;

extern const uint8 S2S_FN_MAX;
#endif

