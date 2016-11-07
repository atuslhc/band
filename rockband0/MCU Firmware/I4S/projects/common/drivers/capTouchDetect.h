#ifndef __CapTouchDetect_H_
#define __CapTouchDetect_H_

#define default_SensorOffDelay     12 //6 // 3   20140927
#define default_Ble_SensorOffDelay 20
#define default_Key_SensorOffDelay 5



//extern bool  SkinTouched;
extern bool allow_sensor;
extern uint8_t  SensorOffDelay;

void KeyTouchDetect(void);
void SkinTouchDetect(void);
void AdjustCapVal(void);


#endif 


