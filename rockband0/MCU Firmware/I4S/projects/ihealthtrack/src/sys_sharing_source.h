#ifndef sys_sharing_source_H
#define sys_sharing_source_H

#define MEMS_SEL 1
#define FLASH_SEL 2
#define BLE_SEL 3

extern bool DiagnoseInfo_Ready;
extern uint8_t my_prevHeartRate;

void ADC_INIT(void);
void ADC_CLOSE(void);
void OutOfBatteryProcess(void);
void BackToWork(void);
void Battery_ADC_Init(void);
void CHECK_BATTERY(void);
void CHECK_PER_Xsecond(void);
void Check_UV_Sensor(bool inoutdoor);
void CheckGoalsAccomplish(void);

//
void LEDFlashingCallback(void*);

//void saveLedFlashPattern();
//void restoreLedFlashPattern();
//void setLedFlashPattern(uint8_t repeat, short num, ...);
void startFlashLed(bool blBackupAndRestore, uint8_t repeat, short num, ...);
void stopFlashLed();

#endif
