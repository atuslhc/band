#include <stdarg.h>

#include "main.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_adc.h"
#include "em_rtc.h"

#include "common_vars.h"
#include "device_task.h"
#include "flash_task.h"
#include "display_task.h"
#include "notification.h"
#include "crc.h"
#include "cortex-m_faults.h"
#include "em_int.h"

#include "Si14x.h"

//bool DiagnoseInfo_Ready=false;


bool hasEnteredOutofbatteryState = false;

void SendDiagnoseInfo(void);
void SimuUart_9600(unsigned char data);
void SimuUart__4800(unsigned char data);
void LowBatteryProcess(void);
void GoStopMCU(void);

void SimuUart__4800(unsigned char data)
{
	char i = 8;
	unsigned char temp = 0;

	GPIO_PinOutClear(Diagnose_GPIOPORT, Diagnose_PIN); //��ʼλ

	switch (SYSCLOCK)
	{
		case 28 :
			SysCtlDelay(1920);
			break;

		case 14 :
			SysCtlDelay(960);
			break;

		case 7  :
			SysCtlDelay(450);
			break;

		default :
			break;
	}

	while(i--)
	{
		temp = data;
		temp &= 0x01;

//����1λ����
		if(temp == 1)
		{
			GPIO_PinOutSet(Diagnose_GPIOPORT, Diagnose_PIN);
		}
		else
		{
			GPIO_PinOutClear(Diagnose_GPIOPORT, Diagnose_PIN);
		}

		switch (SYSCLOCK)
		{
			case 28 :
				SysCtlDelay(1920);
				break;

			case 14 :
				SysCtlDelay(960);
				break;

			case 7  :
				SysCtlDelay(450);
				break;

			default :
				break;
		}

		data = data >> 1;
	}

	GPIO_PinOutSet(Diagnose_GPIOPORT, Diagnose_PIN); //����λ

	switch (SYSCLOCK)
	{
		case 28 :
			SysCtlDelay(1920);
			break;

		case 14 :
			SysCtlDelay(960);
			break;

		case 7  :
			SysCtlDelay(450);
			break;

		default :
			break;
	}

}


void SimuUart_9600(unsigned char data)
{
	char i = 8;
	unsigned char temp = 0;

	GPIO_PinOutClear(Diagnose_GPIOPORT, Diagnose_PIN); //��ʼλ


	switch (SYSCLOCK)
	{
		case 28 :
			SysCtlDelay(960);
			break;

		case 14 :
			SysCtlDelay(480);
			break;

		case 7  :
			SysCtlDelay(222);
			break;

		default :
			break;
	}

	while(i--)
	{
		temp = data;
		temp &= 0x01;

//����1λ����
		if(temp == 1)
		{
			GPIO_PinOutSet(Diagnose_GPIOPORT, Diagnose_PIN);
		}
		else
		{
			GPIO_PinOutClear(Diagnose_GPIOPORT, Diagnose_PIN);
		}

		switch (SYSCLOCK)
		{
			case 28 :
				SysCtlDelay(960);
				break;

			case 14 :
				SysCtlDelay(480);
				break;

			case 7  :
				SysCtlDelay(222);
				break;

			default :
				break;
		}

		data = data >> 1;
	}

	GPIO_PinOutSet(Diagnose_GPIOPORT, Diagnose_PIN); //����λ

	switch (SYSCLOCK)
	{
		case 28 :
			SysCtlDelay(960);
			break;

		case 14 :
			SysCtlDelay(480);
			break;

		case 7  :
			SysCtlDelay(222);
			break;

		default :
			break;
	}
}


void SendDiagnoseInfo(void)
{
//	int i;
//	uint8_t u8buff[20];
//
//	u8buff[0] = 0xff;
//	u8buff[1] = 0xff;
//	u8buff[2] = 8;
//	memcpy(&u8buff[3], &BLE_DevChip.BLE_DeviceInfo[4], 6);
//
//	u8buff[3 + 6] = (uint8_t)systemStatus.blAmbTempSensorOnline
//	                + (uint8_t)(systemStatus.blSkinTempSensorOnline << 1)
//	                + (uint8_t)(systemStatus.blUVSensorOnline << 2)
//	                + (uint8_t)(systemStatus.blHRSensorOnline << 3)
//	                + (uint8_t)(systemStatus.blBleOnline << 4)
//	                + (uint8_t)(systemStatus.blAccelSensorOnline << 5)
//	                + (uint8_t)(systemStatus.blVibratorOnline << 6);
//
//	u8buff[3 + 7] = 0;
//
//	uint16_t temp = CRC_calc(&u8buff[3], &u8buff[3 + 7]);
//
//	u8buff[3 + 8] = (uint8_t)temp;
//	u8buff[3 + 9] = (uint8_t)(temp >> 8);
//
//	SKIN_H();
//
//	for(i = 0; i < 13; i++)
//	{
//		/* Disable interrupts */
//		//INT_Disable();
//
//		SimuUart_9600(u8buff[i]);
//		//SimuUart__4800(u8buff[i]);
//
//		/* Initialization done, enable interrupts globally. */
//		//INT_Enable();
//	}
//
////����Ӳ������״̬��Ϣ�Լ����ӼĴ�����ֵ
//	SendErrorInformation();
//	SKIN_L();


	//====================�����޸ĳ��ʱ�������Э��,��ȥ��������Ϣ=================

	int i;
	uint8_t u8buff[30];

	u8buff[0] = 0xAA;
	u8buff[1] = 0xAA;
	u8buff[2] = 0xAA;
	u8buff[3] = 15;//����ǳ�����Ϣ
	memcpy(&u8buff[4], &BLE_DevChip.BLE_DeviceInfo[4], 6);

	u8buff[4 + 6] = (uint8_t)systemStatus.blAmbTempSensorOnline
	                + (uint8_t)(systemStatus.blSkinTempSensorOnline << 1)
	                + (uint8_t)(systemStatus.blUVSensorOnline << 2)
	                + (uint8_t)(systemStatus.blHRSensorOnline << 3)
	                + (uint8_t)(systemStatus.blBleOnline << 4)
	                + (uint8_t)(systemStatus.blAccelSensorOnline << 5)
	                + (uint8_t)(systemStatus.blVibratorOnline << 6);

	//������ص���
	u8buff[4 + 7] = systemStatus.bBatteryRemaining;

	//app version
	u8buff[4 + 8] = APP_FW_VER_M;
	u8buff[4 + 9] = APP_FW_VER_S;

	//ble version
	u8buff[4 + 10] = BLE_DevChip.BLE_Device.FW_VER1;
	u8buff[4 + 11] = BLE_DevChip.BLE_Device.FW_VER2;

	//bootloader version
	u8buff[4 + 12] = *(uint8_t*) BOOT_FW_VER_M_Add;
	u8buff[4 + 13] = *(uint8_t*) BOOT_FW_VER_S_Add;

	//device model
	u8buff[4 + 14] = systemSetting.bDeviceModel;

	uint16_t temp = CRC_calc(&u8buff[4], &u8buff[4 + 14]);

	u8buff[4 + 15] = (uint8_t)temp;
	u8buff[4 + 16] = (uint8_t)(temp >> 8);

	u8buff[4 + 17] = 0x55;//��0x55
	u8buff[4 + 18] = 0x55;//��0x55

	SKIN_H();

	for(i = 0; i < 23; i++)
	{
		/* Disable interrupts */
		//INT_Disable();

		SimuUart_9600(u8buff[i]);
		//SimuUart__4800(u8buff[i]);

		/* Initialization done, enable interrupts globally. */
		//INT_Enable();
	}

	SKIN_H();
}


void UpdateCalories(void);

const  ADC_InitSingle_TypeDef ECG_ADC_INIT =
{
	adcPRSSELCh0, /* prsSel  PRS ch0 (if enabled). */				\
	adcAcqTime64, /* acqTime 1 ADC_CLK cycle acquisition time. */		   \
	adcRefExtSingle,/* reference 1.25V internal reference. */				\
	adcRes12Bit,/* resolution 12 bit resolution. */ 		   \
	adcSingleInpCh4Ch5,/* input CH0 input selected. */			   \
	true,  /* diff Single ended input. */					 \
	false, /* PRS disabled. */					 \
	false, /*leftAdjust  Right adjust. */				   \
	false  /* Deactivate conversion after one scan sequence. */ \
};

const	ADC_InitSingle_TypeDef BAT_ADC_INIT =
{
	adcPRSSELCh0, /*   PRS ch0 (if enabled). */ 		   \
	adcAcqTime8,  /* 1 ADC_CLK cycle acquisition time. */	  \
	adcRefVDD,	/* 1.25V internal reference. */ 		  \
	adcRes12Bit, /* 12 bit resolution. */			   \
	adcSingleInpCh7, /* CH0 input selected. */		   \
	false, /* Single ended input. */			   \
	false,		/* PRS disabled. */ 		\
	false,	/* Right adjust. */ 								 \
	false  /* Deactivate conversion after one scan sequence. */ \
};

void ADC_INIT(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	/* Init common settings for both single conversion and scan mode */
	init.timebase = ADC_TimebaseCalc(0);

	// init.lpfMode =adcLPFilterRC;//adcLPFilterDeCap;

	//init.warmUpMode=adcWarmupKeepScanRefWarm;

	/* Set ADC clock to 7 MHz, use default HFPERCLK */
	init.prescale = ADC_PrescaleCalc(7000000, 0);

	/* Set oversampling rate */
	//init.ovsRateSel = adcOvsRateSel32;//adcOvsRateSel32;

	ADC_Init(ADC0, &init);

	//ADC_InitSingle(ADC0, &BAT_ADC_INIT);

}

#define Vcc_Buff_Size 4
short Vcc_Buff[Vcc_Buff_Size], Vcc_Buff_Rp = 0;

void Battery_ADC_Init(void)
{
	// �ӵ�����߷��̷��Ƴ����� ������ ֵ = 2607
	ADC_INIT();

	for(int i = 0; i < Vcc_Buff_Size; i++)
		Vcc_Buff[i] = 2607; //2978; // 2.4/3.3 * 4096

	ADC_InitSingle(ADC0, &BAT_ADC_INIT);
}

void Battery_ADC_Read(void)
{
	float BAT_VCC;

	while (ADC0->STATUS & ADC_STATUS_SINGLEACT);

	short val = (short)ADC_DataSingleGet(ADC0);

	if(val)
		Vcc_Buff[Vcc_Buff_Rp++] = val;

	//
	Vcc_Buff_Rp %= Vcc_Buff_Size;  //Atus: should be put into above increment block.
	BAT_VCC = 0;

	for(int i = 0; i < Vcc_Buff_Size; i++)
		BAT_VCC += Vcc_Buff[i];

	BAT_VCC = BAT_VCC / Vcc_Buff_Size;

        //itest = val; 
        //ftest = BAT_VCC;
	BAT_VCC = 2 * BAT_VCC * 3.3 / 4096;

	//================================
	// percent = 1.26V-4.292 // ���� �����ṩ�ĵ������ͼ��õ����Է���
	float temp = 1.26 * BAT_VCC - 4.292;

	if(temp < 0) temp = 0.01;

	temp = temp * 100;

	if(temp < 40)
		temp = temp * 0.9;

	if(temp > 100)
		systemStatus.bBatteryRemaining = 100;
	else
		systemStatus.bBatteryRemaining = (uint8_t)temp;
}


void ADC_CLOSE(void)
{
	CMU_ClockEnable(cmuClock_ADC0, false);
}

float SUM_BMR = 0;
void UpdateCalories(void)
{
	if((isMemsSleeping == true) && (systemStatus.blHRSensorOn == false))
	{
		SUM_BMR += BMR_PER_SECOND ;//+ Calories_at_2second;

		if(SUM_BMR >= 1)
		{
			iCalories += (int)SUM_BMR;
			SUM_BMR = SUM_BMR - (int)SUM_BMR;
		}
	}
}


// =================================================================
// �����ƿ���
#define LED_FLASH_PATTERN_LEN 10

#pragma pack(push, 1)

typedef struct
{
	// settings
	uint8_t led_flash_pattern[LED_FLASH_PATTERN_LEN];
	uint8_t led_flash_repeat;
	uint8_t led_flash_steps;	// ��ǰpattern�ж��ٲ�

	// status
	int8_t led_flash_repeat_count;	// ѭ�������ļ�����
	int8_t led_flash_step;			// ��ǰpattern������
	int8_t led_flash_step_value;	// ��ǰpattern���ֵ
	int8_t led_flash_step_count;	// ��ǰpattern��ļ�����
} LED_FLASH_SETTINGS;

#pragma pack(pop)

bool isLedFlashing = false;					// ��ǰ�Ƿ��� led flashing ����
bool blLedFlashSettingsBackuped = false;	// �Ƿ񱸷��� led flashing ����
LED_FLASH_SETTINGS currentLedFlashSettings;
LED_FLASH_SETTINGS backupLedFlashSettings;
//// settings
//uint8_t led_flash_pattern[LED_FLASH_PATTERN_LEN];
//uint8_t led_flash_repeat = 1;
//uint8_t led_flash_steps = LED_FLASH_PATTERN_LEN;	// ��ǰpattern�ж��ٲ�
//
//// status
//int8_t led_flash_repeat_count = 0;	// ѭ�������ļ�����
//int8_t led_flash_step = -1;			// ��ǰpattern������
//int8_t led_flash_step_value = -1;	// ��ǰpattern���ֵ
//int8_t led_flash_step_count = 0;	// ��ǰpattern��ļ�����

__STATIC_INLINE void saveLedFlashPattern()
{
	blLedFlashSettingsBackuped = true;
	memcpy(&backupLedFlashSettings, &currentLedFlashSettings, sizeof(LED_FLASH_SETTINGS));
}

__STATIC_INLINE void restoreLedFlashPattern()
{
	blLedFlashSettingsBackuped = false;
	memcpy(&currentLedFlashSettings, &backupLedFlashSettings, sizeof(LED_FLASH_SETTINGS));
}

//void setLedFlashPattern(uint8_t repeat, short num, ...)
//{
//	led_flash_repeat = repeat;
//	led_flash_step = -1;
//	led_flash_repeat_count = 0;
//
//	//
//	if (num > LED_FLASH_PATTERN_LEN)
//		num = LED_FLASH_PATTERN_LEN;
//
//	led_flash_steps = num;
//
//	memset(led_flash_pattern, 0, LED_FLASH_PATTERN_LEN);
//
//	//
//	va_list valist;
//	va_start(valist, num);
//
//	uint8_t v = 0;
//	for ( int x = 0; x < num; x++ )
//	{
//		v = va_arg(valist, uint8_t);
//		if (v == 0)
//		{
//			led_flash_steps = x + 1;
//			break;
//		}
//
//		led_flash_pattern[x] = v;
//	}
//
//	va_end(valist);
//}

/* Brief: Setup the LED flashing count and mode.
 * blBackupAndRestore : store the current mode for restore after finish LED command. ex, ble connected while charging.
 * repeat :  repeat count, 0xff = infinite.
 * num : the mode number. the mode arguments must be <= LED_FLASH_PATTERN_LEN
 *  The mode arguments calculated in 100ms. The first argument is On time, second argument is Off time, The third is On time...
 */
void startFlashLed(bool blBackupAndRestore, uint8_t repeat, short num, ...)
{
	// �����ǰ�� led flashing ���ã���ʹ���˱��ݡ��ָ�
	if (blBackupAndRestore && isLedFlashing)
	{
		saveLedFlashPattern();
	}

	//
	currentLedFlashSettings.led_flash_repeat = repeat;
	currentLedFlashSettings.led_flash_step = -1;
	currentLedFlashSettings.led_flash_repeat_count = 0;

	//
	if (num > LED_FLASH_PATTERN_LEN)
		num = LED_FLASH_PATTERN_LEN;

	currentLedFlashSettings.led_flash_steps = num;

	memset(currentLedFlashSettings.led_flash_pattern, 0, LED_FLASH_PATTERN_LEN);

	//
	va_list valist;
	va_start(valist, num);

	uint8_t v = 0;

	for ( int x = 0; x < num; x++ )
	{
		v = va_arg(valist, uint8_t);

		if (v == 0)
		{
			currentLedFlashSettings.led_flash_steps = x + 1;
			break;
		}

		currentLedFlashSettings.led_flash_pattern[x] = v;
	}

	va_end(valist);

	//
	isLedFlashing = true;

//	if (isLedFlashing == false)
	EnableDelayTimer(TIMER_FLAG_LEDFlashing, true, 100, LEDFlashingCallback, NULL);
}

void stopFlashLed()
{
	if (blLedFlashSettingsBackuped)
		restoreLedFlashPattern();
	else
	{
		isLedFlashing = false;
		LED_OFF();//�¼ӵġ�
		DisableDelayTimer(TIMER_FLAG_LEDFlashing);
	}
}

void LEDFlashingCallback(void* data)
{
	if (currentLedFlashSettings.led_flash_step < 0)
	{
		currentLedFlashSettings.led_flash_step = 0;

		currentLedFlashSettings.led_flash_step_value = -1;
		currentLedFlashSettings.led_flash_step_count = 0;
	}
	else if (currentLedFlashSettings.led_flash_step >= currentLedFlashSettings.led_flash_steps)
	{
		// �Ѿ��߹������һ��
		currentLedFlashSettings.led_flash_repeat_count++;

		if (currentLedFlashSettings.led_flash_repeat != 0xFF
		        && currentLedFlashSettings.led_flash_repeat_count >= currentLedFlashSettings.led_flash_repeat)
		{
			//
			LED_OFF();

			//
			stopFlashLed();
		}
		else
		{
			// ����
			currentLedFlashSettings.led_flash_step = 0;

			currentLedFlashSettings.led_flash_step_value = -1;
//			led_flash_step_count = 0;
		}
	}

	// ��ǰpattern���ֵû��ȡ�����ȡ֮
	if (currentLedFlashSettings.led_flash_step_value < 0)
	{
		currentLedFlashSettings.led_flash_step_value = currentLedFlashSettings.led_flash_pattern[currentLedFlashSettings.led_flash_step];
		currentLedFlashSettings.led_flash_step_count = 0;
	}

	if (currentLedFlashSettings.led_flash_step_value > 0)
	{
		if (currentLedFlashSettings.led_flash_step % 2 == 0)
		{
			LED_ON();
		}
		else
		{
			LED_OFF();
		}

		currentLedFlashSettings.led_flash_step_count++;

		// ��ǰpattern���Ѿ���ɣ�������һ��
		if (currentLedFlashSettings.led_flash_step_count >= currentLedFlashSettings.led_flash_step_value)
		{
			currentLedFlashSettings.led_flash_step++;

			//
			currentLedFlashSettings.led_flash_step_value = -1;
		}

		return;
	}
	else
	{
		// ��ǰpattern�� == 0������һ��ѭ��
		currentLedFlashSettings.led_flash_repeat_count++;

		if (currentLedFlashSettings.led_flash_repeat != 0xFF
		        && currentLedFlashSettings.led_flash_repeat_count >= currentLedFlashSettings.led_flash_repeat)
		{
			//
			LED_OFF();

			//
			stopFlashLed();
		}
		else
		{
			// ��Ҫѭ�������ü�����
			currentLedFlashSettings.led_flash_step = -1;

//			// Ȼ���ٴε����Լ�
//			LEDFlashingCallback(0);
		}
	}
}

void CHECK_PER_Xsecond(void)
{
	static uint16_t X_SECONDS_COUNT = 0;
	static uint16_t CHECK_INTERVAL_Changing = 0;
	extern uint8_t FW_Update_sta;

	if(FW_Update_sta == 0) //  if fw updating is going , ignore them for speed up and stable
	{
		if(systemSetting.SystemMode != SYSTEM_MODE_ACTIVATED)
		{
#if 1  // close BLE only, can mask it off

			if(isMemsSleeping == true)
			{
				MemsSleepCount++;

				if(MemsSleepCount == BLE_Sleep_Time) // 5mins  then close BLE and cap sensor
				{
					if(BLE_STATE != BLE_STATE_CONNECTED)
					{
						BLE_Close();
						isMemsDeepSleeping = true;
					}
				}
			}
			else
			{
				isMemsDeepSleeping = false;

				if(MemsSleepCount >= BLE_Sleep_Time)
				{
					if(BLE_STATE == BLE_STATE_IDLE)
						BLE_Open();

					MemsSleepCount = 0;
				}
			}
#endif
		}

		CheckVibrateStatus();
		isMemsError();
		CheckFlashStatus();

		if(isMemsSleeping == true)
		{
			CHECK_INTERVAL_Changing = CHECK_INTERVAL_At_Sleep;

			if(isMemsDeepSleeping == true)
				CHECK_INTERVAL_Changing = CHECK_INTERVAL_At_DeepSleep;
		}
		else
			CHECK_INTERVAL_Changing = CHECK_INTERVAL_At_ACT;

		if(BLE_STATE == BLE_STATE_CONNECTED)
			CHECK_INTERVAL_Changing = CHECK_INTERVAL_AT_BLE;

		X_SECONDS_COUNT++;

		if(X_SECONDS_COUNT >= CHECK_INTERVAL_Changing)
		{
			X_SECONDS_COUNT = 0;

			if (systemStatus.bBatteryLevel >= LOW_BATTERY)
			{
#if BATTERY_LIFE_OPTIMIZATION
//				Start_Cap_Temp();
//				//Check_UV_Sensor(INDOOR);
//				Check_UV_Sensor(OUTDOOR);//�������ȡUVָ����
#else
				Start_Cap_Temp();
				//Check_UV_Sensor(INDOOR);
				Check_UV_Sensor(OUTDOOR);//�������ȡUVָ����
#endif
			}
		}

		//UpdateCalories();
		CHECK_BLE_STUFF();
	}
}

void Check_UV_Sensor(bool inoutdoor)
{
	static uint8_t UV_Warning_Count = 0;
	uint8_t readindex; //[BG023-2] add to filter abnormal value.

#if BATTERY_LIFE_OPTIMIZATION
	systemSetting.blUVSensorEnabled = false;
#endif

	if(systemSetting.blUVSensorEnabled == false)
	{
		AmbientLight = 500;
		return;
	}

	if(inoutdoor == INDOOR)
	{
		InOutdoorChange(0);
		Si114xAlsForce();
		AmbientLight = GetAmbLight();
		InOutdoorChange(1);
	}
	else
	{
		Si114xAlsForce();
		//bUltraViolet = GetUVindex(); //[BG023-2] replace with below.
		if ((readindex = GetUVindex())<=15) //[BG023-2]
		    bUltraViolet = readindex;
		
                /* [BG023] add update bUltraVioletGather */
                if (bUltraViolet > bUltraVioletGather)
                  bUltraVioletGather = bUltraViolet;

		//ȥ��UV����
		if(bUltraViolet >= UV_Warning_Level)
		{
			UV_Warning_Count++;

			if(UV_Warning_Count >= 2) //
			{
				UV_Warning_Count = 0;

				if (systemStatus.blUVAlarmDisabled == false)
				{
					// ��֪ͨ�û���Ӧʹ���ر����ģʽ
//					VibrateCon(StrongBuzz_100, 2, 5);
//					// ��ʾ ��ʾ
//					RaiseNotification(NOTIFY_SERVICE_Intense_UV);
				}
			}
		}
		else
		{
			UV_Warning_Count = 0;

			if (checkNotification(NOTIFY_SERVICE_Intense_UV) > 0)
			{
//				RemoveNotification(NOTIFY_SERVICE_Intense_UV);
//				// ȡ����
//				stopVibrate();
			}
		}
	}
}

void CheckGoalsAccomplish(void)
{
	UINT distanceValue = 0;
	UINT caloriesValue = 0;
	UINT stepValue = 0;
	uint8_t percentage = 0;

#ifdef DEBUG
//	iDistance = 1200;
//	iSteps = 10003;
//	iCalories = 3000;
#endif


	distanceValue = iDistance;
	caloriesValue = iCalories;
	stepValue = iSteps;

	if(blStepAccomplish != true)
	{
		percentage = (uint8_t)((float)stepValue / (float)systemSetting.userGoals[GOAL_STEPS] * 100.0);

		if(percentage >= 100)
		{
			static uint8_t stepShineCount = 0;
			stepShineCount++;

			if(stepShineCount < 9)
				NoTOUCHPressCount = GOAL_SHINE_DELAY;
			else
				NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY;

			if(stepShineCount % 3 == 0)
			{
				RaiseNotification(NOTIFY_SERVICE_Step_Accomplish);
				VibrateCon(StrongBuzz_100, 1, 1);
			}

			if(stepShineCount > 9)
			{
				RemoveNotification(NOTIFY_SERVICE_Step_Accomplish);
				blStepAccomplish = true;
				blAccomplishGoalShine = false;
				stepShineCount = 0;
			}
		}
	}


	if(blCalorieAccomplish != true)
	{
		percentage = (uint8_t)((float)caloriesValue / (float)systemSetting.userGoals[GOAL_CALORIES] * 100.0);

		if(percentage >= 100)
		{
			static uint8_t caloriesShineCount = 0;
			caloriesShineCount++;

			if(caloriesShineCount < 9)
				NoTOUCHPressCount = GOAL_SHINE_DELAY;
			else
				NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY;

			if(caloriesShineCount % 3 == 0)
			{
				RaiseNotification(NOTIFY_SERVICE_Calorie_Accomplish);
				VibrateCon(StrongBuzz_100, 1, 1);
			}


			if(caloriesShineCount > 9)
			{
				RemoveNotification(NOTIFY_SERVICE_Calorie_Accomplish);
				blCalorieAccomplish = true;
				blAccomplishGoalShine = false;
				caloriesShineCount = 0;
			}
		}
	}

	if(blDistanceAccomplish != true)
	{
		percentage = (uint8_t)((float)distanceValue / (float)systemSetting.userGoals[GOAL_DISTANCE] * 100.0);

		if(percentage >= 100)
		{
			static uint8_t distanceShineCount = 0;
			distanceShineCount++;

			if(distanceShineCount < 9)
				NoTOUCHPressCount = GOAL_SHINE_DELAY;//��1��
			else
				NoTOUCHPressCount = LONG_LOCK_SCREEN_DELAY;

			if(distanceShineCount % 3 == 0)
			{
				RaiseNotification(NOTIFY_SERVICE_Distance_Accomplish);

				VibrateCon(StrongBuzz_100, 1, 1);

			}


			if(distanceShineCount > 9)
			{
				RemoveNotification(NOTIFY_SERVICE_Distance_Accomplish);
				blDistanceAccomplish = true;
				blAccomplishGoalShine = false;
				distanceShineCount = 0;
			}
		}
	}
}

#define BATTERY_REMAINING_OUT_OF_BATTERY	13
#define BATTERY_REMAINING_LOW_BATTERY		23
#define BATTERY_LEVEL_TOLERANCE 			3

void CHECK_BATTERY(void)
{
	static unsigned char BAT_CHECK_COUNTER = BAT_CHECK_INTERVAL;
	static unsigned char BAT_AD_FREQ_REF;

	static bool batteryDrainout = false;

	if(GPIO_PinInGet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN) == 0)
	{
		BAT_AD_FREQ_REF = 1;

		if(systemStatus.blBatteryCharging == false)
		{
			systemStatus.blBatteryCharging = true;
			startCharging = true;

			//==========================20140821
			if(systemSetting.SystemMode == SYSTEM_MODE_MANUFACTORING)
			{
				SKIN_TEMP_INIT();
				//AdjustCapVal();
				VibrateCon(BuzzAlert1000ms, 1, 3);
			}

			//========================
			Close_ALLCAPLESENSE();
			GPIO_PinModeSet(Diagnose_GPIOPORT, Diagnose_PIN, gpioModePushPull, 0); //��IOĬ������Ϊ�������0����Ϊģ�⴮�ڵķ���
			GPIO_PinModeSet(Diagnose_GPIOPORT, SKIN_PIN, gpioModePushPull, 0); //��IOĬ������Ϊ�������0����Ϊģ�⴮�ڵķ���

			// �ر�ppg
			AFE44xx_Shutoff();
			SensorOffDelay = 0;
			systemStatus.blSkinTouched = false;

			//DiagnoseInfo_Ready=true;

			//========================
//			USER_EVENT* batteryEvent = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//			batteryEvent->type = EVT_TYPE_BATTERY;
//			batteryEvent->data.v = systemStatus.blBatteryCharging;
//			osMailPut(hDispEventQueue, batteryEvent);


//			USER_EVENT batteryEvent;
//			batteryEvent.type = EVT_TYPE_BATTERY;
//			batteryEvent.data.v = systemStatus.blBatteryCharging;
//
//			xQueueSend(hEvtQueueDisplay, &batteryEvent, 0);
			MESSAGE msg;
			msg.params.type = MESSAGE_BATTERY_CHARGING;

			xQueueSend(hEvtQueueDevice, &msg.id, 0);
			//=======================

//			setLedFlashPattern(0xff, 2, 3, 20);
//			EnableDelayTimer(TIMER_FLAG_LEDFlashing, true, 100, LEDFlashingCallback, NULL);
			startFlashLed(false, 0xff, 2, 3, 20); //��300ms,�ر�2s
		}

		//if(systemSetting.SystemMode!=SYSTEM_MODE_RELEASED)
		{
			SendDiagnoseInfo();
		}
	}
	else
	{
		//============================
#if 0
		if(systemStatus.bBatteryLevel <= LOW_BATTERY)
		{
			flashdelay++;

			if(flashdelay > 20)
			{
				flashdelay = 0;
				EnableDelayTimer(TIMER_FLAG_LEDFlashing);
				LED_ON();
			}
		}

#endif
		//==============================

		BAT_AD_FREQ_REF = BAT_CHECK_INTERVAL;

		if(systemStatus.blBatteryCharging == true)
		{
			systemStatus.blBatteryCharging = false;
			//=====================
			CAPLESENSE_Init();
			UnLockScreen(true); // ������Ļ
			//======================
//			USER_EVENT* batteryEvent = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//			batteryEvent->type = EVT_TYPE_BATTERY;
//			batteryEvent->data.v = systemStatus.blBatteryCharging;
//			osMailPut(hDispEventQueue, batteryEvent);

//			USER_EVENT batteryEvent;
//			batteryEvent.type = EVT_TYPE_BATTERY;
//			batteryEvent.data.v = systemStatus.blBatteryCharging;
//
//			xQueueSend(hEvtQueueDisplay, &batteryEvent, 0);
			MESSAGE msg;
			msg.params.type = MESSAGE_BATTERY_CHARGING;

			xQueueSend(hEvtQueueDevice, &msg.id, 0);

			//
			stopFlashLed();
		}
	}

	static bool oldChargeStatus = false;
	bool currentChargeStatus = false;

        //static time_t time1;
	currentChargeStatus = systemStatus.blBatteryCharging;

	if(oldChargeStatus != currentChargeStatus)
	{
		isChargeStatusChange = 0x01;
		oldChargeStatus = currentChargeStatus;
	}

	BAT_CHECK_COUNTER++;

	if(BAT_CHECK_COUNTER >= BAT_AD_FREQ_REF)
	{
                //itest = BAT_CHECK_COUNTER;
                //utest = time(NULL) - time1;
                //time1 = time(NULL);
		BAT_CHECK_COUNTER = 0;
		Battery_ADC_Read();
		ADC_Start(ADC0, adcStartSingle);
		static uint8_t lowBatteryCount = 0;
		static bool once = true;
		static bool blHasLowBattery = false;


		if((systemStatus.bBatteryRemaining < 25) && once)
		{
			lowBatteryCount++;

			if(lowBatteryCount >= 3)
			{
				lowBatteryCount = 0;
				lowBatteryLevelAlert = 0x01;
				once  = false;
				blHasLowBattery = true;
			}
		}


		if((systemStatus.bBatteryRemaining > 30) && (blHasLowBattery == true)) //ֻ�е���������25%ʱ�����ٴ������͵�ѹ������
		{
			once = true;
			blHasLowBattery = false;
			lowBatteryLevelAlert = 0xAA; //����ѹ����30%ʱ���ٰѵ͵�ѹ״̬��������
		}


		//
		ENUM_BATTERY_LEVEL bOldBatteryLevel = systemStatus.bBatteryLevel;
		ENUM_BATTERY_LEVEL bNewBatteryLevel = bOldBatteryLevel;

		switch (bOldBatteryLevel)
		{
			case BATTERY_NORMAL:
			{
				if(systemStatus.bBatteryRemaining <= BATTERY_REMAINING_OUT_OF_BATTERY - BATTERY_LEVEL_TOLERANCE)
					bNewBatteryLevel = OUT_OF_BATTERY;
				else if(systemStatus.bBatteryRemaining <= BATTERY_REMAINING_LOW_BATTERY - BATTERY_LEVEL_TOLERANCE)
					bNewBatteryLevel = LOW_BATTERY;

				break;
			}

			case LOW_BATTERY:
			{
				if(systemStatus.bBatteryRemaining <= BATTERY_REMAINING_OUT_OF_BATTERY - BATTERY_LEVEL_TOLERANCE)
					bNewBatteryLevel = OUT_OF_BATTERY;
				else if(systemStatus.bBatteryRemaining >= BATTERY_REMAINING_LOW_BATTERY + BATTERY_LEVEL_TOLERANCE)
					bNewBatteryLevel = BATTERY_NORMAL;

				break;
			}

			case OUT_OF_BATTERY:
			{
				if(systemStatus.bBatteryRemaining >= BATTERY_REMAINING_LOW_BATTERY + BATTERY_LEVEL_TOLERANCE)
					bNewBatteryLevel = BATTERY_NORMAL;
				else if (systemStatus.bBatteryRemaining >= BATTERY_REMAINING_OUT_OF_BATTERY + BATTERY_LEVEL_TOLERANCE)
					bNewBatteryLevel = LOW_BATTERY;

				break;
			}
		}

		if (bNewBatteryLevel != bOldBatteryLevel)
		{
			systemStatus.bBatteryLevel = bNewBatteryLevel; // change to new status

			MESSAGE msg;
#if !BATTERY_LIFE_OPTIMIZATION2
			// ���͵�ص����¼������¼��м�� systemStatus.bBatteryLevel ȷ����ص���

			msg.params.type = MESSAGE_BATTERY_LEVEL;
			msg.params.param = bOldBatteryLevel; // old status
			//			osMessagePut(hMsgInterrupt, msg.id, 0);
			xQueueSend(hEvtQueueDevice, &msg.id, 0);

#else

			if(bNewBatteryLevel != LOW_BATTERY)
			{
				//����ص��͵�ѹʱ�������͸���Ϣ����������
				msg.params.type = MESSAGE_BATTERY_LEVEL;
				msg.params.param = bOldBatteryLevel; // old status
//			osMessagePut(hMsgInterrupt, msg.id, 0);
				xQueueSend(hEvtQueueDevice, &msg.id, 0);

			}

#endif


			//
			if (bNewBatteryLevel == OUT_OF_BATTERY)
			{
				if((systemStatus.blBatteryCharging == true))//��ֹMCU�մӵ͹��������ֽ�ȥ�͹��ģ�����Ĳ�����֤����ȫû�е���ٳ��ʱ��������͹��ġ�
					return;

				batteryDrainout = true;
//				OutOfBatteryProcess();//�����״̬��ȫû�磬�͹ر�
			}
			else if(bNewBatteryLevel == LOW_BATTERY)
				LowBatteryProcess();
			else //if (bOldBatteryLevel == OUT_OF_BATTERY)
				BackToWork();
		}
	}

//
	if(batteryDrainout)
	{
		static uint8_t count = 0;
		count++;

		if(count >= 3)
		{
			count = 0;
			batteryDrainout = false;
			OutOfBatteryProcess();
		}
	}
}

void LowBatteryProcess(void)
{
	AFE44xx_Shutoff(); //��low battery  mode
	systemStatus.blHRSensorTempEnabled = false;

	if(hasEnteredOutofbatteryState)
	{
		BLE_Open();
		hasEnteredOutofbatteryState = false;
	}

//���ѵ��ص� 13%���Ͼʹ򿪴��� (ʵ���ϣ������EM4����Ļ�������Ҫ��Щ����Ϊ��EM4�������Ǹ�λ����Щ�����ᱻ�ٴγ�ʼ��)
//	CMU_ClockEnable(cmuClock_LEUART0, true);
//	LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF); // �ظ�����
}

//�������Ϊout of battery�Ĵ���Temperture and UV index will be started at  CHECK_PER_Xsecond() when battery is much bigger than low battery.
extern void SysCtlDelay(unsigned long ulCount);
void OutOfBatteryProcess(void)
{
	/* Enable access to BURTC registers */
	RMU_ResetControl(rmuResetBU, false);

	//----------------------
	//����Щֵ���浽��������
	I_CALORIES = iCalories;
	I_CALORIES_LASTSAVING = iCalories_lastSaving;
	I_STEPS = iSteps;
	I_STEPS_LASTSAVING = iSteps_lastSaving;
	I_DISTANCE = iDistance;
	I_DISTANCE_LASTSAVING = iDistance_lastSaving;
	ACTIVE_LEVEL = active_level;
	ACTIVE_LEVEL_LASTSAVING = active_level_lastSaving;

	//
	hasEnteredOutofbatteryState = true;
	MEMS_CLOSE();

	TEMPSENS_RegisterSet(TMP_I2C, SKIN_TEMP_ADDR, tempsensRegCONFIG, 0x0100);	//temperature close
	Si114xPauseAll();//UV close

	BLE_Close(); //���͹ر����ӵ������������

	Close_ALLCAPLESENSE(); //�رմ���

	//
	GoStopMCU();
}

void BackToWork(void)
{
	MEMS_OPEN();
//	CAPLESENSE_Init();
	BLE_Open();
	systemStatus.blHRSensorTempEnabled = true;
	systemSetting.blTouchSensorEnabled = true; //�ӵ͵�ѹ�����󣬰�ppgsensor���ó��Զ�ģʽ��

}


void GoStopMCU(void)
{

	if((systemStatus.bBatteryLevel != OUT_OF_BATTERY) || (systemStatus.blBatteryCharging == true) || (hasEnteredOutofbatteryState != true))
		return;//�������������ֹ�մ�EM3�������ֽ���EM3.


//����EM4�����Ż���
	EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
	em4Init.lockConfig    = true;              		/* Lock regulator, oscillator and BOD configuration.
                                                         * This needs to be set when using the
                                                         * voltage regulator in EM4 */
	em4Init.osc           = emuEM4Osc_ULFRCO;             /* Select ULFRCO */
	em4Init.buRtcWakeup   = true;                         /* BURTC compare or overflow will generate reset */
	em4Init.vreg          = true;                         /* Enable voltage regulator. Needed for BURTC */
	EMU_EM4Init( &em4Init );



	/* Set pin PF2 mode to input with pull-up resistor */
	GPIO_PinModeSet(gpioPortC, 9, gpioModeInput, 1);

	/* Enable GPIO pin mode retention in EM4 */
	GPIO->CTRL |= GPIO_CTRL_EM4RET;

	/* Clear wake up requests */
	GPIO->CMD |= GPIO_CMD_EM4WUCLR;

	/* Enable wakeup on PF2 */
	GPIO->EM4WUEN = GPIO_EM4WUEN_EM4WUEN_C9;


	//�����ر�һЩʱ��
	LEUART_IntDisable(LEUART0, LEUART_IEN_SIGF);    //������ʣ������ﵽ20%ʱ���ٴ򿪡�
	LETIMER_Enable(LETIMER0, false);               //�ص�tickmessage
	CMU_ClockEnable(cmuClock_LESENSE, false);		/* disenable clock for LESENSE.Ҳ���Ǵ��� */
	CMU_ClockEnable(cmuClock_LEUART0, false);
	CMU_ClockEnable(cmuClock_DMA, false);
	CMU_ClockEnable(cmuClock_CORELE, false);
	RTC_Enable(false);  //��ϵͳ���׹ر�
	NVIC_DisableIRQ(GPIO_ODD_IRQn);  //����������ٶȣ�AFE�����жϡ�
	NVIC_DisableIRQ(TIMER2_IRQn);//���ܹر���cmuClock_HFPER�����ﻹ�ǰ����жϹص�
	CMU_ClockEnable(cmuClock_I2C0, false); //i2c��ʱ��,�ž�������
	CMU_ClockEnable(cmuClock_HFPER, false);

	SysCtlDelay(5000);
//ע�⿴�Ź��ڳ�ʼ��ʱ����Ҫ������EM3��Ҳ���С�
	WDOG_Enable(false);

	while (WDOG->SYNCBUSY & WDOG_SYNCBUSY_CTRL); //����ܹؼ�������ղ����˿��Ź���ʱ�����ͽ���EM3�Ļ����ᵼ�¿��Ź�û�б��رա�

	EMU_EnterEM4();

//	Restart();

}

#if 0
//ʵ���ϴ�EM4���Ѿ��Ǹ�λ������Ҫ����������Щʱ�ӣ��Լ�������Ϣ����������ϵͳ����EM3���Ѳ���Ҫ�����

void Restart(void)
{
	//�ָ�ʱ��
	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);//�����i2c��ʱ��
	CMU_ClockEnable(cmuClock_CORELE, true);
	LETIMER_Enable(LETIMER0, true);//

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_LESENSE, true);
	CMU_ClockEnable(cmuClock_DMA, true);
	RTC_Enable(true);

	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);

	GPIO_IntConfig(BLE_INT_PORT, BLE_INT_PIN, false, true, true);//���´������Ľ����ж����š�

//	GPIO_IntConfig(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, false, true, false);//���°ѳ���������ó�GPIO����Ҫ�жϡ�
//	GPIO_PinModeSet(CHARGER_STA_GPIOPORT, CHARGER_STA_PIN, gpioModeInputPull, 1);//�������óɸߣ����оƬ״̬�Ͳ��ܰ��������͡�

	extern WDOG_Init_TypeDef wathdoginit;
	wathdoginit.enable = true;

	WDOG_Init(&wathdoginit);
	int32_t msg = TICK_Message;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0); //�ڵ���ʱ���֣���RTC�ͷ�����Ϣ���ǿ��Եģ���һ�������ˣ���

}

#endif