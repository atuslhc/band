#include "freertos.h"
#include "task.h"

#include "drv2605.h"
//#include "motor.h"

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "drv2605.h"

#include "common_vars.h"
#include "main.h"

unsigned char DRV2605_TEST_VAL;
//bool DRV2605_ONLINE = false;

extern const WaveForm_TypeDef ERM_ROM;
extern  void Motor_ReadRegs();

uint8_t VibrateBusyTime=0,VibrateRunningTimes=0,VibrateRunningDuration;
VIBRATE_MODE  VibrateRunningMode;

void DRV2605_Driver(VIBRATE_MODE act,uint8_t second);


void DRV2605_Init(void)
{
	GPIO_PinModeSet(VIBRATE_CON_PORT, VIBRATE_CON_PIN, gpioModePushPull, 0);
	//GPIO_PinModeSet(VIBRATE_TRIG_PORT, VIBRATE_TRIG_PIN, gpioModePushPull, 0);
	//GPIO_PinOutClear(VIBRATE_TRIG_PORT, VIBRATE_TRIG_PIN);


//====================================
//= 如果控制马达转动，马上复位，造成驱动死锁，即使复位也无法恢复，必须断电
//= 在 DRV2605_Driver 后加入一点延时后解决，但无法复位驱动的I2C  原因没有解决
//=
	EnableVIBRATE();
	DRV2605_ReadReg(0x00, &DRV2605_TEST_VAL);
	if((DRV2605_TEST_VAL&0x60)==0x60)
		systemStatus.blVibratorOnline=true;
	
    systemStatus.blVibratorOnline=true;  
    DRV2605_WriteReg(MODE_REG, 0X80); //reset the chip
	vTaskDelay(1);
			
	DRV2605_WriteReg(0X02, 0X00); //RTP
	DRV2605_WriteReg(WAVEFORM_SEQUENCER_REG1, 0X01);
	DRV2605_WriteReg(WAVEFORM_SEQUENCER_REG2, 0X00);
	DRV2605_WriteReg(0X0D, 0X00); //ODT
	DRV2605_WriteReg(0X0E, 0X00); //SPT
	DRV2605_WriteReg(0X0F, 0X00); //SNT
	DRV2605_WriteReg(0X10, 0X00); //BRT
	DRV2605_WriteReg(0X13, 0X64); //A2H Vpeak maxinum

	uint8_t control1, control2, control3;
	/* Set the Default control register values, see Haptics.h to define*/
	control1 = DEFAULT_CTRL1;
	control2 = DEFAULT_CTRL2;
	control3 = DEFAULT_CTRL3;

	/* Set Control2
	 * Equivalent: control2 = [(AutoResGain | BlankingTime | IDissTime) & LRA_CTRL2] |
	 * 		[(BiDir_Input | Brake_Stabilizer) & control2] */
	control2 = (0x3F & LRA_CTRL2) | (0xC0 & control2);

	/* Set Control3
	 * Equivalent: control3 = [(ERM_OpenLoop | LRA_DriveMode | LRA_OpenLoop) & (ERM_CTRL3 | LRA_CTRL3)] |
	 * 		[(NG_Thresh | SupplyCompDis | DataFormat_RTP | nPWM_Analog) & control3] */
	control3 = (0x25 & (ERM_OpenLoop  | LRADriveMode_Once | LRA_ClosedLoop	)) | (0xDA & control3);


	//LRADefaultOpenLoopSetting = control3 & LRA_OpenLoop;	// Store LRA open-loop bit
	uint8_t ERM_AutoCal_FB = 0x36;

	uint8_t ERM_RatedVoltage = Voltage_2p7;
	uint8_t ERM_ODClamp = Voltage_3p3;

	DRV2605_WriteReg(0x16, ERM_RatedVoltage);
	DRV2605_WriteReg(0x17, ERM_ODClamp);
	DRV2605_WriteReg(0x1a, ERM_AutoCal_FB);
	DRV2605_WriteReg(0x1b, control1);
	DRV2605_WriteReg(0x1c, control2);
	DRV2605_WriteReg(0x1d, control3);
	DRV2605_WriteReg(MODE_REG, 0x07);
	DRV2605_WriteReg(0X1E, 0X30);
	VibrateCon(BuzzAlert1000ms,1,1);
	
   
}

void CheckVibrateStatus(void)
{
	if(VibrateBusyTime)
	{
		VibrateBusyTime--;
		if(VibrateBusyTime==0)
		{
			if(VibrateRunningTimes)
			{
				VibrateRunningTimes--;
				DRV2605_Driver(VibrateRunningMode,VibrateRunningDuration);
			}
			else
				DisableVIBRATE();
		}
	}
}

void DRV2605_Driver(VIBRATE_MODE act, uint8_t second)
{
	EnableVIBRATE();
	DRV2605_WriteReg(LIBRARY_SELECTION_REG, LIBRARY_A);
	DRV2605_WriteReg(WAVEFORM_SEQUENCER_REG1, act);
	DRV2605_WriteReg(MODE_REG, MODE_INTERNAL_TRIGGER);
	DRV2605_WriteReg(GO_REG, GO);	
	VibrateBusyTime=second;	
	SysCtlDelay(2000);//vTaskDelay(1);
}


void VibrateCon(VIBRATE_MODE act, uint8_t duration,uint8_t times)
{
	VibrateRunningMode=act;
	if(times==0)
		times=1;
	VibrateRunningTimes=times-1;
	VibrateRunningDuration=duration;
	DRV2605_Driver(VibrateRunningMode,VibrateRunningDuration);
}

void stopVibrate()
{
	VibrateRunningTimes = 0;
}

unsigned char DRV2605_ReadReg(unsigned char Reg, unsigned char* Data)
{
	unsigned char ret;

	ret = I2CReadNByte(TMP_I2C, DRV2605_I2C_ADD, I2C_SUBA_ONEBYTE, Reg, Data, 1);

	if (ret == i2cTransferDone)
	{
		return 0;
	}
	else
	{
		return 0xFF;
	}
}


unsigned char DRV2605_WriteReg(unsigned char WriteAddr, unsigned char Data)
{
	unsigned char ret;

	if(systemStatus.blVibratorOnline==false)
			return 0xff;
	
	ret = I2CWriteNByte(TMP_I2C, DRV2605_I2C_ADD, I2C_SUBA_ONEBYTE, WriteAddr, &Data, 1);

	if (ret == i2cTransferDone)
	{
		return 0;
	}
	else
	{
		//while(1);
		return 0xff;
	}
}

void VibrateCallback(void)
{

}


