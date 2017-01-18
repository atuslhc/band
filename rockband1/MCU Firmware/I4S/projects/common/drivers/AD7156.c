#include "FreeRTOS.h"
#include "AD7156.h"
#include "I2c_driver.h"
#include "em_gpio.h"
#include "task.h"
#include "GlobalData.h"
#include "common_vars.h"
#include "main.h"

#if (CAP_SUPPORT==2) //[BG037]

//extern volatile I2C_TransferReturn_TypeDef I2C0_Status;
extern uint32_t I2C0_error_count, I2C1_error_count;

float ad7156Channel1Range = 2;
float ad7156Channel2Range = 2;

//int AD7156_I2CReadNByte_GetRegData(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);
int AD7156_WriteReg(uint8_t WriteAddr, uint8_t Data);
//int AD7156_ReadReg(BYTE Reg, BYTE* nonused);
int AD7156_ReadReg(uint8_t Reg, uint8_t* data);

int AD7156BlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values);
int AD7156BlockRead(uint8_t address, uint8_t length, uint8_t* values);

void AD7156_IIC_Init(void)
{
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	uint8_t i = 0;

	if(I2C0_used == false)
	{
		I2C0_used = true;

		CMU_ClockEnable(cmuClock_GPIO, true);
		CMU_ClockEnable(AD7156_I2C_cmuClock_I2C, true);
		GPIO_PinModeSet(AD7156_I2C_gpioPort, AD7156_I2C_SCL_PIN, gpioModePushPull, 1);
		GPIO_PinModeSet(AD7156_I2C_gpioPort, AD7156_I2C_SDA_PIN, gpioModePushPull, 1); //

		/*产生几个脉冲使IIC总线稳定*/
		for (i = 0; i < 9; i++)
		{
			GPIO_PinModeSet(AD7156_I2C_gpioPort, AD7156_I2C_SCL_PIN, gpioModeWiredAnd, 0);
			GPIO_PinModeSet(AD7156_I2C_gpioPort, AD7156_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		}

		/*确保IIC的两个引脚都输出高*/
		GPIO_PinModeSet(AD7156_I2C_gpioPort, AD7156_I2C_SCL_PIN, gpioModeWiredAnd, 1);//配置成线与(也就是开漏) 数据是1
		GPIO_PinModeSet(AD7156_I2C_gpioPort, AD7156_I2C_SDA_PIN, gpioModeWiredAnd, 1);

		/* Enable pins at location 3*/
		AD7156_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | AD7156_I2C_LOC;
		I2C_Init(AD7156_I2C, &i2cInit);//后一个参数是指针，所以这里把变量取地址传入。

		/* Clear and enable interrupt from I2C module */
		NVIC_ClearPendingIRQ(AD7156_I2C_IRQn);
		NVIC_EnableIRQ(AD7156_I2C_IRQn);
	}
}

/*最终获取的数据在data中存储*/
int AD7156_ReadReg(uint8_t Reg, uint8_t* data)
{
	uint8_t ret;


	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return i2cTransferSwFault;

	// =============================================================
	ret = I2CReadNByte(AD7156_I2C, AD7156_IIC_ADDR, I2C_SUBA_ONEBYTE, Reg, data, 1);


	// =============================================================
	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);

	xSemaphoreGive(hI2CSemaphore);


	// =============================================================
	//
	if(ret == i2cTransferDone)
	{
		return i2cTransferDone; //0;
	}
	else
	{
		return 0xff;
	}
}

/*写成功返回0，失败返回0xff*/
int AD7156_WriteReg(uint8_t WriteAddr, uint8_t Data)
{
	int ret;


	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != pdTRUE)
		return i2cTransferSwFault;


	// =============================================================
	ret = I2CWriteNByte(AD7156_I2C, AD7156_IIC_ADDR, I2C_SUBA_ONEBYTE, WriteAddr, &Data, 1);


	// =============================================================
	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);
	xSemaphoreGive(hI2CSemaphore);

	// =========================================
	if (ret == i2cTransferDone)
	{
		return 0;
	}
	else
	{
		return 0xff;
	}

}

//块写入，用于配置参数
int AD7156_BlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values)
{
	uint8_t  retval, counter;

	for ( counter = 0; counter < length; counter++)
	{
		retval = AD7156_WriteReg(address + counter, values[counter]);
	}

	return retval;
}


int AD7156_BlockRead(uint8_t address, uint8_t length, uint8_t* values)
{
	uint8_t counter;
	//   BYTE nonused = 0;

	for ( counter = 0; counter < length; counter++)
	{
		//这里把数据放在value数组中。
		AD7156_ReadReg(address + counter, values);
		values++;
	}

	return 0;
}



void AD7156_Reset(void)
{
	//SysCtlDelay(1000);
	AD7156_WriteReg(AD7156_RESET, 0x1);
	SysCtlDelay(3000);  // 2ms at least.append 1ms for safe.
}


void AD7156_Init(void)
{
	uint8_t data;

	AD7156_IIC_Init();

	AD7156_ReadReg(AD7156_CHIP_ID, &data); 

//	systemStatus.blCAPSensorOnline = false;

	if(data != AD7156_PART_ID)// Device id
		return;

//	systemStatus.blCAPSensorOnline = true;


	AD7156_Reset();//
}


/*******************************************************************************
 * @brief Sets the converter mode of operation.
 *
 * @oaram pwrMode - Mode of operation option.
 *		    Example: AD7156_CONV_MODE_IDLE - Idle
 *                           AD7156_CONV_MODE_CONT_CONV  - Continuous conversion
 *                           AD7156_CONV_MODE_SINGLE_CONV - Single conversion
 *                           AD7156_CONV_MODE_PWR_DWN - Power-down
 *
 * @return None.
*******************************************************************************/
void AD7156_SetPowerMode(uint8_t pwrMode)
{
    unsigned char oldConfigReg = 0;
    unsigned char newConfigReg = 0;

    AD7156_ReadReg(AD7156_CONFIG, &oldConfigReg);
    oldConfigReg &= ~AD7156_CONFIG_MD(0x3);
    newConfigReg = oldConfigReg| AD7156_CONFIG_MD(pwrMode);
    AD7156_WriteReg(AD7156_CONFIG, newConfigReg); 
}

/*******************************************************************************
 * @brief Enables or disables conversion on the selected channel.
 *
 * @param channel    - Channel option.
 *                      Example: AD7156_CHANNEL1
 *                               AD7156_CHANNEL2
 * @param enableConv - The state of channel activity.
 *                      Example: 0 - disable conversion on selected channel.
 *                               1 - enable conversion on selected channel.
 *
 * @return None.
*******************************************************************************/
void AD7156_ChannelState(uint8_t channel, uint8_t enableConv)
{
    unsigned char oldConfigReg = 0;
    unsigned char newConfigReg = 0;
    unsigned char channelMask  = 0;
    
    channelMask = (channel == 1) ? AD7156_CONFIG_EN_CH1 : AD7156_CONFIG_EN_CH2;

    AD7156_ReadReg(AD7156_CONFIG, &oldConfigReg);
    oldConfigReg &= ~channelMask;
    newConfigReg = oldConfigReg | (channelMask * enableConv);
    AD7156_WriteReg(AD7156_CONFIG, newConfigReg);
}

/*******************************************************************************
 * @brief Sets the input range of the specified channel.
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2 
 * @param range   - Input range option.
 *                  Example: AD7156_CDC_RANGE_2_PF   - 2pF input range.
 *                           AD7156_CDC_RANGE_0_5_PF - 0.5pF input range.
 *                           AD7156_CDC_RANGE_1_PF   - 1pF input range.
 *                           AD7156_CDC_RANGE_4_PF   - 4pF input range.
 *
 * @return None.
*******************************************************************************/
void AD7156_SetRange(unsigned char channel, unsigned char range)
{
    unsigned char oldSetupReg = 0;
    unsigned char newSetupReg = 0;
    unsigned char regAddress  = 0;

    regAddress = (channel == 1) ? AD7156_CH1_SETUP : AD7156_CH2_SETUP;
    AD7156_ReadReg(regAddress, &oldSetupReg);
    oldSetupReg &= ~AD7156_CH1_SETUP_RANGE(0x3);
    newSetupReg = oldSetupReg | AD7156_CH1_SETUP_RANGE(range);
    AD7156_WriteReg(regAddress, newSetupReg);
    /* Update global variables that hold range information. */
    if(channel == 1)
    {
        ad7156Channel1Range = AD7156_GetRange(channel);
    }
    else
    {
        ad7156Channel2Range = AD7156_GetRange(channel);
    }
}

/***************************************************************************//**
 * @brief Reads the range bits from the device and returns the range in pF.
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2 
 *
 * @return The capacitive input range(pF).
*******************************************************************************/
float AD7156_GetRange(unsigned char channel)
{
    unsigned char setupReg    = 0;
    unsigned char regAddress  = 0;
    float range = 0;
    
    regAddress = (channel == 1) ? AD7156_CH1_SETUP : AD7156_CH2_SETUP;
    AD7156_ReadReg(regAddress, &setupReg);
    setupReg = (setupReg & AD7156_CH1_SETUP_RANGE(0x3)) >> 6;
    switch(setupReg)
    {
        case AD7156_CDC_RANGE_2_PF:
            range =  2.0;
            break;
        case AD7156_CDC_RANGE_0_5_PF:
            range = 0.5;
            break;
        case AD7156_CDC_RANGE_1_PF:
            range =  1.0;
            break;
        case AD7156_CDC_RANGE_4_PF:
            range =  4.0;
            break;
    }
    /* Update global variables that hold range information. */
    if(channel == 1)
    {
        ad7156Channel1Range = range;
    }
    else
    {
        ad7156Channel2Range = range;
    }

    return range;
}
/***************************************************************************//**
 * @brief Selects the threshold mode of operation.
 *
 * @param thrMode  - Output comparator mode.
 *                   Example: AD7156_THR_MODE_NEGATIVE
 *                            AD7156_THR_MODE_POSITIVE
 *                            AD7156_THR_MODE_IN_WINDOW
 *                            AD7156_THR_MODE_OU_WINDOW
 * @param thrFixed - Selects the threshold mode.
 *                   Example: AD7156_ADAPTIVE_THRESHOLD
 *                            AD7156_FIXED_THRESHOLD
 *
 * @return None.
*******************************************************************************/
void AD7156_SetThrMode(unsigned char thrMode, unsigned char thrFixed)
{
    unsigned char oldConfigReg = 0;
    unsigned char newConfigReg = 0;

    AD7156_ReadReg(AD7156_CONFIG, &oldConfigReg);
    oldConfigReg &= ~(AD7156_CONFIG_THR_FIXED | AD7156_CONFIG_THR_MD(0x3));
    newConfigReg = oldConfigReg |
                   (AD7156_CONFIG_THR_FIXED * thrFixed) |
                   (AD7156_CONFIG_THR_MD(thrMode));
    AD7156_WriteReg(AD7156_CONFIG, newConfigReg);
}

/***************************************************************************//**
 * @brief Writes to the threshold register when threshold fixed mode is enabled.
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param pFthr   - The threshold value in picofarads(pF). The value must not be
 *                  out of the selected input range.
 *
 * @return None.
*******************************************************************************/
void AD7156_SetThreshold(unsigned char channel, float pFthr)
{
    unsigned char  thrRegAddress  = 0;
    unsigned short rawThr         = 0;
    float  range                  = 0;

    thrRegAddress = (channel == 1) ? AD7156_CH1_THR_H :
                                     AD7156_CH2_THR_H;
    range = AD7156_GetRange(channel);
    rawThr = (unsigned short)((pFthr * 0xA000 / range) + 0x3000);
    if(rawThr > 0xD000)
    {
        rawThr = 0xD000;
    }
    else if(rawThr < 0x3000)
    {
        rawThr = 0x3000;
    }
    AD7156_BlockWrite(thrRegAddress, 2, (uint8_t*) &rawThr);
}

/***************************************************************************//**
 * @brief Writes a value(pF) to the sensitivity register. This functions 
 * should be used when adaptive threshold mode is selected.
 *
 * @param channel       - Channel option.
 *                        Example: AD7156_CHANNEL1
 *                                 AD7156_CHANNEL2
 * @param pFsensitivity - The sensitivity value in picofarads(pF).
 *
 * @return None.
*******************************************************************************/
void AD7156_SetSensitivity(unsigned char channel, float pFsensitivity)
{
    unsigned char  sensitivityRegAddr = 0;
    unsigned short rawSensitivity     = 0;
    float range = 0;

    sensitivityRegAddr = (channel == 1) ? AD7156_CH1_SENS : AD7156_CH2_SENS;
    range = (channel == 1) ? ad7156Channel1Range : ad7156Channel2Range;
    rawSensitivity = (unsigned short)(pFsensitivity * 0xA00 / range);
    rawSensitivity = (rawSensitivity << 4) & 0x0FF0;
    AD7156_BlockWrite(sensitivityRegAddr, 2, (uint8_t *)&rawSensitivity);
}

/***************************************************************************//**
 * @brief Reads a 12-bit sample from the selected channel.
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @return Conversion result from the selected channel.
*******************************************************************************/
unsigned short AD7156_ReadChData(unsigned char channel)
{
    unsigned short chResult   = 0;
    unsigned char  regData[2] = {0, 0};
    unsigned char  chAddress  = 0;

    if(channel == AD7156_CHANNEL1)
    {
        chAddress = AD7156_CH1_DATA_H;
    }
    else
    {
        chAddress = AD7156_CH2_DATA_H;
    }
    AD7156_BlockRead(chAddress, 2, regData);
    chResult = (regData[0] << 8) + regData[1];

    return chResult;
}

uint16_t GetAD7156_Ch1(void)
{
//	if(systemStatus.blCapSensorOnline == false)
//		return 0;

	return AD7156_ReadChData(AD7156_CHANNEL1);
}

uint16_t GetAD7156_Ch2(void)
{
//	if(systemStatus.blCapSensorOnline == false)
//		return 0;

	return AD7156_ReadChData(AD7156_CHANNEL2);
}

/***************************************************************************//**
 * @brief Waits for a finished CDC conversion and reads a 12-bit sample from
 *        the selected channel.
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @return Conversion result form the selected channel.
*******************************************************************************/
unsigned short AD7156_WaitReadChData(unsigned char channel)
{
    unsigned short chResult   = 0;
    unsigned char  regData[2] = {0, 0};
    unsigned char  status     = 0;
    unsigned char  chRdyMask  = 0;
    unsigned char  chAddress  = 0;

    if(channel == 1)
    {
        chRdyMask = AD7156_STATUS_RDY1;
        chAddress = AD7156_CH1_DATA_H;
    }
    else
    {
        chRdyMask = AD7156_STATUS_RDY2;
        chAddress = AD7156_CH2_DATA_H;
    }
    do
    {
        AD7156_ReadReg(AD7156_STATUS, &status);
    }while((status & chRdyMask) != 0);
    AD7156_BlockRead(chAddress, 2, regData);
    chResult = (regData[0] << 8) + regData[1];

    return chResult;
}

/***************************************************************************//**
 * @brief Reads a sample the selected channel and converts the data to
 *        picofarads(pF).
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @return Conversion result form the selected channel as picofarads(pF).
*******************************************************************************/
float AD7156_ReadChCapacitance(unsigned char channel)
{
    unsigned short rawCh = 0;
    float chRange = 0;
    float pFdata = 0;

    chRange = (channel == AD7156_CHANNEL1) ? ad7156Channel1Range : ad7156Channel2Range;
    rawCh = AD7156_ReadChData(channel);
    if(rawCh < 0x3000)
    {
        rawCh= 0x3000;
    }
    else if(rawCh > 0xD000)
    {
        rawCh = 0xD000;
    }
    pFdata = (((rawCh) - 0x3000) * chRange) / 0xA000;

    return pFdata;
}

/***************************************************************************//**
 * @brief Waits for a finished CDC conversion the selected channel, reads a
 *        sample and converts the data to picofarads(pF).
 *
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @return Conversion result form the selected channel as picofarads(pF).
*******************************************************************************/
float AD7156_WaitReadChCapacitance(unsigned char channel)
{
    unsigned short rawCh = 0;
    float chRange = 0;
    float pFdata = 0;

    chRange = (channel == AD7156_CHANNEL1) ? ad7156Channel1Range : ad7156Channel2Range;
    rawCh = AD7156_WaitReadChData(channel);
    if(rawCh < 0x3000)
    {
        rawCh= 0x3000;
    }
    else if(rawCh > 0xD000)
    {
        rawCh = 0xD000;
    }
    pFdata = (((rawCh) - 0x3000) * chRange) / 0xA000;

    return pFdata;
}

#endif
