#include "FreeRTOS.h"
#include "LPS22HB.h"
#include "I2c_driver.h"
#include "em_gpio.h"
#include "task.h"
#include "GlobalData.h"
#include "common_vars.h"
#include "main.h"

#if (BAROMETER_SUPPORT==1) //[BG037]

//extern volatile I2C_TransferReturn_TypeDef I2C0_Status;
extern uint32_t I2C0_error_count, I2C1_error_count;

uint8_t TempByf; 
uint16_t PressureByf;



//int LPS22HB_I2CReadNByte_GetRegData(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);
int LPS22HB_WriteReg(uint8_t WriteAddr, uint8_t Data);
//int LPS22HB_ReadReg(BYTE Reg, BYTE* nonused);
int LPS22HB_ReadReg(uint8_t Reg, uint8_t* data);

int LPS22HBBlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values);
int LPS22HBBlockRead(uint8_t address, uint8_t length, uint8_t* values);

void LPS22HB_IIC_Init(void)
{
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	uint8_t i = 0;

	if(I2C0_used == false)
	{
		I2C0_used = true;

		CMU_ClockEnable(cmuClock_GPIO, true);
		CMU_ClockEnable(LPS22HB_I2C_cmuClock_I2C, true);
		GPIO_PinModeSet(LPS22HB_I2C_gpioPort, LPS22HB_I2C_SCL_PIN, gpioModePushPull, 1);
		GPIO_PinModeSet(LPS22HB_I2C_gpioPort, LPS22HB_I2C_SDA_PIN, gpioModePushPull, 1); //

		/*产生几个脉冲使IIC总线稳定*/
		for (i = 0; i < 9; i++)
		{
			GPIO_PinModeSet(LPS22HB_I2C_gpioPort, LPS22HB_I2C_SCL_PIN, gpioModeWiredAnd, 0);
			GPIO_PinModeSet(LPS22HB_I2C_gpioPort, LPS22HB_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		}

		/*确保IIC的两个引脚都输出高*/
		GPIO_PinModeSet(LPS22HB_I2C_gpioPort, LPS22HB_I2C_SCL_PIN, gpioModeWiredAnd, 1);//配置成线与(也就是开漏) 数据是1
		GPIO_PinModeSet(LPS22HB_I2C_gpioPort, LPS22HB_I2C_SDA_PIN, gpioModeWiredAnd, 1);

		/* Enable pins at location 3*/
		LPS22HB_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | LPS22HB_I2C_LOC;
		I2C_Init(LPS22HB_I2C, &i2cInit);//后一个参数是指针，所以这里把变量取地址传入。

		/* Clear and enable interrupt from I2C module */
		NVIC_ClearPendingIRQ(LPS22HB_I2C_IRQn);
		NVIC_EnableIRQ(LPS22HB_I2C_IRQn);
	}
}

/*最终获取的数据在data中存储*/
int LPS22HB_ReadReg(uint8_t Reg, uint8_t* data)
{
	uint8_t ret;


	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return i2cTransferSwFault;

	// =============================================================
	ret = I2CReadNByte(LPS22HB_I2C, LPS22HB_IIC_ADDR, I2C_SUBA_ONEBYTE, Reg, data, 1);


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
int LPS22HB_WriteReg(uint8_t WriteAddr, uint8_t Data)
{
	int ret;


	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != pdTRUE)
		return i2cTransferSwFault;


	// =============================================================
	ret = I2CWriteNByte(LPS22HB_I2C, LPS22HB_IIC_ADDR, I2C_SUBA_ONEBYTE, WriteAddr, &Data, 1);


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
int LPS22HBBlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values)
{
	uint8_t  retval, counter;

	for ( counter = 0; counter < length; counter++)
	{
		retval = LPS22HB_WriteReg(address + counter, values[counter]);
	}

	return retval;
}


int LPS22HBBlockRead(uint8_t address, uint8_t length, uint8_t* values)
{
	uint8_t counter;
	//   BYTE nonused = 0;

	for ( counter = 0; counter < length; counter++)
	{
		//这里把数据放在value数组中。
		LPS22HB_ReadReg(address + counter, values);
		values++;
	}

	return 0;
}


/***************************************关于UV校正的一些函数********************************************/



void LPS22HB_RST(void)
{
	uint8_t t8_l=0, t8_h=0; 

    LPS22HB_ReadReg(LPS22HB_CTRL_REG1, &t8_l);
    LPS22HB_ReadReg(LPS22HB_CTRL_REG2, &t8_h);
    
	LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 0x04);
    LPS22HB_ReadReg(LPS22HB_CTRL_REG2, &t8_h);

	SysCtlDelay(1000);
    LPS22HB_ReadReg(LPS22HB_CTRL_REG2, &t8_h);
    
    LPS22HB_ReadReg(LPS22HB_CTRL_REG1, &t8_h);

}

void LPS22HB_Disabled(void)
{
    if (systemStatus.blPressureSensorOnline==0x01)
    {
      LPS22HB_RST();
      systemStatus.blPressureSensorOnline = false;
    }
    GPIO_PinModeSet(PRES_INT_PORT, PRES_INT_PIN, gpioModeDisabled, 0); /* PRES_INT */

}

int LPS22HB_Init(uint8_t mode)
{
	uint8_t nonused;

	LPS22HB_IIC_Init();

	LPS22HB_ReadReg(LPS22HB_WHO_AM_I, &nonused); 

	systemStatus.blPressureSensorOnline = false;

	if(nonused != LPS22HB_PART_ID)// Device id 0xB1
		return DEVICE_NOTEXIST;

	systemStatus.blPressureSensorOnline = true;

	LPS22HB_RST();//
    
    if (mode==0)
    {
      LPS22HB_Disabled();
    }

    return DEVICE_SUCCESS;
}


uint8_t GeLPS22HB_Temperature(void)
{
	uint8_t t8_l=0, t8_h=0; 
	uint16_t ret = 0; 

	if(systemStatus.blPressureSensorOnline == false)
		return 0;

	ret |= LPS22HB_ReadReg(LPS22HB_TEMP_OUT_L, &t8_l); 
	ret |= LPS22HB_ReadReg(LPS22HB_TEMP_OUT_H, &t8_h);
	//Atus: [BG023-2] should check read fail condition.
	if (ret != i2cTransferDone) //[BG023-2] add.
	{
	 	return 0; //return last value.
	}

	ret = (t8_h * 256 + t8_l) / Tsens;

	return (uint8_t)ret;
}

uint16_t GetLPS22HB_Pressure(void)
{
	uint8_t t8_xl, t8_l, t8_h;
	uint32_t ret = 0;

	if(systemStatus.blPressureSensorOnline == false)
		return 0;

	LPS22HB_ReadReg(LPS22HB_PRESS_OUT_XL , &t8_xl);
	LPS22HB_ReadReg(LPS22HB_PRESS_OUT_L , &t8_l);
	LPS22HB_ReadReg(LPS22HB_PRESS_OUT_H , &t8_h);

	ret = (t8_h * 0x10000 + t8_l * 0x100 + t8_xl ) / Psens;

	return (uint16_t)ret;


}

void LPS22HB_start_conversion(void)
{
	uint8_t ret = 0; 
	
	if(systemStatus.blPressureSensorOnline == false)
		return;
	
	LPS22HB_ReadReg(LPS22HB_CTRL_REG2, &ret);
	LPS22HB_WriteReg(LPS22HB_CTRL_REG2, ret|ONE_SHOT);

}

	
void LPS22HB_Read_converter(void)
{
	uint8_t ret = 0; 
	
	TempByf = 0;
	PressureByf = 0;
	
	if(systemStatus.blPressureSensorOnline == false)
		return;
	
	//SysCtlDelay(1000);
	LPS22HB_ReadReg(LPS22HB_STATUS, &ret);
	// if((ret&T_OR) &&(ret&T_DA))
	if(ret&T_DA)
		TempByf =GeLPS22HB_Temperature();
	// if((ret&P_OR) &&(ret&P_DA))
	if(ret&P_DA)
		PressureByf =GetLPS22HB_Pressure();
	
}
#endif
