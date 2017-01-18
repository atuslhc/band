#include "FreeRTOS.h"
#include "LPS35HW.h"
#include "I2c_driver.h"
#include "em_gpio.h"
#include "task.h"
#include "GlobalData.h"
#include "common_vars.h"
#include "main.h"

#if (BAROMETER_SUPPORT==2) //[BG037]

//extern volatile I2C_TransferReturn_TypeDef I2C0_Status;
extern uint32_t I2C0_error_count, I2C1_error_count;

uint8_t TempByf; 
uint16_t PressureByf;



//int LPS35HW_I2CReadNByte_GetRegData(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);
int LPS35HW_WriteReg(uint8_t WriteAddr, uint8_t Data);
//int LPS35HW_ReadReg(BYTE Reg, BYTE* nonused);
int LPS35HW_ReadReg(uint8_t Reg, uint8_t* data);

int LPS35HWBlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values);
int LPS35HWBlockRead(uint8_t address, uint8_t length, uint8_t* values);

void LPS35HW_IIC_Init(void)
{
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	uint8_t i = 0;

	if(I2C0_used == false)
	{
		I2C0_used = true;

		CMU_ClockEnable(cmuClock_GPIO, true);
		CMU_ClockEnable(LPS35HW_I2C_cmuClock_I2C, true);
		GPIO_PinModeSet(LPS35HW_I2C_gpioPort, LPS35HW_I2C_SCL_PIN, gpioModePushPull, 1);
		GPIO_PinModeSet(LPS35HW_I2C_gpioPort, LPS35HW_I2C_SDA_PIN, gpioModePushPull, 1); //

		/*������������ʹIIC�����ȶ�*/
		for (i = 0; i < 9; i++)
		{
			GPIO_PinModeSet(LPS35HW_I2C_gpioPort, LPS35HW_I2C_SCL_PIN, gpioModeWiredAnd, 0);
			GPIO_PinModeSet(LPS35HW_I2C_gpioPort, LPS35HW_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		}

		/*ȷ��IIC���������Ŷ������*/
		GPIO_PinModeSet(LPS35HW_I2C_gpioPort, LPS35HW_I2C_SCL_PIN, gpioModeWiredAnd, 1);//���ó�����(Ҳ���ǿ�©) ������1
		GPIO_PinModeSet(LPS35HW_I2C_gpioPort, LPS35HW_I2C_SDA_PIN, gpioModeWiredAnd, 1);

		/* Enable pins at location 3*/
		LPS35HW_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | LPS35HW_I2C_LOC;
		I2C_Init(LPS35HW_I2C, &i2cInit);//��һ��������ָ�룬��������ѱ���ȡ��ַ���롣

		/* Clear and enable interrupt from I2C module */
		NVIC_ClearPendingIRQ(LPS35HW_I2C_IRQn);
		NVIC_EnableIRQ(LPS35HW_I2C_IRQn);
	}
}

/*���ջ�ȡ��������data�д洢*/
int LPS35HW_ReadReg(uint8_t Reg, uint8_t* data)
{
	uint8_t ret;


	// =============================================================
	// ���i2c�ź���
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // ��ȡ�ź���ʧ��

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return i2cTransferSwFault;

	// =============================================================
	ret = I2CReadNByte(LPS35HW_I2C, LPS35HW_IIC_ADDR, I2C_SUBA_ONEBYTE, Reg, data, 1);


	// =============================================================
	// �ͷ�I2C�ź���
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

/*д�ɹ�����0��ʧ�ܷ���0xff*/
int LPS35HW_WriteReg(uint8_t WriteAddr, uint8_t Data)
{
	int ret;


	// =============================================================
	// ���i2c�ź���
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // ��ȡ�ź���ʧ��

	if (xSemaphoreTake(hI2CSemaphore, 20) != pdTRUE)
		return i2cTransferSwFault;


	// =============================================================
	ret = I2CWriteNByte(LPS35HW_I2C, LPS35HW_IIC_ADDR, I2C_SUBA_ONEBYTE, WriteAddr, &Data, 1);


	// =============================================================
	// �ͷ�I2C�ź���
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

//��д�룬�������ò���
int LPS35HWBlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values)
{
	uint8_t  retval, counter;

	for ( counter = 0; counter < length; counter++)
	{
		retval = LPS35HW_WriteReg(address + counter, values[counter]);
	}

	return retval;
}


int LPS35HWBlockRead(uint8_t address, uint8_t length, uint8_t* values)
{
	uint8_t counter;
	//   BYTE nonused = 0;

	for ( counter = 0; counter < length; counter++)
	{
		//��������ݷ���value�����С�
		LPS35HW_ReadReg(address + counter, values);
		values++;
	}

	return 0;
}


/***************************************����UVУ����һЩ����********************************************/



void LPS35HW_RST(void)
{

	//SysCtlDelay(1000);
	//LPS35HW_WriteReg(REG_IRQ_STATUS, 0xff);

	//SysCtlDelay(1000);

}


void LPS35HW_Init(void)
{
	uint8_t nonused;

	LPS35HW_IIC_Init();//�����Sensor��mcuͨ�ŵĳ�ʼ��

	LPS35HW_ReadReg(LPS35HW_WHO_AM_I, &nonused); //��ȡID

	systemStatus.blPressureSensorOnline = false;///????

	if(nonused != 0xB1)//��������������ȷ�Ķ����˸�оƬ��ID
		return;

	systemStatus.blPressureSensorOnline = true;


	LPS35HW_RST();//�����λ


}


uint8_t GeLPS35HW_Temperature(void)
{
	uint8_t t8_l=0, t8_h=0; 
	uint16_t ret = 0; 

	if(systemStatus.blPressureSensorOnline == false)
		return 0;

	ret |= LPS35HW_ReadReg(LPS35HW_TEMP_OUT_L, &t8_l); 
	ret |= LPS35HW_ReadReg(LPS35HW_TEMP_OUT_H, &t8_h);
	//Atus: [BG023-2] should check read fail condition.
	if (ret != i2cTransferDone) //[BG023-2] add.
	{
	 	return 0;//bUltraViolet; //return last value.
	}

	ret = (t8_h * 256 + t8_l) / Tsens;

	return (uint8_t)ret;
}

uint16_t GetLPS35HW_Pressure(void)
{
	uint8_t t8_xl, t8_l, t8_h;
	uint32_t ret = 0;

	if(systemStatus.blPressureSensorOnline == false)
		return 0;

	LPS35HW_ReadReg(LPS35HW_PRESS_OUT_XL , &t8_xl);
	LPS35HW_ReadReg(LPS35HW_PRESS_OUT_L , &t8_l);
	LPS35HW_ReadReg(LPS35HW_PRESS_OUT_H , &t8_h);

	ret = (t8_h * 0x10000 + t8_l * 0x100 + t8_xl ) / Psens;

	return (uint16_t)ret;


}

void LPS35HW_start_conversion(void)
{
	uint8_t ret = 0; 
	
	if(systemStatus.blPressureSensorOnline == false)
		return;
	
	LPS35HW_ReadReg(LPS35HW_CTRL_REG2, &ret);
	LPS35HW_WriteReg(LPS35HW_CTRL_REG2, ret|ONE_SHOT);

}

	
void LPS35HW_Read_converter(void)
{
	uint8_t ret = 0; 
	
	TempByf = 0;
	PressureByf = 0;
	
	if(systemStatus.blPressureSensorOnline == false)
		return;
	
	//SysCtlDelay(1000);
	LPS35HW_ReadReg(LPS35HW_STATUS, &ret);
	// if((ret&T_OR) &&(ret&T_DA))
	if(ret&T_DA)
		TempByf =GeLPS35HW_Temperature();
	// if((ret&P_OR) &&(ret&P_DA))
	if(ret&P_DA)
		PressureByf =GetLPS35HW_Pressure();
	
}
#endif
