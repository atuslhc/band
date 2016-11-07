#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "common_vars.h"
#include "main.h"

#include "MCP9804.h"

//bool AMB_TMP_ONLINE=0;
//bool SKIN_TMP_ONLINE=0;

uint8_t AMB_TMP[2], SKIN_TMP[2];


/*******************************************************************************
 **************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/
extern I2C_TransferReturn_TypeDef I2C1_Status;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/



/***************************************************************************//**
 * @brief
 *   Initalize basic I2C master mode driver for use on the DVK.
 *
 * @details
 *   This driver only supports master mode, single bus-master. In addition
 *   to configuring the EFM32 I2C peripheral module, it also configures DVK
 *   specific setup in order to use the I2C bus.
 *
 * @param[in] init
 *   Pointer to I2C initialization structure.
 ******************************************************************************/
uint16_t test_val;

void Pre_INIT_I2C1(void)
{

	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	CMU_ClockEnable(TMP_I2C_cmuClock_I2C, true);

	/* Output value must be set to 1 to not drive lines low... We set */
	/* SCL first, to ensure it is high before changing SDA. */
	GPIO_PinModeSet(TMP_I2C_gpioPort, TMP_I2C_SCL_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(TMP_I2C_gpioPort, TMP_I2C_SDA_PIN, gpioModeWiredAnd, 1);

	/* In some situations (after a reset during an I2C transfer), the slave */
	/* device may be left in an unknown state. Send 9 clock pulses just in case. */
	for (int i = 0; i < 9; i++)
	{
		/*
		 * TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
		 * optimization when running at 14MHz. A bit high for standard mode devices,
		 * but DVK only has fast mode devices. Need however to add some time
		 * measurement in order to not be dependable on frequency and code executed.
		 */
		GPIO_PinModeSet(TMP_I2C_gpioPort, TMP_I2C_SCL_PIN, gpioModeWiredAnd, 0);
		SysCtlDelay(SYSCLOCK * 10);
		GPIO_PinModeSet(TMP_I2C_gpioPort, TMP_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		SysCtlDelay(SYSCLOCK * 10);
	}

	/* Enable pins at location 2 */
	TMP_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | TMP_I2C_LOC;
	I2C_Init(TMP_I2C, &i2cInit);

	/* Clear and enable interrupt from I2C module */
	NVIC_ClearPendingIRQ(I2C1_IRQn);
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn, I2C1_IRQn_Level);

}

void MCP9804_Init(void)
{

	systemStatus.blAmbTempSensorOnline = 0;
	systemStatus.blSkinTempSensorOnline = 0;

	TEMPSENS_RegisterGet(TMP_I2C, AMB_TEMP_ADDR, tempsensRegMFTID, &test_val);

	if(test_val == MCP9804_DEV_ID)
	{
		systemStatus.blAmbTempSensorOnline = 1;
		TEMPSENS_RegisterSet(TMP_I2C, AMB_TEMP_ADDR, tempsensRegRES, 0x01); //0.25C
		TEMPSENS_RegisterSet(TMP_I2C, AMB_TEMP_ADDR, tempsensRegCONFIG, 0x0100); //close
	}

	SKIN_TEMP_INIT();

}

void SKIN_TEMP_INIT(void)
{
	test_val = 0;
	TEMPSENS_RegisterGet(TMP_I2C, SKIN_TEMP_ADDR, tempsensRegMFTID, &test_val);

	if(test_val == MCP9804_DEV_ID)
	{
		systemStatus.blSkinTempSensorOnline = 1;
		TEMPSENS_RegisterSet(TMP_I2C, SKIN_TEMP_ADDR, tempsensRegRES, 0x01); // res=0.25C
		TEMPSENS_RegisterSet(TMP_I2C, SKIN_TEMP_ADDR, tempsensRegCONFIG, 0x0100);	//close
	}
	else
		systemStatus.blSkinTempSensorOnline = 0;

}


void Start_Cap_Temp(void)
{
	if(systemSetting.blSkinTempSensorEnabled == true)
	{
		if(systemStatus.blSkinTempSensorOnline)
		{
			TEMPSENS_RegisterSet(TMP_I2C, SKIN_TEMP_ADDR, tempsensRegCONFIG, 0x0000);
			EnableDelayTimer(TIMER_FLAG_TEMP, false, 100, NULL, NULL);
		}
	}

	if( systemSetting.blAmbTempSensorEnabled == true)
	{
		if(systemStatus.blAmbTempSensorOnline)
		{
			TEMPSENS_RegisterSet(TMP_I2C, AMB_TEMP_ADDR, tempsensRegCONFIG, 0x0000);
			// will be delay 100ms (65mm) to get the temp data
			EnableDelayTimer(TIMER_FLAG_TEMP, false, 100, NULL, NULL);
		}
	}
}

void TempReadAndStop(void)
{
	DisableDelayTimer(TIMER_FLAG_TEMP);

	if(systemStatus.blSkinTempSensorOnline)
	{
		TEMPSENS_RegisterSet(TMP_I2C, SKIN_TEMP_ADDR, tempsensRegCONFIG, 0x0100); //close

		if(MCP9804_TemperatureGet(TMP_I2C, SKIN_TEMP_ADDR, SKIN_TMP) != i2cTransferDone)
			systemStatus.blSkinTempSensorOnline = 0;

	}

	if(systemStatus.blAmbTempSensorOnline)
	{
		TEMPSENS_RegisterSet(TMP_I2C, AMB_TEMP_ADDR, tempsensRegCONFIG, 0x0100); //close
		MCP9804_TemperatureGet(TMP_I2C, AMB_TEMP_ADDR, AMB_TMP);
	}
}

extern uint32_t I2C1_error_count;


int TEMPSENS_RegisterGet(I2C_TypeDef* i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t* val)
{
	I2C_TransferSeq_TypeDef seq;
	uint8_t regid[1];
	uint8_t data[2];

	seq.addr = addr;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Select register to be read */
	regid[0] = ((uint8_t)reg) & 0xf;
	seq.buf[0].data = regid;
	seq.buf[0].len = 1;
	/* Select location/length to place register */

	seq.buf[1].data = data;
	seq.buf[1].len = 2;

	I2C1_Status = I2C_TransferInit(i2c, &seq);
	I2C1_error_count = 0;

	while (I2C1_Status == i2cTransferInProgress)
	{
		EMU_EnterEM1();
		I2C1_error_count++;

		if(I2C1_error_count > 100)break;
	}



	if (I2C1_Status != i2cTransferDone)
	{
		return((int)I2C1_Status);
	}

	*val = (((uint16_t)(data[0])) << 8) | data[1];

	return(0);
}


int TEMPSENS_RegisterSet(I2C_TypeDef* i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t val)
{
	I2C_TransferSeq_TypeDef seq;
	uint8_t data[3];

	seq.addr = addr;
	seq.flags = I2C_FLAG_WRITE;
	/* Select register to be written */
	data[0] = ((uint8_t)reg) & 0xf;
	seq.buf[0].data = data;

	if(reg == tempsensRegRES)
	{
		data[1] = (uint8_t)val;
		seq.buf[0].len = 2;
	}
	else
	{
		data[1] = (uint8_t)(val >> 8);
		data[2] = (uint8_t)val;
		seq.buf[0].len = 3;
	}

	I2C1_Status = I2C_TransferInit(i2c, &seq);

	I2C1_error_count = 0;

	while (I2C1_Status == i2cTransferInProgress)
	{
		EMU_EnterEM1();
		I2C1_error_count++;

		if(I2C1_error_count > 100)break;
	}

	if (I2C1_Status != i2cTransferDone)
	{
		return((int)I2C1_Status);
	}

	return(i2cTransferDone);
}

void MCP9804_DOWN(I2C_TypeDef* i2c, uint8_t addr)
{
	TEMPSENS_RegisterSet(i2c, addr, tempsensRegCONFIG, 0x0100);
}

int MCP9804_TemperatureGet(I2C_TypeDef* i2c,
                           uint8_t addr,
                           uint8_t* temp)
{

	uint16_t tmp;
	uint16_t val ;
	float f_temp;

	int sta = TEMPSENS_RegisterGet(i2c, addr, tempsensRegTEMP, &val);

	tmp = val & 0x1fff; //拿出13位 数据
	temp[0] = (tmp & 0xfff) >> 4; //仅仅那数据，不拿符号位，而且拿出的仅仅是整数位

	if ((tmp & 0x1000) == 0x1000) //TA <0
	{
		temp[0] |= 0x80; //把整数部分加上符号位
	}

	temp[1] = (val & 0x0C) >> 2; //拿出小数部分的2位
	f_temp = temp[1] * 2.5 + 0.5;
	temp[1] = (uint8_t)f_temp;
	return(sta);

}
