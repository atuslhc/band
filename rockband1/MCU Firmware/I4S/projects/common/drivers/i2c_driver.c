/**************************************************************************//**
* @file
* @brief i2c_driver  for EFM32TG110
* @author Energy Micro AS
* @version 2.0.0
******************************************************************************
* @section License
* <b>(C) Copyright 2009 Energy Micro AS, http://www.energymicro.com</b>
******************************************************************************
*
* This source code is the property of Energy Micro AS. The source and compiled
* code may only be used on Energy Micro "EFM32" microcontrollers.
*
* This copyright notice may not be removed from the source code nor changed.
*
* DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
* obligation to support this Software. Energy Micro AS is providing the
* Software "AS IS", with no express or implied warranties of any kind,
* including, but not limited to, any implied warranties of merchantability
* or fitness for any particular purpose or warranties against infringement
* of any proprietary rights of a third party.
*
* Energy Micro AS will not be liable for any consequential, incidental, or
* special damages, or any other relief, or for any claim by any third party,
* arising from your use of this Software.
*
*****************************************************************************/
#include "i2c_driver.h"

#include "main.h"

uint32_t I2C0_error_count=0,I2C1_error_count=0;

volatile I2C_TransferReturn_TypeDef I2C0_Status;
volatile I2C_TransferReturn_TypeDef I2C1_Status;

/*******************************************************************************
**************************   GLOBAL FUNCTIONS   *******************************
******************************************************************************/

/**************************************************************************//**
* @brief I2C Interrupt Handler.
*        The interrupt table is in assembly startup file startup_efm32.s
*****************************************************************************/
void I2C0_IRQHandler(void)
{
	/* Just run the I2C_Transfer function that checks interrupts flags and returns */
	/* the appropriate status */
	I2C0_Status = I2C_Transfer(I2C0);
}

#if 1


/**************************************************************************//**
* @brief I2C Interrupt Handler.
*        The interrupt table is in assembly startup file startup_efm32.s
*****************************************************************************/
void I2C1_IRQHandler(void)
{
	/* Just run the I2C_Transfer function that checks interrupts flags and returns */
	/* the appropriate status */
	I2C1_Status = I2C_Transfer(I2C1);
}

#endif
/***************************************************************************//**
* @brief
*   Initalize basic I2C0 master mode driver
*
* @details
*   This driver only supports master mode, single bus-master. In addition
*   to configuring the EFM32 I2C peripheral module, it also configures
*   specific setup in order to use the I2C bus.
*
* @param[in] init
*   Pointer to I2C initialization structure.
******************************************************************************/

void I2C0Init(void)
{
	int i;
	
	
	/* Initialize I2C driver, using standard rate. */
	/* Devices on DK itself supports fast mode, */
	/* but in case some slower devices are added on */
	/* prototype board, we use standard mode. */
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	
	//i2cInit.freq = I2C_FREQ_FAST_MAX;
	
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	
	/* Use location 2: SDA - Pin D6, SCL - Pin D7 */
	/* Output value must be set to 1 to not drive lines low... We set */
	/* SCL first, to ensure it is high before changing SDA. */
	GPIO_PinModeSet(gpioPortI2C0, I2C0_SCL_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortI2C0, I2C0_SDA_PIN, gpioModeWiredAnd, 1);
	
	/* In some situations (after a reset during an I2C transfer), the slave */
	/* device may be left in an unknown state. Send 9 clock pulses just in case. */
	for (i = 0; i < 9; i++)
	{
		/*
		* TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
		* optimization when running at 14MHz. A bit high for standard mode devices,
		* but DVK only has fast mode devices. Need however to add some time
		* measurement in order to not be dependable on frequency and code executed.
		*/
		GPIO_PinModeSet(gpioPortI2C0, I2C0_SDA_PIN, gpioModeWiredAnd, 0);
		GPIO_PinModeSet(gpioPortI2C0, I2C0_SDA_PIN, gpioModeWiredAnd, 1);
	}
	
	/* Enable pins at location 6 */
	I2C0->ROUTE = I2C_ROUTE_SDAPEN |
		I2C_ROUTE_SCLPEN |
			(I2C0_LOCATION << _I2C_ROUTE_LOCATION_SHIFT);
	
	I2C_Init(I2C0, &i2cInit);
	
	/* Clear and enable interrupt from I2C module */
	NVIC_ClearPendingIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(I2C0_IRQn);
	
	NVIC_SetPriority(I2C0_IRQn, I2C0_IRQn_Level); 
}

/***************************************************************************//**
* @brief
*   Initalize basic I2C1 master mode driver
*
* @details
*   This driver only supports master mode, single bus-master. In addition
*   to configuring the EFM32 I2C peripheral module, it also configures
*   specific setup in order to use the I2C bus.
*
* @param[in] init
*   Pointer to I2C initialization structure.
******************************************************************************/
void I2C1Init(void)
{
	int i;
	
	/* Initialize I2C driver, using standard rate. */
	/* Devices on DK itself supports fast mode, */
	/* but in case some slower devices are added on */
	/* prototype board, we use standard mode. */
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	
	//i2cInit.freq = I2C_FREQ_FAST_MAX;
	
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C1, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	
	/* Use location 3: SDA - Pin E12, SCL - Pin E13 */
	/* Output value must be set to 1 to not drive lines low... We set */
	/* SCL first, to ensure it is high before changing SDA. */
	GPIO_PinModeSet(gpioPortI2C1, I2C1_SCL_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortI2C1, I2C1_SDA_PIN, gpioModeWiredAnd, 1);
	
	/* In some situations (after a reset during an I2C transfer), the slave */
	/* device may be left in an unknown state. Send 9 clock pulses just in case. */
	for (i = 0; i < 9; i++)
	{
		/*
		* TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
		* optimization when running at 14MHz. A bit high for standard mode devices,
		* but DVK only has fast mode devices. Need however to add some time
		* measurement in order to not be dependable on frequency and code executed.
		*/
		GPIO_PinModeSet(gpioPortI2C1, I2C1_SDA_PIN, gpioModeWiredAnd, 0);
		GPIO_PinModeSet(gpioPortI2C1, I2C1_SDA_PIN, gpioModeWiredAnd, 1);
	}
	
	/* Enable pins at location 0 */
	I2C1->ROUTE = I2C_ROUTE_SDAPEN |
		          I2C_ROUTE_SCLPEN |
			      (I2C1_LOCATION << _I2C_ROUTE_LOCATION_SHIFT);
	
	I2C_Init(I2C1, &i2cInit);
	
	/* Clear and enable interrupt from I2C module */
	NVIC_ClearPendingIRQ(I2C1_IRQn);
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn, I2C1_IRQn_Level); 
	
}

/***************************************************************************//**
* @brief
*   Read sensor register content.
*
* @details
*   If reading the temperature register, when a measurement is completed inside
*   the sensor device, the new measurement may not be stored. For this reason,
*   the temperature should not be polled with a higher frequency than the
*   measurement conversion time for a given resolution configuration. Please
*   refer to sensor device datasheet.
*
* @param[in] i2c
*   Pointer to I2C peripheral register block.
*
* @param[in] addr
*   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
*   for R/W bit.
*
* @param[in] reg
*   Register to read.
*
* @param[out] val
*   Reference to place register read.
*
* @return
*   Returns 0 if register read, <0 if unable to read register.
******************************************************************************/
int I2CReadNByte(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len)
{
//	I2C_TransferReturn_TypeDef I2C_Status = i2cTransferDone;
	I2C_TransferSeq_TypeDef seq;
	
	uint8_t regid[4];
	
	seq.addr = addr;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Select register to be read */
	*((uint32_t *)regid) = suba;
	seq.buf[0].data = regid;
	seq.buf[0].len = subaType;
	
	seq.buf[1].data = data;
	seq.buf[1].len = len;
	
	
	/* I2C0 */
	if (i2c == I2C0) {
		/* Do a polled transfer */
		I2C0_Status = I2C_TransferInit(i2c, &seq);
		I2C0_error_count=0;
		while (I2C0_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C0_error_count++;
			if(I2C0_error_count>100)break;
			
		}
		
//		if (I2C0_Status != i2cTransferDone)
//		{
			return((int)I2C0_Status);
//		}
	}
	/* I2C1 */
	else //if (i2c == I2C1) 
	{
		/* Do a polled transfer */
		I2C1_Status = I2C_TransferInit(i2c, &seq);
		I2C1_error_count=0;
		while (I2C1_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C1_error_count++;
			if(I2C1_error_count>100)break;
		}
		
//		if (I2C1_Status != i2cTransferDone)
//		{
			return((int)I2C1_Status);
//		}
	}
	
	
//	return I2C_Status;
}

/***************************************************************************//**
* @brief
*   Read sensor register content.
*
* @details
*   If reading the temperature register, when a measurement is completed inside
*   the sensor device, the new measurement may not be stored. For this reason,
*   the temperature should not be polled with a higher frequency than the
*   measurement conversion time for a given resolution configuration. Please
*   refer to sensor device datasheet.
*
* @param[in] i2c
*   Pointer to I2C peripheral register block.
*
* @param[in] addr
*   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
*   for R/W bit.
*
* @param[in] reg
*   Register to read.
*
* @param[out] val
*   Reference to place register read.
*
* @return
*   Returns 0 if register read, <0 if unable to read register.
******************************************************************************/
int I2CReadBytes(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint32_t len)
{
	I2C_TransferSeq_TypeDef seq;
	
	seq.addr = addr;
	seq.flags = I2C_FLAG_READ;
	
	seq.buf[0].data = data;
	seq.buf[0].len = len;
	
	if (i2c == I2C0) {
		/* Do a polled transfer */
		I2C0_Status = I2C_TransferInit(i2c, &seq);
		I2C0_error_count=0;
		while (I2C0_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C0_error_count++;
			if(I2C0_error_count>100)break;
		}
		
		if (I2C0_Status != i2cTransferDone)
		{
			return((int)I2C0_Status);
		}
	}
	
	if ( i2c == I2C1 ) {
		/* Do a polled transfer */
		I2C1_Status = I2C_TransferInit(i2c, &seq);
		I2C1_error_count=0;
		while (I2C1_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C1_error_count++;
			if(I2C1_error_count>100)break;
		}
		
		if (I2C1_Status != i2cTransferDone)
		{
			return((int)I2C1_Status);
		}
	}
	
	return(i2cTransferDone);
}


/***************************************************************************//**
* @brief
*   Write to sensor register.
*
* @param[in] i2c
*   Pointer to I2C peripheral register block.
*
* @param[in] addr
*   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
*   for R/W bit.
*
* @param[in] reg
*   Register to write (temperature register cannot be written).
*
* @param[in] val
*   Value used when writing to register.
*
* @return
*   Returns 0 if register written, <0 if unable to write to register.
******************************************************************************/
int I2CWriteNByte(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len)
{
//	I2C_TransferReturn_TypeDef I2C_Status = i2cTransferDone;	
	I2C_TransferSeq_TypeDef seq;
	uint8_t regid[4];
	
	seq.addr = addr;
	seq.flags = I2C_FLAG_WRITE_WRITE;
	/* Select register to be written */
	*((uint32_t *)regid) = suba;
	
	seq.buf[0].data = regid;
	seq.buf[0].len = subaType;
	
	seq.buf[1].data = data;
	seq.buf[1].len = len;

	
	if (i2c == I2C0) {
		/* Do a polled transfer */
		I2C0_Status = I2C_TransferInit(i2c, &seq);
		I2C0_error_count=0;
		while (I2C0_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C0_error_count++;
			if(I2C0_error_count>100)break;
		}
		
		return(I2C0_Status);
	}
	else //if (i2c == I2C1) 
	{
		/* Do a polled transfer */
		I2C1_Status = I2C_TransferInit(i2c, &seq);
		I2C1_error_count=0;
		while (I2C1_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C1_error_count++;
			if(I2C1_error_count>100)break;
		}
		
		return(I2C1_Status);
	}
	
	
//	return I2C_Status;
}

/***************************************************************************//**
* @brief
*   Write to sensor register.
*
* @param[in] i2c
*   Pointer to I2C peripheral register block.
*
* @param[in] addr
*   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
*   for R/W bit.
*
* @param[in] reg
*   Register to write (temperature register cannot be written).
*
* @param[in] val
*   Value used when writing to register.
*
* @return
*   Returns 0 if register written, <0 if unable to write to register.
******************************************************************************/
int I2CWriteBytes(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint32_t len)
{
	I2C_TransferSeq_TypeDef seq;
	
	seq.addr = addr;
	seq.flags = I2C_FLAG_WRITE;
	
	seq.buf[0].data = data;
	seq.buf[0].len = len;
	
	if (i2c == I2C0) {
		/* Do a polled transfer */
		I2C0_Status = I2C_TransferInit(i2c, &seq);
		I2C0_error_count=0;
		while (I2C0_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C0_error_count++;
			if(I2C0_error_count>100)break;
		}
		
		return(I2C0_Status);
	}
	
	if (i2c == I2C1) {
		/* Do a polled transfer */
		I2C1_Status = I2C_TransferInit(i2c, &seq);
		I2C1_error_count=0;
		while (I2C1_Status == i2cTransferInProgress)
		{
			/* Enter EM1 while waiting for I2C interrupt */
			EMU_EnterEM1();
			I2C1_error_count++;
			if(I2C1_error_count>100)break;
		}
		
		return(I2C1_Status);
	}
	
	return (-6);
}


