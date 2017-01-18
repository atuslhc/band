/**************************************************************************//**
 * @file
 * @brief i2c driver code for Energy Micro EFM32TG110F32
 * @note
 *    null
 * @author Rock
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 * NOTES:      
 ******************************************************************************/
#ifndef __I2C_DRIVER_H
#define __I2C_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "em_leuart.h"

/*******************************************************************************
 **************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/
#define  gpioPortI2C0       gpioPortD
#define  I2C0_LOCATION      1
#define  I2C0_SDA_PIN       6
#define  I2C0_SCL_PIN       7

#define  gpioPortI2C1       gpioPortB
#define  I2C1_LOCATION      1
#define  I2C1_SDA_PIN       11
#define  I2C1_SCL_PIN       12

#define  I2C_SUBA_ONEBYTE    1
#define  I2C_SUBA_TWOBYTE    2
#define  I2C_SUBA_THREEBYTE  3
#define  I2C_SUBA_FOURBYTE   4

//void I2C0Init(void);
//void I2C1Init(void);

//int I2CReadBytes(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint32_t len);
int I2CReadNByte(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);

//int I2CWriteBytes(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint32_t len);
int I2CWriteNByte(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);

//void Write_I2C(uint8_t addr, uint8_t reg, uint8_t data);
//uint8_t Read_I2C(uint8_t addr, uint8_t reg);

#endif