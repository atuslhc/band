/**************************************************************************//**
 * @file
 * @brief Hardfault handler for Cortex-M3, prototypes and definitions
 * @author Joseph Yiu, Frank Van Hooft, Energy Micro AS
 * @version 3.20.5
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __CORTEX_M_FAULTS_H
#define __CORTEX_M_FAULTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef union 
{
	uint32_t data;
	uint8_t buffer[4];
} FOMAT;

//errorDataFomat
extern FOMAT error1, error2, error3, error4, error5;

void HardFault_TrapDivByZero(void);
void HardFault_TrapUnaligned(void);
void HardFault_HandlerC(uint32_t* stack_pointer);
//void SendErrorInformation();

#ifdef __cplusplus
}
#endif

#endif
