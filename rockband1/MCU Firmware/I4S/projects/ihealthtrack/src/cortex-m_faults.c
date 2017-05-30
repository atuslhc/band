/**************************************************************************//**
 * @file
 * @brief Hardfault handler for Cortex-M3
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

#include <stdio.h>
#include "em_device.h"
#include "cortex-m_faults.h"
#include "em_msc.h"
#include "string.h"
#include "common_vars.h"
#include "main.h"

typedef struct
{
	uint32_t tagHead;

	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;    //add 
    uint32_t psr;   //add

	uint32_t tagTail;
} registerDump;

registerDump* dumpData = (registerDump*)0x20007f00;

void WritErrorInfoToUD();


//FOMAT error1, error2, error3, error4, error5;
//FOMAT error_r0, error_r1, error_r2, error_r3, error_r12, error_lr, error_pc, error_psr;
/**************************************************************************//**
 * @brief Enable trapping of divison by zero
 *****************************************************************************/
void HardFault_TrapDivByZero(void)
{
  volatile uint32_t *confctrl = (uint32_t *) 0xE000ED14;

  *confctrl |= (1<<4);
}


/**************************************************************************//**
 * @brief  Enable trapping of unaligned memory accesses
 *****************************************************************************/
void HardFault_TrapUnaligned(void)
{
  volatile uint32_t *confctrl = (uint32_t *) 0xE000ED14;

  *confctrl |= (1<<3);
}

/**************************************************************************//**
 * @brief  Exception handler for Cortex-M3 hard faults
 * @note This code is from http://blog.frankvh.com/, based on Joseph Yiu's
 *       hardfault code. For Keil MDKARM, this assembly code needs to be added
 *       as a separate assembly function, as the RVDS compiler cannot do inline
 *       assembly of thumb instructions required for Cortex-M3.
 *****************************************************************************/
#if defined(__GNUC__)
void HardFault_Handler(void) __attribute__ (( naked ));
#endif
void HardFault_Handler(void)
{
	/*
	 * Get the appropriate stack pointer, depending on our mode,
	 * and use it as the parameter to the C handler. This function
	 * will never return
	 */
	errlocated.mspTop = __get_MSP();
	errlocated.pspTop = __get_PSP();
	__asm("TST   LR, #4"); //把LR中的值和4比较，然后更新状态寄存器
#if 0
	__asm("ITE   EQ");		//下面2条指令是条件执行
	__asm("MRSEQ R0, MSP");//如果相等则把MSP数据放到R0
	__asm("MRSNE R0, PSP");//如果不相等则把PSP数据放到R0
#else  //IAR before ARM 7.40.1
    __asm("ITE EQ \n"
	      "MRSEQ R0, MSP \n"
	      "MRSNE R0, PSP");
#endif

	__asm("B HardFault_HandlerC");
}

//void MemManage_Handler(void)
//{
//	/*
//	 * Get the appropriate stack pointer, depending on our mode,
//	 * and use it as the parameter to the C handler. This function
//	 * will never return
//	 */
//	__asm("TST   LR, #4");
//	__asm("ITE   EQ");
//	__asm("MRSEQ R0, MSP");
//	__asm("MRSNE R0, PSP");
//	__asm("B MemManage_HandlerC");
//}
//
//
//void BusFault_Handler(void)
//{
//	/*
//	 * Get the appropriate stack pointer, depending on our mode,
//	 * and use it as the parameter to the C handler. This function
//	 * will never return
//	 */
//	__asm("TST   LR, #4");
//	__asm("ITE   EQ");
//	__asm("MRSEQ R0, MSP");
//	__asm("MRSNE R0, PSP");
//	__asm("B BusFault_HandlerC");
//}
//
//void UsageFault_Handler(void)
//{
//	/*
//	 * Get the appropriate stack pointer, depending on our mode,
//	 * and use it as the parameter to the C handler. This function
//	 * will never return
//	 */
//	__asm("TST   LR, #4");
//	__asm("ITE   EQ");
//	__asm("MRSEQ R0, MSP");
//	__asm("MRSNE R0, PSP");
//	__asm("B UsageFault_HandlerC");
//}

int test_index;
/**************************************************************************//**
 * @brief Exception handler for Cortex-M3 hard faults
 * @param[in] hardfault_args Stack frame location
 * @note  From Joseph Yiu, minor edits by FVH and Energy Micro AS.
 *        Hard fault handler in C
 *****************************************************************************/
void HardFault_HandlerC(uint32_t *stack_pointer)
{
#if 0	//[BG025] add to remove never used warning.
  uint32_t stacked_r0;
  uint32_t stacked_r1;
  uint32_t stacked_r2;
  uint32_t stacked_r3;
  uint32_t stacked_r12;
  uint32_t stacked_lr;
  uint32_t stacked_pc;
  uint32_t stacked_psr;

  stacked_r0 = ((uint32_t) stack_pointer[0]);
  stacked_r1 = ((uint32_t) stack_pointer[1]);
  stacked_r2 = ((uint32_t) stack_pointer[2]);
  stacked_r3 = ((uint32_t) stack_pointer[3]);

  stacked_r12 = ((uint32_t) stack_pointer[4]);
  stacked_lr =  ((uint32_t) stack_pointer[5]);
  stacked_pc =  ((uint32_t) stack_pointer[6]);
  stacked_psr = ((uint32_t) stack_pointer[7]);

  //printf("\n\n[HardFault]\n");
  //printf("R0        = %08x\n", (unsigned int)stacked_r0);
  //printf("R1        = %08x\n", (unsigned int)stacked_r1);
  //printf("R2        = %08x\n", (unsigned int)stacked_r2);
  //printf("R3        = %08x\n", (unsigned int)stacked_r3);
  //printf("R12       = %08x\n", (unsigned int)stacked_r12);
  //printf("LR [R14]  = %08x - Subroutine Call return address\n", (unsigned int)stacked_lr);
  //printf("PC [R15]  = %08x - Program Counter\n", (unsigned int)stacked_pc);
  //printf("PSR       = %08x\n", (unsigned int)stacked_psr);
  //printf("BFAR      = %08x - Bus Fault SR/Address causing bus fault\n",
         //(unsigned int) (*((volatile uint32_t *)(0xE000ED38))));
 // printf("CFSR      = %08x - Config. Fault SR\n",
        // (unsigned int) (*((volatile uint32_t *)(0xE000ED28))));
#endif

  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<25))
  {
    test_index=1;
    //printf("  :UsageFault->DivByZero\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<24))
  {
	  test_index=1;

	 //printf("  :UsageFault->Unaligned access\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<18))
  {
	  test_index=1;

	 //printf("  :UsageFault->Integrity check error\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<0))
  {
    test_index=1;
	//printf("  :MemFault->Data access violation\n");
  }
  if((*((volatile uint32_t *)(0xE000ED28)))&(1<<0))
  {
    test_index=1;
    //printf("  :MemFault->Instruction access violation\n");
  }
  //printf("HFSR      = %08x - Hard Fault SR\n",
    //     (unsigned int)(*((volatile uint32_t *)(0xE000ED2C))));
  
  if((*((volatile uint32_t *)(0xE000ED2C)))&(1UL<<1))
  {
    test_index=1;
    //printf("  :VECTBL, Failed vector fetch\n");
  }
  if((*((volatile uint32_t *)(0xE000ED2C)))&(1UL<<30))
  {
    test_index=1;
	//printf("  :FORCED, Bus fault/Memory management fault/usage fault\n");
  }
  if((*((volatile uint32_t *)(0xE000ED2C)))&(1UL<<31))
  {
    test_index=1;
    //printf("  :DEBUGEVT, Hard fault triggered by debug event\n");
  }
  //printf("DFSR      = %08x - Debug Fault SR\n", (unsigned int)(*((volatile uint32_t *)(0xE000ED30))));
  //printf("MMAR      = %08x - Memory Manage Address R\n", (unsigned int)(*((volatile uint32_t *)(0xE000ED34))));
  //printf("AFSR      = %08x - Auxilirary Fault SR\n", (unsigned int)(*((volatile uint32_t *)(0xE000ED3C))));
  //printf("SCB->SHCSR= %08x - System Handler Control and State R (exception)\n", (unsigned int)SCB->SHCSR);

//	error_r0.data = stacked_r0;
//	error_r1.data = stacked_r1;
//	error_r2.data = stacked_r2;
//	error_r3.data = stacked_r3;
//
//	error_r12.data = stacked_r12;//通用寄存器
//	error_lr.data  = stacked_lr;//连接寄存器
//	error_pc.data  = stacked_pc;//程序指针
//	error_psr.data = stacked_psr;//程序状态寄存器
//

	dumpData->tagHead = 0x12345678;

#if (1)
    dumpData->r0      = ((uint32_t) stack_pointer[0]);
	dumpData->r1      = ((uint32_t) stack_pointer[1]);
	dumpData->r2      = ((uint32_t) stack_pointer[2]);
	dumpData->r3      = ((uint32_t) stack_pointer[3]);
	dumpData->r12     = ((uint32_t) stack_pointer[4]);
	dumpData->lr      = ((uint32_t) stack_pointer[5]);
	dumpData->pc      = ((uint32_t) stack_pointer[6]);
	dumpData->psr     = ((uint32_t) stack_pointer[7]);
#else
    dumpData->r0      = (uint32_t) * ((uint32_t*)(errlocated.pspTop) + 6);
	dumpData->r1      = (uint32_t)SCB->CFSR;
	dumpData->r2      = (uint32_t)SCB->HFSR;
	dumpData->r3      = (uint32_t)SCB->MMFAR;
	dumpData->r12     = (uint32_t)SCB->BFAR;
	dumpData->lr      = (uint32_t) * ((uint32_t*)(errlocated.pspTop) + 5);
#endif
	dumpData->tagTail = 0x89abcdef;

//	error1.data = (uint32_t)SCB->CFSR;
//	error2.data = (uint32_t)SCB->HFSR;
//	error3.data = (uint32_t)SCB->MMFAR;
//	error4.data = (uint32_t)SCB->BFAR;
//	error5.data = stacked_lr;




//	WritErrorInfoToUD();

//	uint32_t k = 50000;
//
//	while(k--);


	RESET_MCU();

	while(1);
}

//void MemManage_Handler(void)
//{
//	uint8_t mmfsr = SCB->CFSR & 0xFF;
//	uint32_t mmfar = SCB->MMFAR;
//	
//	while(1);
//}
//
//void BusFault_Handler(void)
//{
//	uint8_t bfsr = (SCB->CFSR >> 8) & 0xFF;
//	uint32_t bfar = SCB->BFAR;
//	
//	while(1);
//}
//
//void UsageFault_Handler(void)
//{
//	uint16_t ufsr = (SCB->CFSR >> 16) & 0xFFFF;
//	
//	while(1);
//}
