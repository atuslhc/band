//******************************************************************************
//  Praveen Aroul
//  HealthTech, MHR
//  (C) Texas Instruments Inc., 2013
//  All Rights Reserved.
//  Built with IAR Workbench 5.50.2
//
//-------------------------------------------------------------------------------
// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
//--------------------------------------------------------------------------------

#ifndef AFE44x0_H_
#define AFE44x0_H_

#include "em_gpio.h"
#include "em_cmu.h"

#define AFE_REG_READ 1
#define AFE_REG_WRITE 0

// AFE_WORKING status definition




//#define ADC_MAX_VAL 0x1fffff

#define Light_CONFIR_TIME  6

#define AMBINENT_Thre  (50000/64)  //   (x/0x200000)*1.2v , 0x200000=2097152, 100000/2097152*1.2=0.057V
#define TOUCHED_Thre   (100)  //  0.086V

#if (BG013_LED_ADJ_METHOD==0)
#define REG_VAL_MAX     (int16_t)0x7fff
#define REG_VAL_MIN     (int16_t)0x8000
#define WORKING_H_LEVEL  (2000000*1.0/64)  //   x/2097152*1.2=1.0V  ,31250=0x7a12
#define WORKING_L_LEVEL  (2000000*0.6/64)  // 1223338/2097152*1.2=0.5V, 18750=0x493e
#elif (BG013_LED_ADJ_METHOD==1)
#define REG_VAL_MAX     (int16_t)0x7fff
#define REG_VAL_MIN     (int16_t)0x8000
#define VAL_RANGE        (0x200000)     //int22bit abs max 2097152
#define WORKING_H_LEVEL  (VAL_RANGE*10/12/64)  // keep range 10/12 as max boundary  0x6aaa
#define WORKING_L_LEVEL  (VAL_RABGE*6/12/64)  // keep range 6/12 as min boundary 0x4000
#elif (BG013_LED_ADJ_METHOD==2)
#define REG_VAL_MAX     (int32_t)0x7fffffff
#define REG_VAL_MIN     (int32_t)0x80000000
#define VAL_RANGE        (0x200000)     //int22bit abs max 2097152
#define WORKING_H_LEVEL  (VAL_RANGE*10/12)  // keep range 10/12 as max boundary  0x6aaa
#define WORKING_L_LEVEL  (VAL_RABGE*6/12)  // keep range 6/12 as min boundary 0x4000
#else
#error not specify WORKING_X_LEVEL for BG013_LED_ADJ_METHOD!
#endif

//#define WORKING_H_LEVEL  (1747626/64)  //   x/2097152*1.2=1.0V
//#define WORKING_L_LEVEL  (873813/64)  // 1223338/2097152*1.2=0.5V



#define WORKING_M_LEVEL ((WORKING_H_LEVEL+WORKING_L_LEVEL)/2)   // 25000=0x61a8   0x5555

#define PPG_SAM_FREQ 32


#define LED_Intensity_Init_val LED_INTENSITY_MS_MIN //5//128 //150

#define AMBINENT_DETECT  1
#define SKIN_TOUCHED_DETECT  2 
#define BEGIN_ADJ_LIGHT  3
#define BEGIN_PPG_Measurent     4
#define BEGIN_PPG_Locking      5
/* above definition no use and reference by code */
//#define PPG_0_6V 0x100000
//#define PPG_0_3V 0x80000


//=====================

#define AFE44x0_OK 0x00000000

//=========AFE address=========

//#define  AFE_LEDCNTRL  0x22
//#define  AFE_TIA_AMB_GAIN 0x21
typedef enum 
{
    AFE_CONTROL0        = 0, //Write only.
    AFE_LED2STC         = 1,
    AFE_LED2ENDC        = 2,
    AFE_LED2LEDSTC      = 3,
    AFE_LED2LEDENDC     = 4,
    AFE_ALED2STC        = 5,
    AFE_ALED2ENDC       = 6,
    AFE_LED1STC         = 7,
    AFE_LED1ENDC        = 8,
    AFE_LED1LEDSTC      = 9,
    AFE_LED1LEDENDC     = 10,
    AFE_ALED1STC        = 11,
    AFE_ALED1ENDC       = 12,
    AFE_LED2CONVST      = 13,
    AFE_LED2CONVEND     = 14,
    AFE_ALED2CONVST     = 15,
    AFE_ALED2CONVEND    = 16,
    AFE_LED1CONVST      = 17,
    AFE_LED1CONVEND     = 18,
    AFE_ALED1CONVST     = 19,
    AFE_ALED1CONVEND    = 20,
    AFE_ADCRSTSTCT0     = 21,
    AFE_ADCRSTENDCT0    = 22,
    AFE_ADCRSTSTCT1     = 23,
    AFE_ADCRSTENDCT1    = 24,
    AFE_ADCRSTSTCT2     = 25,
    AFE_ADCRSTENDCT2    = 26,
    AFE_ADCRSTSTCT3     = 27,
    AFE_ADCRSTENDCT3    = 28,
    AFE_PRPCOUNT        = 29,
    AFE_CONTROL1        = 30,
    AFE_SPARE1          = 31,
    AFE_TIAGAIN         = 32,
    AFE_TIA_AMB_GAIN    = 33,   //0x21
    AFE_LEDCNTRL        = 34,   //0x22
    AFE_CONTROL2        = 35,
    AFE_SPARE2          = 36,
    AFE_SPARE3          = 37,
    AFE_SPARE4          = 38,
    AFE_SPARE5          = 39,
    AFE_SPARE6          = 40,
    AFE_ALARM           = 41,
    AFE_LED2VAL         = 42,
    AFE_ALED2VAL        = 43,
    AFE_LED1VAL         = 44,
    AFE_ALED1VAL        = 45,
    AFE_LED2_ALED2VAL   = 46,
    AFE_LED1_ALED1VAL   = 47,
    AFE_DIAGNOSTICS     = 48,
} AFE_REG_ADDRESS;
//==========================

/****************************************************************/
/* Global functions*/
/****************************************************************/
void Init_AFE44xx_Resource(void);
void AFE44xx_PowerOn_Init(void);


void AFE44xx_Default_Reg_Init(void);
void AFE44xx_Reg_Write(unsigned char Reg_address, uint32_t Reg_data);
uint32_t AFE44xx_Reg_Read(unsigned char Reg_address);

void Init_AFE44xx_DRDY_Interrupt (void);
void Enable_AFE44xx_DRDY_Interrupt (void);

void AFE44xx_Read_All_Regs(uint32_t AFE44xxeg_buf[]);

void AFE44xx_SW_CMD (unsigned char ch);


void AFE44xx_Shutoff(void);
void AFE_READ(void);
//uint32_t AFE44xx_Reg_Read(unsigned char Reg_address);

void LED_Val_AMB_Cancellation(int32_t led_val,int32_t amb_uA);

extern uint32_t   afe_sample_int_counter;
extern bool afe_online;
extern int32_t ecg_val,pri_ecg_val;

#if (BG013_LED_ADJ_METHOD==0)
extern int16_t read_IR_reg;
extern int16_t DBG_IR_reg_min, DBG_IR_reg_max;
extern int16_t read_AB_reg;
extern int16_t DBG_AB_reg_min, DBG_AB_reg_max;
#elif (BG013_LED_ADJ_METHOD==1)
extern int16_t read_IR_reg;
extern int16_t DBG_IR_reg_min, DBG_IR_reg_max;
extern int16_t read_AB_reg;
extern int16_t DBG_AB_reg_min, DBG_AB_reg_max;
#elif (BG013_LED_ADJ_METHOD==2)
extern int32_t read_IR_reg;
extern int32_t DBG_IR_reg_min, DBG_IR_reg_max;
extern int32_t read_AB_reg;
extern int32_t DBG_AB_reg_min, DBG_AB_reg_max;
#else
extern int32_t read_IR_reg;
#endif
extern uint8_t LED_INTENSITY;
extern uint8_t AMB_uA;
extern int32_t read_IR_reg_22bit;
extern int32_t read_AB_reg_22bit;

#endif /*AFE44x0_H_*/
