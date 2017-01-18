/**************************************************************************//**
 * @file main.c
 * @brief USB HID keyboard device example.
 * @author Energy Micro AS
 * @version 1.2.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
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
#include <stdio.h>
#include "efm32.h"
#include "em_cmu.h"
#include "em_msc.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_wdog.h"
#include "em_letimer.h"
#include "m25pxx.h"
#include "ble.h"
#include "em_leuart.h"
//#include "globaldata.h"
#include "main.h"
#include "config.h"
#include "crc.h"
#include "boot.h"
//#include "res.h"
#include "em_common.h"



void DealError(void);
bool JudgeFlashBusy();
/*** Typedef's and defines. ***/

//LED definition
#if (BOARD_TYPE==0)
#define LED_GPIOPORT  (gpioPortC)
#define LED_PIN   	  (10)
#elif (BOARD_TYPE==1)
#define LED_GPIOPORT  (gpioPortB)
#define LED_PIN   	  (5)
#elif (BOARD_TYPE==2)
#define LED_GPIOPORT  (gpioPortC)
#define LEDC_PIN        (0)
#define LEDR_PIN        (1)
#define LEDB_PIN        (2)
#define LEDG_PIN        (3)
#define LED_PIN         LEDR_PIN    //redirect mapping, backward compatible.
#endif
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
#define LED_ON()        GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
#define LED_OFF()       GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )
#define LED_TOGGLE()    GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#elif (BOARD_TYPE==2)
// pin PC0 is 3V feedback, input pin.
//#define LEDC_High()      GPIO_PinOutSet(LED_GPIOPORT,LED_PIN )
//#define LEDC_Low()      GPIO_PinOutClear(LED_GPIOPORT,LED_PIN )
//#define LEDC_TOGGLE() GPIO_PinOutToggle(LED_GPIOPORT,LED_PIN)
#define GetLEDC() GPIO_PinInGet(LED_GPIOPORT,LEDC_PIN)

#define LEDR_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDR_PIN)  //low active
#define LEDR_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDR_PIN)
#define LEDR_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDR_PIN)

#define LEDG_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDG_PIN) 
#define LEDG_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDG_PIN)
#define LEDG_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDG_PIN)

#define LEDB_ON()       GPIO_PinOutClear(LED_GPIOPORT, LEDB_PIN)
#define LEDB_OFF()      GPIO_PinOutSet(LED_GPIOPORT, LEDB_PIN)
#define LEDB_TOGGLE()   GPIO_PinOutToggle(LED_GPIOPORT, LEDB_PIN)

#define LED_ON()        GPIO_PinOutClear(LED_GPIOPORT, LED_PIN)
#define LED_OFF()       GPIO_PinOutSet(LED_GPIOPORT, LED_PIN)
#define LED_TOGGLE()    GPIO_PinOutToggle(LED_GPIOPORT, LED_PIN)
#endif


//FW macro defined
#define FW_STARTADDR           0    //FW put on the ext flash begin 0x0
#define FW_BIN_STARTADDR       16     //upgrade firmware stored in ext flash, start 0 with header 16 bytes.(FW_APPSTART_ADDR)
#define FWHEAD_LENGTH          16
#define ONCEREADSIZE           2048   //once read size base on flash sector size.
#define FWTYPE_MCU             1    //MCU_App firmware
#define FWTYPE_BLE             2    //BLE firmware
#define FWTYPE_BOOT            3    //Bootloader firmware(MCU)
#define RESET_MCU()            SCB->AIRCR = 0x05FA0004
#define FW_BOARD_I4   0x40
#define FW_BOARD_I4S  0x41
#define FW_BOARD_I4B  0x42
#define FW_BOARD_RB0  0x41
#define FW_BOARD_RB1  0x50


//ble statue of upgrade
#define CHECK_BLE_ONLINE        0
#define CHECK_BLE_STATE         1   //CHECK_BLE_STATAE >> CHECK_BLE_STATE
#define SEND_START_CMMD         2
#define WRITE_BLE_FLASH         3
#define SEND_STOP_CMMD          4
#define ERROR                   5
#define APP_STATE               1
#define BOOT_STATE              0

#define UNDEFINE_STATUS         10

uint32_t flashSize, flashPageSize;
volatile unsigned char mcuUpdatingShiningCount = 0;
volatile unsigned char bleUpdatingShiningCount = 0;
volatile unsigned char bootUpdatingShiningCount = 0;
uint16_t  waitBleStopCommandresponse = 0;

EFM32_PACK_START( 1 )

union _FW_INFO
{
	struct _INFO
	{
		uint16_t fw_type;
		uint16_t ver_num;
		uint32_t fw_length;
		uint16_t fw_crc;
		uint32_t Rev1;
		uint16_t Head_CRC;
	} INFO;
	uint8_t INFO_BUF[16];
};
union _FW_INFO FW_INFO;

EFM32_PACK_END()

union _CHIP DevChip;

#if 0   //Atus: duplicate in main.c, but boot usbhhidkbd and Bootld proj refer this, remark main.c.
void SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif

//watchdog initialize setting.
WDOG_Init_TypeDef init =
{
	.enable     = true,               /* Start watchdog when init done */
	.debugRun   = false,              /* WDOG not counting during debug halt */
	.em2Run     = true,               /* WDOG counting when in EM2 */
	.em3Run     = true,               /* WDOG counting when in EM3 */
	.em4Block   = false,              /* EM4 can be entered */
	.swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */
	.lock       = false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
	.clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */
	.perSel     = wdogPeriod_32k,      /* Set the watchdog period to 4097 clock periods (ie ~2 seconds)*/
};


void LETIMER_setup(void)
{

	CMU_OscillatorEnable(cmuOsc_LFXO, true, true); //wait low osc enable.
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); // external Low Frequency Crystal Osc(LFXO=32.768Hz) as clock source.
	CMU_ClockEnable(cmuClock_CORELE, true); //Low Energy clock module for RTC and LE equipment, and watchdog source.
	CMU_ClockEnable(cmuClock_LETIMER0, true); //turn on Timer0

	LETIMER_Init_TypeDef leTimerinit = LETIMER_INIT_DEFAULT; //initialize

	leTimerinit.debugRun = true;
	leTimerinit.comp0Top = true;
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0); //comparer0 interrupt
	NVIC_EnableIRQ(LETIMER0_IRQn); //NestedVectorInterruptController enable
	LETIMER_CompareSet(LETIMER0, 0, 500); //setup comparer0 timer 15ms
	LETIMER_Init(LETIMER0, &leTimerinit);

}


//Low Energy Timer0 interrupt handler
void LETIMER0_IRQHandler(void)
{

	LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0); //clear COMP0 interrupt flag.
	mcuUpdatingShiningCount++;
	bleUpdatingShiningCount++;
	bootUpdatingShiningCount++;
	waitBleStopCommandresponse++;

	if(waitBleStopCommandresponse > 400)
	{
		waitBleStopCommandresponse = 0;
	}

	if((FW_INFO.INFO.fw_type == 1) && (mcuUpdatingShiningCount == 12))
	{
		mcuUpdatingShiningCount = 0;
		LED_TOGGLE();
	}
	else if((FW_INFO.INFO.fw_type == 2) && (bleUpdatingShiningCount == 30))
	{
		bleUpdatingShiningCount = 0;
		LED_TOGGLE();
	}
	else if((FW_INFO.INFO.fw_type == 3) && (bootUpdatingShiningCount == 3))
	{
		bootUpdatingShiningCount = 0;
		LED_TOGGLE();

	}
}

//��ȡƬ��flash�Ĵ�С��Ϣ
void FLASH_CalcPageSize(void)
{
	uint8_t family = *(uint8_t*)EFM32_PART_FAMILY;

	flashSize = *(uint16_t*)EFM32_INFO_FLASH * 1024;

	flashPageSize = 4096;                 /* Assume Giant, 'H' */

	if ( family == 74 ) //Leopard Gecko
	{
		flashPageSize = 2048;               /* Leopard, 'J' */
	}
	else if ( family == 75 ) //Wonder Gecko
	{
		flashPageSize = 2048;               /* Leopard, 'W' */
	}
}


//��Ƭ��flash�����ݶ���������д��Ƭ��flash,ÿ�δ�Ƭ���2k,Ȼ��д��
void  InsideFlashWR(void)
{
	unsigned int i , j ;
	unsigned char appBuf[ONCEREADSIZE] = {0};
	unsigned int integer = 0;
	unsigned int decimal = 0;
	int align4 = 0;//4�ֽڶ���
	
	int8_t writeStatus = UNDEFINE_STATUS;
	int8_t eraseStatus = UNDEFINE_STATUS;


	MSC_Init(); //enable built-in flash controller for write operation.
	FLASH_CalcPageSize(); //get the built-in flash page size(erase unit).


	integer = FW_INFO.INFO.fw_length / ONCEREADSIZE;
	decimal = FW_INFO.INFO.fw_length % ONCEREADSIZE;

	SysCtlDelay(500000); //The delay to keep small size app can write EFM32 built-in flash successful.

	for(i = 0; i < integer; i++) //full 2k part.
	{
		JudgeFlashBusy();
		FlashRead((FW_BIN_STARTADDR + ONCEREADSIZE * i), appBuf, ONCEREADSIZE); //ÿ�ζ���Ƭ���2k
		JudgeFlashBusy();

        //erase built-in flash MCU-APP area, ONCEREADSIZE(2k) each time.
		eraseStatus = MSC_ErasePage((uint32_t*)(MCUAPP_ADDR + i * ONCEREADSIZE)); //BOOTLOADER_SIZE >> MCUAPP_ADDR

		while(eraseStatus != mscReturnOk); //BUG: fail lock withoug update command.
#if 0
        do {
            eraseStatus = MSC_ErasePage((uint32_t*)(MCUAPP_ADDR + i * ONCEREADSIZE)); //BOOTLOADER_SIZE >> MCUAPP_ADDR
        } while (earseStatus != mscReturnOk);
#endif        
        

		eraseStatus = UNDEFINE_STATUS;

//ͨ���鿴����ȫ���ɹ� ��
		for(j = 0; j < ONCEREADSIZE; j++)
		{
			if(*((uint8_t*)(MCUAPP_ADDR + i * ONCEREADSIZE + j)) != 0xFF) //���û�в�����ȷ������  //BOOTLOADER_SIZE >> MCUAPP_ADDR
				while(1);
		}

		writeStatus = MSC_WriteWord((uint32_t*)(MCUAPP_ADDR + i * ONCEREADSIZE), appBuf , ONCEREADSIZE); //ÿ��д2k //BOOTLOADER_SIZE >> MCUAPP_ADDR

		while(writeStatus != mscReturnOk);

		writeStatus = UNDEFINE_STATUS;

		WDOG_Feed();
//������Ҫ�ϳ�ʱ�䣬�ڴ�ι��
	}

	if(decimal > 0)//ȷ������ʣ��
	{
		JudgeFlashBusy();

		FlashRead((FW_BIN_STARTADDR + ONCEREADSIZE * integer), appBuf, decimal); //appBuf�д洢�����ϴε����ݡ������ٶ�ȡһ�Σ�����ֻ��һ����

		JudgeFlashBusy();

		eraseStatus = MSC_ErasePage((uint32_t*)(MCUAPP_ADDR + integer * ONCEREADSIZE)); //�ٶ����һ��ҳ������д���ʣ�µĲ���һ��ҳ������ //BOOTLOADER_SIZE >> MCUAPP_ADDR

		while(eraseStatus != mscReturnOk);

		eraseStatus = UNDEFINE_STATUS;

		for(j = 0; j < decimal; j++)
		{
			if(*((uint8_t*)(MCUAPP_ADDR + integer * ONCEREADSIZE + j)) != 0xff) //BOOTLOADER_SIZE >> MCUAPP_ADDR
				while(1);
		}

		if((decimal % 4) == 0)//˵��ʣ����ֽ����ܱ�4��������ֱ��д��flash
		{
			writeStatus = MSC_WriteWord((uint32_t*)(MCUAPP_ADDR + integer * ONCEREADSIZE), appBuf , decimal); //BOOTLOADER_SIZE >> MCUAPP_ADDR

			while(writeStatus != mscReturnOk);

			writeStatus = UNDEFINE_STATUS;

			WDOG_Feed();
		}
		else
		{
			align4 = decimal % 4;

			switch(align4)
			{
				case 1://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff; //ע�ⲹ����ʼ��ַ
					appBuf[decimal + 1] = 0xff;
					appBuf[decimal + 2] = 0xff;
					writeStatus = MSC_WriteWord((uint32_t*)(MCUAPP_ADDR + integer * ONCEREADSIZE), appBuf , decimal + 3);  //BOOTLOADER_SIZE >> MCUAPP_ADDR

					while(writeStatus != mscReturnOk);

					writeStatus = UNDEFINE_STATUS;
					WDOG_Feed();

				}
				break;

				case 2://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					appBuf[decimal + 1] = 0xff;
					writeStatus = MSC_WriteWord((uint32_t*)(MCUAPP_ADDR + integer * ONCEREADSIZE), appBuf , decimal + 2);  //BOOTLOADER_SIZE >> MCUAPP_ADDR

					while(writeStatus != mscReturnOk);

					writeStatus = UNDEFINE_STATUS;
					WDOG_Feed();
				}
				break;

				case 3://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					writeStatus = MSC_WriteWord((uint32_t*)(MCUAPP_ADDR + integer * ONCEREADSIZE), appBuf , decimal + 1);  //BOOTLOADER_SIZE >> MCUAPP_ADDR

					while(writeStatus != mscReturnOk);

					writeStatus = UNDEFINE_STATUS;
					WDOG_Feed();
				}
				break;

				default:
					break;
			}
		}
	}
}

void  CalcInsideFlashCRCandWrIn(void)
{
	unsigned char i = 0;
	unsigned char temp[4] = {0};
	unsigned int  inSideFlashCRC = 0;//Ƭ��flash��bootloadersize�����4���ֽڵ�CRC
	unsigned char flag = UNDEFINE_STATUS;

	for (i = 0; i < 16; i++)
	{
		DevChip.EFM32DeviceInfo[i] = *((unsigned char*)(EFM32_ADDRESS + i)); //��EFM32_ADDRESS��ַ���16���ֽ��ó�����
	}

	inSideFlashCRC = CRC_calc((uint8_t*)MCUAPP_ADDR, (uint8_t*)(DevChip.Device.memFlash * 1024 - 8));  //BOOTLOADER_SIZE >> MCUAPP_ADDR

	//�������CRCд�뵽Ƭ��flash�����4���ֽڴ�
	temp[0] = (unsigned char)(inSideFlashCRC & 0xff);
	temp[1] = (unsigned char)((inSideFlashCRC >> 8) & 0xff);
	temp[2] = (unsigned char)((inSideFlashCRC >> 16) & 0xff);
	temp[3] = (unsigned char)((inSideFlashCRC >> 24) & 0xff); //ʹ�ã�uint32_t*������ֱ�ӵ�ַ���������ȶ������Ǹ��ֽ�

	MSC_Init();
	FLASH_CalcPageSize();//�Ȼ�ȡƬ��flash����Ϣ
	flag = MSC_ErasePage((uint32_t*)0x3F800);//������254-256k������,�����һҳ
	WDOG_Feed();

	while(flag !=  mscReturnOk);

	flag = UNDEFINE_STATUS;

	flag = MSC_WriteWord((uint32_t*)(DevChip.Device.memFlash * 1024 - 4), temp , 4); //д�������CRC,д�ĵ�ַ��Ƭ��flash�����4���ֽ�

	while(flag != mscReturnOk);

}

/* 
 * To Deal the error of firmware upgrade data check error.
 */
void DealError(void)
{
	unsigned char i = 0;
	volatile unsigned int flashCrcApp = 0;

    /* get the EFM32 chip info, the last 16 bytes */
	for (i = 0; i < 16; i++)
	{
		DevChip.EFM32DeviceInfo[i] = *((unsigned char*)(EFM32_ADDRESS + i));
	}

    /* calculate the APP crc checksum. MCUAPP_ADDR .. endFlash - 8 */
	flashCrcApp = CRC_calc((uint8_t*)MCUAPP_ADDR, (uint8_t*)(DevChip.Device.memFlash * 1024 - 8)); //notice: range from 0x9800~last 8 bytes.

#if defined(SHOWLED)
    for (i=0 ; i< 20 ; i++)
    {
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
      LED_TOGGLE();
#elif (BOARD_TYPE==2)
      LEDB_TOGGLE();
#endif
      SysCtlDelay(9000);
    }
#endif

#if (1)  //Atus: for dev board test, skip check reset
 		WDOG_Feed();

		BOOT_boot();//jump to run App. This is normal case power on bootup or reset boot.
#endif   
    /* crcApp store in the last 4 bytes */
	if(flashCrcApp != (*((uint32_t*)(DevChip.Device.memFlash * 1024 - 4))))
	{ /* mismatch, reset the MCU */
		RESET_MCU(); //reset MCU
        
	}
	else //�����ȥִ��App������֮ǰ�ص����Ź�������ص����Ź���ϵͳ�ز�ͣ�ĸ�λ��
	{ /* match, execute App */
        //feed dog, if we turn off watchdog here, the system will repeat reset
		WDOG_Feed();

		BOOT_boot();//jump to run App. This is normal case power on bootup or reset boot.
	}

}

void BootWriteIn(void)
{
	unsigned int i , j ;
	unsigned char appBuf[ONCEREADSIZE] = {0};
	unsigned int integer = 0;
	unsigned int decimal = 0;
	unsigned int align4 = 0;//4�ֽڶ���
	uint8_t writeStatus = UNDEFINE_STATUS;//������ʾ�Ƿ�д�ɹ���
	uint8_t eraseFlag = UNDEFINE_STATUS;



	MSC_Init();//��Ƭ��flash���г�ʼ��������Ҫ������д��
	FLASH_CalcPageSize();//�Ȼ�ȡƬ��flash����Ϣ


	integer = FW_INFO.INFO.fw_length / ONCEREADSIZE;
	decimal = FW_INFO.INFO.fw_length % ONCEREADSIZE;

	SysCtlDelay(500000);//�����ʱȷ������С��app��ʱ���ܹ�˳����д��efm32��ָ��λ�á�

	for(i = 0; i < integer; i++) //����2k��������
	{
		JudgeFlashBusy();
		FlashRead((FW_BIN_STARTADDR + ONCEREADSIZE * i), appBuf, ONCEREADSIZE); //ÿ�ζ���Ƭ���2k
		JudgeFlashBusy();

		eraseFlag = MSC_ErasePage((uint32_t*)(BOOTLOADER_ADDR + i * ONCEREADSIZE)); //erase 2k every time. start from  0. FW_STARTADDR >> BOOTLOADER_ADDR.

		while(eraseFlag != mscReturnOk);

		eraseFlag = UNDEFINE_STATUS;


		for(j = 0; j < ONCEREADSIZE; j++)
		{
			if(*((uint8_t*)(BOOTLOADER_ADDR + i * ONCEREADSIZE + j)) != 0xFF) //FW_STARTADDR >> BOOTLOADER_ADDR
				while(1);
		}

		writeStatus = MSC_WriteWord((uint32_t*)(BOOTLOADER_ADDR + i * ONCEREADSIZE), appBuf , ONCEREADSIZE); //FW_STARTADDR >> BOOTLOADER_ADDR

		while(writeStatus != mscReturnOk);

		writeStatus = UNDEFINE_STATUS;

		WDOG_Feed();
//������Ҫ�ϳ�ʱ�䣬�ڴ�ι��
	}

	if(decimal > 0)//ȷ������ʣ��
	{
		JudgeFlashBusy();

		FlashRead((FW_BIN_STARTADDR + ONCEREADSIZE * integer), appBuf, decimal); //appBuf�д洢�����ϴε����ݡ������ٶ�ȡһ�Σ�����ֻ��һ����

		JudgeFlashBusy();

		eraseFlag = MSC_ErasePage((uint32_t*)(BOOTLOADER_ADDR + integer * ONCEREADSIZE)); //erase one sector for remains.  //FW_STARTADDR >> BOOTLOADER_ADDR

		while(eraseFlag != mscReturnOk);

		eraseFlag = UNDEFINE_STATUS;

		for(j = 0; j < decimal; j++)
		{
			if(*((uint8_t*)(BOOTLOADER_ADDR + integer * ONCEREADSIZE + j)) != 0xff)  //FW_STARTADDR >> BOOTLOADER_ADDR
				while(1);
		}

		if((decimal % 4) == 0)//˵��ʣ����ֽ����ܱ�4��������ֱ��д��flash
		{
			writeStatus = MSC_WriteWord((uint32_t*)(BOOTLOADER_ADDR + integer * ONCEREADSIZE), appBuf , decimal);  //FW_STARTADDR >> BOOTLOADER_ADDR

			while(writeStatus != mscReturnOk);

			writeStatus = UNDEFINE_STATUS;
			WDOG_Feed();
		}
		else
		{
			align4 = decimal % 4;

			switch(align4)
			{
				case 1://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff; //ע�ⲹ����ʼ��ַ
					appBuf[decimal + 1] = 0xff;
					appBuf[decimal + 2] = 0xff;
					writeStatus =  MSC_WriteWord((uint32_t*)(BOOTLOADER_ADDR + integer * ONCEREADSIZE), appBuf , decimal + 3); //FW_STARTADDR >> BOOTLOADER_ADDR

					while(writeStatus != mscReturnOk);

					writeStatus = UNDEFINE_STATUS;

					WDOG_Feed();
				}
				break;

				case 2://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					appBuf[decimal + 1] = 0xff;
					writeStatus = MSC_WriteWord((uint32_t*)(BOOTLOADER_ADDR + integer * ONCEREADSIZE), appBuf , decimal + 2); //FW_STARTADDR >> BOOTLOADER_ADDR

					while(writeStatus != mscReturnOk);

					writeStatus = UNDEFINE_STATUS;
					WDOG_Feed();
				}
				break;

				case 3://��Ҫ��3���ֽ�,�����ֽڶ���0xff
				{
					appBuf[decimal + 0] = 0xff;
					writeStatus = MSC_WriteWord((uint32_t*)(BOOTLOADER_ADDR + integer * ONCEREADSIZE), appBuf , decimal + 1);  //FW_STARTADDR >> BOOTLOADER_ADDR

					while(writeStatus != mscReturnOk);

					writeStatus = UNDEFINE_STATUS;
					WDOG_Feed();
				}
				break;

				default:
					break;
			}
		}
	}
}



void BootUpdate(void)
{
	unsigned int bootCRC = 0;
	uint8_t retry = 0;

	bootCRC = FlashCRC(0 + 16, FW_INFO.INFO.fw_length); //calculate entire firmware CRC.

	//validate the firmware with header, and write to internal flash.
	if(bootCRC == FW_INFO.INFO.fw_crc)
	{

		BootWriteIn(); //��ɴ�Ƭ���bootloader������д��Ƭ��flash��

		bootCRC = CRC_calc((uint8_t*)BOOTLOADER_ADDR, (uint8_t*)(BOOTLOADER_ADDR + FW_INFO.INFO.fw_length - 1)); //FW_STARTADDR >> BOOTLOADER_ADDR

		if(bootCRC == FW_INFO.INFO.fw_crc)//У��ղ�д���bootloader����ȷ��
		{

			JudgeFlashBusy();

			FlashSectorErase( 0 );

			JudgeFlashBusy();

			RESET_MCU();//���и�λ
		}
		else//���д���ˣ�������д4��
		{
			while(retry < 4)
			{
				retry++;
				BootWriteIn();
				bootCRC = CRC_calc((uint8_t*)BOOTLOADER_ADDR, (uint8_t*)(BOOTLOADER_ADDR + FW_INFO.INFO.fw_length - 1)); //FW_STARTADDR >> BOOTLOADER_ADDR

				if(bootCRC == FW_INFO.INFO.fw_crc)
				{
					JudgeFlashBusy();

					FlashSectorErase( 0 );

					JudgeFlashBusy();

					RESET_MCU(); //���и�λ
				}
			}

			DealError();//�����д4�κ��ǲ��ԣ�ֱ�ӽ����쳣����
		}
	}
	else
	{

		DealError();
	}
}


void McuFwUpdate(void)
{
	unsigned int  calcCRC = 0;
	static uint8_t retry = 0;

	calcCRC = FlashCRC(0 + 16, FW_INFO.INFO.fw_length); //calculate entire firmware CRC.

	//validate the firmware with header, and write to internal flash.
	if(calcCRC == FW_INFO.INFO.fw_crc)
	{

		InsideFlashWR(); //��ȡƬ��flash�е����ݲ�д��Ƭ�ڡ�����Ҫ����Ƭ��flash��CRC
		calcCRC = CRC_calc((uint8_t*)MCUAPP_ADDR, (uint8_t*)(MCUAPP_ADDR + FW_INFO.INFO.fw_length - 1)); //BOOTLOADER_SIZE >> MCUAPP_ADDR

		if(calcCRC == FW_INFO.INFO.fw_crc)//���д�뵽Ƭ��Flash��App��CRC��Ƭ���ļ�ͷ�д洢��CRC��һ��Ҫ����д
		{
			//calculate the built-in flash MCU-App flash range CRC and write to last 4 bytes.
			CalcInsideFlashCRCandWrIn();

            /* erase ext flash upgrade firmware header sector to avoid repeat upgrade */
			JudgeFlashBusy();

			FlashSectorErase( 0 ); //erase ext flash first sector.

			JudgeFlashBusy();

			RESET_MCU(); //reset to bootup again. Why did not switch to MCU-App or DealError()?
		}
		else//���Ƶ�Ƭ��flash�����ݵ�CRC��Ƭ��flash���ļ�ͷ�м�¼�Ĳ�һ��,ֱ���˳����쳣����
		{
			while(retry < 3)
			{
				retry++;
				InsideFlashWR();//������дһ��
				calcCRC = CRC_calc((uint8_t*)MCUAPP_ADDR, (uint8_t*)(MCUAPP_ADDR + FW_INFO.INFO.fw_length - 1)); //BOOTLOADER_SIZE >> MCUAPP_ADDR

				if(calcCRC == FW_INFO.INFO.fw_crc)
				{
					CalcInsideFlashCRCandWrIn();

					JudgeFlashBusy();

					FlashSectorErase( 0 );

					JudgeFlashBusy();

					RESET_MCU();
				}
			}

			DealError(); //�����ظ�д4�Σ����ǲ��ԵĻ����ͻ���ȥ��λ����������ߵ���������������
		}
	}
	else//Ƭ��flash���ݵ�CRC�����ļ�ͷ�д洢�Ĳ�һ��
	{
		DealError();
	}
}

void ReadBLEFwFromExflashAndWrIn(void)
{

	unsigned char bleFwBuf[BLE_FRAME_SIZE] = {0}; //128 >> BLE_FRAME_SIZE
	int integer = 0;
	int decimal = 0;
	int temp    = 0;
	volatile unsigned int i = 0;
	static char page = 0;


	integer = FW_INFO.INFO.fw_length / sizeof (bleFwBuf); //BLE_FRAME_SIZE; // total will be 1952*128
	decimal = FW_INFO.INFO.fw_length % sizeof (bleFwBuf); // should be 0 for BLE padded.

	for( i = 0; i < integer; i++)
	{

		memset(CopyRxBuff, 0, 64);

		JudgeFlashBusy();

		FlashRead((FW_BIN_STARTADDR + 128 * i), bleFwBuf, sizeof (bleFwBuf)); //read BLE_FRAME_SIZE(128) every time.

		JudgeFlashBusy();

		WriteCC254xFlash(bleFwBuf);

		char countTx = 60;

		while(!BLE_Responsed  && countTx ) //��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue
		{
			countTx--;
			SysCtlDelay(10000);//����������ʱ�ٻᶪ����
		}//ÿ����1�����ݣ�Ҫ���㹻��ʱ����BLE��Ӧ��Ȼ���Ӧ���ͻ��ˡ�

		if(countTx > 0)//˵��û�г�ʱ
		{

			if( (BLE_DevChip.BLE_Device.FW_VER1 == 2) && (BLE_DevChip.BLE_Device.FW_VER2 == 0))
			{
				//���ݾɰ汾��BLE��bootloader�����ﲻ������
			}
			else
			{
				page = (unsigned char)(i / 16);

				if((CopyRxBuff[0] == 0x3c ) && (CopyRxBuff[2] == 0x05))//�鿴���ص�ҳ�Բ��ԣ����ҳ����ֱ�ӵ��豸APP
				{
					if(CopyRxBuff[5] != (page + 1))
					{
						DealError();
					}
				}
			}
		}

		BLE_Responsed = false;

		if(i % 5 == 0)//ÿд5��ι��
		{
			WDOG_Feed();
		}

	}

	if(decimal > 0)
	{
		temp = decimal % 4;//BLE��flashҲҪ4���ֽڶ���д��

		FlashRead((FW_BIN_STARTADDR + 128 * integer), bleFwBuf, decimal); //����ֻ��ʣ�µ�decimal�ֽ�

		JudgeFlashBusy();

		if(temp == 0)
		{

			MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

			WDOG_Feed();

			while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

			BLE_Responsed = false;
		}
		else
		{
			WDOG_Feed();

			switch(temp)
			{
				case 1:
				{
					bleFwBuf[decimal + 0] = 0xff;
					bleFwBuf[decimal + 1] = 0xff;
					bleFwBuf[decimal + 2] = 0xff;

					MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

					while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

					BLE_Responsed = false;

				}
				break;

				case 2:
				{
					bleFwBuf[decimal + 0] = 0xff;
					bleFwBuf[decimal + 1] = 0xff;

					MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

					while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

					BLE_Responsed = false;

				}
				break;

				case 3:
				{
					bleFwBuf[decimal + 0] = 0xff;

					MyLEUARTSentByDma(UART_CMD_UPGRADE_DATA, bleFwBuf, decimal);

					while(!BLE_Responsed);//��BLE�յ����ݺ󣬻��mcuһ��Ӧ�����ж��������⵽���Ӧ����Ѹñ�����λtrue

					BLE_Responsed = false;
				}
				break;

				default:
					break;
			}
		}
	}
}


void BleFwUpdate(void)
{
	uint16_t calcCRC = 0;
	uint8_t  bleUpdSta = 0;
	static int8_t resetCount = 0;
	static int8_t changeCount = 0;
	static int8_t startCmdCount = 0;
	static int8_t stopCount = 0;
	static int8_t rebootCount = 3;

	calcCRC = FlashCRC(0 + 16, FW_INFO.INFO.fw_length); //calculate ext flash image CRC.

	if(calcCRC == FW_INFO.INFO.fw_crc)
	{
		while(1)
		{
			switch(bleUpdSta)
			{
				case CHECK_BLE_ONLINE: //0
				{
					if(BLE_ONLINE == true) //set by LEUART0_IRQHandler() get data.
					{
						bleUpdSta = CHECK_BLE_STATE;
						BLE_ONLINE = false;
					}
					else
					{
						resetCount++;
						BLE_RST_L();
						SysCtlDelay(8000 * SYSCLOCK);
						BLE_RST_H();

						SysCtlDelay(800000);
						SysCtlDelay(800000);

						bleUpdSta = CHECK_BLE_ONLINE;
					}

					if(resetCount == 3)
					{
						bleUpdSta = ERROR;
					}
				}
				break;

				case CHECK_BLE_STATE:
				{
					if(BLE_DevChip.BLE_Device.WORKSTA == BOOT_STATE)
					{
						SysCtlDelay(800000);//it needs some time between sending two commands
						bleUpdSta =	SEND_START_CMMD;
					}
					else if(BLE_DevChip.BLE_Device.WORKSTA == APP_STATE)
					{
						changeCount++;
						AppToBoot(); //set BLE state App to Boot.
						SysCtlDelay(90000);//waiting for interrupt
						bleUpdSta = CHECK_BLE_STATE;
					}

					if(changeCount > 5)
					{
						bleUpdSta = ERROR;
					}
				}
				break;

				case SEND_START_CMMD:
				{

					MyBLE_Update_Start();
					SysCtlDelay(90000);//waiting for interrupt.The BLE will make HAL_SYSTEM_RESET();

					if(BLE_Responsed == true)
					{
						bleUpdSta = WRITE_BLE_FLASH;
						BLE_Responsed = false; //clean flag for next stage.
					}
					else
					{
						startCmdCount++;
						SysCtlDelay(90000);//waiting for interrupt
						bleUpdSta = SEND_START_CMMD;
					}

					if(startCmdCount > 5)
					{
						bleUpdSta = ERROR;
					}
				}
				break;

				case WRITE_BLE_FLASH:
				{
					ReadBLEFwFromExflashAndWrIn();
					bleUpdSta = SEND_STOP_CMMD;
				}
				break;

				case SEND_STOP_CMMD:
				{
					SysCtlDelay(5000);//wait some time before sending stop command
					BLE_Update_End(FW_INFO.INFO.fw_crc);//this command needs much time for response.

					//the response is "3c 08 05 05 ....",and device information
					waitBleStopCommandresponse = 0;

					while((waitBleStopCommandresponse <= 300) && (BLE_Responsed != true));//�ȴ�4.5�롣

					if(waitBleStopCommandresponse < 300)
					{
						waitBleStopCommandresponse = 0;

						while((bleStatusFlag != 1) &&  rebootCount)
						{
							rebootCount--;
							SysCtlDelay(900000);
						}

						if(rebootCount > 0)
						{
							JudgeFlashBusy();

							FlashSectorErase( 0 );  //clean ext flash first sector(header) to avoid reupgrade.

							JudgeFlashBusy();
						}

						DealError();//updated ,goto  app of main MCU

					}
					else
					{
						stopCount++;
						bleUpdSta = SEND_STOP_CMMD;
					}

					if(stopCount > 5)
					{
						bleUpdSta = ERROR;
					}

				}
				break;

				case ERROR:
				{
					DealError();
				}
				break;

				default:
					break;
			}
		}
	}
	else
	{
		DealError();
	}
}


int main(void)
{

	unsigned int  fwHeadCRC;
	uint8_t  deviceTypeCheck = 0;//

#ifndef DEBUG
	SCB->VTOR = 0x20000000; //loader will copy the code to ram 0x20000000 and execute.
#endif
#if defined(SHOWLED)
    
    CMU_ClockEnable(cmuClock_GPIO, true); //turn on GPIO clock.
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
	GPIO_PinModeSet(LED_GPIOPORT, LED_PIN, gpioModePushPull, 1); //set LED gpio mode.
#elif (BOARD_TYPE==2)
	GPIO_PinModeSet(LED_GPIOPORT, LEDR_PIN, gpioModePushPull, 1); //set LEDR gpio mode.
	GPIO_PinModeSet(LED_GPIOPORT, LEDB_PIN, gpioModePushPull, 0); //set LEDB gpio mode.
	GPIO_PinModeSet(LED_GPIOPORT, LEDG_PIN, gpioModePushPull, 1); //set LEDG gpio mode.
	//GPIO_PinModeSet(LED_GPIOPORT, LED_PIN, gpioModePushPull, 0); //set LED gpio mode.
#endif
    SysCtlDelay(10000); //5000
#if (BOARD_TYPE==0 || BOARD_TYPE==1)
    LED_OFF();
#elif (BOARD_TYPE==2)
    LEDR_OFF();
    LEDB_OFF();
    LEDG_OFF();
#endif

#endif
    
	LETIMER_setup(); //config LETimer0, use in LED blink freq at difference case and watchdog feed freq.
	BLE_INIT(); //turn on MCU to BLE CLK_32K clock output and LEuart dma.
#if (0)    
	FLASH_POWER_BACK(); //Atus: duplicated sub-process in M25Pxx_INIT.
	SysCtlDelay(10000);
#endif
	M25Pxx_INIT();  //initialize ext flash chip: power and SPI(USART0) interface and read identifier.
	SysCtlDelay(10000);

	WDOG_Init(&init);
    
    /* read header from ext flash */
	JudgeFlashBusy();
	FlashRead(FW_STARTADDR , FW_INFO.INFO_BUF , FWHEAD_LENGTH); //Atus: should check the flash status.
	JudgeFlashBusy();


	fwHeadCRC = CRC_calc(&FW_INFO.INFO_BUF[0], &FW_INFO.INFO_BUF[13]);//calculate header crc [0x0..0xd]

    /* check upgrade board type */
	if((FW_INFO.INFO_BUF[10] == 0x40)  || (FW_INFO.INFO_BUF[10] == 0x41) || (FW_INFO.INFO_BUF[10] == 0x42))//check devicetype I4,I4S.I4B
	{
		deviceTypeCheck = 1;
	}

	if(fwHeadCRC == FW_INFO.INFO.Head_CRC) //header crc correct.
	{

		switch(FW_INFO.INFO.fw_type)
		{
        case FWTYPE_MCU://1:mcu app firmware
			{
				if(deviceTypeCheck)
				{
					McuFwUpdate();
				}
				else
				{
					DealError();
				}
			}
			break;

        case FWTYPE_BLE : //2:BLE firmware
			{
				BleFwUpdate();
			}
			break;

		case FWTYPE_BOOT: //3:bootloader self upgrade
			{
				BootUpdate();
			}
			break;

		default: //Not specify firmware type.
			{
				DealError();
				break;
			}
		}
	}
	else //header CRC mismatch.
	{
		DealError();
	}

//  while(1);//loader����Զ����ִ�е��������ϵͳ������loader����
	/*
	  �þ䲻��Ҫ�������һ����д��Ƭ��flash��û�����ݣ��ᵼ�������ļ�ͷCRC�ļ��ͨ��
	  ���ǣ���û�к��ʵ��ļ�ͷ������ͻ��˵������ôϵͳ����Զ����loader�г���ȥ��
	  ����취��������switch�е�default�м���dealError������ʹϵͳ�Զ���ת��MCU�ڲ�CRC������
	  �ж���Ҫִ��App���Ǹ�λ��
	  */

}


bool JudgeFlashBusy()
{
	bool isBusy = true;
	unsigned int judgeFlashbusy = 0;

	while(IsFlashBusy() && (judgeFlashbusy < 65530))
	{
		judgeFlashbusy++;
	}

	if(judgeFlashbusy > 65530)
	{
		//flash always busy ,so go to the APP
		DealError();
	}
	else
	{
		isBusy = false;
	}

	return isBusy;
}


