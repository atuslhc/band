/**************************************************************************//**
 * @file
 * @brief Bootloader Configuration.
 *    This file defines how the bootloader is set up.
 * @author Energy Micro AS
 * @version 1.02
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2011 Energy Micro AS, http://www.energymicro.com</b>
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
#ifndef CONFIG_H
#define CONFIG_H

/************ DEBUG #define's ******************************************/

// 2013.10.25
/** The size of the bootloader flash image */
#define BOOTLOADER_ADDR           (0)
#define BOOTLOADER_SIZE           (38*1024)       /* 0x9800=38KB */ //The space must be enough with bootload extension to avoid bootload overrite App.
#define MCUAPP_ADDR               BOOTLOADER_SIZE
   
/** The maximum flash size of any EFM32 part */
#define MAX_SIZE_OF_FLASH         (1024*1024)     /* 1 MB */

/** The size of a mass erase block */
#define MASSERASE_BLOCK_SIZE      (512*1024)      /* 512 KB */

#define EFM32_ADDRESS (0x0FE081F0)  //refer EFM32WG-RM chapter 5.6, the unique number.(0x0FE080C0~0x0FE081FF)
#define EFM32_INFO_FLASH (0x0FE081F8)  //EFM32 Flash size, kbyte count as unsigned integer. eg. 256
#define EFM32_PART_FAMILY (0x0FE081FE) //EFM32 part family number (Gecko=71, .. Wonder Gecko=75)

#define EFM32_INFOBLOCK   0x0FE00000 //efm32 information memory user data start address. (0x0FE00000~0x0FE007FF)

#ifdef ewarm //compiler flag, apply in release mode.
#pragma pack(1)
#endif
#ifdef rvmdk
__packed 
#endif
union _CHIP{
  struct _DEVICE{  //refer EFM32WG-RM chapter5.6
    uint32_t unique0;   //unique number0
    uint32_t unique1;   //unique number1
    uint16_t memFlash;  //flash size,kbyte count.
    uint16_t memRAM;    //ram size, kytes count.
    uint16_t partNum;   //EFM32 part number.
    uint8_t partFamily; //EFM32 part family number (Gecko=71, Giant Gecko=72, Tiny Gecko=73, Leopard Gecko=74, Wonder Gecko=75).
    uint8_t prodRev;    //EFM32 Production ID.
  }Device;
  uint8_t EFM32DeviceInfo[16];
};
extern union _CHIP DevChip;

#ifdef ewarm
#pragma pack()
#endif
//升级状态
#define UPGRADE_IDLE       (0)
#define UPGRADE_RUNNING    (1)
#define UPGRADE_DONE       (2)
#define UPGRADE_READDEVICE (3)
#define UPGRADE_RESTART    (4)
#define UPGRADE_BOOT       (5)

extern uint32_t GulUpgradeFlag;


#ifdef ewarm
#pragma pack(1)
#endif
#ifdef rvmdk
__packed 
#endif
//Star Send [Start Upgrade Cmd:1Byte] + [Start Address 4byte] + [End Address 4byte] +
//[Packet Cnt 4byte] + [FlashCRC 2byte] + [Firmware Version 4byte]
//2013.10.25
//
// UPDATE_TYPE = 1 (Update App)
// UPDATE_TYPE = 2 (Update BOOT)

#define  UPDATE_APP 1
#define  UPDATE_BOOT 2

//升级参数
union _UPGRADE_PARAM{
  struct _PARAM{
    uint8_t cmd;
    uint32_t StartAddr;
    uint32_t EndAddr;
    uint32_t PacketCnt;
    uint16_t FlashCRC;
    uint32_t FirmwareVersion;
	uint16_t UPDATE_TYPE;
    uint8_t rev[60-19];
  }Param;
  uint8_t data[60];
};
extern union _UPGRADE_PARAM gUpgrade;

#ifdef ewarm
#pragma pack()
#endif

#endif
