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

/** The size of the bootloader flash image */
#define BOOTLOADER_SIZE           (28*1024)       /* 28 KB */

/** The maximum flash size of any EFM32 part */
#define MAX_SIZE_OF_FLASH         (1024*1024)     /* 1 MB */

/** The size of a mass erase block */
#define MASSERASE_BLOCK_SIZE      (512*1024)      /* 512 KB */

#define EFM32_ADDRESS (0x0FE081F0)

#ifdef ewarm
#pragma pack(push, 1)
#endif
#ifdef rvmdk
__packed 
#endif
union _CHIP{
  struct _DEVICE{
    uint32_t unique0;
    uint32_t unique1;
    uint16_t memFlash;
    uint16_t menRAM;
    uint16_t Number;
    uint8_t Family;
    uint8_t Rev;
  }Device;
  uint8_t EFM32DeviceInfo[16];
};
extern union _CHIP DevChip;

#ifdef ewarm
#pragma pack(pop)
#endif

#define UPGRADE_IDLE       (0)
#define UPGRADE_RUNNING    (1)
#define UPGRADE_DONE       (2)
#define UPGRADE_READDEVICE (3)
#define UPGRADE_RESTART    (4)
#define UPGRADE_BOOT       (5)
#define APP_UPGRADE        (0x20)

#define BLE_SWITCH_TO_BOOTLOAD  (0x40)
#define BLE_UPGRADE_START 0x41
#define BLE_UPGRADE_RUNNING (0x42)
#define BLE_UPGRADE_DONE (0x43)

#define DATA_GET_LATEST_TIMESTAMP 	(0x50)
#define DATA_PREPARE_UPLOAD		 	(0x51)
#define DATA_START_UPLOAD		 	(0x52)
//#define DATA_CONTINUE_UPLOAD		(0x53)

// 下面两个命令由设备发送，报告数据上传的状态
#define DATA_BATCH_UPLOAD_DONE 		(0x58)
#define DATA_UPLOAD_DONE 			(0x59)


#ifdef ewarm
#pragma pack(push, 1)
#endif
#ifdef rvmdk
__packed 
#endif
//Star Send [Start Upgrade Cmd:1Byte] + [Start Address 4byte] + [End Address 4byte] +
//[Packet Cnt 4byte] + [FlashCRC 2byte] + [Firmware Version 4byte]
union _UPGRADE_PARAM{
  struct _PARAM{
    uint8_t cmd;
    uint32_t StartAddr;
    uint32_t EndAddr;
    uint32_t PacketCnt;
    uint16_t FlashCRC;
    uint32_t FirmwareVersion;
    uint8_t rev[60-17];
  }Param;
  uint8_t data[60];
};
extern union _UPGRADE_PARAM gUpgrade;

#ifdef ewarm
#pragma pack(pop)
#endif

#endif
