
#ifndef _M25PXX_H_
#define _M25PXX_H_

#include <stdbool.h>
#include <stdio.h>
//#include "debug.h"
#include <string.h>
#include "efm32.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"


#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE /* undefine this macro to read One-Byte Signature*/

#define USE_JEDEC_M25P32
#ifdef USE_JEDEC_M25P128
   #ifdef  USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
        #define EXPECTED_DEVICE (0x2018)    /* Device Identification for the USE_M25P28 */
   #else
        #define EXPECTED_DEVICE (0x16)      
   #endif
 
   //#define FLASH_SIZE (0x1000000)             /* Total device size in Bytes 16M*/
   #define FLASH_BYTES_PER_PAGE (0x100)       /*Page size 256byte*/
   //#define FLASH_PAGE_COUNT   (0x10000)       /* Total Pages 64k*/
   //#define FLASH_SECTOR_SIZE (0x40000)       /* Sector size 256K*/
   #define FLASH_SECTOR_COUNT (0x40)        /* Sectors 64*/
   #define FLASH_WRITE_BUFFER_SIZE 0x100    /* Write Buffer = 256 bytes */ 
   #define FLASH_MWA 1                      /* Minimum Write Access */ 
   #define BE_TIMEOUT (0x160)               /* Timeout in seconds suggested for Bulk Erase Operation*/
   #define NO_DEEP_POWER_DOWN_SUPPORT       /* No support for Deep Power-down feature*/    
#endif


#ifdef USE_JEDEC_M25P32
   
   #define EXPECTED_DEVICE (0x2016)      

   //#define FLASH_SIZE (0x400000)             /* Total device size in Bytes 4M*/
   #define FLASH_BYTES_PER_PAGE (0x100)       /*Page size 256byte*/
   //#define FLASH_PAGE_COUNT   (0x4000)       /* Total Pages 16k*/
   //#define FLASH_SECTOR_SIZE (0x10000)       /* Sector size 64K*/
   #define FLASH_SECTOR_COUNT (0x40)        /* Sectors 64*/
   #define FLASH_WRITE_BUFFER_SIZE 0x100    /* Write Buffer = 256 bytes */ 
   #define FLASH_MWA 1                      /* Minimum Write Access */ 
   #define BE_TIMEOUT (0x160)               /* Timeout in seconds suggested for Bulk Erase Operation*/
   #define NO_DEEP_POWER_DOWN_SUPPORT       /* No support for Deep Power-down feature*/    
#endif

#ifdef USE_JEDEC_M25P64
   
   #ifdef  USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
        #define EXPECTED_DEVICE (0xC237)
   #else
        #define EXPECTED_DEVICE (0x37)      
   #endif

   //#define FLASH_SIZE (0x800000)          /* Total device size in Bytes 8M*/
   #define FLASH_BYTES_PER_PAGE (0x100)     /*Page size 256byte*/
   //#define FLASH_PAGE_COUNT   (0x8000)     /* Total Pages 32k*/
   //#define FLASH_SECTOR_SIZE (0x1000)     /* Sector size 4K*/
   #define FLASH_SECTOR_COUNT (0x800)       /* Sectors 2048*/
   #define FLASH_WRITE_BUFFER_SIZE 0x100    /* Write Buffer = 256 bytes */ 
   #define FLASH_MWA 1                      /* Minimum Write Access */ 
   #define BE_TIMEOUT (0x160)               /* Timeout in seconds suggested for Bulk Erase Operation*/
   #define NO_DEEP_POWER_DOWN_SUPPORT       /* No support for Deep Power-down feature*/    
#endif


typedef enum { 
	Flash_AddressInvalid, 
	Flash_MemoryOverflow, 
	Flash_PageEraseFailed, 
	Flash_PageNrInvalid, 
	Flash_SectorNrInvalid, 
	Flash_BlockNrInvalid, 
	Flash_FunctionNotSupported,
	Flash_NoInformationAvailable,
	Flash_OperationOngoing, 
	Flash_OperationTimeOut, 
	Flash_ProgramFailed, 
	Flash_WrongType,
	Flash_Success
} ReturnType; 

enum
{
	SPI_FLASH_SRWD	= 0x80,				// Status Register Write Protect
	SPI_FLASH_BP2	= 0x10,				// Block Protect Bit2
	SPI_FLASH_BP1	= 0x08,				// Block Protect Bit1
	SPI_FLASH_BP0	= 0x04,				// Block Protect Bit0
	SPI_FLASH_WEL	= 0x02,				// write enable latch
	SPI_FLASH_WIP	= 0x01				// write/program/erase in progress indicator
};

typedef enum
{
	//Instruction set
	SPI_FLASH_INS_WREN        = 0x06,		// write enable
	SPI_FLASH_INS_WRDI        = 0x04,		// write disable
	SPI_FLASH_INS_RDSR        = 0x05,		// read status register
	SPI_FLASH_INS_WRSR        = 0x01,		// write status register
	SPI_FLASH_INS_READ        = 0x03,		// read data bytes
	SPI_FLASH_INS_FAST_READ   = 0x0B,		// read data bytes at higher speed
	SPI_FLASH_INS_PP          = 0x02,		// page program
	SPI_FLASH_INS_SE          = 0x20,		// sector erase
	SPI_FLASH_INS_BE          = 0xD8,		// block erase

	SPI_FLASH_INS_RES         = 0xAB,		
	//#ifndef NO_DEEP_POWER_DOWN_SUPPORT
	SPI_FLASH_INS_DP          = 0xB9,		// deep power-down
	//#endif

	SPI_FLASH_INS_RDID        = 0x9F,		// read identification
    
	SPI_FLASH_INS_CE          = 0xC7		// chip erase
} M25PXX_COMMAND_SET;


#define HRWritingInterval     240
#define TempWritingInterval   60

//extern unsigned char Flash_ID[4];
//extern unsigned long systemStatus.flashInfo.flashSize, 
//extern unsigned long systemStatus.flashInfo.sectors,systemStatus.flashInfo.sectorSize;  
extern unsigned char FLASH_SECTOR_Num,erasing_sector_add;
extern unsigned long FlashSizeLeft,FlashWritingP;
//extern unsigned short BlockCount;


// =================================================================
// º¯Êý

void M25Pxx_INIT(void);

// ------------------------------
// power management
void FLASH_POWER_DOWN();
bool FLASH_POWER_BACK();

void FLASH_DEEP_SLEEP(void);
void FLASH_SLEEP_BACK(void);

// ------------------------------
// status
void waitFlashOperaDone();
unsigned char IsFlashBusy();

// ------------------------------
// read
ReturnType FlashRead(unsigned long address, unsigned char* data_ptr, unsigned short length);

// ------------------------------
// write
ReturnType FlashProgram( unsigned long udAddr, unsigned char *pArray, unsigned long udNrOfElementsInArray );
//ReturnType FlashPageProgram(unsigned long address, unsigned char* data_ptr, unsigned short length);

// ------------------------------
// erase
ReturnType  FlashSectorErase( int sectorNo );
ReturnType  FlashBlockErase( int blockNo );
ReturnType  FlashBulkErase( void );

// ------------------------------
uint16_t FlashCRC(int address,int length);

#endif
