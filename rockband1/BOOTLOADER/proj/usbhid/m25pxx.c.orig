#include "main.h"

#include "m25pxx.h"

#include "common_vars.h"


SYSTEM_STATUS systemStatus;
FLASH_INFO* pFlashInfo = &(systemStatus.flashInfo);
volatile FLASH_STORAGE_INDICATOR* pFlashStorageIndicator = &(systemStatus.flashStorageIndicator);


//unsigned char Flash_ID[4];
unsigned char error_happened = 0;
unsigned long test_address = 0;
//Flash_saving_format saving_buff[2];
//volatile uint8_t flashStatus = ReadytoWrite;
//bool blMemoryFull = false;
//bool systemStatus.blFlashOnline = false;

//unsigned long systemStatus.flashInfo.flashSize,
//unsigned long systemStatus.flashInfo.sectors, systemStatus.flashInfo.sectorSize;
//unsigned char FLASH_SECTOR_Num,
//unsigned short BlockCount;
unsigned char erasing_sector_add;
unsigned long FlashSizeLeft, FlashWritingP;

ReturnType FlashReadStatusRegister(unsigned char* data_ptr);
void FlashWritingTest(void);

bool IsFlashPowerOn=false;

void M25Pxx_INIT(void)
{
    
    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	
   	GPIO_PinModeSet(FLASH_PW_PORT, FLASH_PW_PIN, gpioModePushPull, 1); /* Power */
	GPIO_DriveModeSet(FLASH_PW_PORT, gpioDriveModeHigh);
	FLASH_PW_ON();
//	vTaskDelay(1);该函数是task中的，这里没有用系统，不用这个函数。

 	GPIO_PinModeSet(FLASH_SPI_CS_GPIOPORT, FLASH_SPI_CSPIN, gpioModePushPull, 1); /* CS */
	GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_MOSIPIN,gpioModePushPull, 1);  /* MOSI */
	GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_MISOPIN,gpioModeInputPull,1);  /* MISO */
	GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_CLKPIN,gpioModePushPull, 1);  /* Clock */
  
	init.baudrate	   = 8000000;
	init.databits	   = usartDatabits8;
	init.msbf		   = 1;
	init.master	   = 1;
	init.clockMode    = usartClockMode3;//usartClockMode0
	init.prsRxEnable  = 0;
	init.autoTx	   = 0;
	USART_InitSync(USART0, &init);
	  
	USART0->ROUTE =USART_ROUTE_TXPEN|USART_ROUTE_RXPEN|USART_ROUTE_CLKPEN|USART_ROUTE_LOCATION_LOC0;

	//FLASH_SLEEP_BACK();

    SysCtlDelay(5000);//延时一段时间，使信号稳定保证flash初始化成功
  	FlashReadManufacturerIdentification(pFlashInfo->chipID.rawData);

	GetFlashCapacity(pFlashInfo->chipID.rawData[2]);
        

#if 0
	FlashBulkErase();
	//flashStatus = BulkErasing;
#endif
}

void SPI_Send_Byte (unsigned char datum)
{
	uint8_t rxed_byte;

	/* Check that transmit buffer is empty */
	while (!(FLASH_SPI->STATUS & USART_STATUS_TXBL));

	FLASH_SPI->TXDATA = (uint32_t)datum;

	while(!(FLASH_SPI->STATUS & USART_STATUS_TXC));

	//if not receive empty,receive
	if ((FLASH_SPI->STATUS & USART_STATUS_RXDATAV) != 0)
		rxed_byte = FLASH_SPI->RXDATA;
}

unsigned char SPI_Send_And_Read_Byte (unsigned char datum)
{
	unsigned char rxed_byte;

	//if not receive empty,receive
	if ((FLASH_SPI->STATUS & USART_STATUS_RXDATAV) != 0)
		rxed_byte = FLASH_SPI->RXDATA;

	//send datum
	while (!(FLASH_SPI->STATUS & USART_STATUS_TXBL));

	FLASH_SPI->TXDATA = (uint32_t)datum;

	while(!(FLASH_SPI->STATUS & USART_STATUS_TXC));

	//receive rxed_byte
	while(!(FLASH_SPI->STATUS & USART_STATUS_RXDATAV));

	rxed_byte = FLASH_SPI->RXDATA;

	return (rxed_byte);
}

void SPI_Send_And_Read_Bytes (unsigned short num_bytes, unsigned char* bytes_to_tx, unsigned char* rxed_bytes)
{
	unsigned short bytes_sent = 0;

	//if not receive empty,receive
	if ((FLASH_SPI->STATUS & USART_STATUS_RXDATAV) != 0)
		rxed_bytes[0] = FLASH_SPI->RXDATA;

	//send num_bytes,and receive num_bytes
	while (bytes_sent < num_bytes)
	{
		while (!(FLASH_SPI->STATUS & USART_STATUS_TXBL));

		FLASH_SPI->TXDATA = bytes_to_tx[bytes_sent];

		while (!(FLASH_SPI->STATUS & USART_STATUS_TXC))
		{
		}

		rxed_bytes[bytes_sent] = FLASH_SPI->RXDATA;

		bytes_sent++;
	}
}

void SPI_Send_Bytes (unsigned short num_bytes, unsigned char* bytes_to_tx)
{
	unsigned short bytes_sent = 0;
	unsigned char  rxed_byte;

	if ((FLASH_SPI->STATUS & USART_STATUS_RXDATAV) != 0)
		rxed_byte = FLASH_SPI->RXDATA;

	while (bytes_sent < num_bytes)
	{
		FLASH_SPI->TXDATA = bytes_to_tx[bytes_sent];

		while (!(FLASH_SPI->STATUS & USART_STATUS_TXC))
		{
		}

		rxed_byte = FLASH_SPI->RXDATA;

		bytes_sent++;
	}

	rxed_byte = rxed_byte;
}


void SPI_Read_Bytes (unsigned short num_bytes, unsigned char byte_to_tx, unsigned char* rxed_bytes)
{
	unsigned short bytes_sent = 0;

	if ((FLASH_SPI->STATUS & USART_STATUS_RXDATAV) != 0)
		*rxed_bytes = FLASH_SPI->RXDATA;

	while (bytes_sent < num_bytes)
	{
		FLASH_SPI->TXDATA = byte_to_tx;

		while (!(FLASH_SPI->STATUS & USART_STATUS_TXC))
		{
		}

		rxed_bytes[bytes_sent] = FLASH_SPI->RXDATA;

		bytes_sent++;
	}
}

unsigned char IsFlashBusy()
{
	unsigned char ucSR;

	FlashReadStatusRegister(&ucSR);

	if(ucSR & SPI_FLASH_WIP)//ucSR中数据为1表示忙。
		return 1;
	else
		return 0;
}

ReturnType FlashTimeOut(unsigned long udSeconds)
{
	static unsigned long udCounter = 0;

	if (udSeconds == 0)
	{
		udCounter = 0;
	}

	if (udCounter == (udSeconds * 10000))
	{
		udCounter = 0;
		return Flash_OperationTimeOut;
	}
	else
	{
		udCounter++;
		return Flash_OperationOngoing;
	}
}

ReturnType FlashWriteEnable(void)
{
	FLASH_CS_L();
	SPI_Send_Byte (SPI_FLASH_INS_WREN);
	FLASH_CS_H();

	return Flash_Success;
}


ReturnType FlashWriteDisable(void)
{
	FLASH_CS_L();
	SPI_Send_Byte (SPI_FLASH_INS_WRDI);
	FLASH_CS_H();

	return Flash_Success;
}

ReturnType FlashReadStatusRegister(unsigned char* data_ptr)
{
	unsigned char cmd_to_read [2];
	unsigned char readback [2];

	cmd_to_read[0] = SPI_FLASH_INS_RDSR;
	cmd_to_read[1] = 0x00;

	FLASH_CS_L();
	SPI_Send_And_Read_Bytes(2, cmd_to_read, readback);
	*data_ptr = readback[1];
	FLASH_CS_H();

	return Flash_Success;
}

ReturnType FlashWriteStatusRegister(unsigned char contents)
{
	FlashWriteEnable();

	FLASH_CS_L();
	SPI_Send_Byte (SPI_FLASH_INS_WRSR);
	SPI_Send_Byte (contents);
	FLASH_CS_H();
	return Flash_Success;
}

#if 0
void FlashWritingTest(void)
{
	unsigned short i, j;
//unsigned long test_address=0;
	unsigned char* des;
	unsigned char* sou;

	des = (unsigned char*)&saving_buff[0];
	sou = (unsigned char*)&saving_buff[1];

	FlashBulkErase();
	flashStatus = BulkErasing;

	while(flashStatus == BulkErasing);

	for(i = 0; i < WriteBlockSize; i++)
	{
		*(des + i) = i;
		*(sou + i) = 0;
	}

	for(i = 0; i < systemStatus.flashInfo.blocks; i++)
	{
		FlashPageProgram(test_address, des, 256);
		SysCtlDelay(10 * 1500 / 3);
		FlashPageProgram(test_address + 256, des + 256, 256);
		SysCtlDelay(10 * 1500 / 3);
		//FlashProgram(test_address,des,WriteBlockSize);
		//__delay_cycles(10*5000);
		FlashRead(test_address, sou, WriteBlockSize);

		for(j = 0; j < WriteBlockSize; j++)
		{
			error_happened |=  *(des + j) ^ *(sou + j);

			if(error_happened != 0)
				error_happened = 5;
		}

		test_address += WriteBlockSize;
	}

}

void SearchLeftCapacity(void)
{
	unsigned long timestamp_temp = 0, timestamp_address = 0;
	unsigned short i;
	unsigned char first_flag = 1;
	unsigned long earlier_timestamp = 0xffffffff, earlier_add = 0;

	FlashSizeLeft = systemStatus.flashInfo.flashSize;
	timestamp_address = 0;

	for(i = 0; i < systemStatus.flashInfo.sectors; i++)
	{
		FlashRead(timestamp_address, (unsigned char*)&timestamp_temp, 4);

		if(timestamp_temp != 0xffffffff)
		{
			FlashSizeLeft -= WriteBlockSize;

			if(timestamp_temp > 0)
			{
				if(timestamp_temp < earlier_timestamp)
				{
					earlier_timestamp = timestamp_temp;
					earlier_add = timestamp_address;
					//so here we can get the sector address which will be erased first
				}
			}
		}
		else if(first_flag)
		{
			first_flag = 0;
			FlashWritingP = timestamp_address;
		}

		timestamp_address += WriteBlockSize;
	}

	if(FlashSizeLeft == 0)blMemoryFull = 1;

	erasing_sector_add = (unsigned char)(earlier_add / systemStatus.flashInfo.sectorSize);
	erasing_sector_add %= systemStatus.flashInfo.sectors;
}
#endif

void GetFlashCapacity(unsigned char data)
{
	switch(data)
	{
	   case 0x15: //2M bytes
			systemStatus.flashInfo.flashSize  = 0x200000;
			systemStatus.flashInfo.blockSize  = 0x10000;
			systemStatus.flashInfo.sectorSize = 0x100;
			systemStatus.flashInfo.sectors    = 0x2000;
			systemStatus.flashInfo.blocks     = 0x20;
			systemStatus.flashInfo.sectorsPerBlock = 0x100;

			systemStatus.blFlashOnline = true;
			systemStatus.blFlashPowerOn = true;
			break;
			
		case 0x16: // 4M bytes
						
			systemStatus.flashInfo.flashSize  = 0x400000;
			systemStatus.flashInfo.blockSize  = 0x8000;
			systemStatus.flashInfo.sectorSize = 0x800;
			systemStatus.flashInfo.sectors    = 0x400;
			systemStatus.flashInfo.blocks     = 0x40;
			systemStatus.flashInfo.sectorsPerBlock = 0x10;

			systemStatus.blFlashOnline = true;
			systemStatus.blFlashPowerOn = true;
			
			break;

		case 0x17: // 8M bytes
			systemStatus.flashInfo.flashSize  = 0x800000;
			systemStatus.flashInfo.blockSize  = 0x10000;
			systemStatus.flashInfo.sectorSize = 0x1000;
			systemStatus.flashInfo.sectors    = 0x800;
			systemStatus.flashInfo.blocks     = 0x80;
			systemStatus.flashInfo.sectorsPerBlock = 0x10;

			systemStatus.blFlashOnline = true;
			systemStatus.blFlashPowerOn = true;
			break;

	    case 0x37: // 8M bytes   MX25U6435E
			systemStatus.flashInfo.flashSize  = 0x800000;
			systemStatus.flashInfo.blockSize  = 0x10000;//64k
			systemStatus.flashInfo.sectorSize = 0x1000;//4k
			systemStatus.flashInfo.sectors    = 0x800;//2048个sector
			systemStatus.flashInfo.blocks     = 0x80;//128个block
			systemStatus.flashInfo.sectorsPerBlock = 0x10;

			systemStatus.blFlashOnline = true;
			systemStatus.blFlashPowerOn = true;

			break;
		case 0x18: // 16M bytes
		    systemStatus.flashInfo.flashSize  = 0x1000000;
			systemStatus.flashInfo.blockSize  = 0x10000;
			systemStatus.flashInfo.sectorSize = 0x1000;
			systemStatus.flashInfo.sectors    = 0x1000;
			systemStatus.flashInfo.blocks     = 0x100;
			systemStatus.flashInfo.sectorsPerBlock = 0x10;

			systemStatus.blFlashOnline = true;
			systemStatus.blFlashPowerOn = true;
		
		
			break;	

		default:
			systemStatus.blFlashOnline = false;
			break;
	}

}


void  FlashReadManufacturerIdentification(unsigned char* data_ptr)
{
	unsigned char byte_from_flash;

	FLASH_CS_L();

	SPI_Send_Byte (SPI_FLASH_INS_RDID);

	byte_from_flash = SPI_Send_And_Read_Byte(0x00);
	*data_ptr++ = (byte_from_flash);

	byte_from_flash = SPI_Send_And_Read_Byte(0x00);
	*data_ptr++ = byte_from_flash;

	byte_from_flash = SPI_Send_And_Read_Byte(0x00);
	*data_ptr++ = byte_from_flash;

	FLASH_CS_H();
	return;
}

ReturnType FlashRead(unsigned long address, unsigned char* data_ptr, unsigned short length)
{
	unsigned char cmd_to_read [4];

	if(!(address < systemStatus.flashInfo.flashSize))
		return Flash_AddressInvalid;

	FLASH_CS_L();

	cmd_to_read[0] = SPI_FLASH_INS_READ;
	cmd_to_read[1] = ((unsigned char)(address >> 16));
	cmd_to_read[2] = ((unsigned char)(address >> 8));
	cmd_to_read[3] = ((unsigned char)(address));
	SPI_Send_Bytes(4, cmd_to_read);

	SPI_Read_Bytes (length, 0x00, data_ptr);

	FLASH_CS_H();

	return Flash_Success;
}

ReturnType  FlashSectorErase( int SectorNo )
{
	unsigned char cmd_to_erase [4];
	unsigned long erase_address;

	if(SectorNo >= systemStatus.flashInfo.sectors) 
		return Flash_SectorNrInvalid;

	if(IsFlashBusy()) 
		return Flash_OperationOngoing;

	FlashWriteEnable();

	FLASH_CS_L();

	erase_address = (unsigned long)systemStatus.flashInfo.sectorSize * (unsigned long)(SectorNo);

	cmd_to_erase[0] = SPI_FLASH_INS_SE;
	cmd_to_erase[1] = (unsigned char)(erase_address >> 16);
	cmd_to_erase[2] = (unsigned char)(erase_address >> 8);
	cmd_to_erase[3] = (unsigned char)(erase_address & 0xFF);

	SPI_Send_Bytes(4, cmd_to_erase);

	FLASH_CS_H();

	return Flash_Success;
}

ReturnType FlashBlockErase( int BlockNo )
{
	unsigned char cmd_to_erase [4];
	unsigned long erase_address;

	if(BlockNo >= systemStatus.flashInfo.blocks) 
		return Flash_BlockNrInvalid;

	if(IsFlashBusy()) 
		return Flash_OperationOngoing;

	FlashWriteEnable();

	FLASH_CS_L();

	erase_address = (unsigned long)systemStatus.flashInfo.blockSize * (unsigned long)(BlockNo);

	cmd_to_erase[0] = SPI_FLASH_INS_BE;
	cmd_to_erase[1] = (unsigned char)(erase_address >> 16);
	cmd_to_erase[2] = (unsigned char)(erase_address >> 8);
	cmd_to_erase[3] = (unsigned char)(erase_address & 0xFF);

	SPI_Send_Bytes(4, cmd_to_erase);

	FLASH_CS_H();

	return Flash_Success;
}

ReturnType  FlashBulkErase( void )
{
	if(IsFlashBusy()) return Flash_OperationOngoing;

	FlashWriteEnable();

	FLASH_CS_L();

	unsigned char cmd[1];
	cmd[0] = SPI_FLASH_INS_CE;
	SPI_Send_Bytes(1, cmd);
//	while (!(FLASH_SPI->STATUS & USART_STATUS_TXBL));
//
//	FLASH_SPI->TXDATA = SPI_FLASH_INS_CE;
//
//	while(FLASH_SPI->STATUS & USART_STATUS_TXC);

	FLASH_CS_H();

	return Flash_Success;
}

void FLASH_POWER_DOWN(void)
{
 if(IsFlashPowerOn==true)
 	{
     GPIO_PinModeSet(FLASH_SPI_CS_GPIOPORT, FLASH_SPI_CSPIN, gpioModeDisabled, 0); /* CS */
     GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_MOSIPIN,gpioModeDisabled, 0);  /* MOSI */
     GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_MISOPIN,gpioModeDisabled,0);  /* MISO */
     GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_CLKPIN,gpioModeDisabled, 0);  /* Clock */
    	
     CMU_ClockEnable(cmuClock_USART0, false);
     FLASH_PW_OFF();
     IsFlashPowerOn=false;
 	}
}

void FLASH_POWER_BACK(void)
{
 if(IsFlashPowerOn==false)
 	{ 	
     FLASH_PW_ON();
     GPIO_PinModeSet(FLASH_SPI_CS_GPIOPORT, FLASH_SPI_CSPIN, gpioModePushPull, 1); /* CS */
     GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_MOSIPIN,gpioModePushPull, 1);  /* MOSI */
     GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_MISOPIN,gpioModeInputPull,1);  /* MISO */
     GPIO_PinModeSet(FLASH_SPI_GPIOPORT,FLASH_SPI_CLKPIN,gpioModePushPull, 1);  /* Clock */    
     CMU_ClockEnable(cmuClock_USART0, true);
	 IsFlashPowerOn=true;
 	}
 
 }

void FLASH_DEEP_SLEEP(void)
{
	unsigned char cmd[4];

	FLASH_CS_L();

	cmd[0] = SPI_FLASH_INS_DP;

	SPI_Send_Bytes(1, cmd);

	FLASH_CS_H();

	systemStatus.blFlashPowerOn = false;
	
}


void FLASH_SLEEP_BACK(void)
{
	unsigned char cmd[4];

	FLASH_CS_L();

	cmd[0] = SPI_FLASH_INS_RES;

	SPI_Send_Bytes(1, cmd);

	FLASH_CS_H();

	systemStatus.blFlashPowerOn = true;
	
}


ReturnType FlashPageProgram(unsigned long address, unsigned char* data_ptr, unsigned short length)
{
	unsigned char cmd_to_program [4];

	if(!(address <  systemStatus.flashInfo.flashSize))
		return Flash_AddressInvalid;

	if(IsFlashBusy())
		return Flash_OperationOngoing;

	FlashWriteEnable();

	FLASH_CS_L();

	cmd_to_program[0] = SPI_FLASH_INS_PP;
	cmd_to_program[1] = ((unsigned char)(address >> 16));
	cmd_to_program[2] = ((unsigned char)(address >> 8));
	cmd_to_program[3] = ((unsigned char)(address & 0xFF));
	SPI_Send_Bytes(4, cmd_to_program);

	SPI_Send_Bytes(length, data_ptr);

	FLASH_CS_H();

    while (IsFlashBusy() == 1);
	 
	return Flash_Success;
}

ReturnType FlashProgram( unsigned long udAddr, unsigned char* pArray, unsigned long udNrOfElementsInArray )
{
	unsigned short ucMargin;
	unsigned short ucPageCount, ucRemainder;
	ReturnType typeReturn;

	if(udAddr >=  systemStatus.flashInfo.flashSize)
		return Flash_AddressInvalid;

	if(udAddr + udNrOfElementsInArray > systemStatus.flashInfo.flashSize) 
		return Flash_MemoryOverflow;

	ucMargin = (unsigned char)(~udAddr) + 1;

	if(udNrOfElementsInArray > ucMargin)
	{
		typeReturn = FlashPageProgram(udAddr, pArray, ucMargin);
        //while (IsFlashBusy() == 1);
		 
		if(Flash_Success != typeReturn) return typeReturn;

		udNrOfElementsInArray -= ucMargin;
		pArray += ucMargin;
		udAddr += ucMargin;
		ucPageCount = udNrOfElementsInArray / FLASH_WRITE_BUFFER_SIZE;
		ucRemainder = udNrOfElementsInArray % FLASH_WRITE_BUFFER_SIZE;

		while(ucPageCount--)
		{
			typeReturn = FlashPageProgram(udAddr, pArray, FLASH_WRITE_BUFFER_SIZE);
            //while (IsFlashBusy() == 1);
			
			if(Flash_Success != typeReturn) return typeReturn;

			pArray += FLASH_WRITE_BUFFER_SIZE;
			udAddr += FLASH_WRITE_BUFFER_SIZE;
		};

		return FlashPageProgram(udAddr, pArray, ucRemainder);
	}
	else
	{
		return FlashPageProgram(udAddr, pArray, udNrOfElementsInArray);
	}
}



uint16_t FlashCRC(int address,int length)
{
	unsigned char cmd_to_read [4],rxed_bytes;
    int bytes_sent=0;
	uint16_t crc_save=0;//这里的类型是uint16_t，最终返回的uint16_t的

	FLASH_CS_L();

	cmd_to_read[0] = SPI_FLASH_INS_READ;
	cmd_to_read[1] = ((unsigned char)(address >> 16));
	cmd_to_read[2] = ((unsigned char)(address >> 8));
	cmd_to_read[3] = ((unsigned char)(address));
	SPI_Send_Bytes(4, cmd_to_read);
  
	if ((FLASH_SPI->STATUS & USART_STATUS_RXDATAV) != 0)
	rxed_bytes = FLASH_SPI->RXDATA;//它的数据类型是char。

	while (bytes_sent < length)
	{
		FLASH_SPI->TXDATA = 0;
		while (!(FLASH_SPI->STATUS & USART_STATUS_TXC));		
		rxed_bytes= FLASH_SPI->RXDATA;
		bytes_sent++;

		//==CRC====
		
		crc_save  = (crc_save >> 8) | (crc_save << 8);
		crc_save ^= rxed_bytes;
		crc_save ^= (crc_save & 0xff) >> 4;
		crc_save ^= crc_save << 12;
		crc_save ^= (crc_save & 0xff) << 5;
		

		//============
	}

	//========================

	FLASH_CS_H();

	return crc_save;
}


