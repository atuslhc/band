/*
 * usb_task.c
 *
 *  Created on: 2015-10-27
 *      Author: Administrator
 */
#include <assert.h>

#include <stdio.h>
//#include "debug.h"

//#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "em_usb.h"
#include "usbconfig.h"
#include "config.h"
#include "crc.h"
#include "main.h"
#include "descriptors.h"
#include "em_msc.h"
#include "m00930.h"
#include "ble.h"
#include "flash_task.h"
#include "subMenu.h"

#define USB_RX_SIZE  64*3
#define POLL_TIMER              0
#define INTR_IN_EP_ADDR         0x81
#define DEFAULT_POLL_TIMEOUT    24
#define UPGRADE_STATUS_LOW_POWER 0xf0

union _USB_PACKET gHIDOutPacket, gHIDInPacket __attribute__ ((aligned(4)));
static int pollTimeout = DEFAULT_POLL_TIMEOUT;
static uint8_t  idleRate;
static uint32_t SetIdleBuffer;
static bool onceReadFirmwareHead = false;


uint8_t USB_RX_BUFF[USB_RX_SIZE] __attribute__ ((aligned(4)));
uint8_t USB_WR_BUFF[128];

uint8_t USB_RX_RP, USB_RX_WP;
uint32_t BLE_Updata_Packet_Count, USB_RX_LEFT_COUNT;




int  OutputReportReceived(USB_Status_TypeDef status,
                          uint32_t xferred,
                          uint32_t remaining);
int  InputReportTransmitted(USB_Status_TypeDef status,
                            uint32_t xferred,
                            uint32_t remaining);
void StateChange(USBD_State_TypeDef oldState,
                 USBD_State_TypeDef newState);
int SetupCmd(const USB_Setup_TypeDef* setup);
void  USBHIDDATAPARSING(void);
static void PollTimeout(void);
void UsbAppInit(void);
uint8_t VerifyUSBPacketChecksum(void);
void USBAckPacket(uint8_t ack);

void onGetDevInfo(uint8_t ack);
void onStartUpgradeFirmware(uint8_t ack, uint8_t* data);
void onGetSectorCRC(uint8_t ack, uint8_t* data);
void onEraseFlashSector(uint8_t ack, uint8_t* data);
void onWritingFlashSector(uint8_t ack, uint8_t* data);
void onWriteFlashSector(uint8_t ack, uint8_t* data);
union _UPGRADE_PARAM gUpgrade;


void UsbAppInit(void)
{
	USBD_Init(&initstruct);
}
/**************************************************************************//**
 * @brief
 *   Called on timer elapsed event. This function is called at a rate set
 *   by the host driver with the SET_IDLE setup command.
 *****************************************************************************/

static void PollTimeout(void)
{
	/* Restart HID poll timer */
	if (pollTimeout)
	{
		USBTIMER_Start(POLL_TIMER, pollTimeout, PollTimeout);
	}
	else
	{
		USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
	}
}


/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts HID operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
void StateChange(USBD_State_TypeDef oldState,
                 USBD_State_TypeDef newState)
{
	if (newState == USBD_STATE_CONFIGURED)
	{
		/* We have been configured, start HID functionality ! */
		if (oldState != USBD_STATE_SUSPENDED)	/* Resume ?   */
		{
			idleRate	  = DEFAULT_POLL_TIMEOUT / 4;
			pollTimeout = DEFAULT_POLL_TIMEOUT;
		}

		USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
	}

	else if ((oldState == USBD_STATE_CONFIGURED) &&
	         (newState != USBD_STATE_SUSPENDED))
	{
		/* We have been de-configured, stop HID functionality */
		USBTIMER_Stop(POLL_TIMER);
	}

	else if (newState == USBD_STATE_SUSPENDED)
	{
		/* We have been suspended, stop HID functionality */
		/* Reduce current consumption to below 2.5 mA.	  */
		USBTIMER_Stop(POLL_TIMER);
	}

}


/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements HID class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
int SetupCmd(const USB_Setup_TypeDef* setup)
{
	int retVal = USB_STATUS_REQ_UNHANDLED;

	if ((setup->Type == USB_SETUP_TYPE_STANDARD) &&
	        (setup->Direction == USB_SETUP_DIR_IN) &&
	        (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
	{
		/* A HID device must extend the standard GET_DESCRIPTOR command   */
		/* with support for HID descriptors.							  */
		switch (setup->bRequest)
		{
			case GET_DESCRIPTOR:

				/********************/
				if ((setup->wValue >> 8) == USB_HID_REPORT_DESCRIPTOR)
				{
					USBD_Write(0, (void*) ReportDescriptor,
					           EFM32_MIN(sizeof(ReportDescriptor), setup->wLength),
					           NULL);
					retVal = USB_STATUS_OK;
				}
				else if ((setup->wValue >> 8) == USB_HID_DESCRIPTOR)
				{
					USBD_Write(0, (void*) &configDesc[ USB_CONFIG_DESCSIZE +
					                                   USB_INTERFACE_DESCSIZE ],
					           EFM32_MIN(USB_HID_DESCSIZE, setup->wLength),
					           NULL);
					retVal = USB_STATUS_OK;
				}

				break;
		}
	}

	else if ((setup->Type == USB_SETUP_TYPE_CLASS) &&
	         (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
	{
		/* Implement the necessary HID class specific commands. 		  */
		switch (setup->bRequest)
		{
			case USB_HID_SET_REPORT:

				/********************/
				if (((setup->wValue >> 8) == 2) &&			/* Output report */
				        ((setup->wValue & 0xFF) == 0) &&			/* Report ID	 */
				        (setup->wIndex == 0) &&					/* Interface no. */
				        (setup->wLength == 0x40) &&				/* Report length */
				        (setup->Direction == USB_SETUP_DIR_OUT))
				{
					/* Receive OUT Report */ //PC site data via report format to MCU. get the USB FIFO data to Buffer.
					USBD_Read(0, (void*) (gHIDOutPacket.Buffer), 64, OutputReportReceived);
					retVal = USB_STATUS_OK;
				}

				break;

			case USB_HID_GET_REPORT:

				/********************/
				if (((setup->wValue >> 8) == 1) &&			/* Input report  */
				        ((setup->wValue & 0xFF) == 0) &&			/* Report ID	 */
				        (setup->wIndex == 0) &&					/* Interface no. */
				        (setup->wLength == 0x40) &&				/* Report length */
				        (setup->Direction == USB_SETUP_DIR_IN))
				{
					/* Send IN report */  // send data to PC site.
					USBD_Write(0, (void*)(gHIDInPacket.Buffer), 64, InputReportTransmitted);
					retVal = USB_STATUS_OK;
				}

				break;

			case USB_HID_SET_IDLE:

				/********************/
				if (((setup->wValue & 0xFF) == 0) &&			/* Report ID	 */
				        (setup->wIndex == 0) &&					/* Interface no. */
				        (setup->wLength == 0) &&
				        (setup->Direction != USB_SETUP_DIR_IN))
				{
					idleRate	= setup->wValue >> 8;
					pollTimeout = 4 * idleRate;

					if (pollTimeout > DEFAULT_POLL_TIMEOUT)
					{
						pollTimeout = DEFAULT_POLL_TIMEOUT;
					}

					retVal = USB_STATUS_OK;
				}

				break;

			case USB_HID_GET_IDLE:

				/********************/
				if ((setup->wValue == 0) &&					/* Report ID	 */
				        (setup->wIndex == 0) &&					/* Interface no. */
				        (setup->wLength == 1) &&
				        (setup->Direction == USB_SETUP_DIR_IN))
				{
					*((uint8_t*)SetIdleBuffer) = idleRate;
					USBD_Write(0, (void*) SetIdleBuffer, 1, NULL);
					retVal = USB_STATUS_OK;
				}

				break;
		}
	}

	return retVal;
}


/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_SET_REPORT
 *   setup command has completed.
 *  When HID device report (got PC site data) then call it.
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
int OutputReportReceived(USB_Status_TypeDef status,
                         uint32_t xferred,
                         uint32_t remaining)
{
	(void) remaining;

	/* We have received new data for NumLock, CapsLock and ScrollLock LED's */
	if ((status == USB_STATUS_OK) && (xferred == 64))
	{
		USBHIDDATAPARSING();
	}

	return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_GET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int InputReportTransmitted(USB_Status_TypeDef status,
                                  uint32_t xferred,
                                  uint32_t remaining)
{
	(void) remaining;

	/* We have received new data for NumLock, CapsLock and ScrollLock LED's */
	if ((status == USB_STATUS_OK) && (xferred == 64))
	{
		memset(gHIDInPacket.Buffer, 0, 64);
	}

	return USB_STATUS_OK;
}


void AssembleUSBPacket(uint8_t ack, void* data, int size)
{
	CCASSERT(size < PACKET_DATA_LEN);

	//
	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
	gHIDInPacket.Packet.Data[0] = ack;

	if (data != 0 && size > 0)
	{
		memcpy(gHIDInPacket.Packet.Data + 1, data, size);
	}

	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;
}


// get usb command£ºDATA_GET_LATEST_TIMESTAMP
// request get the latest timestamp
void onUsbCmdGetLatestTimestamp()
{
	// the latest timestamp extract from pFlashStorageIndicator 
	// The newer pFlashStorageIndicator are written into flash
	// or read from the stored data.

	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;

	memcpy(gHIDInPacket.Packet.Data, (void*) & (pFlashStorageIndicator->endTimestamp), sizeof(time_t));

	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;
}

// Prepare upload start
void onUsbCmdPrepareUpload(time_t t)
{
	// search all sector, and find the maximum of close <= t 
//	time_t st = searchMatchedTimestamp(t);
//
//	//
	//	AssembleUSBPacket(CMD_ACK, &st, sizeof(time_t));  //remark 2015/10/22 10:55:44
}

// Start transmit(upload) via USB
// Need get the DATA_PREPARE_UPLOAD  command to prepare upload.
// Upload from dataUploadStartPointer pointer start.
// upload all data until newest.
void onUsbCmdStartUpload()
{
	CCTRACE("\n^^ on command upload ...\n");

	// send the response ack first
	USBAckPacket(CMD_ACK);
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);


	// send command to flash_task, transmit data in the task.
//	FLASH_COMMAND* cmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmd->cmd = FLASH_CMD_UPLOAD;
//	cmd->data.p = (void*) &(gHIDInPacket.Packet);

// Remark some protocol here.
//	MESSAGE msg;
//	msg.params.type = FLASH_CMD_UPLOAD;
//
//	xQueueSend(hEvtQueueUSB, &msg.id, 0);

//	osMailPut(hFlashCommandQueue, cmd);
}

void onUsbCmdDoUpload()
{
	CCTRACE("\n^^ on command upload ...\n");

	//  doFlashUpload(); //remark 2015/10/22 10:55:08
//	// send response command first
//	USBAckPacket(CMD_ACK);
//	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
//
//
//	// send command to flash_task, transmit data in the task
//	FLASH_COMMAND* cmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmd->cmd = FLASH_CMD_UPLOAD;
////	cmd->data.p = (void*) &(gHIDInPacket.Packet);
//
//	osMailPut(hFlashCommandQueue, cmd);
}

void USBHIDDATAPARSING(void)

{
	uint8_t ret, i, command;

	uint32_t CheckSum;

	//Verify USBPacket Checksum
	ret = false;

	if(VerifyUSBPacketChecksum())
		ret = true;

	if (gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER)
	{
		//The command header
		command = gHIDOutPacket.Packet.Data[0];
		ret = true;
	}
	else if(gHIDOutPacket.Packet.FrameHeader == FRAME_DATA_HEADER)
	{
		//The command header
		command = BLE_UPGRADE_RUNNING;
		ret = true;
	}
	else
		ret = false;

	if (ret == false)
	{
		USBAckPacket(CMD_NACK); //The ACK already sent, just sent NACK.
		return;
	}



	switch(command)
	{
		case GET_DEV_INFO:
			//Feedback Device Information packet
			onGetDevInfo(CMD_ACK);//  also to send the BLE info.

			break;

		case FW_Update_COMM:
			//start upgrade
			onStartUpgradeFirmware(CMD_ACK, &gHIDOutPacket.Packet.Data[1]);
			break;

		case FW_CRC_GET:
			onGetSectorCRC(CMD_ACK, &gHIDOutPacket.Packet.Data[1]);
			break;

		case ERASE_SECTOR_COMM:
			onEraseFlashSector(CMD_ACK, &gHIDOutPacket.Packet.Data[1]);
			break;

		case WR_SECTOR_COMM:
			onWriteFlashSector(CMD_ACK, &gHIDOutPacket.Packet.Data[1]);
			break;

		case FW_Update_DATA:
			onWritingFlashSector(CMD_ACK, &gHIDOutPacket.Packet.Data[1]);
			break;

		//=======================================
		default:
			break;
	}

}

uint8_t VerifyUSBPacketChecksum(void)
{
	uint8_t ret = true;
	uint16_t packetCRC;

	if ((gHIDOutPacket.Packet.FrameHeader != FRAME_CMD_HEADER) &&
	        (gHIDOutPacket.Packet.FrameHeader != FRAME_DATA_HEADER))
	{
		ret = false;
	}

	if(gHIDOutPacket.Packet.FrameTail != FRAME_TAIL)
	{
		ret = false;
	}

	packetCRC = CRC_calc(gHIDOutPacket.Packet.Data, (&gHIDOutPacket.Packet.Data[PACKET_DATA_LEN - 1]));

	if (packetCRC != gHIDOutPacket.Packet.CRC )
	{
		ret = false;
	}

	return ret;
}


void USBAckPacket(uint8_t ack)
{
	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
	gHIDInPacket.Packet.Data[0] = ack;

	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;

	//
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
}

//============================================================
//The data format must match with PC site decode format, or can not decoding.
//All of PC site command process as below.
void onGetDevInfo(uint8_t ack)
{
	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;

	gHIDInPacket.Packet.Data[0] = ack;

	memcpy((&gHIDInPacket.Packet.Data[1]), DevChip.EFM32DeviceInfo, sizeof(DevChip.EFM32DeviceInfo)); //size 16

	gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo) + 1] = APP_FW_VER_M;
	gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo) + 2] = APP_FW_VER_S; //MCU FW Version

	memcpy((&gHIDInPacket.Packet.Data[1 + 16 + 2 + 8]), BLE_DevChip.BLE_DeviceInfo, sizeof(BLE_DevChip.BLE_DeviceInfo));


	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));

	gHIDInPacket.Packet.CRC = packetCRC;

	//
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
}

// action 0x01: start upgrade, 0x02: stop/end upgrade
void onStartUpgradeFirmware(uint8_t ack, uint8_t* data)
{
	uint16_t packetCRC;
	uint8_t action = 0;
	uint8_t updateStatus = 0;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;

	gHIDInPacket.Packet.Data[0] = ack;

	action = data[0];

	if( action == 0x01)
	{
		//start upgrade
		if (systemStatus.bBatteryLevel == OUT_OF_BATTERY) //The condition maybe can ignore in USB mode.
		{
			updateStatus = UPGRADE_STATUS_LOW_POWER; // out of battery
		}
		else
		{
			AFE44xx_Shutoff();

			if((systemSetting.blHRSensorEnabled == true) && (systemStatus.blHRSensorTempEnabled == true))
				blTurnedOffPpgByBleTask = true;

//					systemSetting.blHRSensorEnabled = false;
//					systemStatus.blHRSensorEnabled  = false;

			// enable a timer to check overtime in upgrade process.
//					blFwUpgradingFlag=true;
			EnableLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA, false,
			                TIME_OUT_UPGRADE_FW, onBleTaskTimeOut,
			                (void*)BLE_TASK_TYPE_FW_UPGRADING);

			//
			SetFlashBusyBit(FLASH_BUSY_BIT_BLE);
			WakeFlashUp();
			updateStatus = 2;

			if(data[1] == 0x55)//As the API, the third byte 0x55 will have 2 bytes of upgrade data sector offset
				EXT_FLASH_SECTOR_OFFSET = data[2] + (uint16_t)(data[3] << 8);
			else
				EXT_FLASH_SECTOR_OFFSET = 0;

		}
	}
	else if(action == 0x02)
	{
		//stop/end upgrade
		DisableLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA);

		if(blTurnedOffPpgByBleTask == true)
		{
			blTurnedOffPpgByBleTask = false;
//					systemSetting.blHRSensorEnabled = true;
			systemStatus.blHRSensorTempEnabled = true;
		}

		//
		updateStatus = 1; //  if update_sta==1 ,update failed,  ==2 ,success
		FlashRead(EXT_FLASH_SECTOR_OFFSET * 4096, FW_INFO.INFO_BUF, 16);

		if(FW_INFO.INFO.Head_CRC == CRC_calc(&FW_INFO.INFO_BUF[0], &FW_INFO.INFO_BUF[13]))
		{
			CRC_TEMP = FlashCRC(EXT_FLASH_SECTOR_OFFSET * 4096 + 16, FW_INFO.INFO.fw_length);

			if(FW_INFO.INFO.fw_crc == CRC_TEMP) //download is correct
			{
				//Download complete, clean screen and turn off screen.
				blDuringDownload = false;
				onceReadFirmwareHead = false;
				clearScreen(false);
				OLEDOff();

				updateStatus = 2; //  if update_sta==1 ,update failed,  ==2 ,success
				gHIDInPacket.Packet.Data[1] = action;
				gHIDInPacket.Packet.Data[2] = updateStatus;
				packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
				gHIDInPacket.Packet.CRC = packetCRC;

				//
				USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);

				vTaskDelay(100);
				RESET_MCU();

				while(1);
			}
		}
	}

	gHIDInPacket.Packet.Data[1] = action;
	gHIDInPacket.Packet.Data[2] = updateStatus;
	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;

	//
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
}

void onGetSectorCRC(uint8_t ack, uint8_t* data)
{
	uint16_t packetCRC;
	int startSectorNo, endStartNo, startSectorAddress;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
	gHIDInPacket.Packet.Data[0] = ack;

	startSectorNo = data[0] + (uint16_t)(data[1] << 8);
	endStartNo   = data[2] + (uint16_t)(data[3] << 8);
	startSectorAddress = startSectorNo * systemStatus.flashInfo.sectorSize;

	CRC_TEMP = FlashCRC(EXT_FLASH_SECTOR_OFFSET * 4096 + startSectorAddress, 4096);

	memcpy(&gHIDInPacket.Packet.Data[1], &data[0], 4);
	gHIDInPacket.Packet.Data[1 + 4] = (uint8_t)CRC_TEMP;
	gHIDInPacket.Packet.Data[1 + 5] = (uint8_t)(CRC_TEMP >> 8);
	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;

	//
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
}

void onEraseFlashSector(uint8_t ack, uint8_t* data)
{
	uint16_t packetCRC;
	int eraseSectorNo = 0;
	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;

	gHIDInPacket.Packet.Data[0] = ack;

	eraseSectorNo = data[0] + (uint16_t)(data[1] << 8);

	if(eraseSectorNo < systemStatus.flashInfo.sectorSize)
	{
		FlashSectorErase(EXT_FLASH_SECTOR_OFFSET + eraseSectorNo);
	}

	gHIDInPacket.Packet.Data[1] = eraseSectorNo;
	gHIDInPacket.Packet.Data[2] = (uint8_t) (eraseSectorNo >> 8);
	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;

	//
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
}

void onWriteFlashSector(uint8_t ack, uint8_t* data)
{
	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
	gHIDInPacket.Packet.Data[0] = ack;

	WR_SEC_STA = data[2];//get the write sector action: 0x01:start, 0x02:stop/end

	if(WR_SEC_STA == 0x01)
	{
		WR_SEC_LOC = data[0] + (uint16_t)(data[1] << 8);

		WR_SEC_LOC = WR_SEC_LOC * systemStatus.flashInfo.sectorSize; //The location of sector would written.
		WR_SEC_OFFSET = 0;
		FW_RX_BUFF_WP = 0;
		Left_SEC_DATA = systemStatus.flashInfo.sectorSize;//The remain(left) data not be written.
		WR_SEC_COUNT = 0;
	}
	else if(WR_SEC_STA == 0x02)
	{
		WR_SEC_OFFSET = 0;
	}

	gHIDInPacket.Packet.Data[1] = (uint8_t)WR_SEC_LOC;
	gHIDInPacket.Packet.Data[2] = (uint8_t)(WR_SEC_LOC >> 8);
	gHIDInPacket.Packet.Data[3] = WR_SEC_STA;
	gHIDInPacket.Packet.Data[4] = (uint8_t) WR_SEC_COUNT;
	gHIDInPacket.Packet.Data[5] = (uint8_t)(WR_SEC_COUNT >> 8);
	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;

	//
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);

}

void onWritingFlashSector(uint8_t ack, uint8_t* data)
{


	ResetLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA);

	WR_SEC_COUNT++;

	for(uint8_t i = 0; i < 19; i++)
	{
		FW_RX_BUFF[FW_RX_BUFF_WP++] = data[i];
	}

	if(Left_SEC_DATA >= FW_RX_BUFF_SIZE)
	{
		if(FW_RX_BUFF_WP >= FW_RX_BUFF_SIZE)
		{
			Left_SEC_DATA -= FW_RX_BUFF_SIZE;
			FW_RX_BUFF_WP = 0;
			FlashProgram(EXT_FLASH_SECTOR_OFFSET * 4096 + WR_SEC_LOC + WR_SEC_OFFSET, FW_RX_BUFF, FW_RX_BUFF_SIZE);
			WR_SEC_OFFSET += FW_RX_BUFF_SIZE;



			//Calculate the progress of download
			if(onceReadFirmwareHead == false)
			{
				//firmware header is 16 bytes, it should be written into flash at the begin. Just read back to extract total length in the header.
				FlashRead(EXT_FLASH_SECTOR_OFFSET * 4096, FW_INFO.INFO_BUF, 16);
				allSector = FW_INFO.INFO.fw_length / 4096.0;
				onceReadFirmwareHead = true;
			}

			LED_TOGGLE();
		}
	}
	else
	{
		if(FW_RX_BUFF_WP >= Left_SEC_DATA)
		{
			FlashProgram(EXT_FLASH_SECTOR_OFFSET * 4096 + WR_SEC_LOC + WR_SEC_OFFSET, FW_RX_BUFF, Left_SEC_DATA);

			Left_SEC_DATA = 0;
		}
	}
}
