/*
 * usb_task.c
 *
 *  Created on: 2013-6-16
 *      Author: Administrator
 */
#include <assert.h>

#include <stdio.h>
//#include "debug.h"

#include "cmsis_os.h"
#include "em_usb.h"
#include "usbconfig.h"
#include "config.h"
#include "crc.h"
#include "main.h"
#include "descriptors.h"
#include "em_msc.h"

#include "ble.h"
#include "flash_task.h"

union _USB_PACKET gHIDOutPacket, gHIDInPacket __attribute__ ((aligned(4)));

static int pollTimeout = DEFAULT_POLL_TIMEOUT;
static uint8_t  idleRate;
static uint32_t SetIdleBuffer;

#define USB_RX_SIZE  64*3

uint8_t USB_RX_BUFF[USB_RX_SIZE] __attribute__ ((aligned(4)));
uint8_t USB_WR_BUFF[128];

uint8_t USB_RX_RP,USB_RX_WP;
uint32_t BLE_Updata_Packet_Count,USB_RX_LEFT_COUNT;


#define POLL_TIMER              0
#define INTR_IN_EP_ADDR         0x81
#define DEFAULT_POLL_TIMEOUT    24

int  OutputReportReceived(USB_Status_TypeDef status,
                          uint32_t xferred,
                          uint32_t remaining);
int  InputReportTransmitted(USB_Status_TypeDef status,
                            uint32_t xferred,
                            uint32_t remaining);
void StateChange(USBD_State_TypeDef oldState,
                 USBD_State_TypeDef newState);
int SetupCmd(const USB_Setup_TypeDef *setup);
void USBReadDeviceAckPacket(uint8_t ack);
void  USBHIDDATAPARSING(void);
static void PollTimeout(void);
void UsbAppInit(void);
uint8_t VerifyUSBPacketChecksum(void);
void USBAckPacket(uint8_t ack);

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
int SetupCmd(const USB_Setup_TypeDef *setup)
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
					/* Receive OUT Report */
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
					/* Send IN report */
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
					*((uint8_t *)SetIdleBuffer) = idleRate;
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
 *
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


// 收到usb command：DATA_GET_LATEST_TIMESTAMP
// 要求获得最新的时间戳
void onUsbCmdGetLatestTimestamp()
{
	// 最新的时间戳直接从 pFlashStorageIndicator 中读取
	// 因为更新在 pFlashStorageIndicator 中的时间戳已经写入flash
	// 或从已存储的数据中读取出来
	
	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;
	
	memcpy(gHIDInPacket.Packet.Data, (void*) &(pFlashStorageIndicator->endTimestamp), sizeof(time_t));

	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN - 1]));
	gHIDInPacket.Packet.CRC = packetCRC;
}

// 预备上传起始
void onUsbCmdPrepareUpload(time_t t)
{
	// 搜索所有sector，找到最 <= t 的最大时间戳
	time_t st = searchMatchedTimestamp(t);
	
	//
	AssembleUSBPacket(CMD_ACK, &st, sizeof(time_t));
}

// 开始通过USB上传数据
// 需先通过 DATA_PREPARE_UPLOAD 指令预备上传
// 上传操作从 dataUploadStartPointer 指示的位置开始
// 上传所有数据直到最新数据
void onUsbCmdStartUpload()
{
	CCTRACE("\n^^ on command upload ...\n");
	
	// 先响应命令
	USBAckPacket(CMD_ACK);
	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);

	
	// 发送指令到 flash_task，在task中上传数据
	FLASH_COMMAND* cmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
	cmd->cmd = FLASH_CMD_UPLOAD;
//	cmd->data.p = (void*) &(gHIDInPacket.Packet);

	osMailPut(hFlashCommandQueue, cmd);
}

void onUsbCmdDoUpload()
{
	CCTRACE("\n^^ on command upload ...\n");

	doFlashUpload();
//	// 先响应命令
//	USBAckPacket(CMD_ACK);
//	USBD_Write(0, (void*) &gHIDInPacket, sizeof(struct _PACKET), 0);
//
//	
//	// 发送指令到 flash_task，在task中上传数据
//	FLASH_COMMAND* cmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmd->cmd = FLASH_CMD_UPLOAD;
////	cmd->data.p = (void*) &(gHIDInPacket.Packet);
//
//	osMailPut(hFlashCommandQueue, cmd);
}

void USBHIDDATAPARSING(void)

{
	uint8_t ret, i,command;
	//uint16_t FlashCRC;
	uint32_t CheckSum;

	//Verify USBPacket Checksum
	ret=false;
	if(VerifyUSBPacketChecksum())
		ret=true;

	if (gHIDOutPacket.Packet.FrameHeader == FRAME_CMD_HEADER)
	{
		command= gHIDOutPacket.Packet.Data[0];
		ret=true;
	}
	else if(gHIDOutPacket.Packet.FrameHeader==FRAME_DATA_HEADER)
	{
		command= BLE_UPGRADE_RUNNING;
		ret=true;
	}
	else
		ret=false;

	if (ret == false)
	{
		USBAckPacket(CMD_NACK);
		return;
	}



	switch(command)
	{
		case UPGRADE_READDEVICE:

            getBleDeviceInfo();
			
			//Feedback Device Information packet
			USBReadDeviceAckPacket(CMD_ACK);//  also to send the BLE info.

			break;

		case APP_UPGRADE:
			USBAckPacket(CMD_ACK);
			MSC_Init();
			MSC_WriteWord((uint32_t *)(DevChip.Device.memFlash*1024-4),(uint32_t *)&CheckSum,4);
			//MSC_Deinit()
			SysCtlDelay(60000);
			/* Write to the Application Interrupt/Reset Command Register to reset
			       * the EFM32. See section 9.3.7 in the reference manual. */
			RESET_MCU();
			break;

		// update BLE firmare
		//=======================================

		case BLE_SWITCH_TO_BOOTLOAD:

			if(BLE_ONLINE==true)
			{
				if(BLE_DevChip.BLE_Device.WORKSTA==BLE_APP)
				{
					BLE_ONLINE=false;//because it need to reset
					BLE_Update_Start();
				}
				else
				{
				}
			}

			USBAckPacket(CMD_ACK);

			break;

		case BLE_UPGRADE_START:

			memcpy(gUpgrade.data, gHIDOutPacket.Packet.Data, 60);
			USB_RX_RP=0;
			USB_RX_WP=0;
			USB_RX_LEFT_COUNT=0;
			BLE_Updata_Packet_Count=(0x3D800-0x800)/128;

#if 1
			//BLE_Responsed=false;
			BLE_Update_Start();

			//while(BLE_Responsed==false)
			//{
			//	EMU_EnterEM1();//DMA Not Done
			//};
#endif
			USBAckPacket(CMD_ACK);
			break;

		case BLE_UPGRADE_RUNNING:
			//TEST_L();

			if(BLE_Updata_Packet_Count)
			{
				for (i=0; i<PACKET_DATA_LEN; i++) // 60 bytes each packet
				{
					USB_RX_BUFF[USB_RX_RP++] = gHIDOutPacket.Packet.Data[i];
					USB_RX_RP%=USB_RX_SIZE;
				}

				USB_RX_LEFT_COUNT+=PACKET_DATA_LEN;

				if(USB_RX_LEFT_COUNT>=128)
				{
					USB_RX_LEFT_COUNT-=128;
					for (i=0; i<128; i++)
					{
						USB_WR_BUFF[i]=USB_RX_BUFF[USB_RX_WP++];
						USB_RX_WP%=USB_RX_SIZE;
					}

					//BLE_Responsed=false;
					WriteCC254xFlash(USB_WR_BUFF);
					//while(BLE_Responsed==false)
					//{
					// EMU_EnterEM1();//DMA Not Done
					//};
					//BLE_Updata_Packet_Count--;
				}
			}
			USBAckPacket(CMD_ACK);
			// TEST_H();
			break;


		case BLE_UPGRADE_DONE:

			BLE_Update_End(gUpgrade.Param.FlashCRC);
			USBAckPacket(CMD_ACK);
			BLE_ONLINE=false;
			break;

		case DATA_PREPARE_UPLOAD:
		{
			time_t t;
			memcpy(&t, gHIDOutPacket.Packet.Data + 1, sizeof(time_t));
			
			onUsbCmdPrepareUpload(t);
			
			break;
		}

		case DATA_START_UPLOAD:
		{
//			onUsbCmdStartUpload();
			onUsbCmdDoUpload();
			
			break;
		}

//		case DATA_CONTINUE_UPLOAD:
//		{
//			onUsbCmdContinueUpload();
//			
//			break;
//		}
		
		//=======================================
		default:
			break;
	}

}

uint8_t VerifyUSBPacketChecksum(void)
{
	uint8_t ret = true;
	uint16_t packetCRC;

	if ((gHIDOutPacket.Packet.FrameHeader != FRAME_CMD_HEADER)&&
	        (gHIDOutPacket.Packet.FrameHeader != FRAME_DATA_HEADER))
	{
		ret = false;
	}
	if(gHIDOutPacket.Packet.FrameTail != FRAME_TAIL)
	{
		ret = false;
	}
	packetCRC = CRC_calc(gHIDOutPacket.Packet.Data, (&gHIDOutPacket.Packet.Data[PACKET_DATA_LEN-1]));
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

	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN-1]));
	gHIDInPacket.Packet.CRC = packetCRC;
}

void USBReadDeviceAckPacket(uint8_t ack)
{
	uint16_t packetCRC;

	memset(gHIDInPacket.Buffer, 0, sizeof(gHIDInPacket.Buffer));

	gHIDInPacket.Packet.FrameHeader = FRAME_CMD_HEADER;
	gHIDInPacket.Packet.FrameTail  = FRAME_TAIL;

	gHIDInPacket.Packet.Data[0] = ack;

	memcpy((&gHIDInPacket.Packet.Data[1]), DevChip.EFM32DeviceInfo, sizeof(DevChip.EFM32DeviceInfo));

	// 20130620
	gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo)+1] = FW_TYPE_APP;
	gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo)+2] = APP_FW_VER_M;
	gHIDInPacket.Packet.Data[sizeof(DevChip.EFM32DeviceInfo)+3] = APP_FW_VER_S;

	if(BLE_ONLINE==true)
		memcpy((&gHIDInPacket.Packet.Data[1+16+3+8]),BLE_DevChip.BLE_DeviceInfo,sizeof(BLE_DevChip.BLE_DeviceInfo));
	else
		memset((&gHIDInPacket.Packet.Data[1+16+3+8]),0xff,sizeof(BLE_DevChip.BLE_DeviceInfo));


	packetCRC = CRC_calc(gHIDInPacket.Packet.Data, (&gHIDInPacket.Packet.Data[PACKET_DATA_LEN-1]));
	gHIDInPacket.Packet.CRC = packetCRC;
}
