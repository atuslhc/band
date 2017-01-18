/***************************************************************************//**
 * @file descriptors.h
 * @brief USB descriptors for HID keyboard example project.
 * @author Energy Micro AS
 * @version 1.2.1
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2011 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
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
 ******************************************************************************/

/* Created with usb.org's "HID Descriptor Tool(DT)", version 2.4 */
/*
const u8 ReportDescriptor[SIZ_REPORT_DESC] =
  {
    0x05, 0xFF,                    // USAGE_PAGE(User define)
    0x09, 0xFF,                    // USAGE(User define)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x01,                    // USAGE_PAGE(1)
    0x19, 0x00,                    //   USAGE_MINIMUM(0)
    0x29, 0xFF,                    //   USAGE_MAXIMUM(255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,                    //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x40,                    //   REPORT_COUNT (64)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x05, 0x02,                    // USAGE_PAGE(2)
    0x19, 0x00,                    //   USAGE_MINIMUM (0)
    0x29, 0xFF,                    //   USAGE_MAXIMUM (255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,                    //   LOGICAL_MAXIMUM (255)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x75, 0x40,                    //   REPORT_SIZE (64)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
  };  //ReportDescriptor
*/
EFM32_ALIGN(4)
static const char ReportDescriptor[40] __attribute__ ((aligned(4)))=
{
    0x06, 0x00,0xFF,               // USAGE_PAGE (Generic Desktop)
    0x09, 0xFF,                    // USAGE (User Define)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x01,                    //   USAGE_PAGE (1)
    0x19, 0,                       //   USAGE_MINIMUM (0)
    0x29, 255,                     //   USAGE_MAXIMUM (255)
    0x15, 0,                       //   LOGICAL_MINIMUM (0)
    0x25, 255,                     //   LOGICAL_MAXIMUM (255)
    0x75, 8,                       //   REPORT_SIZE (8)
    0x95, 64,                      //   REPORT_COUNT (64)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//11     
    0x05, 0x02,                    //   USAGE_PAGE (2)
    0x19, 0,                       //   USAGE_MINIMUM (0)
    0x29, 255,                     //   USAGE_MAXIMUM (255)
    0x15, 0,                       //   LOGICAL_MINIMUM (0)
    0x25, 255,                     //   LOGICAL_MAXIMUM (255)
    0x75, 8,                       //   REPORT_SIZE (8)
    0x95, 64,                      //   REPORT_COUNT (64)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
//19    
    0xc0                           // END_COLLECTION
};


EFM32_ALIGN(4)
static const USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4)))=
{
  .bLength            = USB_DEVICE_DESCSIZE,
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = USB_EP0_SIZE,
  .idVendor           = 0x2544,
  .idProduct          = 0x0002,
  .bcdDevice          = 0x0000,
  .iManufacturer      = 1,
  .iProduct           = 2,
  .iSerialNumber      = 3,
  .bNumConfigurations = 1
};

EFM32_ALIGN(4)
static const uint8_t configDesc[] __attribute__ ((aligned(4)))=
{
  /*** Configuration descriptor ***/
  USB_CONFIG_DESCSIZE,    /* bLength                                   */
  USB_CONFIG_DESCRIPTOR,  /* bDescriptorType                           */

  USB_CONFIG_DESCSIZE +   /* wTotalLength (LSB)                        */
  USB_INTERFACE_DESCSIZE +
  USB_HID_DESCSIZE +
  (USB_ENDPOINT_DESCSIZE * NUM_EP_USED),

  (USB_CONFIG_DESCSIZE +  /* wTotalLength (MSB)                        */
  USB_INTERFACE_DESCSIZE +
  USB_HID_DESCSIZE +
  (USB_ENDPOINT_DESCSIZE * NUM_EP_USED))>>8,

  1,                      /* bNumInterfaces                            */
  1,                      /* bConfigurationValue                       */
  0,                      /* iConfiguration                            */
  CONFIG_DESC_BM_RESERVED_D7 |   /* bmAttrib: Self powered             */
  CONFIG_DESC_BM_SELFPOWERED,
  CONFIG_DESC_MAXPOWER_mA( 100 ),/* bMaxPower: 100 mA                  */

  /*** Interface descriptor ***/
  USB_INTERFACE_DESCSIZE, /* bLength               */
  USB_INTERFACE_DESCRIPTOR,/* bDescriptorType      */
  0,                      /* bInterfaceNumber      */
  0,                      /* bAlternateSetting     */
  NUM_EP_USED,            /* bNumEndpoints         */
  0x03,                   /* bInterfaceClass (HID) */
  0,                      /* bInterfaceSubClass    */
  1,                      /* bInterfaceProtocol    */
  0,                      /* iInterface            */

  /*** HID descriptor ***/
  USB_HID_DESCSIZE,       /* bLength               */
  USB_HID_DESCRIPTOR,     /* bDescriptorType       */
  0x11,                   /* bcdHID (LSB)          */
  0x01,                   /* bcdHID (MSB)          */
  0,                      /* bCountryCode          */
  1,                      /* bNumDescriptors       */
  USB_HID_REPORT_DESCRIPTOR,     /* bDecriptorType        */
  sizeof( ReportDescriptor ),    /* wDescriptorLength(LSB)*/
  sizeof( ReportDescriptor )>>8, /* wDescriptorLength(MSB)*/

  /*** Endpoint descriptor ***/
  USB_ENDPOINT_DESCSIZE,  /* bLength               */
  USB_ENDPOINT_DESCRIPTOR,/* bDescriptorType       */
  INTR_IN_EP_ADDR,        /* bEndpointAddress (IN) */
  USB_EPTYPE_INTR,        /* bmAttributes          */
  64,                     /* wMaxPacketSize (LSB)  */
  0,                      /* wMaxPacketSize (MSB)  */
  DEFAULT_POLL_TIMEOUT,   /* bInterval             */
};

STATIC_CONST_STRING_DESC_LANGID( langID, 0x04, 0x09         );
STATIC_CONST_STRING_DESC( iManufacturer, L"CNVSYSTEMS Ltd." );
STATIC_CONST_STRING_DESC( iProduct     , L" ihealthTrack "    );
STATIC_CONST_STRING_DESC( iSerialNumber, L"000000001234"    );

static const void * const strings[] =
{
  &langID,
  &iManufacturer,
  &iProduct,
  &iSerialNumber
};

//下面把USB部分的内容注释掉
/* Endpoint buffer sizes */
/* 1 = single buffer, 2 = double buffering, 3 = triple buffering ... */
//static const uint8_t bufferingMultiplier[ NUM_EP_USED + 1 ] = { 1, 1 };
//
//static const USBD_Callbacks_TypeDef callbacks =
//{
//  .usbReset        = NULL,
//  .usbStateChange  = StateChange,
//  .setupCmd        = SetupCmd,
//  .isSelfPowered   = NULL,
//  .sofInt          = NULL
//};
//
//static const USBD_Init_TypeDef initstruct =
//{
//  .deviceDescriptor    = &deviceDesc,
//  .configDescriptor    = configDesc,
//  .stringDescriptors   = strings,
//  .numberOfStrings     = sizeof(strings)/sizeof(void*),
//  .callbacks           = &callbacks,
//  .bufferingMultiplier = bufferingMultiplier,
//  .reserved            = 0
//};

#define FRAME_CMD_HEADER   (0xAA)
#define FRAME_DATA_HEADER  (0xA5)
#define FRAME_TAIL         (0x55)

#define CMD_ACK            (0x00)
#define CMD_NACK           (0x01)
#define CMD_UPGRADE_FAIL   (0x03)

#define PACKET_DATA_LEN  (60)

EFM32_PACK_START(1)
union _USB_PACKET{
  struct _PACKET{
    unsigned char FrameHeader;
    unsigned char Data[PACKET_DATA_LEN];
    unsigned short CRC; 
    unsigned char FrameTail;
  }Packet;
  unsigned char Buffer[PACKET_DATA_LEN + 4];
};

static union _USB_PACKET gHIDOutPacket, gHIDInPacket __attribute__ ((aligned(4)));
static uint8_t GucFlashBuffer[2][60] __attribute__ ((aligned(4)));//这里60表示60个字节，这里的2表示？
EFM32_PACK_END()


