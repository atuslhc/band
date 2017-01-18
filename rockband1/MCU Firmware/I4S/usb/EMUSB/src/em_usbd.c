/**************************************************************************//**
 * @file
 * @brief USB protocol stack library, device API.
 * @author Energy Micro AS
 * @version 3.20.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
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
#include "em_device.h"
#if defined( USB_PRESENT ) && ( USB_COUNT == 1 )
#include "em_usb.h"
#if defined( USB_DEVICE )

#include "em_cmu.h"
#include "em_usbtypes.h"
#include "em_usbhal.h"
#include "em_usbd.h"
#include "cmsis_os.h"
#include "main.h"

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
extern osMutexId hMtxUsb;
static USBD_Device_TypeDef device;
USBD_Device_TypeDef *dev = &device;

static const char *stateNames[] =
{
  [ USBD_STATE_NONE       ] = "NONE      ",
  [ USBD_STATE_ATTACHED   ] = "ATTACHED  ",
  [ USBD_STATE_POWERED    ] = "POWERED   ",
  [ USBD_STATE_DEFAULT    ] = "DEFAULT   ",
  [ USBD_STATE_ADDRESSED  ] = "ADDRESSED ",
  [ USBD_STATE_CONFIGURED ] = "CONFIGURED",
  [ USBD_STATE_SUSPENDED  ] = "SUSPENDED ",
  [ USBD_STATE_LASTMARKER ] = "UNDEFINED "
};

/** @endcond */

/***************************************************************************//**
 * @brief
 *   Abort all pending transfers.
 *
 * @details
 *   Aborts transfers for all endpoints currently in use. Pending
 *   transfers on the default endpoint (EP0) are not aborted.
 ******************************************************************************/
void USBD_AbortAllTransfers( void )
{
  USBDHAL_AbortAllTransfers( USB_STATUS_EP_ABORTED );
}

/***************************************************************************//**
 * @brief
 *   Abort a pending transfer on a specific endpoint.
 *
 * @param[in] epAddr
 *   The address of the endpoint to abort.
 ******************************************************************************/
int USBD_AbortTransfer( int epAddr )
{
  USB_XferCompleteCb_TypeDef callback;
  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( epAddr );

  if ( ep == NULL )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_AbortTransfer(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( ep->num == 0 )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_AbortTransfer(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( ep->state == D_EP_IDLE )
  {
    INT_Enable();
    return USB_STATUS_OK;
  }

  USBD_AbortEp( ep );

  ep->state = D_EP_IDLE;
  if ( ep->xferCompleteCb )
  {
    callback = ep->xferCompleteCb;
    ep->xferCompleteCb = NULL;

    if ( ( dev->lastState == USBD_STATE_CONFIGURED ) &&
         ( dev->state     == USBD_STATE_ADDRESSED  )    )
    {
      USBDHAL_DeactivateEp( ep );
    }

    DEBUG_TRACE_ABORT( USB_STATUS_EP_ABORTED );
    callback( USB_STATUS_EP_ABORTED, ep->xferred, ep->remaining );
  }

  return USB_STATUS_OK;
}

/***************************************************************************//**
 * @brief
 *   Start USB device operation.
 *
 * @details
 *   Device operation is started by connecting a pullup resistor on the
 *   appropriate USB data line.
 ******************************************************************************/
void USBD_Connect( void )
{
  USBDHAL_Connect();
}

/***************************************************************************//**
 * @brief
 *   Stop USB device operation.
 *
 * @details
 *   Device operation is stopped by disconnecting the pullup resistor from the
 *   appropriate USB data line. Often referred to as a "soft" disconnect.
 ******************************************************************************/
void USBD_Disconnect( void )
{
  USBDHAL_Disconnect();
}

/***************************************************************************//**
 * @brief
 *   Check if an endpoint is busy doing a transfer.
 *
 * @param[in] epAddr
 *   The address of the endpoint to check.
 *
 * @return
 *   True if endpoint is busy, false otherwise.
 ******************************************************************************/
bool USBD_EpIsBusy( int epAddr )
{
  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( epAddr );

  if ( ep == NULL )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_EpIsBusy(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( ep->state == D_EP_IDLE )
    return false;

  return true;
}

/***************************************************************************//**
 * @brief
 *   Get current USB device state.
 *
 * @return
 *   Device USB state. See @ref USBD_State_TypeDef.
 ******************************************************************************/
USBD_State_TypeDef USBD_GetUsbState( void )
{
  return dev->state;
}

/***************************************************************************//**
 * @brief
 *   Get a string naming a device USB state.
 *
 * @param[in] state
 *   Device USB state. See @ref USBD_State_TypeDef.
 *
 * @return
 *   State name string pointer.
 ******************************************************************************/
const char *USBD_GetUsbStateName( USBD_State_TypeDef state )
{
  if ( state > USBD_STATE_LASTMARKER )
    state = USBD_STATE_LASTMARKER;

  return stateNames[ state ];
}

/***************************************************************************//**
 * @brief
 *   Initializes USB device hardware and internal protocol stack data structures,
 *   then connects the data-line (D+ or D-) pullup resistor to signal host that
 *   enumeration can begin.
 *
 * @note
 *   You may later use @ref USBD_Disconnect() and @ref USBD_Connect() to force
 *   reenumeration.
 *
 * @param[in] p
 *   Pointer to device initialization struct. See @ref USBD_Init_TypeDef.
 *
 * @return
 *   @ref USB_STATUS_OK on success, else an appropriate error code.
 ******************************************************************************/
int USBD_Init( const USBD_Init_TypeDef *p )
{
  int numEps;
  USBD_Ep_TypeDef *ep;
  uint8_t txFifoNum;
  uint8_t *conf, *confEnd;
  USB_EndpointDescriptor_TypeDef *epd;
  uint32_t totalRxFifoSize, totalTxFifoSize, numInEps, numOutEps;

  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
  
#if 0 //20131203
#if ( USB_USBC_32kHz_CLK == USB_USBC_32kHz_CLK_LFXO )
  CMU_OscillatorEnable( cmuOsc_LFXO, true, false );
#else
  CMU_OscillatorEnable( cmuOsc_LFRCO, true, false );
#endif
#endif


  memset( dev, 0, sizeof( USBD_Device_TypeDef ) );

  dev->setup                = dev->setupPkt;
  dev->deviceDescriptor     = p->deviceDescriptor;
  dev->configDescriptor     = (USB_ConfigurationDescriptor_TypeDef*)
                              p->configDescriptor;
  dev->stringDescriptors    = p->stringDescriptors;
  dev->numberOfStrings      = p->numberOfStrings;
  dev->state                = USBD_STATE_LASTMARKER;
  dev->savedState           = USBD_STATE_NONE;
  dev->lastState            = USBD_STATE_NONE;
  dev->callbacks            = p->callbacks;
  dev->remoteWakeupEnabled  = false;

  /* Initialize EP0 */

  ep                 = &dev->ep[ 0 ];
  ep->in             = false;
  ep->buf            = NULL;
  ep->num            = 0;
  ep->mask           = 1;
  ep->addr           = 0;
  ep->type           = USB_EPTYPE_CTRL;
  ep->txFifoNum      = 0;
  ep->packetSize     = USB_EP0_SIZE;
  ep->remaining      = 0;
  ep->xferred        = 0;
  ep->state          = D_EP_IDLE;
  ep->xferCompleteCb = NULL;
  ep->fifoSize       = USB_EP0_SIZE / 4;

  totalTxFifoSize = ep->fifoSize * p->bufferingMultiplier[ 0 ];
  totalRxFifoSize = (ep->fifoSize + 1) * p->bufferingMultiplier[ 0 ];


  /* Parse configuration decriptor */
  numEps = 0;
  numInEps  = 0;
  numOutEps = 0;
  conf = (uint8_t*)dev->configDescriptor;
  confEnd = conf + dev->configDescriptor->wTotalLength;

  txFifoNum = 1;
  while ( conf < confEnd )
  {
    if ( *conf == 0 )
    {
      DEBUG_USB_API_PUTS( "\nUSBD_Init(), Illegal configuration descriptor" );
      EFM_ASSERT( false );
      return USB_STATUS_ILLEGAL;
    }

    if ( *(conf + 1) == USB_ENDPOINT_DESCRIPTOR )
    {
      numEps++;
      epd = (USB_EndpointDescriptor_TypeDef*)conf;

      ep                 = &dev->ep[ numEps ];
      ep->in             = ( epd->bEndpointAddress & USB_SETUP_DIR_MASK ) != 0;
      ep->buf            = NULL;
      ep->addr           = epd->bEndpointAddress;
      ep->num            = ep->addr & USB_EPNUM_MASK;
      ep->mask           = 1 << ep->num;
      ep->type           = epd->bmAttributes & CONFIG_DESC_BM_TRANSFERTYPE;
      ep->packetSize     = epd->wMaxPacketSize;
      ep->remaining      = 0;
      ep->xferred        = 0;
      ep->state          = D_EP_IDLE;
      ep->xferCompleteCb = NULL;
      if ( ep->in )
      {
        numInEps++;
        ep->txFifoNum = txFifoNum++;
        ep->fifoSize = (ep->packetSize/4) * p->bufferingMultiplier[ numEps ];
        dev->inEpAddr2EpIndex[ ep->num ] = numEps;
        totalTxFifoSize += ep->fifoSize;
        if ( ep->num > MAX_NUM_IN_EPS )
        {
          DEBUG_USB_API_PUTS( "\nUSBD_Init(), Illegal IN EP address" );
          EFM_ASSERT( false );
          return USB_STATUS_ILLEGAL;
        }
      }
      else
      {
        numOutEps++;
        ep->fifoSize = (ep->packetSize/4 + 1) * p->bufferingMultiplier[ numEps ];
        dev->outEpAddr2EpIndex[ ep->num ] = numEps;
        totalRxFifoSize += ep->fifoSize;
        if ( ep->num > MAX_NUM_OUT_EPS )
        {
          DEBUG_USB_API_PUTS( "\nUSBD_Init(), Illegal OUT EP address" );
          EFM_ASSERT( false );
          return USB_STATUS_ILLEGAL;
        }
      }
    }
    conf += *conf;
  }

  /* Rx-FIFO size: SETUP packets : 4*n + 6    n=#CTRL EP's
   *               GOTNAK        : 1
   *               Status info   : 2*n        n=#OUT EP's (EP0 included) in HW
   */
  totalRxFifoSize += 10 + 1 + ( 2 * (MAX_NUM_OUT_EPS + 1) );

  if ( numEps != NUM_EP_USED )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Init(), Illegal EP count" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( numInEps > MAX_NUM_IN_EPS )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Init(), Illegal IN EP count" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( numOutEps > MAX_NUM_OUT_EPS )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Init(), Illegal OUT EP count" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }


  /* Enable USB clock */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_USB | CMU_HFCORECLKEN0_USBC;
  CMU_ClockSelectSet( cmuClock_USBC, cmuSelect_HFCLK );

  USBHAL_DisableGlobalInt();

  if ( USBDHAL_CoreInit( totalRxFifoSize, totalTxFifoSize ) == USB_STATUS_OK )
  {
    USBDHAL_EnableUsbResetInt();
    USBHAL_EnableGlobalInt();

	/* at the lowest interrupt priority. */
	NVIC_SetPriority(USB_IRQn, USB_INT_LEVEL);//2013.12.15
	
    NVIC_ClearPendingIRQ( USB_IRQn );
    NVIC_EnableIRQ( USB_IRQn );
  }
  else
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Init(), FIFO setup error" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

#if ( USB_PWRSAVE_MODE & USB_PWRSAVE_MODE_ONVBUSOFF )
  if ( USBHAL_VbusIsOn() )
  {
    USB_ON_CLKTO48MHZ();
		
    USBD_SetUsbState( USBD_STATE_POWERED );
  }
  else
#endif
  {   
	USB_OFF_CLKTO14MHZ();
		
    USBD_SetUsbState( USBD_STATE_NONE );
  }

  return USB_STATUS_OK;
}

/***************************************************************************//**
 * @brief
 *   Start a read (OUT) transfer on an endpoint.
 *
 * @note
 *   The transfer buffer length must be a multiple of 4 bytes in length and
 *   WORD (4 byte) aligned. When allocating the buffer, round buffer length up.
 *   If it is possible that the host will send more data than your device
 *   expects, round buffer size up to the next multiple of maxpacket size.
 *
 * @param[in] epAddr
 *   Endpoint address.
 *
 * @param[in] data
 *   Pointer to transfer data buffer.
 *
 * @param[in] byteCount
 *   Transfer length.
 *
 * @param[in] callback
 *   Function to be called on transfer completion. Supply NULL if no callback
 *   is needed. See @ref USB_XferCompleteCb_TypeDef.
 *
 * @return
 *   @ref USB_STATUS_OK on success, else an appropriate error code.
 ******************************************************************************/
int USBD_Read( int epAddr, void *data, int byteCount,
               USB_XferCompleteCb_TypeDef callback )
{
  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( epAddr );

  if ( ep == NULL )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( (   byteCount > MAX_XFER_LEN                           ) ||
       ( ( byteCount / ep->packetSize ) > MAX_PACKETS_PR_XFER )    )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Illegal transfer size" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

#if !defined( USB_SLAVEMODE )
  if ( (uint32_t)data & 3 )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Misaligned data buffer" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }
#endif

  if ( USBDHAL_EpIsStalled( ep ) )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Endpoint is halted" );
    return USB_STATUS_EP_STALLED;
  }

  if ( ep->state != D_EP_IDLE )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Endpoint is busy" );
    return USB_STATUS_EP_BUSY;
  }

  if ( ( ep->num > 0 ) && ( USBD_GetUsbState() != USBD_STATE_CONFIGURED ) )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Device not configured" );
    return USB_STATUS_DEVICE_UNCONFIGURED;
  }

  ep->buf       = (uint8_t*)data;
  ep->remaining = byteCount;
  ep->xferred   = 0;

  if ( ep->num == 0 )
  {
    ep->in = false;
  }
  else if ( ep->in != false )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Read(), Illegal EP direction" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  ep->state          = D_EP_RECEIVING;
  ep->xferCompleteCb = callback;

  USBD_ArmEp( ep );
  return USB_STATUS_OK;
}

/***************************************************************************//**
 * @brief
 *   Perform a remote wakeup signalling sequence.
 *
 * @note
 *   It is the responsibility of the application to ensure that remote wakeup
 *   is not attempted before the device has been suspended for at least 5
 *   miliseconds. This function should not be called from within an interrupt
 *   handler.
 *
 * @return
 *   @ref USB_STATUS_OK on success, else an appropriate error code.
 ******************************************************************************/
int USBD_RemoteWakeup( void )
{

  if ( ( dev->state != USBD_STATE_SUSPENDED ) ||
       ( dev->remoteWakeupEnabled == false  )    )
  {
    INT_Enable();
    DEBUG_USB_API_PUTS( "\nUSBD_RemoteWakeup(), Illegal remote wakeup" );
    return USB_STATUS_ILLEGAL;
  }

  USBDHAL_SetRemoteWakeup();
  osDelay( 10 );
  USBDHAL_ClearRemoteWakeup();
  return USB_STATUS_OK;
}

/***************************************************************************//**
 * @brief
 *   Check if it is ok to enter energy mode EM2.
 *
 * @note
 *   Before entering EM2 both the USB hardware and the USB stack must be in a
 *   certain state, this function checks if all conditions for entering EM2
 *   is met.
 *   Refer to the @ref usb_device_powersave section for more information.
 *
 * @return
 *   True if ok to enter EM2, false otherwise.
 ******************************************************************************/
bool USBD_SafeToEnterEM2( void )
{
#if ( USB_PWRSAVE_MODE )
  return USBD_poweredDown ? true : false;
#else
  return false;
#endif
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

void USBD_SetUsbState( USBD_State_TypeDef newState )
{
  USBD_State_TypeDef currentState;

  currentState = dev->state;
  if ( newState == USBD_STATE_SUSPENDED )
  {
    dev->savedState = currentState;
  }

  dev->lastState = dev->state;
  dev->state = newState;

  if ( ( dev->callbacks->usbStateChange ) &&
       ( currentState != newState       )    )
  {
    dev->callbacks->usbStateChange( currentState, newState );
  }
}

/** @endcond */

/***************************************************************************//**
 * @brief
 *   Set an endpoint in the stalled (halted) state.
 *
 * @param[in] epAddr
 *   The address of the endpoint to stall.
 *
 * @return
 *   @ref USB_STATUS_OK on success, else an appropriate error code.
 ******************************************************************************/
int USBD_StallEp( int epAddr )
{
  USB_Status_TypeDef retVal;
  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( epAddr );

  if ( ep == NULL )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_StallEp(), Illegal request" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( ep->num == 0 )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_StallEp(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  retVal = USBDHAL_StallEp( ep );

  if ( retVal != USB_STATUS_OK )
  {
    retVal = USB_STATUS_ILLEGAL;
  }

  return retVal;
}

/***************************************************************************//**
 * @brief
 *   Stop USB device stack operation.
 *
 * @details
 *   The data-line pullup resistor is turned off, USB interrupts are disabled,
 *   and finally the USB pins are disabled.
 ******************************************************************************/
void USBD_Stop( void )
{
  USBD_Disconnect();
  NVIC_DisableIRQ( USB_IRQn );
  USBHAL_DisableGlobalInt();
  USB->IEN   = _USB_IEN_RESETVALUE;
  USB->ROUTE = _USB_ROUTE_RESETVALUE;
  USBD_SetUsbState( USBD_STATE_NONE );
}

/***************************************************************************//**
 * @brief
 *   Reset stall state on a stalled (halted) endpoint.
 *
 * @param[in] epAddr
 *   The address of the endpoint to un-stall.
 *
 * @return
 *   @ref USB_STATUS_OK on success, else an appropriate error code.
 ******************************************************************************/
int USBD_UnStallEp( int epAddr )
{
  USB_Status_TypeDef retVal;
  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( epAddr );

  if ( ep == NULL )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_UnStallEp(), Illegal request" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( ep->num == 0 )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_UnStallEp(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  retVal = USBDHAL_UnStallEp( ep );

  if ( retVal != USB_STATUS_OK )
  {
    retVal = USB_STATUS_ILLEGAL;
  }

  return retVal;
}

/***************************************************************************//**
 * @brief
 *   Start a write (IN) transfer on an endpoint.
 *
 * @param[in] epAddr
 *   Endpoint address.
 *
 * @param[in] data
 *   Pointer to transfer data buffer. This buffer must be WORD (4 byte) aligned.
 *
 * @param[in] byteCount
 *   Transfer length.
 *
 * @param[in] callback
 *   Function to be called on transfer completion. Supply NULL if no callback
 *   is needed. See @ref USB_XferCompleteCb_TypeDef.
 *
 * @return
 *   @ref USB_STATUS_OK on success, else an appropriate error code.
 ******************************************************************************/
int USBD_Write( int epAddr, void *data, int byteCount,
                USB_XferCompleteCb_TypeDef callback )
{
  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( epAddr );

  if ( ep == NULL )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Illegal endpoint" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  if ( (   byteCount > MAX_XFER_LEN                           ) ||
       ( ( byteCount / ep->packetSize ) > MAX_PACKETS_PR_XFER )    )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Illegal transfer size" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

#if !defined( USB_SLAVEMODE )
  if ( (uint32_t)data & 3 )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Misaligned data buffer" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }
#endif

  if ( USBDHAL_EpIsStalled( ep ) )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Endpoint is halted" );
    return USB_STATUS_EP_STALLED;
  }

  if ( ep->state != D_EP_IDLE )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Endpoint is busy" );
    return USB_STATUS_EP_BUSY;
  }

  if ( ( ep->num > 0 ) && ( USBD_GetUsbState() != USBD_STATE_CONFIGURED ) )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Device not configured" );
    return USB_STATUS_DEVICE_UNCONFIGURED;
  }

  ep->buf       = (uint8_t*)data;
  ep->remaining = byteCount;
  ep->xferred   = 0;

  if ( ep->num == 0 )
  {
    ep->in = true;
  }
  else if ( ep->in != true )
  {
    DEBUG_USB_API_PUTS( "\nUSBD_Write(), Illegal EP direction" );
    EFM_ASSERT( false );
    return USB_STATUS_ILLEGAL;
  }

  ep->state          = D_EP_TRANSMITTING;
  ep->xferCompleteCb = callback;

  USBD_ArmEp( ep );
  return USB_STATUS_OK;
}


#endif /* defined( USB_DEVICE ) */
#endif /* defined( USB_PRESENT ) && ( USB_COUNT == 1 ) */
