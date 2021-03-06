/**************************************************************************//**
 * @file
 * @brief USB protocol stack library, USB device peripheral interrupt handlers.
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
#include "common_vars.h"


/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

#define HANDLE_INT( x ) if ( status & x ) { Handle_##x(); status &= ~x; }

static void Handle_USB_GINTSTS_ENUMDONE(void);
static void Handle_USB_GINTSTS_IEPINT(void);
static void Handle_USB_GINTSTS_OEPINT(void);
static void Handle_USB_GINTSTS_RESETDET(void);
#if defined( USB_SLAVEMODE )
static void Handle_USB_GINTSTS_RXFLVL ( void );
#endif
static void Handle_USB_GINTSTS_SOF(void);
static void Handle_USB_GINTSTS_USBRST(void);
static void Handle_USB_GINTSTS_USBSUSP(void);
static void Handle_USB_GINTSTS_WKUPINT(void);

static void __EMUSB_IRQHandler( void );

#if ( USB_PWRSAVE_MODE )
/* Variables and prototypes for USB powerdown (suspend) functionality. */
volatile bool USBD_poweredDown = false;
static bool UsbPowerDown(void);
static bool UsbPowerUp(void);

/* Storage for backing up USB core registers. */
static uint32_t x_USB_GINTMSK;
static uint32_t x_USB_GOTGCTL;
static uint32_t x_USB_GAHBCFG;
static uint32_t x_USB_GUSBCFG;
static uint32_t x_USB_GRXFSIZ;
static uint32_t x_USB_GNPTXFSIZ;
static uint32_t x_USB_DCFG;
static uint32_t x_USB_DCTL;
static uint32_t x_USB_DAINTMSK;
static uint32_t x_USB_DIEPMSK;
static uint32_t x_USB_DOEPMSK;
static uint32_t x_USB_PCGCCTL;

#if ( NUM_EP_USED > 0 )
static uint32_t x_USB_EP_CTL[NUM_EP_USED];
static uint32_t x_USB_EP_TSIZ[NUM_EP_USED];
static uint32_t x_USB_EP_DMAADDR[NUM_EP_USED];
#endif

#if ( NUM_EP_USED > MAX_NUM_TX_FIFOS )
#define FIFO_CNT MAX_NUM_TX_FIFOS
#else
#define FIFO_CNT NUM_EP_USED
#endif

#if ( FIFO_CNT > 0 )
static uint32_t x_USB_DIEPTXFS[FIFO_CNT];
#endif

#endif /* if ( USB_PWRSAVE_MODE ) */
//void UsbCoreTask(const void *arg);
//osThreadDef(UsbCoreTask, osPriorityHigh, 1, 512);

osMessageQDef(MsgUsb, 10, uint32_t);
osMessageQId hMsgUsb;


void initUsbTask()
{
	hMsgUsb = osMessageCreate(osMessageQ(MsgUsb), osThreadGetId());
	
}

#if 1
void UsbCoreTask(const void * arg)
{
	osEvent event;	
		
	
	while (1)
	{
		uint32_t value;

		event = osMessageGet(hMsgUsb, osWaitForever);
		value = event.value.v;

		if(value == 0){
			// TEST_H();
			__EMUSB_IRQHandler();
			
			NVIC_EnableIRQ(USB_IRQn);
		} else {
			((USBTIMER_Callback_TypeDef)value)();
		}

	}
}

void EMUSB_IRQHandler(void)
{
    //TEST_L();
	//__EMUSB_IRQHandler();
	//TEST_H();
	NVIC_DisableIRQ(USB_IRQn);
	osMessagePut(hMsgUsb,0,0);
}


#else
void UsbCoreTask(const void * arg)
{
	osEvent event;

	hMsgUsb = osMessageCreate(osMessageQ(MsgUsb), osThreadGetId());

	while (1)
	{
		uint32_t value;

		event = osMessageGet(hMsgUsb, osWaitForever);
		value = event.value.v;

		if(value == 0){
			// TEST_H();
			//__EMUSB_IRQHandler();
			
			//NVIC_EnableIRQ(USB_IRQn);
		} else {
			((USBTIMER_Callback_TypeDef)value)();
		}

	}
}

void EMUSB_IRQHandler(void)
{
    //TEST_L();
	__EMUSB_IRQHandler();
	//TEST_H();
	//NVIC_DisableIRQ(USB_IRQn);
	//osMessagePut(hMsgUsb,0,0);
}


#endif



static void __EMUSB_IRQHandler( void )
{
  uint32_t status;

#if ( USB_PWRSAVE_MODE )
  if ( USBD_poweredDown )
  {
    /* Switch USBC clock from 32kHz to HFCLK to be able to read USB */
    /* peripheral registers.                                        */
    /* If we woke up from EM2, HFCLK is now HFRCO.                  */

    CMU_OscillatorEnable( cmuOsc_HFXO, true, false);  /* Prepare HFXO. */
    CMU->CMD = CMU_CMD_USBCCLKSEL_HFCLKNODIV;
    while ( !( CMU->STATUS & CMU_STATUS_USBCHFCLKSEL ) ){}
  }
#endif /* if ( USB_PWRSAVE_MODE ) */




#if ( USB_PWRSAVE_MODE & USB_PWRSAVE_MODE_ONVBUSOFF )
 
 

  if ( USB->IF )
  { 
    if ( USB->IF & USB_IF_VREGOSH )
    {
      USB->IFC = USB_IFC_VREGOSH;

      if ( USB->STATUS & USB_STATUS_VREGOS )
      {
        if ( UsbPowerUp() )
          USBDHAL_EnableUsbResetInt();

        USBD_SetUsbState( USBD_STATE_POWERED );
      }
    	}
	

    if ( USB->IF & USB_IF_VREGOSL )
    {
      USB->IFC = USB_IFC_VREGOSL;

      if ( !( USB->STATUS & USB_STATUS_VREGOS ) )
      {
     
        USB->GINTMSK = 0;
        USB->GINTSTS = 0xFFFFFFFF;

        UsbPowerDown();
        USBD_SetUsbState( USBD_STATE_NONE );
      }
    }
  }

#endif

 

  status = USBHAL_GetCoreInts();
  if ( status == 0 )
  {
    return;
  	}
 
  HANDLE_INT( USB_GINTSTS_RESETDET   )
  HANDLE_INT( USB_GINTSTS_WKUPINT    )
  HANDLE_INT( USB_GINTSTS_USBSUSP    )
  HANDLE_INT( USB_GINTSTS_SOF        )
#if defined( USB_SLAVEMODE )
  HANDLE_INT( USB_GINTSTS_RXFLVL     )
#endif
  HANDLE_INT( USB_GINTSTS_ENUMDONE   )
  HANDLE_INT( USB_GINTSTS_USBRST     )
  HANDLE_INT( USB_GINTSTS_IEPINT     )
  HANDLE_INT( USB_GINTSTS_OEPINT     )

}



/*
 * Handle port enumeration interrupt. This has nothing to do with normal
 * device enumeration.
 */
static void Handle_USB_GINTSTS_ENUMDONE(void)
{
#if ( USB_PWRSAVE_MODE )
	UsbPowerUp();
#endif /* if ( USB_PWRSAVE_MODE ) */

	USBDHAL_Ep0Activate();
	dev->ep[0].state = D_EP_IDLE;
	USBDHAL_EnableInts(dev);
	
}

/*
 * Handle IN endpoint transfer interrupt.
 */
static void Handle_USB_GINTSTS_IEPINT(void)
{
	int epnum;
	uint16_t epint;
	uint16_t epmask;
	uint32_t status;
	USBD_Ep_TypeDef *ep;
	//TEST_L();

	DEBUG_USB_INT_HI_PUTCHAR( 'i' );

	epint = USBDHAL_GetAllInEpInts();
	for (epnum = 0, epmask = 1; epnum <= MAX_NUM_IN_EPS; epnum++, epmask <<= 1)
	{
		if (epint & epmask)
		{
			ep = USBD_GetEpFromAddr(USB_SETUP_DIR_MASK | epnum);
			status = USBDHAL_GetInEpInts(ep);

			if (status & USB_DIEP_INT_XFERCOMPL)
			{
#if defined( USB_SLAVEMODE )
				/* Disable Tx FIFO empty interrupt */
				USB->DIEPEMPMSK &= ~epmask;
				USB_DINEPS[ epnum ].INT = USB_DIEP_INT_TXFEMP;
				status &= ~USB_DIEP_INT_TXFEMP;
#endif
				USB_DINEPS [epnum].INT = USB_DIEP_INT_XFERCOMPL;

				DEBUG_USB_INT_HI_PUTCHAR( 'c' );

				if (epnum == 0)
				{
#if !defined( USB_SLAVEMODE )
					if (ep->remaining > ep->packetSize)
					{
						ep->remaining -= ep->packetSize;
						ep->xferred += ep->packetSize;
					}
					else
					{
						ep->xferred += ep->remaining;
						ep->remaining = 0;
					}
#endif
					USBDEP_Ep0Handler(dev);
				}
				else
				{
#if !defined( USB_SLAVEMODE )
					ep->xferred = ep->remaining
							- ((USB_DINEPS [epnum].TSIZ
									& _USB_DIEP_TSIZ_XFERSIZE_MASK)
									>> _USB_DIEP_TSIZ_XFERSIZE_SHIFT);
					ep->remaining -= ep->xferred;
#endif
					USBDEP_EpHandler(ep->addr);
				}
			}

#if defined( USB_SLAVEMODE )
			if ( status & USB_DIEP_INT_TXFEMP )
			{
				USB_DINEPS[ epnum ].INT = USB_DIEP_INT_TXFEMP;

				if ( ep->state != D_EP_IDLE )
				{
					DEBUG_USB_INT_HI_PUTCHAR( 'f' );
					USBDHAL_FillFifo( ep );
				}
			}
#endif
		}
	}
	//TEST_H();
}

/*
 * Handle OUT endpoint transfer interrupt.
 */
static void Handle_USB_GINTSTS_OEPINT(void)
{
	int epnum;
	uint16_t epint;
	uint16_t epmask;
	uint32_t status;
	USBD_Ep_TypeDef *ep;

	DEBUG_USB_INT_HI_PUTCHAR( 'o' );
	epint = USBDHAL_GetAllOutEpInts();
	for (epnum = 0, epmask = 1; epnum <= MAX_NUM_OUT_EPS; epnum++, epmask <<= 1)
	{
		if (epint & epmask)
		{
			ep = USBD_GetEpFromAddr(epnum);
			status = USBDHAL_GetOutEpInts(ep);

			if (status & USB_DOEP_INT_XFERCOMPL)
			{
				USB_DOUTEPS [epnum].INT = USB_DOEP_INT_XFERCOMPL;

				DEBUG_USB_INT_HI_PUTCHAR( 'c' );

				if (epnum == 0)
				{
#if !defined( USB_SLAVEMODE )
					if (ep->remaining > ep->packetSize)
					{
						ep->remaining -= ep->packetSize;
						ep->xferred += ep->packetSize;
					}
					else
					{
						ep->xferred += ep->remaining;
						ep->remaining = 0;
					}
#endif
					USBDEP_Ep0Handler(dev);
				}
				else
				{
#if !defined( USB_SLAVEMODE )
					ep->xferred = ep->hwXferSize
							- ((USB_DOUTEPS [epnum].TSIZ
									& _USB_DOEP_TSIZ_XFERSIZE_MASK)
									>> _USB_DOEP_TSIZ_XFERSIZE_SHIFT);
					ep->remaining -= ep->xferred;
#endif
					USBDEP_EpHandler(ep->addr);
				}
			}

			/* Setup Phase Done */
			if (status & USB_DOEP_INT_SETUP)
			{
				

#if !defined( USB_SLAVEMODE )
				if (USB ->DOEP0INT & USB_DOEP_INT_BACK2BACKSETUP)
				{ /* Back to back setup packets received */
					USB ->DOEP0INT = USB_DOEP_INT_BACK2BACKSETUP;
					

					dev->setup = (USB_Setup_TypeDef*) (USB ->DOEP0DMAADDR
							- USB_SETUP_PKT_SIZE);
				}
				else
				{
					/* Read SETUP packet counter from hw. */
					int supCnt = (USB ->DOEP0TSIZ & _USB_DOEP0TSIZ_SUPCNT_MASK)
							>> _USB_DOEP0TSIZ_SUPCNT_SHIFT;

					if (supCnt == 3)
						supCnt = 2;

					dev->setup = &dev->setupPkt[2 - supCnt];
				}
#endif
				USB ->DOEP0TSIZ |= 3 << _USB_DOEP0TSIZ_SUPCNT_SHIFT;
				USB ->DOEP0DMAADDR = (uint32_t) dev->setupPkt;
				USB ->DOEP0INT = USB_DOEP_INT_SETUP;
				USBDEP_Ep0Handler(dev); /* Call the SETUP process for the EP0  */
			}
		}
	}
}

/*
 * Handle USB reset detectet interrupt in suspend mode.
 */
static void Handle_USB_GINTSTS_RESETDET(void)
{
	USB ->GINTSTS = USB_GINTSTS_RESETDET;

#if ( USB_PWRSAVE_MODE )
	UsbPowerUp();
#endif /* if ( USB_PWRSAVE_MODE ) */

	USBD_SetUsbState(USBD_STATE_DEFAULT);
	
}

#if defined( USB_SLAVEMODE )
/*
 * Handle receive FIFO full interrupt.
 */
static void Handle_USB_GINTSTS_RXFLVL( void )
{
	USBD_Ep_TypeDef *ep;
	uint32_t status, byteCount, count, residue;

	DEBUG_USB_INT_HI_PUTCHAR( 'q' );

	status = USB->GRXSTSP; /* Get status from top of FIFO */

	ep = USBD_GetEpFromAddr( status & _USB_GRXSTSP_CHEPNUM_MASK );

	switch ( status & _USB_GRXSTSP_PKTSTS_MASK )
	{
		case GRXSTSP_PKTSTS_DEVICE_DATAOUTRECEIVED:
		byteCount = (status & _USB_GRXSTSP_BCNT_MASK) >> _USB_GRXSTSP_BCNT_SHIFT;
		if ( byteCount )
		{
			DEBUG_USB_INT_HI_PUTCHAR( 'e' );
			if ( ep->state != D_EP_IDLE )
			{
				/* Check for possible buffer overflow */
				if ( byteCount > ep->remaining )
				{
					residue = byteCount - ep->remaining;
				}
				else
				{
					residue = 0;
				}

				count = EFM32_MIN( byteCount, ep->remaining );
				USBHAL_ReadFifo( ep->buf, count );
				ep->xferred += count;
				ep->remaining -= count;
				ep->buf += count;

				if ( residue )
				{
					USBHAL_FlushFifo( residue );
				}
			}
		}
		break;

		case GRXSTSP_PKTSTS_DEVICE_SETUPRECEIVED:
		USBHAL_ReadFifo( (uint8_t*)dev->setup, USB_SETUP_PKT_SIZE );
		break;
	}
}
#endif

/*
 * Handle Start Of Frame (SOF) interrupt.
 */
static void Handle_USB_GINTSTS_SOF(void)
{
	USB ->GINTSTS = USB_GINTSTS_SOF;

	if (dev->callbacks->sofInt)
	{
		dev->callbacks->sofInt(
				(USB ->DSTS & _USB_DSTS_SOFFN_MASK) >> _USB_DSTS_SOFFN_SHIFT);
	}
}

/*
 * Handle USB port reset interrupt.
 */
static void Handle_USB_GINTSTS_USBRST(void)
{
	int i;


  /* Clear Remote Wakeup Signalling */
  USB->DCTL &= ~( DCTL_WO_BITMASK | USB_DCTL_RMTWKUPSIG );
  USBHAL_FlushTxFifo( 0 );

	/* Clear pending interrupts */
	for (i = 0; i <= MAX_NUM_IN_EPS; i++)
	{
		USB_DINEPS [i].INT = 0xFFFFFFFF;
	}

	for (i = 0; i <= MAX_NUM_OUT_EPS; i++)
	{
		USB_DOUTEPS [i].INT = 0xFFFFFFFF;
	}

	USB ->DAINTMSK = USB_DAINTMSK_INEPMSK0 | USB_DAINTMSK_OUTEPMSK0;
	USB ->DOEPMSK = USB_DOEPMSK_SETUPMSK | USB_DOEPMSK_XFERCOMPLMSK;
	USB ->DIEPMSK = USB_DIEPMSK_XFERCOMPLMSK;

  /* Reset Device Address */
  USB->DCFG &= ~_USB_DCFG_DEVADDR_MASK;

	/* Setup EP0 to receive SETUP packets */
	USBDHAL_StartEp0Setup(dev);
	USBDHAL_EnableInts(dev);

	if (dev->callbacks->usbReset)
	{
		dev->callbacks->usbReset();
	}

	USBD_SetUsbState(USBD_STATE_DEFAULT);
	USBDHAL_AbortAllTransfers(USB_STATUS_DEVICE_RESET);
}

/*
 * Handle USB port suspend interrupt.
 */
static void Handle_USB_GINTSTS_USBSUSP(void)
{
	USBD_State_TypeDef state;

	USB ->GINTSTS = USB_GINTSTS_USBSUSP;
	USBDHAL_AbortAllTransfers(USB_STATUS_DEVICE_SUSPENDED);
	
	state = USBD_GetUsbState();
	if ((state == USBD_STATE_POWERED) || (state == USBD_STATE_DEFAULT)
			|| (state == USBD_STATE_ADDRESSED)
			|| (state == USBD_STATE_CONFIGURED))
	{
#if ( USB_PWRSAVE_MODE & USB_PWRSAVE_MODE_ONSUSPEND )
		UsbPowerDown();
#endif /* if ( USB_PWRSAVE_MODE ) */
		USBD_SetUsbState(USBD_STATE_SUSPENDED);
	}
}

/*
 * Handle USB port wakeup interrupt.
 */
static void Handle_USB_GINTSTS_WKUPINT(void)
{
	USB ->GINTSTS = USB_GINTSTS_WKUPINT;

#if ( USB_PWRSAVE_MODE )
	if (UsbPowerUp())
	{
		USBDHAL_StartEp0Setup(dev);
		USBDHAL_Ep0Activate();
	}
#endif /* if ( USB_PWRSAVE_MODE ) */

	USBD_SetUsbState(dev->savedState);
	
}

#if ( USB_PWRSAVE_MODE )
/*
 * Backup essential USB core registers, and set the core in partial powerdown
 * mode. Optionally prepare entry into EM2.
 */
static bool UsbPowerDown(void)
{
#if ( NUM_EP_USED > 0 ) || ( FIFO_CNT > 0 )
	int i;
#endif
#if ( NUM_EP_USED > 0 )
	int epNum;
	USBD_Ep_TypeDef *ep;
#endif

	if (!USBD_poweredDown)
	{
		USBD_poweredDown = true;

		/* Backup USB core registers. */
		x_USB_GINTMSK = USB ->GINTMSK;
		x_USB_GOTGCTL = USB ->GOTGCTL;
		x_USB_GAHBCFG = USB ->GAHBCFG;
		x_USB_GUSBCFG = USB ->GUSBCFG;
		x_USB_GRXFSIZ = USB ->GRXFSIZ;
		x_USB_GNPTXFSIZ = USB ->GNPTXFSIZ;
		x_USB_DCFG = USB ->DCFG;
		x_USB_DCTL = USB ->DCTL;
		x_USB_DAINTMSK = USB ->DAINTMSK;
		x_USB_DIEPMSK = USB ->DIEPMSK;
		x_USB_DOEPMSK = USB ->DOEPMSK;
		x_USB_PCGCCTL = USB ->PCGCCTL;

#if ( NUM_EP_USED > 0 )
		for (i = 0; i < NUM_EP_USED; i++)
		{
			ep = &dev->ep[i + 1];
			epNum = ep->num;
			if (ep->in)
			{
				x_USB_EP_CTL[i] = USB_DINEPS [epNum].CTL;
				x_USB_EP_TSIZ[i] = USB_DINEPS [epNum].TSIZ;
				x_USB_EP_DMAADDR[i] = USB_DINEPS [epNum].DMAADDR;
			}
			else
			{
				x_USB_EP_CTL[i] = USB_DOUTEPS [epNum].CTL;
				x_USB_EP_TSIZ[i] = USB_DOUTEPS [epNum].TSIZ;
				x_USB_EP_DMAADDR[i] = USB_DOUTEPS [epNum].DMAADDR;
			}
		}
#endif

#if ( FIFO_CNT > 0 )
		for (i = 0; i < FIFO_CNT; i++)
		{
			x_USB_DIEPTXFS[i] = USB_DIEPTXFS [i];
		}
#endif

    /* Prepare for wakeup on resume and reset. */
    USB->DCFG    = (USB->DCFG & ~_USB_DCFG_RESVALID_MASK) |
                   (4 << _USB_DCFG_RESVALID_SHIFT);
    USB->DCFG   |= USB_DCFG_ENA32KHZSUSP;
    USB->GINTMSK = USB_GINTMSK_RESETDETMSK | USB_GINTMSK_WKUPINTMSK;

    /* Enter partial powerdown mode. */
    USB->PCGCCTL |= USB_PCGCCTL_PWRCLMP;
    USB->PCGCCTL |= USB_PCGCCTL_RSTPDWNMODULE;
    USB->PCGCCTL |= USB_PCGCCTL_STOPPCLK;

#if ( USB_PWRSAVE_MODE & USB_PWRSAVE_MODE_ENTEREM2 )
		/* Enter EM2 on interrupt exit. */
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk;
#endif

		/* Switch USBC clock to 32 kHz. */
#if ( USB_USBC_32kHz_CLK == USB_USBC_32kHz_CLK_LFXO )
		CMU ->CMD = CMU_CMD_USBCCLKSEL_LFXO;
#else
		CMU->CMD = CMU_CMD_USBCCLKSEL_LFRCO;
#endif
		while (CMU ->STATUS & CMU_STATUS_USBCHFCLKSEL)
		{
		 }
	
	    USB_OFF_CLKTO14MHZ();

		//===============================
        //SystemParam->app.statusflag&=~STATUS_USB_CONNECT;	
		//SystemParam->app.statusflag|=STATUS_BAR_UPDATA;	
		//===============================
		
		return true;
	}
	return false;
}
#endif /* if ( USB_PWRSAVE_MODE ) */

#if ( USB_PWRSAVE_MODE )
/*
 * Exit USB core partial powerdown mode, restore essential USB core registers.
 * Optionally prevent re-entry back to EM2.
 */
static bool UsbPowerUp(void)
{
#if ( NUM_EP_USED > 0 ) || ( FIFO_CNT > 0 )
	int i;
#endif
#if ( NUM_EP_USED > 0 )
	int epNum;
	uint32_t tmp;
	USBD_Ep_TypeDef *ep;
#endif

#if ( USB_PWRSAVE_MODE & USB_PWRSAVE_MODE_ONVBUSOFF )
	if (USBD_poweredDown && (USB ->STATUS & USB_STATUS_VREGOS))
#else
	if ( USBD_poweredDown )
#endif
	{		
	    USBD_poweredDown = false;
		USB_ON_CLKTO48MHZ();

        /* Exit partial powerdown mode. */
        USB->PCGCCTL &= ~USB_PCGCCTL_STOPPCLK;
        USB->PCGCCTL &= ~(USB_PCGCCTL_PWRCLMP | USB_PCGCCTL_RSTPDWNMODULE);

		/* Restore USB core registers. */USB ->GUSBCFG = x_USB_GUSBCFG;
		USB ->DCFG = x_USB_DCFG;

#if ( FIFO_CNT > 0 )
		for (i = 0; i < FIFO_CNT; i++)
		{
			USB_DIEPTXFS [i] = x_USB_DIEPTXFS[i];
		}
#endif

#if ( NUM_EP_USED > 0 )
		for (i = 0; i < NUM_EP_USED; i++)
		{
			ep = &dev->ep[i + 1];
			epNum = ep->num;
			if (ep->in)
			{
				tmp = (ep->packetSize << _USB_DIEP_CTL_MPS_SHIFT)
						| (ep->type << _USB_DIEP_CTL_EPTYPE_SHIFT)
						| (ep->txFifoNum << _USB_DIEP_CTL_TXFNUM_SHIFT)
						| USB_DIEP_CTL_USBACTEP | USB_DIEP_CTL_SNAK;

				if (x_USB_EP_CTL[i] & USB_DIEP_CTL_DPIDEOF)
					tmp |= USB_DIEP_CTL_SETD1PIDOF;
				else
					tmp |= USB_DIEP_CTL_SETD0PIDEF;

				USB_DINEPS [epNum].CTL = tmp;
				USB_DINEPS [epNum].TSIZ = x_USB_EP_TSIZ[i];
				USB_DINEPS [epNum].DMAADDR = x_USB_EP_DMAADDR[i];
			}
			else
			{
				if (x_USB_EP_CTL[i] & USB_DOEP_CTL_DPIDEOF)
					USB_DOUTEPS [epNum].CTL = x_USB_EP_CTL[i]
							| USB_DOEP_CTL_SETD1PIDOF;
				else
					USB_DOUTEPS [epNum].CTL = x_USB_EP_CTL[i]
							| USB_DOEP_CTL_SETD0PIDEF;

				USB_DOUTEPS [epNum].TSIZ = x_USB_EP_TSIZ[i];
				USB_DOUTEPS [epNum].DMAADDR = x_USB_EP_DMAADDR[i];
			}
		}
#endif

		USB ->PCGCCTL = x_USB_PCGCCTL;
		USB ->DOEPMSK = x_USB_DOEPMSK;
		USB ->DIEPMSK = x_USB_DIEPMSK;
		USB ->DAINTMSK = x_USB_DAINTMSK;
		USB ->DCTL = x_USB_DCTL;
		USB ->GNPTXFSIZ = x_USB_GNPTXFSIZ;
		USB ->GRXFSIZ = x_USB_GRXFSIZ;
		USB ->GAHBCFG = x_USB_GAHBCFG;
		USB ->GOTGCTL = x_USB_GOTGCTL;
		USB ->GINTMSK = x_USB_GINTMSK;

		USB ->DCTL |= USB_DCTL_PWRONPRGDONE;

#if ( USB_PWRSAVE_MODE & USB_PWRSAVE_MODE_ENTEREM2 )
		/* Do not reenter EM2 on interrupt exit. */
		SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
#endif

		return true;
	}
	return false;
}
#endif /* if ( USB_PWRSAVE_MODE ) */

/** @endcond */

#endif /* defined( USB_DEVICE ) */
#endif /* defined( USB_PRESENT ) && ( USB_COUNT == 1 ) */

