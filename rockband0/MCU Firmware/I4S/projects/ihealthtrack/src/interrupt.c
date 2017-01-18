#include "em_rtc.h"
//#include "cmsis_os.h"

#include "bsp.h"
#include "main.h"
#include "common_vars.h"
#include "device_task.h"

extern void SVC_Handler (void);
extern void PendSV_Handler (void);
extern void SysTick_Handler (void);

extern void vPortSVCHandler (void);
extern void xPortPendSVHandler (void);
extern void xPortSysTickHandler (void);
extern void xPortRTCTickHandler (void);

extern void EMUSB_IRQHandler(void);

//extern osSemaphoreId hSemaClock;


#if defined (__GNUC__)
/* Because of the way the FreeRTOS SVC handler is implemented,
 * we must make sure that we do not manipulate the stack or the LR register here.
 * It may otherwise happen depending on optimization level!
 */
__attribute__((naked)) void SVC_Handler (void);
void SVC_Handler (void)
{
    __asm (
        " b.w vPortSVCHandler \n\t"
    );
}

__attribute__((naked)) void SysTick_Handler (void);
void SysTick_Handler (void)
{
    __asm (
        " b.w xPortSysTickHandler \n\t"
    );
}

__attribute__((naked)) void PendSV_Handler (void);
void PendSV_Handler (void)
{
    __asm (
        " b.w xPortPendSVHandler \n\t"
    );
}

#else
#pragma required=vPortSVCHandler
#pragma required=xPortPendSVHandler
#pragma required=xPortSysTickHandler

//void SVC_Handler (void)
//{
//    asm(" ldr r2,[pc,#0] \n"
//        " bx r2\n"
//        " dcd vPortSVCHandler \n"); /* jump over constant */
//}

//void SysTick_Handler (void)
//{
//    xPortSysTickHandler();
//}
//
//void PendSV_Handler (void)
//{
//    asm(" ldr r2,[pc,#0] \n"
//        " bx r2\n"
//        " dcd xPortPendSVHandler \n"); /* jump over constant */
//}

#endif /*__GNUC__*/

#if (configUSE_TICKLESS_IDLE == 3)
void RTC_IRQHandler (void)
{
	uint32_t ifc;

	ifc = RTC_IntGet();
	if((ifc&RTC_IF_COMP0) == RTC_IF_COMP0){
		RTC_IntClear(RTC_IF_COMP0);
	}
	/* Clear interrupt source */
	if((ifc&RTC_IF_COMP1) == RTC_IF_COMP1){
		xPortRTCTickHandler();
	}
}
#elif (configUSE_TICKLESS_IDLE == 4)
void RTC_IRQHandler (void)
{
	xPortRTCTickHandler();
}
#endif

void USB_IRQHandler(void)
{
	//EMUSB_IRQHandler();
}

//void HardFault_Handler()
//{
//	while(1);
//}


void LETIMER0_IRQHandler()
{
  /* Clear LETIMER0 underflow interrupt flag */
   LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);


//   osMessagePut(hMsgInterrupt, TICK_Message, 0);
	int32_t msg = TICK_Message;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);

 }


void GPIO_EVEN_IRQHandler(void)
{
  uint32_t status;
  
  status = GPIO->IF;
  GPIO->IFC = status;
  
 // if (status & (1 << KEY_PIN)) {
     //KeyIntHandler();
  //}
  extern void EnableLeUart(void);
  if (status & (1 << BLE_INT_PIN)) {
     EnableLeUart();
  }

 // if (status & (1 << CHARGER_STA_PIN)) 
  //{  
  //osMessagePut(hMsgInterrupt,MESSAGE_BATTERY_CHARGING,0);  	   
  //} 
}
  


/**************************************************************************//**
 * @brief GPIO ODD IRQ
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  uint32_t flags = GPIO_IntGet();
  GPIO_IntClear(flags);
  
  if (flags&(1<<AFE_ADC_DRDY_PIN)) 
     {   
      extern void AFE_INT_CALLBACK(void);
  	  AFE_INT_CALLBACK(); 
     }   
  
  if (flags&(1<<MEMS_INT1_PIN)) 
     {	 
     extern void MEMS_INT1_CALLBACK(void);
     MEMS_INT1_CALLBACK(); 
     }   
  
  if (flags&(1<<MEMS_INT2_PIN)) 
     {	 
     extern void MEMS_INT2_CALLBACK(void);
     MEMS_INT2_CALLBACK(); 
     }   
}

extern void DelayTimerCallback(void);
void TIMER2_IRQHandler(void)
{
  uint32_t IntFlag; // NextTime; [BG025] remark.
  
  IntFlag = TIMER_DEALY->IF;
  TIMER_IntClear(TIMER_DEALY, IntFlag);

  if (IntFlag & TIMER_IF_OF)
  	        DelayTimerCallback();

 }


