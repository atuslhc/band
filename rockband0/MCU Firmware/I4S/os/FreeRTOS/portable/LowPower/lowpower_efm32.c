/*
 * lowpower_efm32.c
 *
 *  Created on: 2013-6-13
 *      Author: Administrator
 */
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_device.h"
#include "core_cm4.h"
#include "core_cmFunc.h"

#include "efm32.h"
#include "em_rtc.h"
#include "em_emu.h"
#include "em_cmu.h"

#include "bsp.h"

#if defined(__ICCARM__)
#define __WEAK __weak
#elif defined(__GNUC__)
#define __WEAK __attribute__(( weak ))
#endif


#if (configUSE_TICKLESS_IDLE == 1) || (configUSE_TICKLESS_IDLE == 2)
#ifndef configSYSTICK_CLOCK_HZ
#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
#endif
#endif


#define configRTCTICK_CLOCK_HZ          (32768)
#define configRTC_GET_COUNT()			RTC_CounterGet()
#define configRTC_RESET_COUNT()			RTC_CounterReset()
#define configRTC_SET_COMPARE(x)		RTC_CompareSet(0,(x))
#define configRTC_CNT_MASK              _RTC_CNT_MASK
#define configRTC_ENABLE()				RTC_Enable(true)
#define configRTC_DISABLE()             RTC_Enable(false)
#define configSLEEP_PROCESSING()        EMU_EnterEM2(true)

#define OSTICK_TO_RTCTICK(x)  (configRTCTICK_CLOCK_HZ*(x)/configTICK_RATE_HZ)
#define RTCTICK_TO_OSTICK(x)  (configTICK_RATE_HZ*(x)/configRTCTICK_CLOCK_HZ)
#define RTCTICK_TO_SYSTICK(x) (configSYSTICK_CLOCK_HZ*(x)/configRTCTICK_CLOCK_HZ)

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile unsigned long * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile unsigned long * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile unsigned long * ) 0xe000e018 ) )
#define portNVIC_INT_CTRL_REG				( * ( ( volatile unsigned long * ) 0xe000ed04 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile unsigned long * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_CLK_BIT			( 1UL << 2UL )
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVSET_BIT				( 1UL << 28UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )


/*-----------------------------------------------------------*/
/*
 * The number of SysTick increments that make up one tick period.
 */
#if configUSE_TICKLESS_IDLE == 1 || configUSE_TICKLESS_IDLE == 2 || configUSE_TICKLESS_IDLE == 3 || configUSE_TICKLESS_IDLE == 4
static unsigned long ulTimerReloadValueForOneTick = 0;
#endif

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if configUSE_TICKLESS_IDLE == 1 || configUSE_TICKLESS_IDLE == 2 || configUSE_TICKLESS_IDLE == 3 || configUSE_TICKLESS_IDLE == 4
static unsigned long xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if configUSE_TICKLESS_IDLE == 1 || configUSE_TICKLESS_IDLE == 2 || configUSE_TICKLESS_IDLE == 3 || configUSE_TICKLESS_IDLE == 4
static unsigned long ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */



/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
#if configUSE_TICKLESS_IDLE == 1
__WEAK void vPortSuppressTicksAndSleep( portTickType xExpectedIdleTime )
{
    unsigned long ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickIncrements;
    portTickType xModifiableIdleTime;


    /* Make sure the SysTick reload value does not overflow the counter. */
    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods, and the fraction of a tick period is
    accounted for later. */
    ulReloadValue = ( ulTimerReloadValueForOneTick * ( xExpectedIdleTime - 1UL ) );
    if( ulReloadValue > ulStoppedTimerCompensation )
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Stop the SysTick momentarily.  The time the SysTick is stopped for
    is accounted for as best it can be, but using the tickless mode will
    inevitably result in some tiny drift of the time maintained by the
    kernel with respect to calendar time. */
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;

    /* Adjust the reload value to take into account that the current time
    slice is already partially complete. */
    ulReloadValue += ( portNVIC_SYSTICK_LOAD_REG - ( portNVIC_SYSTICK_LOAD_REG - portNVIC_SYSTICK_CURRENT_VALUE_REG ) );

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;

        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        __enable_irq();
    }
    else
    {
        /* Set the new reload value. */
        portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

        /* Clear the SysTick count flag and set the count value back to
        zero. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if( xModifiableIdleTime > 0 )
        {
            __asm volatile( "wfi" );
        }
        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        /* Stop SysTick.  Again, the time the SysTick is stopped for is
        accounted for as best it can be, but using the tickless mode will
        inevitably result in some tiny drift of the time maintained by the
        kernel with respect to calendar time. */
        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;

        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        __enable_irq();

        if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
        {
            /* The tick interrupt has already executed, and the SysTick
            count reloaded with the portNVIC_SYSTICK_LOAD_REG value.
            Reset the portNVIC_SYSTICK_LOAD_REG with whatever remains of
            this tick period. */
            portNVIC_SYSTICK_LOAD_REG = ulTimerReloadValueForOneTick - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
            Work out how long the sleep lasted. */
            ulCompletedSysTickIncrements = ( xExpectedIdleTime * ulTimerReloadValueForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

            /* How many complete tick periods passed while the processor
            was waiting? */
            ulCompleteTickPeriods = ulCompletedSysTickIncrements / ulTimerReloadValueForOneTick;

            /* The reload value is set to whatever fraction of a single tick
            period remains. */
            portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1 ) * ulTimerReloadValueForOneTick ) - ulCompletedSysTickIncrements;
        }

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
        again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
        value. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;

        vTaskStepTick( ulCompleteTickPeriods );
    }
}
#endif /* #if configUSE_TICKLESS_IDLE */
#if configUSE_TICKLESS_IDLE == 2
void OS_RTC_TickInit (void);
void OS_RTC_TickSetup(uint32_t tick);

__WEAK void vPortSuppressTicksAndSleep( portTickType xExpectedIdleTime )
{
    unsigned long ulReloadValue, ulRTCCurCnt, ulRTCStoreCnt, ulCompleteTickPeriods, ulRTCTickPeriods;
    portTickType xModifiableIdleTime;

    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods, and the fraction of a tick period is
    accounted for later. */
    ulReloadValue = OSTICK_TO_RTCTICK( xExpectedIdleTime - 1);
    if( ulReloadValue > ulStoppedTimerCompensation )
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }


    __disable_irq();

    /* If a context switch is pending or a task is waiting for the scheduler
     to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        /* Re-enable interrupts - see comments above the cpsid instruction()
        above. */
        __enable_irq();
    }
    else
    {

        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;

        ulRTCCurCnt = 0;
        ulRTCStoreCnt = ulRTCCurCnt;
        ulReloadValue += ulRTCCurCnt;

        /* Set the new reload value. */
        configRTC_RESET_COUNT();
        configRTC_SET_COMPARE(ulReloadValue);
        configRTC_ENABLE();



        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
         set its parameter to 0 to indicate that its implementation contains
         its own wait for interrupt or wait for event instruction, and so wfi
         should not be executed again.  However, the original expected idle
         time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if( xModifiableIdleTime > 0 )
        {
            configSLEEP_PROCESSING();
        }
        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );


        ulRTCCurCnt = configRTC_GET_COUNT();
        configRTC_DISABLE();
        if( ulRTCCurCnt >= ulReloadValue)
        {
            /* The tick interrupt has already executed, and the SysTick
             count reloaded with the portNVIC_SYSTICK_LOAD_REG value.
             Reset the portNVIC_SYSTICK_LOAD_REG with whatever remains of
             this tick period. */

            /* The tick interrupt handler will already have pended the tick
             processing in the kernel.  As the pending tick will be
             processed as soon as this function exits, the tick value
             maintained by the tick is stepped forward by one less than the
             time spent waiting. */
            ulCompleteTickPeriods = xExpectedIdleTime;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
             Work out how long the sleep lasted. */
            ulRTCTickPeriods  = (ulRTCCurCnt - ulRTCStoreCnt)&configRTC_CNT_MASK;
            ulCompleteTickPeriods = RTCTICK_TO_OSTICK(ulRTCTickPeriods);


        }

        portNVIC_SYSTICK_LOAD_REG = ulTimerReloadValueForOneTick -
                                    (RTCTICK_TO_SYSTICK((ulRTCCurCnt - ulRTCStoreCnt)&configRTC_CNT_MASK) -
                                     portNVIC_SYSTICK_LOAD_REG + portNVIC_SYSTICK_CURRENT_VALUE_REG) %
                                    ulTimerReloadValueForOneTick;
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
         again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
         value. */
        vTaskStepTick( ulCompleteTickPeriods );
        __enable_irq();
    }
}
#endif

#if configUSE_TICKLESS_IDLE == 3
void  OS_RTC_TickInit (void);
void  OS_RTC_TickSetup(uint32_t tick);

bool IsSafeToEnterEM2(void)
{
    extern bool USBD_SafeToEnterEM2(void);
    return USBD_SafeToEnterEM2();
}

__WEAK void vPortSuppressTicksAndSleep( portTickType xExpectedIdleTime )
{
    unsigned long ulReloadValue, ulRTCTickPeriods, ulRTCCurCnt, ulRTCStoreCnt, ulCompleteTickPeriods;
    portTickType xModifiableIdleTime;

    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    ulReloadValue = OSTICK_TO_RTCTICK(xExpectedIdleTime);
    if( ulReloadValue > ulStoppedTimerCompensation )
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    //__disable_interrupt();
    __disable_irq();

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep || !IsSafeToEnterEM2())
    {
        __enable_irq();
        return;
    }
    else
    {
        ulRTCCurCnt = configRTC_GET_COUNT();
        ulRTCStoreCnt = ulRTCCurCnt;

        ulReloadValue = OSTICK_TO_RTCTICK(
                            RTCTICK_TO_OSTICK(ulRTCCurCnt) + xExpectedIdleTime);

        if(ulReloadValue > configRTCTICK_CLOCK_HZ)
        {
            ulReloadValue = configRTCTICK_CLOCK_HZ;
        }

        configRTC_SET_COMPARE(ulReloadValue);



        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if( xModifiableIdleTime > 0 )
        {

            if (GetSysEnergyModeFlag() == 0)
            {
                EMU_EnterEM2(true);
            }
            else
            {
                EMU_EnterEM1();//DMA Not Done

                ulRTCCurCnt = configRTC_GET_COUNT();
                if( ((ulRTCCurCnt - ulRTCStoreCnt)&configRTC_CNT_MASK ) <
                        ulReloadValue)
                {
                    EMU_EnterEM2(true);
                }
            }
        }
        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        ulRTCCurCnt = configRTC_GET_COUNT();

        if(ulRTCCurCnt > ulRTCStoreCnt)
        {
            ulRTCTickPeriods = ulRTCCurCnt - ulRTCStoreCnt;
        }
        else
        {
            ulRTCTickPeriods = configRTCTICK_CLOCK_HZ + ulRTCCurCnt - ulRTCStoreCnt;
        }

        ulCompleteTickPeriods = RTCTICK_TO_OSTICK(ulRTCTickPeriods);
        if(ulRTCTickPeriods >= configRTCTICK_CLOCK_HZ)
        {
            ulRTCTickPeriods = OSTICK_TO_RTCTICK(1);
        }
        else
        {
            ulRTCTickPeriods = OSTICK_TO_RTCTICK(ulCompleteTickPeriods + 1);
        }

        configRTC_SET_COMPARE(ulRTCTickPeriods);

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
        again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
        value. */
        vTaskStepTick( ulCompleteTickPeriods - 1);
        __enable_irq();


        ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
        {
            vTaskIncrementTick();
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );

#if configUSE_PREEMPTION == 1
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
#endif
    }
}
#endif

#if configUSE_TICKLESS_IDLE == 4
void  OS_RTC_TickInit (void);
void  OS_RTC_TickSetup(uint32_t tick);

bool IsSafeToEnterEM2(void)
{
    extern bool USBD_SafeToEnterEM2(void);
    return USBD_SafeToEnterEM2();
}

__WEAK void vPortSuppressTicksAndSleep( portTickType xExpectedIdleTime )
{
    unsigned long ulReloadValue, ulRTCTickPeriods, ulRTCCurCnt, ulRTCStoreCnt, ulCompleteTickPeriods;
    portTickType xModifiableIdleTime;

    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    ulReloadValue = OSTICK_TO_RTCTICK(xExpectedIdleTime);
    if( ulReloadValue > ulStoppedTimerCompensation )
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    //__disable_interrupt();
    __disable_irq();

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep || !IsSafeToEnterEM2())
    {
        __enable_irq();
        return;
    }
    else
    {
        ulRTCCurCnt = configRTC_GET_COUNT();
        configRTC_SET_COMPARE((ulReloadValue + ulRTCCurCnt)&configRTC_CNT_MASK);

        ulRTCStoreCnt = ulRTCCurCnt;




        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if( xModifiableIdleTime > 0 )
        {

            if (GetSysEnergyModeFlag() == 0)
            {
                EMU_EnterEM2(true);
            }
            else
            {
                EMU_EnterEM1();//DMA Not Done

                ulRTCCurCnt = configRTC_GET_COUNT();
                if( ((ulRTCCurCnt - ulRTCStoreCnt)&configRTC_CNT_MASK ) <
                        ulReloadValue)
                {
                    if (GetSysEnergyModeFlag() == 0)  //EM1 wakeup but not clear(timer example)
                    {
                        EMU_EnterEM2(true);
                    }
                }
            }
        }
        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        ulRTCCurCnt = configRTC_GET_COUNT();
        if( ((ulRTCCurCnt - ulRTCStoreCnt)&configRTC_CNT_MASK ) >=
                ulReloadValue)
        {
            /* The tick interrupt has already executed, and the SysTick
            count reloaded with the portNVIC_SYSTICK_LOAD_REG value.
            Reset the portNVIC_SYSTICK_LOAD_REG with whatever remains of
            this tick period. */


            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods = (xExpectedIdleTime);
        }
        else
        {
            ulRTCTickPeriods  = (ulRTCCurCnt - ulRTCStoreCnt)&configRTC_CNT_MASK;
            ulCompleteTickPeriods = RTCTICK_TO_OSTICK(ulRTCTickPeriods);
        }

        configRTC_RESET_COUNT();
        configRTC_SET_COMPARE(ulTimerReloadValueForOneTick -
                              ulRTCCurCnt % ulTimerReloadValueForOneTick);
        RTC_IntClear(RTC_IFC_COMP0);
        NVIC_ClearPendingIRQ(RTC_IRQn);

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
        again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
        value. */
        if(ulCompleteTickPeriods >= 2){
          vTaskStepTick( ulCompleteTickPeriods - 1);
        }
        __enable_irq();

        ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
        {
          if(ulCompleteTickPeriods >= 1){
            vTaskIncrementTick();
          }
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );

#if configUSE_PREEMPTION == 1
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
#endif
    }
}
#endif


/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    /* If using preemption, also force a context switch. */
#if configUSE_PREEMPTION == 1
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
#endif

    /* Only reset the systick load register if configUSE_TICKLESS_IDLE is set to
    1.  If it is set to 0 tickless idle is not being used.  If it is set to a
    value other than 0 or 1 then a timer other than the SysTick is being used
    to generate the tick interrupt. */
#if configUSE_TICKLESS_IDLE == 1 || configUSE_TICKLESS_IDLE == 2
    portNVIC_SYSTICK_LOAD_REG = ulTimerReloadValueForOneTick;
#endif

    ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
    {
        vTaskIncrementTick();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__WEAK void vPortSetupTimerInterrupt( void )
{

    /* Calculate the constants required to configure the tick interrupt. */
#if (configUSE_TICKLESS_IDLE == 1)
    {
        ulTimerReloadValueForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
        xMaximumPossibleSuppressedTicks = 0xffffffUL / ( ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL );
        ulStoppedTimerCompensation = 45UL / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
    }
#endif /* configUSE_TICKLESS_IDLE */

#if (configUSE_TICKLESS_IDLE == 2)
    {
        ulTimerReloadValueForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
        xMaximumPossibleSuppressedTicks = 0xffffffUL / ( ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL );
        ulStoppedTimerCompensation = 0;
    }
#endif /* configUSE_TICKLESS_IDLE */
#if (configUSE_TICKLESS_IDLE == 3)
    {
        ulTimerReloadValueForOneTick = ( configRTCTICK_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = 0xffffffUL / ( ( configRTCTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL ) - 1UL;
        ulStoppedTimerCompensation = 0;
    }
    OS_RTC_TickInit();
    OS_RTC_TickSetup(ulTimerReloadValueForOneTick);
#elif(configUSE_TICKLESS_IDLE == 4)
    {
        ulTimerReloadValueForOneTick = ( configRTCTICK_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = 0xffffffUL / ( ( configRTCTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL ) - 1UL;
        ulStoppedTimerCompensation = 0;
    }
    OS_RTC_TickInit();
    OS_RTC_TickSetup(ulTimerReloadValueForOneTick);
#else
    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
#endif

#if (configUSE_TICKLESS_IDLE == 2)
    OS_RTC_TickInit();
#endif
}





/*-----------------------------------------------------------*/

#if (configUSE_TICKLESS_IDLE == 2)||(configUSE_TICKLESS_IDLE == 3)||(configUSE_TICKLESS_IDLE == 3)
/*
 * Setup RTC Timer
 */
__WEAK void OS_RTC_TickInit (void)
{
    RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;
    /* Configuring clocks in the Clock Management Unit (CMU) */
    /* Starting LFRCO and waiting until it is stable */
    //CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    /* Routing the LFXO clock to the RTC */
    //CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFRCO);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Enabling clock to the interface of the low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);


    RTC_Init(&rtcInit);
    RTC_IntEnable(RTC_IF_COMP0 | RTC_IF_COMP1);
    RTC_CompareSet(0, configRTCTICK_CLOCK_HZ);

    /* Enabling Interrupt from RTC */
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);

    RTC_Enable(false);
}

__WEAK void OS_RTC_TickSetup(uint32_t tick)
{
    /* Setting up RTC */
    RTC_CompareSet(1, tick);

    /* Enabling the RTC */
    RTC_Enable(true);

}

#endif

#if (configUSE_TICKLESS_IDLE == 4)
/*
 * Setup RTC Timer
 */
__WEAK void OS_RTC_TickInit (void)
{
    RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

    rtcInit.comp0Top = false;
    /* Configuring clocks in the Clock Management Unit (CMU) */
    /* Starting LFRCO and waiting until it is stable */
    //CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    /* Routing the LFXO clock to the RTC */
    //CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFRCO);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Enabling clock to the interface of the low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);


    RTC_Init(&rtcInit);
    RTC_IntEnable(RTC_IF_COMP0);

    /* Enabling Interrupt from RTC */
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);

    RTC_Enable(false);
}

__WEAK void OS_RTC_TickSetup(uint32_t tick)
{
    /* Setting up RTC */
    RTC_CompareSet(0, tick);

    /* Enabling the RTC */
    RTC_Enable(true);
}

#endif

#if (configUSE_TICKLESS_IDLE == 2)
void xPortRTCTickHandler(void)
{
    RTC_IntClear(RTC_IntGet());
}

#endif

#if (configUSE_TICKLESS_IDLE == 3)
void  xPortRTCTickHandler (void)
{
    uint32_t num;
    uint32_t count;

    RTC_IntClear(RTC_IF_COMP1);

    count = configRTC_GET_COUNT();

    num = RTCTICK_TO_OSTICK(count) + 1;
    count = OSTICK_TO_RTCTICK(num);

    configRTC_SET_COMPARE(count);


    /* If using preemption, also force a context switch. */
#if configUSE_PREEMPTION == 1
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
#endif

    ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
    {
        vTaskIncrementTick();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}
#endif
#if (configUSE_TICKLESS_IDLE == 4)
void  xPortRTCTickHandler (void)
{
    uint32_t ifc;

    ifc = RTC_IntGet();
    /* Clear interrupt source */
    if((ifc & RTC_IF_COMP0) != RTC_IF_COMP0)
    {
        RTC_IntClear(ifc);
        return;
    }
    RTC_IntClear(ifc);

    RTC_CounterReset();

    configRTC_SET_COMPARE(ulTimerReloadValueForOneTick);

    /* If using preemption, also force a context switch. */
#if configUSE_PREEMPTION == 1
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
#endif

    ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
    {
        vTaskIncrementTick();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}
#endif
