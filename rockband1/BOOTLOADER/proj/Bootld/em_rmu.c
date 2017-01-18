/***************************************************************************/
 #include "em_rmu.h"
 #if defined(RMU_COUNT) && (RMU_COUNT > 0)
 
 #include "em_emu.h"
 #include "em_bitband.h"
 
 /***************************************************************************/
 /***************************************************************************/
 /*******************************************************************************
  **************************   GLOBAL FUNCTIONS   *******************************
  ******************************************************************************/
 
 /***************************************************************************/
 void RMU_ResetControl(RMU_Reset_TypeDef reset, bool enable)
 {
   BITBAND_Peripheral(&(RMU->CTRL), (uint32_t)reset, (uint32_t)enable);
 }
 
 
 /***************************************************************************/
 void RMU_ResetCauseClear(void)
 {
   uint32_t locked;
 
   RMU->CMD = RMU_CMD_RCCLR;
 
   /* Clear some reset causes not cleared with RMU CMD register */
   /* (If EMU registers locked, they must be unlocked first) */
   locked = EMU->LOCK & EMU_LOCK_LOCKKEY_LOCKED;
   if (locked)
   {
     EMU_Unlock();
   }
 
   BITBAND_Peripheral(&(EMU->AUXCTRL), 0, 1);
   BITBAND_Peripheral(&(EMU->AUXCTRL), 0, 0);

   if (locked)
   {
     EMU_Lock();
   }
 }

 /***************************************************************************/
 uint32_t RMU_ResetCauseGet(void)
 {
   uint32_t ret = RMU->RSTCAUSE;
 
   /* Inspect and decode bits. The decoding must be done in correct order, */
   /* since some reset causes may trigger other reset causes due to internal */
   /* design. We are only interested in the main cause. */
 #if defined( RMU_RSTCAUSE_EM4RST )
   /* Clear "stray" bits if EM4 bit is set, they will always be active */
   if (ret & RMU_RSTCAUSE_EM4RST)
     ret &= ~(RMU_RSTCAUSE_BODREGRST|
              RMU_RSTCAUSE_BODUNREGRST|
              RMU_RSTCAUSE_LOCKUPRST|
              RMU_RSTCAUSE_SYSREQRST);
   }
 #endif
   if (ret & RMU_RSTCAUSE_PORST)
   {
     ret = RMU_RSTCAUSE_PORST;
   }
   else if ((ret & 0x83) == RMU_RSTCAUSE_BODUNREGRST)
   {
     ret = RMU_RSTCAUSE_BODUNREGRST;
   }
   else if ((ret & 0x1f) == RMU_RSTCAUSE_BODREGRST)
   {
     ret = RMU_RSTCAUSE_BODREGRST;
   }
   /* Both external and watchdog reset may occur at the same time */
   else if ((ret & 0x1b) & (RMU_RSTCAUSE_EXTRST | RMU_RSTCAUSE_WDOGRST))
   {
     ret &= RMU_RSTCAUSE_EXTRST | RMU_RSTCAUSE_WDOGRST;
   }
   /* Both lockup and system reset may occur at the same time */
   else if ((ret & 0x7ff) & (RMU_RSTCAUSE_LOCKUPRST | RMU_RSTCAUSE_SYSREQRST))
   {
     ret &= RMU_RSTCAUSE_LOCKUPRST | RMU_RSTCAUSE_SYSREQRST;
   }
 #if defined( RMU_RSTCAUSE_BODAVDD0 )
   /* EM4 wake up and pin retention support */
   else if (ret & RMU_RSTCAUSE_BODAVDD0)
   {
     ret = RMU_RSTCAUSE_BODAVDD0;
   }
   else if (ret & RMU_RSTCAUSE_BODAVDD1)
   {
     ret = RMU_RSTCAUSE_BODAVDD1;
   }
   else if (ret & (RMU_RSTCAUSE_EM4WURST|RMU_RSTCAUSE_EM4RST))
   {
     ret &= (RMU_RSTCAUSE_EM4WURST|
 #if defined( RMU_RSTCAUSE_BUMODERST )
             RMU_RSTCAUSE_BUMODERST|
 #endif
             RMU_RSTCAUSE_EM4RST);
   }
   else if (ret & (RMU_RSTCAUSE_EM4RST|RMU_RSTCAUSE_EXTRST))
   {
     ret &= (RMU_RSTCAUSE_EM4RST|RMU_RSTCAUSE_EXTRST);
   }
 #endif
 #if defined( RMU_RSTCAUSE_BUBODVDDDREG )
   /* Backup power domain support */
   else if (ret & (RMU_RSTCAUSE_BUBODVDDDREG))
   {
     /* Keep backup mode flag, will only be present in this scenario */
     ret &= (RMU_RSTCAUSE_BUBODVDDDREG|RMU_RSTCAUSE_BUMODERST);
   }
   else if (ret & (RMU_RSTCAUSE_BUBODBUVIN))
   {
     ret &= (RMU_RSTCAUSE_BUBODBUVIN);
   }
   else if (ret & (RMU_RSTCAUSE_BUBODUNREG))
   {
     ret &= (RMU_RSTCAUSE_BUBODUNREG);
   }
   else if (ret & (RMU_RSTCAUSE_BUBODREG))
  {
     ret &= (RMU_RSTCAUSE_BUBODREG);
   }
 #endif
   else
   {
    ret = 0;
   }
   return ret;
 }
 
 
 #endif /* defined(RMU_COUNT) && (RMU_COUNT > 0) */