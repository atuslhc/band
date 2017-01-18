 /***************************************************************************/
 #ifndef __EM_RMU_H
 #define __EM_RMU_H
 
 #include "em_device.h"
 #if defined(RMU_COUNT) && (RMU_COUNT > 0)
 
 #include <stdbool.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /***************************************************************************/
 /***************************************************************************/
 /*******************************************************************************
  ********************************   ENUMS   ************************************
  ******************************************************************************/
 
 typedef enum
 {
 #if defined( RMU_CTRL_BURSTEN )
 
   rmuResetBU = _RMU_CTRL_BURSTEN_SHIFT,
 #endif
 
   rmuResetLockUp = _RMU_CTRL_LOCKUPRDIS_SHIFT
 } RMU_Reset_TypeDef;
 
 /*******************************************************************************
  *****************************   PROTOTYPES   **********************************
  ******************************************************************************/
 
 #define RMU_LockupResetDisable(A) RMU_ResetControl(rmuResetLockUp, A)
 
 void RMU_ResetControl(RMU_Reset_TypeDef reset, bool enable);
 void RMU_ResetCauseClear(void);
 uint32_t RMU_ResetCauseGet(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* defined(RMU_COUNT) && (RMU_COUNT > 0) */
 #endif /* __EM_RMU_H */