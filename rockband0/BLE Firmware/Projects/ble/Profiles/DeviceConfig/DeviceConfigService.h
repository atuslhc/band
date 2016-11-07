#ifndef IRTEMPSERVICE_H
#define IRTEMPSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define DRIVER_CONFIG_DATA              0       // R  uint8 - Profile Attribute value
#define DRIVER_CONFIG_CONF              1       // RW uint8 - Profile Attribute value

// Service UUID
#define DRIVER_CONFIG_SERV_UUID         0xCC00  // F0000000-0451-4000-B000-00000000-AA00
#define DRIVER_CONFIG_NOTIF_UUID        0xCC01  
#define DRIVER_CONFIG_DATE_TIME_UUID    0xCC02  
#define DRIVER_CONFIG_ALARM_UUID        0xCC03  
#define DRIVER_CONFIG_USR_PROFILE_UUID  0xCC04  
  
  
  
#ifdef __cplusplus
}
#endif

#endif /* IRTEMPSERVICE_H */


