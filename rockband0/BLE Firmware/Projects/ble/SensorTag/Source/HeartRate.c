#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "heartrateservice.h"
#include "peripheral.h"
#include "OnBoard.h"
#include "sensorTag.h"

static attHandleValueNoti_t stHeartRateMeas;
/*********************************************************************
 * @fn      heartRateCB
 *
 * @brief   Callback function for heart rate service.
 *
 * @param   event - service event
 *
 * @return  none
 */
extern gaprole_States_t gapProfileState ;
extern uint8 sensorTag_TaskID; 
extern uint16 gapConnHandle;
void heartRateCB(uint8 event)
{
  if (event == HEARTRATE_MEAS_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( sensorTag_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
    } 
  }
  else if (event == HEARTRATE_MEAS_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( sensorTag_TaskID, HEART_PERIODIC_EVT );
  }
  else if (event == HEARTRATE_COMMAND_SET)
  {
    // reset energy expended
    //heartRateEnergy = 0;
  }
}



/*********************************************************************
 * @fn      heartRateMeasNotify
 *
 * @brief   Prepare and send a heart rate measurement notification
 *
 * @return  none
 */
static uint8 heartRateBpm = 70;

void heartRateMeasNotify(void)
{
  uint8 *p = stHeartRateMeas.value;
  uint8 flags = 0;
  {
    /*
    static const uint8 heartRateFlags[FLAGS_IDX_MAX] =
{
  HEARTRATE_FLAGS_CONTACT_NOT_SUP,
  HEARTRATE_FLAGS_CONTACT_NOT_DET,
  HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP,
  HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_RR,
  HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP | HEARTRATE_FLAGS_RR,
  HEARTRATE_FLAGS_FORMAT_UINT16 | HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP | HEARTRATE_FLAGS_RR,
  0x00
};
    */
  }
  // build heart rate measurement structure from simulated values
  *p++ = flags;
  *p++ = heartRateBpm;
  if (flags & HEARTRATE_FLAGS_FORMAT_UINT16)
  {
    // additional byte for 16 bit format
    *p++ = 0;
  }
  if (flags & HEARTRATE_FLAGS_ENERGY_EXP)
  {
    *p++ = 0x55;
    *p++ = 0x66;
  }
  if (flags & HEARTRATE_FLAGS_RR)
  {
    *p++ = 0x11;
    *p++ = 0x22;  
    *p++ = 0x33;
    *p++ = 0x44;  
  }
  stHeartRateMeas.len = (uint8) (p - stHeartRateMeas.value);
  HeartRate_MeasNotify( gapConnHandle, &stHeartRateMeas );
  heartRateBpm++;
  if(heartRateBpm==80)
    heartRateBpm=70;
  }



/*********************************************************************
 * @fn      heartRatePeriodicTask
 *
 * @brief   Perform a periodic heart rate application task.
 *
 * @param   none
 *
 * @return  none
 */
void heartRatePeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // send heart rate measurement notification
    heartRateMeasNotify();
    
    // Restart timer
    osal_start_timerEx( sensorTag_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
  }
}