#ifndef __HEART_RATE_H
#define __HEART_RATE_H
#define  DEFAULT_HEARTRATE_PERIOD             1000
void heartRateCB(uint8 event);
void heartRatePeriodicTask( void );
#endif