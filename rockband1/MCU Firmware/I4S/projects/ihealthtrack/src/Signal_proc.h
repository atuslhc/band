

#ifndef _Signal_proc_H_
#define _Signal_proc_H_
#include "main.h"

void PPG_INIT(void);
void PPG_PROCESSING(void);
int32_t DSP_PROCESSING(int32_t s_ppg, int32_t s_amb);

uint32_t XYZFilter_TRACK(void);
void XYZ_FIR_INIT(uint8_t freq);
#endif

