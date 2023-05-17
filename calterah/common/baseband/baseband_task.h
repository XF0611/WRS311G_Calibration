#ifndef BASEBAND_TASK_H
#define BASEBAND_TASK_H


#include "rtwtypes.h"
#define BB_ISR_QUEUE_LENGTH 8
#define BB_1P4_QUEUE_LENGTH 1
#define BB_DOA_QUEUE_LENGTH 1
#define WDT_FEED_QUEUE_LENGTH 1
#define FFT1D_FINISH_QUEUE_LENGTH 1


extern creal32_T fft1d_data[50];
void baseband_task(void *parameters);
void bb_clk_switch();
void bb_clk_restore();
void initial_flag_set(bool data);

#endif
