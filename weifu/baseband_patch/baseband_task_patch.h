/*
 * baseband_task_patch.h
 *
 *  Created on: 2019Äê9ÔÂ20ÈÕ
 *      Author: xerxes
 */

#ifndef _BASEBAND_TASK_PATCH_H_
#define _BASEBAND_TASK_PATCH_H_

#include "embARC_toolchain.h"
#include "clkgen.h"
#include "sensor_config.h"
#include "baseband_task.h"
#include "baseband.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "arc_exception.h"
#include "alps_hardware.h"
#include "embARC_debug.h"



/*--- INCLUDE ------------------------*/
#include <math.h>
#include <string.h>
#include "calterah_math.h"
#include "track.h"
#include "track_cli.h"
#include "ekf_track.h"
#include "baseband.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "sensor_config.h"
#include "console.h"
#include "can_signal_interface.h"
#include "track_cli.h"


#define BB_READ_REG(bb_hw, RN) baseband_read_reg(bb_hw, BB_REG_##RN)
#define MAX_Q_IDX 15
#define MAX_Q_NUM 32




int fft1d_get_with_para(baseband_t *bb, int rng_index, int chrip_index,int ch_index, complex_t *fft1d);
void rng_of_position_calc(baseband_t *bb, float rng, float *rng_pos);
void position_of_rng_calc(baseband_t *bb, float *rng, float rng_pos);
int fft1d_peak_get_with_para(baseband_t *bb, int chirp_index, int ch_index, float rng_start, float rng_end,
    complex_t *fft1d_peak, float *fft1d_peak_rng);
extern int fft1d_get_with_position(baseband_t *bb, int chirp_index, int ch_index, int fft_position_start,
    int fft_position_end, complex_t *fft1d_data);

int doppler_data_print(baseband_t *bb,uint32_t fft_flag);
extern uint32_t  baseband_read_mem_table(baseband_hw_t *bb_hw, uint32_t offset);
extern void doa_hw_read(track_t *track);

#endif /* _BASEBAND_TASK_PATCH_H_ */
