/*
 * uds_misc.h
 *
 *  Created on: 2023Äê1ÔÂ8ÈÕ
 *      Author: Ðì·É
 */

#ifndef CALTERAH_FREERTOS_UDS_UDS_MISC_H_
#define CALTERAH_FREERTOS_UDS_UDS_MISC_H_



typedef struct  {
	float *send_data;
	uint32_t total_length;
}uds_calib_data;

BaseType_t ant_calib_uds(float calib_ang_trans,float calib_rang_min_trans,float calib_rang_max_trans,uint32_t calib_cnt_trans,uint32_t rng_index_trans,uint32_t frame_period_trans);
BaseType_t uds_ant_calib(float calib_ang,float calib_rang_min,float calib_rang_max,uint32_t calib_cnt,uint32_t rng_index,uint32_t frame_period);

#endif /* CALTERAH_FREERTOS_UDS_UDS_MISC_H_ */
