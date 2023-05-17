/*
 * uds_misc.c
 *
 *  Created on: 2023Äê1ÔÂ8ÈÕ
 *      Author: Ðì·É
 */

#include "embARC_toolchain.h"
#include "embARC_assert.h"
#include "embARC.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "FreeRTOS.h"
#include "sensor_config.h"
#include "baseband_reg.h"
#include "baseband_hw.h"
#include "baseband.h"
#include "baseband_alps_FM_reg.h"
#include "uds_misc.h"



static volatile bool frame_ready = true;
static xTimerHandle xTimerFrame = NULL;
uint32_t times_cnt;
float test[10] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
float calib_data[4097] = {0};
float real_range_data;
void vTimerFrameCallback1(xTimerHandle xTimer)
{
	frame_ready = true;
}

BaseType_t ant_calib_uds(float calib_ang_trans,float calib_rang_min_trans,float calib_rang_max_trans,uint32_t calib_cnt_trans,uint32_t rng_index_trans,uint32_t frame_period_trans)
{
        baesband_frame_interleave_cnt_clr(); // clear recfg count in frame interleaving
#if DDM_EN
        baseband_t *bb = baseband_get_rtl_frame_type();
#else
        /* reconfigure the frame pattern for frame-interleaving */
        baseband_t *bb = baseband_frame_interleave_recfg();
#endif
        sensor_config_t *cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;


        int calib_ang = 0;                      // calib angle
        float calib_rang_min = 0.5;             // minimum target distance, unit: meter
        float calib_rang_max = 50.0;            // maximum target distance, unit: meter
        int calib_num = 10, calib_cnt = 0;      // calib times
        uint32_t rng_index = 0;                 // target 2D-FFT range index
        uint16_t frame_period = 50;             // Frame Repetition Period in ms

        times_cnt = 0;
        memset(calib_data,0,sizeof(calib_data));


        bool timer_en = false;
        uint8_t bpm_idx_min = 0;
        uint8_t bpm_idx_max = cfg->nvarray - 1;

        if (cfg->anti_velamb_en) {
                bpm_idx_min = 1;
                bpm_idx_max = cfg->nvarray;
        }

        /* get paremeter */

		calib_ang = (int)calib_ang_trans;
		if (calib_ang > 90 || calib_ang < -90)
				calib_ang = 0;


		calib_rang_min = calib_rang_min_trans;
		if (calib_rang_min < 2*bb->sys_params.rng_delta)
				calib_rang_min = 0.5;


		calib_rang_max = calib_rang_max_trans;
		if (calib_rang_max < 2*bb->sys_params.rng_delta)
				calib_rang_max = 50.0;


		calib_num = calib_cnt_trans;
		if (calib_num < 1)
				calib_num = 10;


		rng_index = rng_index_trans;
		if (rng_index > 200 || calib_num < 5)
				rng_index = 0;


		if(frame_period_trans)
		{
			timer_en = true;
			frame_period = frame_period_trans;
			if(frame_period < 10 || frame_period > 5000)
					frame_period = 50;
		}


        // calib_cnt = calib_num / cfg->nvarray;

        calib_cnt = calib_num;

        /* Initialize Timer */
        if(timer_en){
                if(xTimerFrame == NULL){
                        /* if timer has not been created before, create timer */
                        xTimerFrame = xTimerCreate("FrameTimer", pdMS_TO_TICKS(frame_period), pdTRUE, (void *)1, vTimerFrameCallback1);
                        if(xTimerFrame == NULL){
                                EMBARC_PRINTF("Timer Initialization failed.\r\n");
                        }
                }else{
                        /* if timer has already been created, update the frame repeatition period */
                        xTimerChangePeriod(xTimerFrame, pdMS_TO_TICKS(frame_period), 0);
                }
        }

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER)
                baseband_write_cascade_cmd(pcCommandString); /* spi tx CommandString to slave*/
#endif
        /* turn off zero doppler removel */
        bool old_zer_dpl = BB_READ_REG(bb_hw, FFT_ZER_DPL_ENB);
        BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB  , 0);

        dmu_adc_reset(); // ADC should reset in cascade

        while(1){
                if((frame_ready) && (calib_cnt > 0)){
                        if(timer_en){
                                frame_ready = false;
                                if( xTimerStart( xTimerFrame, 0 ) != pdPASS ){
                                        EMBARC_PRINTF("Timer not started.\r\n");
                                        break;
                                }
                        }

#if DDM_EN
                        baseband_data_proc_t* dpc = baseband_get_dpc();
                        /* Clear event bit before bb start */
                        uint32_t event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
                        if( event_bits != E_OK)
                        {
                                EMBARC_PRINTF("Event bit set to %d before start @%s", event_bits, __func__);
                        }

                        int index = 0;
                        while (!baseband_data_proc_run(&dpc[index++]))
                                ;
                        BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);
                        // update bb, cfg, bb_hw here, or 'bpm_idx_max' and baseband_hw_get_fft_mem(bb_hw, ...)
                        // in following lines will generate error results
                        bb = baseband_get_rtl_frame_type();
                        cfg = (sensor_config_t*)(bb->cfg);
                        bb_hw = &bb->bb_hw;
                        bpm_idx_min = 0;
                        bpm_idx_max = cfg->nvarray - 1;

#else
                        /* start baseband */
                        baseband_start_with_params(bb, true, true,
                                                (   (SYS_ENABLE_SAM_MASK << SYS_ENABLE_SAM_SHIFT)
                                                | (SYS_ENABLE_FFT_2D_MASK << SYS_ENABLE_FFT_2D_SHIFT)
                                                | (SYS_ENABLE_CFR_MASK << SYS_ENABLE_CFR_SHIFT)
                                                | (SYS_ENABLE_BFM_MASK << SYS_ENABLE_BFM_SHIFT)),
                                                true, BB_IRQ_ENABLE_SAM_DONE, false);
                        /* wait done */
                        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);

#endif
                        // Search test target peak in 2D-FFT plane
                        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
                        /* search peak */
                        if ( rng_index==0 ) {
                                float peak_power = 0;
                                complex_t complex_fft;
                                uint32_t fft_mem;
                                int bpm_index;
                                int ch_index;
                                int tmp_rng_ind  = (int)(calib_rang_min/bb->sys_params.rng_delta);
                                int rng_ind_end  = (int)(calib_rang_max/bb->sys_params.rng_delta);
                                for ( ; tmp_rng_ind <= rng_ind_end; tmp_rng_ind++) {
                                        float tmp_power = 0;
                                        for (bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                                for (ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                        fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, tmp_rng_ind, 0, bpm_index);
                                                        complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
                                                        tmp_power += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 64; // Non-coherent accumulation of 4 channel 2D-FFT value
                                                }
                                        }
                                        if (peak_power < tmp_power) {        // find the peak
                                                peak_power = tmp_power;
                                                rng_index = tmp_rng_ind;     // peak index
                                        }
                                }
                        }

                        /* output phase */
                        int ch_index;
                        int bpm_index;
                        uint32_t fft_mem;
                        complex_t complex_fft;
                        float angle;
                        float power;
                        baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
#ifdef CHIP_CASCADE
                        if (chip_cascade_status() == CHIP_CASCADE_SLAVE) {
                                cascade_write_buf_req();
                                cascade_write_buf(rng_index); // write rng_index to spi
                                for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                        for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, 0, bpm_index);
                                                cascade_write_buf(fft_mem); // write FFT data to spi
                                        }
                                }
                                cascade_write_buf_done();
                        }

                        uint32_t slv_fft_buf[MAX_NUM_RX*MAX_NUM_RX];
                        uint32_t rx_buf_offset = 0;
                        uint32_t slv_rng_index = 0;
                        if (chip_cascade_status() == CHIP_CASCADE_MASTER) { // read slave FFT data
                                if (E_OK == cascade_read_buf_req(WAIT_TICK_NUM)) { /* wait 30ms */
                                        slv_rng_index = cascade_read_buf(rx_buf_offset++); // read rng_index from spi
                                        rng_index = slv_rng_index;  // use slave peak range index for master FFT access
                                        for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                                for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                        // now rx_buf_offset == 1
                                                        *(slv_fft_buf + rx_buf_offset-1) = cascade_read_buf(rx_buf_offset); // read fft data from spi, store first then print
                                                        rx_buf_offset++;
                                                }
                                        }
                                        cascade_read_buf_done(); /* release rx buffer */
                                        /*
                                        rx_buf_offset = 0;
                                        for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                                for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                        uint32_t fft2d_data = *(slv_fft_buf + rx_buf_offset++);
                                                        complex_t complex_fft = cfl_to_complex(fft2d_data, 14, 14, true, 4, false);
                                                        float angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                                                        float power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;
                                                        EMBARC_PRINTF("%7.2f  %7.2f  ", angle, power);
                                                }
                                        }
                                        EMBARC_PRINTF("  rng_index %d  range %.2f slave\n", slv_rng_index, slv_rng_index*bb->sys_params.rng_delta);
                                        */
                                } else {
                                        EMBARC_PRINTF("!!! merge timeout ~~~\n"); // To be modified for python identification
                                } /* endif E_OK */
                        }
#endif
                        for (bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++){
                                for (ch_index = 0; ch_index < MAX_NUM_RX; ch_index++){
                                        fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, 0, bpm_index);
                                        complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
                                        angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                                        power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;

                                        calib_data[times_cnt++] = angle;
                                        calib_data[times_cnt++] = power;
                                        //EMBARC_PRINTF("%7.2f  %7.2f  ", angle, power);
                                }
                        }
                        //EMBARC_PRINTF("  rng_index %d  range %.2f master\n", rng_index, rng_index*bb->sys_params.rng_delta);
                        baseband_switch_mem_access(bb_hw, old);

#ifdef CHIP_CASCADE

                        rx_buf_offset = 0;
                        if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
                                for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                        for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                uint32_t fft2d_data = *(slv_fft_buf + rx_buf_offset++);
                                                complex_t complex_fft = cfl_to_complex(fft2d_data, 14, 14, true, 4, false);
                                                float angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                                                float power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;
                                                EMBARC_PRINTF("%7.2f  %7.2f  ", angle, power);
                                        }
                                }
                                EMBARC_PRINTF("  rng_index %d  range %.2f slave\n", slv_rng_index, slv_rng_index*bb->sys_params.rng_delta);
                        }

#endif
                        calib_cnt--;
                }else if (calib_cnt == 0){
                        break;
                }else{
                        taskYIELD();
                }
        }

#if ENA_PRINT_ANT_CALIB
        /*print the result of last calib to check the memory read is correct*/
        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_RLT);
        volatile obj_info_t *obj_info = (volatile obj_info_t *)BB_MEM_BASEADDR;
        int obj_num = BB_READ_REG(bb_hw, CFR_NUMB_OBJ); ;
#ifdef CHIP_CASCADE
        obj_num = BB_READ_REG(bb_hw, DOA_NUMB_OBJ);
#endif

        for (int i = 0; i < obj_num; i++)
                EMBARC_PRINTF("obj_ind = %d rng = %d vel = %d  noi = %d sig_0 = %d\n", i, obj_info[i].rng_idx, obj_info[i].vel_idx,  obj_info[i].noi, obj_info[i].doa[0].sig_0);

        baseband_switch_mem_access(bb_hw, old);
#endif

        if(timer_en){
                xTimerStop(xTimerFrame, 0);
                frame_ready = true;
        }

#if(0) // set 1 to print temperature
        float temperature = 0;

        /* clean write buffer */
        memset( pcWriteBuffer, 0x00, xWriteBufferLen );

        temperature = fmcw_radio_get_temperature(NULL);

        /* display info */
        sprintf(pcWriteBuffer, "\r\n radio temperature: %.2f \r\n", temperature);
#endif
        /* restore zero doppler removel */
        BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB, old_zer_dpl);

        return pdFALSE;
}

BaseType_t uds_ant_calib(float calib_ang,float calib_rang_min,float calib_rang_max,uint32_t calib_cnt,uint32_t rng_index,uint32_t frame_period)
{
        baesband_frame_interleave_cnt_clr(); // clear recfg count in frame interleaving

        /* reconfigure the frame pattern for frame-interleaving */
        baseband_t *bb = baseband_frame_interleave_recfg();

        sensor_config_t *cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;

//        EMBARC_PRINTF("calib_ang is %f\r\n",calib_ang);
//        EMBARC_PRINTF("calib_rang_min is %f\r\n",calib_rang_min);
//        EMBARC_PRINTF("calib_rang_max is %f\r\n",calib_rang_max);
//        EMBARC_PRINTF("calib_cnt is %d\r\n",calib_cnt);
//        EMBARC_PRINTF("frame_period is %d\r\n",frame_period);
        //EMBARC_PRINTF("rng_index is %d\r\n",rng_index);
        //EMBARC_PRINTF("rng delta is %f\r\n",bb->sys_params.rng_delta);
        real_range_data = rng_index*bb->sys_params.rng_delta;
        EMBARC_PRINTF("test rng delta is %f\r\n",real_range_data);
        EMBARC_PRINTF("  rng_index %d  range %.2f master\n", rng_index, rng_index*bb->sys_params.rng_delta);

        times_cnt = 0;
        memset(calib_data,0,sizeof(calib_data));

        bool timer_en = false;
        uint8_t bpm_idx_min = 0;
        uint8_t bpm_idx_max = cfg->nvarray - 1;
        //EMBARC_PRINTF("cfg->nvarray is %d\r\n",cfg->nvarray);
        if (cfg->anti_velamb_en) {
                bpm_idx_min = 1;
                bpm_idx_max = cfg->nvarray;
        }

        /* get paremeter */

		if (calib_ang > 90.0 || calib_ang < -90.0)
				calib_ang = 0.0;

		if (calib_rang_min < 2*bb->sys_params.rng_delta)
				calib_rang_min = 0.5;


		if (calib_rang_max < 2*bb->sys_params.rng_delta)
				calib_rang_max = 50.0;


		if (calib_cnt < 1)
				calib_cnt = 10;


		if (rng_index > 200 || calib_cnt < 5)
				rng_index = 0;


		if(frame_period)
		{
			timer_en = true;
			if(frame_period < 10 || frame_period > 5000)
					frame_period = 50;
		}


		//EMBARC_PRINTF("timer_en is %d\r\n",timer_en);
        /* Initialize Timer */
        if(timer_en){
                if(xTimerFrame == NULL){
                        /* if timer has not been created before, create timer */
                        xTimerFrame = xTimerCreate("FrameTimer", pdMS_TO_TICKS(frame_period), pdTRUE, (void *)1, vTimerFrameCallback1);
                        if(xTimerFrame == NULL){
                                EMBARC_PRINTF("Timer Initialization failed.\r\n");
                        }
                }else{
                        /* if timer has already been created, update the frame repeatition period */
                        xTimerChangePeriod(xTimerFrame, pdMS_TO_TICKS(frame_period), 0);
                }
        }

        /* turn off zero doppler removel */
        bool old_zer_dpl = BB_READ_REG(bb_hw, FFT_ZER_DPL_ENB);
        BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB  , 0);

        dmu_adc_reset(); // ADC should reset in cascade



        while(1){

            if((frame_ready) && (calib_cnt > 0)){
                    if(timer_en){
                            frame_ready = false;
                            if( xTimerStart( xTimerFrame, 0 ) != pdPASS ){
                                    EMBARC_PRINTF("Timer not started.\r\n");
                                    break;
                            }
                    }
#if DDM_EN
                        baseband_data_proc_t* dpc = baseband_get_dpc();
                        /* Clear event bit before bb start */
                        uint32_t event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
                        if( event_bits != E_OK)
                        {
                                EMBARC_PRINTF("Event bit set to %d before start @%s", event_bits, __func__);
                        }

                        int index = 0;
                        while (!baseband_data_proc_run(&dpc[index++]))
                                ;
                        BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);
                        // update bb, cfg, bb_hw here, or 'bpm_idx_max' and baseband_hw_get_fft_mem(bb_hw, ...)
                        // in following lines will generate error results
                        bb = baseband_get_rtl_frame_type();
                        cfg = (sensor_config_t*)(bb->cfg);
                        bb_hw = &bb->bb_hw;
                        bpm_idx_min = 0;
                        bpm_idx_max = cfg->nvarray - 1;

#else
                        /* start baseband */
                        baseband_start_with_params(bb, true, true,
                                                (   (SYS_ENABLE_SAM_MASK << SYS_ENABLE_SAM_SHIFT)
                                                | (SYS_ENABLE_FFT_2D_MASK << SYS_ENABLE_FFT_2D_SHIFT)
                                                | (SYS_ENABLE_CFR_MASK << SYS_ENABLE_CFR_SHIFT)
                                                | (SYS_ENABLE_BFM_MASK << SYS_ENABLE_BFM_SHIFT)),
                                                true, BB_IRQ_ENABLE_SAM_DONE, false);
                        /* wait done */
                        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);
#endif

                        // Search test target peak in 2D-FFT plane
                        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
                        /* search peak */
                        if ( rng_index==0 ) {
                                float peak_power = 0;
                                complex_t complex_fft;
                                uint32_t fft_mem;
                                int bpm_index;
                                int ch_index;
                                int tmp_rng_ind  = (int)(calib_rang_min/bb->sys_params.rng_delta);
                                int rng_ind_end  = (int)(calib_rang_max/bb->sys_params.rng_delta);
                                for ( ; tmp_rng_ind <= rng_ind_end; tmp_rng_ind++) {
                                        float tmp_power = 0;
                                        for (bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                                for (ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                        fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, tmp_rng_ind, 0, bpm_index);
                                                        complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
                                                        tmp_power += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 64; // Non-coherent accumulation of 4 channel 2D-FFT value
                                                }
                                        }
                                        if (peak_power < tmp_power) {        // find the peak
                                                peak_power = tmp_power;
                                                rng_index = tmp_rng_ind;     // peak index
                                        }
                                }
                        }

                        /* output phase */
                        int ch_index;
                        int bpm_index;
                        uint32_t fft_mem;
                        complex_t complex_fft;
                        float angle;
                        float power;
                        baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
#ifdef CHIP_CASCADE
                        if (chip_cascade_status() == CHIP_CASCADE_SLAVE) {
                                cascade_write_buf_req();
                                cascade_write_buf(rng_index); // write rng_index to spi
                                for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                        for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, 0, bpm_index);
                                                cascade_write_buf(fft_mem); // write FFT data to spi
                                        }
                                }
                                cascade_write_buf_done();
                        }

                        uint32_t slv_fft_buf[MAX_NUM_RX*MAX_NUM_RX];
                        uint32_t rx_buf_offset = 0;
                        uint32_t slv_rng_index = 0;
                        if (chip_cascade_status() == CHIP_CASCADE_MASTER) { // read slave FFT data
                                if (E_OK == cascade_read_buf_req(WAIT_TICK_NUM)) { /* wait 30ms */
                                        slv_rng_index = cascade_read_buf(rx_buf_offset++); // read rng_index from spi
                                        rng_index = slv_rng_index;  // use slave peak range index for master FFT access
                                        for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                                for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                        // now rx_buf_offset == 1
                                                        *(slv_fft_buf + rx_buf_offset-1) = cascade_read_buf(rx_buf_offset); // read fft data from spi, store first then print
                                                        rx_buf_offset++;
                                                }
                                        }
                                        cascade_read_buf_done(); /* release rx buffer */
                                        /*
                                        rx_buf_offset = 0;
                                        for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                                for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                        uint32_t fft2d_data = *(slv_fft_buf + rx_buf_offset++);
                                                        complex_t complex_fft = cfl_to_complex(fft2d_data, 14, 14, true, 4, false);
                                                        float angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                                                        float power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;
                                                        EMBARC_PRINTF("%7.2f  %7.2f  ", angle, power);
                                                }
                                        }
                                        EMBARC_PRINTF("  rng_index %d  range %.2f slave\n", slv_rng_index, slv_rng_index*bb->sys_params.rng_delta);
                                        */
                                } else {
                                        EMBARC_PRINTF("!!! merge timeout ~~~\n"); // To be modified for python identification
                                } /* endif E_OK */
                        }
#endif

                        for (bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++){
                                for (ch_index = 0; ch_index < MAX_NUM_RX; ch_index++){
                                        fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, 0, bpm_index);
                                        complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
                                        angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                                        power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;


                                        calib_data[times_cnt++] = angle;
                                        calib_data[times_cnt++] = power;
                                        //EMBARC_PRINTF("%7.2f  %7.2f  ", angle, power);
                                }
                        }


                        //calib_data[times_cnt++] = rng_index*bb->sys_params.rng_delta;


                        baseband_switch_mem_access(bb_hw, old);

#ifdef CHIP_CASCADE

                        rx_buf_offset = 0;
                        if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
                                for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                        for (int ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                uint32_t fft2d_data = *(slv_fft_buf + rx_buf_offset++);
                                                complex_t complex_fft = cfl_to_complex(fft2d_data, 14, 14, true, 4, false);
                                                float angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                                                float power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;
                                                EMBARC_PRINTF("%7.2f  %7.2f  ", angle, power);
                                        }
                                }
                                EMBARC_PRINTF("  rng_index %d  range %.2f slave\n", slv_rng_index, slv_rng_index*bb->sys_params.rng_delta);
                        }

#endif
                    calib_cnt--;
            }else if(calib_cnt == 0)
            {

            	EMBARC_PRINTF("times_cnt is %d\r\n",times_cnt);
            	//times_cnt = 0;
            	break;
            }else
            {
            	taskYIELD();
            }

        }

#if ENA_PRINT_ANT_CALIB
		/*print the result of last calib to check the memory read is correct*/
		uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_RLT);
		volatile obj_info_t *obj_info = (volatile obj_info_t *)BB_MEM_BASEADDR;
		int obj_num = BB_READ_REG(bb_hw, CFR_NUMB_OBJ); ;
	#ifdef CHIP_CASCADE
		obj_num = BB_READ_REG(bb_hw, DOA_NUMB_OBJ);
	#endif

		for (int i = 0; i < obj_num; i++)
				EMBARC_PRINTF("obj_ind = %d rng = %d vel = %d  noi = %d sig_0 = %d\n", i, obj_info[i].rng_idx, obj_info[i].vel_idx,  obj_info[i].noi, obj_info[i].doa[0].sig_0);

		baseband_switch_mem_access(bb_hw, old);
#endif

		if(timer_en){
                xTimerStop(xTimerFrame, 0);
                frame_ready = true;
        }

        /* restore zero doppler removel */
        BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB, old_zer_dpl);

        return pdFALSE;
}
