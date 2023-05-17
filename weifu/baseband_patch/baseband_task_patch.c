/*
 * baseband_task_patch.c
 *
 *  Created on: 2019��9��20��
 *      Author: xerxes
 */

#include "baseband_task_patch.h"
#include "baseband_cli.h"
extern bb_dump_data_t bb_dump_data_info;
/*
 * description: get 1d fft data by range index, velocity and channel index.
 * @bb: baseband handle.
 * @rng_index: range index.
 * @chrip_index: chrip index.
 * @ch_index: channel index.
 * @fft1d: get one 1d fft data.
 * @author: xerxes
 */
#define doppler_checkparams_ok 0
#define doppler_checkparams_not_ok -1
int fft1d_get_with_para(baseband_t *bb, int rng_index, int chrip_index,int ch_index, complex_t *fft1d)
{
    baseband_hw_t *bb_hw = &bb->bb_hw;
    uint32_t off_set_r = 0;
    uint32_t fft_mem = 0;
    complex_t complex_fft;

    if(rng_index < 0 || rng_index > bb->sys_params.rng_nfft)
    {
        EMBARC_PRINTF("rng_index=%d error! should less than %d\r\n", rng_index, bb->sys_params.rng_nfft);
        return -1;
    }
    if(chrip_index < 0 || chrip_index > bb->sys_params.vel_nfft)
    {
        EMBARC_PRINTF("vel_index=%d error! should less than %d\r\n", chrip_index, bb->sys_params.vel_nfft);
        return -1;
    }
    if(ch_index < 0 || ch_index > 3)
    {
        EMBARC_PRINTF("ch_index=%d error! should [0,3]\r\n", ch_index);
        return -1;
    }

    off_set_r = chrip_index * MAX_NUM_RX * (bb->sys_params.rng_nfft >> 1) + ch_index * (bb->sys_params.rng_nfft >> 1) + rng_index;
    fft_mem = baseband_read_mem_table(bb_hw, off_set_r);

    complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
    *fft1d = complex_fft;

    return 0;
}

/*
 * description: calculate the range position of range(m).
 * @bb: baseband handle.
 * @rng: range meters.
 * @rng_pos: get range index in 1d fft position
 * @author: xerxes
 */
void rng_of_position_calc(baseband_t *bb, float rng, float *rng_pos)
{
    const float vol = 299792458.0;
    float aim_freq;
    float fs;//sample rate
    float tof;//time of fly
    float slope;
    float n;
    float p;

    fs = bb->sys_params.Fs * 1.0e6;//Hz
    slope = bb->sys_params.bandwidth / bb->sys_params.chirp_rampup * 1.0e12;//Hz/s
    n = bb->sys_params.rng_nfft;
    /*
    aim_freq = (p-1)*fs/n;
    tof = aim_freq/slope;
    r = tof * vol / 2;
    */
    tof = 2 * rng / vol;
    aim_freq = tof * slope;
    p = aim_freq * n / fs + 1;
    *rng_pos = p;
}


/*
 * description: calculate range(m) of the range position.
 * @bb: baseband handle.
 * @rng: get range meters.
 * @rng_pos: range index in 1d fft position
 * @author: xerxes
 */
void position_of_rng_calc(baseband_t *bb, float *rng, float rng_pos)
{
    const float vol = 299792458.0;
    float aim_freq;
    float fs;//sample rate
    float tof;//time of fly
    float slope;
    float n;
    float r;

    fs = bb->sys_params.Fs * 1.0e6;//Hz
    slope = bb->sys_params.bandwidth / bb->sys_params.chirp_rampup * 1.0e12;//Hz/s
    n = bb->sys_params.rng_nfft;

    aim_freq = (rng_pos-1)*fs/n;
    tof = aim_freq/slope;
    r = tof * vol / 2;

   /*
    tof = 2 * rng / vol;
    aim_freq = tof * slope;
    p = aim_freq * n / fs + 1;
    *rng_pos = p;
    */
    *rng = r;

//    EMBARC_PRINTF("rng = %f, rng_pos = %f \n", r, rng_pos);

}


/*
 * description: get the peak aim in [rng_start, rng_end].
 * @bb: baseband handle.
 * @chirp_index: chirp index.
 * @ch_index: channel index.
 * @rng_start: range start
 * @rng_end: range end
 * @fft1d_peak: get 1d fft peak point
 * @fft1d_peak_rng: get 1d fft peak point position
 * @author: xerxes
 */
int fft1d_peak_get_with_para(baseband_t *bb, int chirp_index, int ch_index, float rng_start, float rng_end,
    complex_t *fft1d_peak, float *fft1d_peak_rng)
{
    baseband_hw_t *bb_hw = &bb->bb_hw;
    uint32_t fft_mem;
    complex_t complex_fft, fft_peak;
    uint32_t rng_start_index, rng_end_index;
    int vel_index = chirp_index;
    float ftmp1, ftmp2;
    float power, peak_power=0.0;
    float rng;
    uint16_t peak_point = 0;


    rng_of_position_calc(bb, rng_start, &ftmp1);
    rng_start_index = (uint32_t)ftmp1;
    rng_of_position_calc(bb, rng_end,   &ftmp2);
    rng_end_index = (uint32_t)ftmp2;

//    EMBARC_PRINTF("rng_start_index=%d,rng_end_index=%d , (%f,%f)\n", rng_start_index, rng_end_index, ftmp1, ftmp2);

    if(rng_start_index > bb->sys_params.rng_nfft/2 || rng_end_index > bb->sys_params.rng_nfft/2)
    {
        EMBARC_PRINTF("rng err!\n");
        return -1;
    }

    for (uint32_t i = rng_start_index; i <= rng_end_index; i++) {
             uint32_t rng_index = i;
             fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, vel_index, 0);
             complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
             power = 10 * log10f(complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i);
             if (i == 0) {
                     peak_power = power;
                     peak_point = rng_index;
                     fft_peak = complex_fft;
             }
             else if (power > peak_power) {
                     peak_power = power;
                     peak_point = rng_index;
                     fft_peak = complex_fft;
             }
     }
     position_of_rng_calc(bb, &rng, (float)peak_point);
//     EMBARC_PRINTF("Channel %d, FFT1D peak is %f FFT1D number is %d (%f,%f)\n", ch_index, peak_power, peak_point, fft_peak.r, fft_peak.i);

    *fft1d_peak = fft_peak;
    *fft1d_peak_rng = rng;

    return 0;
}

/*
 * description: get the array in [fft_position_start, fft_position_end].
 * @bb: baseband handle.
 * @chirp_index: chirp index.
 * @ch_index: channel index.
 * @fft_position_start: fft1d_data start
 * @fft_position_end: fft1d_data end
 * @fft1d_data: get 1d fft point array
 * @author: xerxes
 */
int fft1d_get_with_position(baseband_t *bb, int chirp_index, int ch_index, int fft_position_start,
    int fft_position_end, complex_t *fft1d_data)
{
    baseband_hw_t *bb_hw = &bb->bb_hw;
    uint32_t fft_mem;
    complex_t complex_fft;
    int vel_index = chirp_index;
    uint32_t rng_index = 0;
    int32_t retval = 0;

    if(fft_position_start > bb->sys_params.rng_nfft/2 || fft_position_end > bb->sys_params.rng_nfft/2)
    {
        retval = -1;
    }
    else
    {   //EMBARC_PRINTF("DIFFT_GET_POSITION_START:\n"); // Print unsigned long to UART
        for (rng_index = fft_position_start; rng_index <= fft_position_end; rng_index++) 
        {
            fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, vel_index, 0);
            //EMBARC_PRINTF("index:arr%d ch%d vel%d rng%d\n", 0, ch_index, vel_index, rng_index); // Print unsigned long to UART
            //EMBARC_PRINTF("fft_mem:%x ", fft_mem); // Print unsigned long to UART
            complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
            //EMBARC_PRINTF("cfl value is %.6f\r\n",sqrt(complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i));
            //EMBARC_PRINTF("cfl_to_complex:%.6f + %.6fi|", complex_fft.r, complex_fft.i); // Print unsigned long to UART
                        
            //complex_fft = cfl_to_complex_weifu(fft_mem);
            //EMBARC_PRINTF("cfl_to_complex_weifu:%.6f + %.6fi\n", complex_fft.r, complex_fft.i); // Print unsigned long to UART
                        
            fft1d_data[rng_index - fft_position_start] = complex_fft;
        }
        //EMBARC_PRINTF("DIFFT_GET_POSITION_STOP:\n"); // Print unsigned long to UART
    }

    return retval;
}

int check_doppler_params(bb_dump_data_t * dump_info,eBB_DATDUMP_TYPE dop_type) {
	uint32_t antena_max_num = 4;
	sensor_config_t *cfg = sensor_config_get_cur_cfg();

	 if ((dump_info->data_index[dop_type].T_start_index < 0) || (dump_info->data_index[dop_type].T_start_index > cfg->nvarray)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].T_stop_index < 0) || (dump_info->data_index[dop_type].T_stop_index > cfg->nvarray)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].A_start_index < 0) || (dump_info->data_index[dop_type].A_start_index > antena_max_num)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].A_stop_index < 0) || (dump_info->data_index[dop_type].A_stop_index > antena_max_num)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].V_start_index < 0) || (dump_info->data_index[dop_type].V_start_index > cfg->vel_nfft)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].V_stop_index < 0) || (dump_info->data_index[dop_type].V_stop_index > cfg->vel_nfft)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].R_start_index < 0) || (dump_info->data_index[dop_type].R_start_index > cfg->rng_nfft / 2)) {
		 return doppler_checkparams_not_ok;
	 }

	 if ((dump_info->data_index[dop_type].R_stop_index < 0) || (dump_info->data_index[dop_type].R_stop_index > cfg->rng_nfft / 2)) {
		 return doppler_checkparams_not_ok;
	 }

	 return doppler_checkparams_ok;
}

int doppler_data_print(baseband_t *bb,uint32_t fft_flag)
{
	int32_t retval;

	if ((bb_dump_data_info.bb_dump_datatype & 0x30) == 0x10 && (fft_flag == 2)) {
		retval = check_doppler_params(&bb_dump_data_info,eBB_DATDUMP_DOPV);
		if (retval != doppler_checkparams_not_ok) {
			bb_datdump_uart_print(bb,&bb_dump_data_info,eBB_DATDUMP_DOPV);
		}else {
			return doppler_checkparams_not_ok;
		}

	}else if ((bb_dump_data_info.bb_dump_datatype & 0x30) == 0x20 && (fft_flag == 1)) {
		retval = check_doppler_params(&bb_dump_data_info,eBB_DATDUMP_DOPR);
		if (retval != doppler_checkparams_not_ok) {
			bb_datdump_uart_print(bb,&bb_dump_data_info,eBB_DATDUMP_DOPR);
		}else {
			return doppler_checkparams_not_ok;
		}
	}

		//EMBARC_PRINTF("doppler data type is %d\r\n",bb_dump_data_info.bb_dump_datatype);
		//EMBARC_PRINTF("doppler antena start is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].A_start_index);
		//EMBARC_PRINTF("doppler antena stop is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].A_stop_index);
		//EMBARC_PRINTF("doppler channel start is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].T_start_index);
		//EMBARC_PRINTF("doppler channel stop is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].T_stop_index);
		//EMBARC_PRINTF("doppler vel start is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].V_start_index);
		//EMBARC_PRINTF("doppler vel stop is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].V_stop_index);
		//EMBARC_PRINTF("doppler rng start is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].R_start_index);
		//EMBARC_PRINTF("doppler rng stop is %d\r\n",bb_dump_data_info.data_index[BB_DUMP_MICRD].R_stop_index);
		return doppler_checkparams_ok;
}

void doppler_value(baseband_t *bb)
{
	baseband_hw_t *bb_hw = &bb->bb_hw;
	uint32_t fft_mem;
	complex_t complex_fft;
	float dop;
	for (uint32_t i = 0; i < 1;i++) {
		for(uint32_t chn = 0;chn < 4;chn++) {
			EMBARC_PRINTF("======================channel%d===========================\r\n",chn);
			for(uint32_t vel = 0;vel < 64;vel++) {
				EMBARC_PRINTF("vel%d_doppler = ",vel);
				for(uint32_t rng = 0;rng < 256;rng++) {
					fft_mem = baseband_hw_get_fft_mem(bb_hw, chn, rng, vel, i);

					complex_fft = cfl_to_complex(fft_mem, 14, 14, true, 4, false);
					dop += sqrtf(complex_fft.i * complex_fft.i + complex_fft.r * complex_fft.r);

				}
				EMBARC_PRINTF("%.6f\r\n",dop);
				dop = 0.00;
			}
		}
	}

}

/* read from hardware */
void doa_hw_read(track_t *track)
{


}




