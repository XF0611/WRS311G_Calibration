#include <math.h>
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
#include "baseband_dpc.h"
#include "baseband_cas.h"
#include "cascade.h"
#include "baseband_alps_FM_reg.h"
#include "bb_flow.h"
#include "track_cli.h"
#include "misc_tool.h"
#include "rtwtypes.h"
#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif
#include "baseband_task_patch.h"
#include "baseband_cli.h"

#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)

static uint8_t old_bb_clk;
extern QueueHandle_t queue_fix_1p4;
extern SemaphoreHandle_t mutex_frame_count;
extern SemaphoreHandle_t mutex_live_dection_flag;
int32_t frame_count = 0;
/* fft1d finish */
extern QueueHandle_t queue_fft1d_finish;
bool live_dection = false;
extern uint32_t ldac_flag;
static bool bb_clk_restore_en = false;
/* task variables */
extern SemaphoreHandle_t mutex_initial_flag;

bool initial_flag = true;
creal32_T fft1d_data[50];//store the fft1d data

void bb_clk_switch()
{
        bb_clk_restore_en = true;
        old_bb_clk = raw_readl(REG_CLKGEN_DIV_AHB);
        bus_clk_div(BUS_CLK_100M); /* when dumping debug data(no buffer), bb clock should be switched to dbgbus clock(100MHz) */
}

void bb_clk_restore() /* After dumping sample debug data(no buffer), bb clock should be restored to default 200MHz */
{
        if (bb_clk_restore_en == true) {
                bb_clk_restore_en = false;
                raw_writel(REG_CLKGEN_DIV_AHB, old_bb_clk);
        }
}

void frame_count_ctrl(void)
{
        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
        if (frame_count > 0)
                frame_count--;
        xSemaphoreGive(mutex_frame_count);
}

void initial_flag_set(bool data)
{
        xSemaphoreTake(mutex_initial_flag, portMAX_DELAY);
        initial_flag = data;
        xSemaphoreGive(mutex_initial_flag);
}

void live_dection_flag_set(bool data)
{
    xSemaphoreTake(mutex_live_dection_flag, portMAX_DELAY);
    live_dection = data;
    xSemaphoreGive(mutex_live_dection_flag);
}
#if FRAME_CYCLE_TIME_CHECK
/****************************************************************
;
; This routine:
;       Check whether the BB frame period is as expected
; arg:
;		void
; return:
;       void
; Change tracking:
;       Ver1.0.0 :
;***************************************************************/
void frame_cycle_time_check(void)
{
    sensor_config_t *cfg = sensor_config_get_cur_cfg();
    static uint32_t last_frame_time = 0;
    uint32_t cur_time = xTaskGetTickCount();

    EMBARC_PRINTF("frame time delta is %d\r\n",cur_time - last_frame_time);
    if ((last_frame_time) && (cur_time > last_frame_time) &&
        ((cur_time - last_frame_time) > 1000U / cfg->track_fps)) {
        EMBARC_PRINTF("Frame period[%d] longer than expected[%d]!\r\n",
            cur_time - last_frame_time, 1000U / cfg->track_fps);
    }
    last_frame_time = xTaskGetTickCount();
}
#endif

extern void CAN_Senddata_Temperature(void);
extern uint32_t doppler_flag;

void baseband_task(void *params)
{
    uint32_t event = 0;
    int32_t retval = 0;
    uint32_t event_bits = 0;
    uint16_t status_en;
    uint32_t fft_flag;

    //uint32_t fft1d_temp_finish = 0;
    baseband_t* bb = baseband_get_cur_bb();
    baseband_data_proc_t* dpc = baseband_get_dpc();
    sensor_config_t *cfg = sensor_config_get_cur_cfg();

    uint64_t frame_start,frame_stop,frame_consume;
    //uint64_t time_bb_task_entry,time_bb_task_exit;
    int32_t frame_time;
    /* Get frame fps */
    uint32_t frame_fps = cfg->track_fps;

#ifdef CHIP_CASCADE
    baseband_cascade_handshake();
    baseband_dc_calib_init(NULL, false, false); // dc_calib after handshake
#endif
    while(1) 
    {

    	//if(ldac_flag == on) {
        if (frame_count != 0) 
        {

                bb = baseband_get_rtl_frame_type();
                baseband_hw_t *bb_hw = &bb->bb_hw;
#if FRAME_CYCLE_TIME_CHECK
                /* Check whether the BB frame period is as expected */
                frame_cycle_time_check();
#endif
#ifdef FUNC_SAFETY
                /* bb frame start flag, used for functional-safety SM1 set part handler */
                bb_frame_start_flag = true;
#endif
                /* Clear event bit before bb start */
                event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
                if( event_bits != E_OK)
                {
                    EMBARC_PRINTF("Event bit set to %d before start @%s", event_bits, __func__);
                }


                /* run baseband data proc chain till the end */
                #if 1
                status_en = baseband_read_reg(bb_hw, BB_REG_SYS_ENABLE);
		        status_en = ( 0 << SYS_ENABLE_HIL_SHIFT    )|
		    	            ( 1 << SYS_ENABLE_SAM_SHIFT    )|
		    	            ( 0 << SYS_ENABLE_DMP_MID_SHIFT)|
		    	            ( 0 << SYS_ENABLE_FFT_1D_SHIFT )|
		    	            ( 0 << SYS_ENABLE_FFT_2D_SHIFT )|
		    	            ( 0 << SYS_ENABLE_CFR_SHIFT    )|
		    	            ( 0 << SYS_ENABLE_BFM_SHIFT    )|
		    	            ( 0 << SYS_ENABLE_DMP_FNL_SHIFT);
		        baseband_start_with_params_weifu(baseband_get_bb(0), true, true, status_en, SYS_MEM_ACT_BUF, BB_IRQ_ENABLE_ALL);
		        frame_start = chip_get_cur_us();
		        while(baseband_hw_is_running(bb_hw) == true) chip_hw_mdelay(2); // it is time to get memory
		        Chirp_num=63;
		        Chn_num=1;
		        index_Start=4;
		        index_End=53;
		        

                retval = fft1d_get_with_position(bb, Chirp_num, Chn_num,index_Start, index_End, fft1d_data);
                fft_flag = 1;
                //EMBARC_PRINTF("fft1d finish!\r\n");
               if(Chirp_num!=0)
                {

                    //CAN_Senddata_APP(fft1d_data);
                }
                #endif
                //CAN_Senddata_Temperature();

                if(doppler_flag) {
                	retval = doppler_data_print(bb,fft_flag);
                }else {
                	retval = bb_dumpdata_ctrl(bb);
                }

                if(retval != 0)
                {
                   EMBARC_PRINTF("bb dump data error %d", retval);
                }

		        status_en = ( 0 << SYS_ENABLE_HIL_SHIFT    )|
		                    ( 0 << SYS_ENABLE_SAM_SHIFT    )|
		                    ( 0 << SYS_ENABLE_DMP_MID_SHIFT)|
		                    ( 0 << SYS_ENABLE_FFT_1D_SHIFT )|
		                    ( 1 << SYS_ENABLE_FFT_2D_SHIFT )|
		                    ( 1 << SYS_ENABLE_CFR_SHIFT    )|
		                    ( 1 << SYS_ENABLE_BFM_SHIFT    )|
		                    ( 0 << SYS_ENABLE_DMP_FNL_SHIFT);
		        baseband_start_with_params_weifu(baseband_get_bb(0), false, false, status_en, SYS_MEM_ACT_RLT, BB_IRQ_ENABLE_ALL);
		        fft_flag = 2;
                if(doppler_flag) {
                	retval = doppler_data_print(bb,fft_flag);
                }else {
                	EMBARC_PRINTF("enter bb dump ctrl!\r\n");
                	retval = bb_dumpdata_ctrl(bb);
                }

                if( initial_flag == true ) 
                {
                    track_pre_start(bb->track);
                    //live_dection_flag_set(false);
                    initial_flag_set(false);
                } 
                else 
                {

                	//live_dection_flag_set(true);
                    //track_run(bb->track);
            		doa_sw_read(bb->track);
            		target_doa_read();

                }
                
                /* wait queue for last BB HW run*/
                BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);
#ifdef CHIP_CASCADE
                if (chip_cascade_status() == CHIP_CASCADE_SLAVE) 
                {
                    /* rx "scan stop" to release spi underlying buffer, must before baseband_write_cascade_ctrl*/
                    baseband_scan_stop_rx(CMD_RX_WAIT);
                    baseband_write_cascade_ctrl();
                } 
                else 
                {
                    mst_slv_fram_cnt_chek();
                }
                fram_cnt_inc();
#endif
                frame_count_ctrl();
#if BB_INTERFERENCE_CHECK
#ifndef CHIP_CASCADE
                /*interference identification*/
                check_interference(bb);
#endif
#endif
                track_read(bb->track);

            baseband_workaroud(&bb->bb_hw);


#ifdef FUNC_SAFETY
            /* functional-safety process */
            func_safety_process(NULL);
#endif
            frame_stop = chip_get_cur_us();
            frame_consume = frame_stop - frame_start;

            frame_time = (1000U / frame_fps) - (frame_consume / 1000U) - 1;

            if (frame_time < 0) {

            	taskYIELD();
            }else {
            	vTaskDelay(frame_time);
            }


        } 
        else  
        {
            if( false == initial_flag )
            baseband_stop(bb); /* re-call baseband_stop if not asserted in xQueueReceive */
#ifdef CHIP_CASCADE
            if(chip_cascade_status() == CHIP_CASCADE_SLAVE)
            baseband_read_cascade_cmd(0); /* waiting for master command "scan start/stop" */
#endif
            xQueueReceive(queue_fix_1p4, &event, 1); // add one tick to disturb the idle task loop
            taskYIELD();
        } /* end if */
    }/* end while */
}






