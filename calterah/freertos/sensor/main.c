/* ------------------------------------------
 * Copyright (c) 2017, Synopsys, Inc. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1) Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.

 * 3) Neither the name of the Synopsys, Inc., nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
--------------------------------------------- */

/**
 * \defgroup    EMBARC_APP_FREERTOS_KERNEL  embARC FreeRTOS Kernel Example
 * \ingroup     EMBARC_APPS_TOTAL
 * \ingroup     EMBARC_APPS_OS_FREERTOS
 * \brief       embARC Example for testing FreeRTOS task switch and interrupt/exception handling
 *
 * \details
 * ### Extra Required Tools
 *
 * ### Extra Required Peripherals
 *
 * ### Design Concept
 *     This example is designed to show the functionality of FreeRTOS.
 *
 * ### Usage Manual
 *     Test case for show how FreeRTOS is working by task switching and interrupt/exception processing.
 *     ![ScreenShot of freertos-demo under freertos](pic/images/example/emsk/emsk_freertos_demo.jpg)
 *
 * ### Extra Comments
 *
 */

/**
 * \file
 * \ingroup     EMBARC_APP_FREERTOS_KERNEL
 * \brief       main source file of the freertos demo
 */

/**
 * \addtogroup  EMBARC_APP_FREERTOS_KERNEL
 * @{
 */

#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "command.h"
#include "baseband.h"
#include "baseband_cli.h"
#include "radio_ctrl.h"
#include "radio_reg.h"
#include "sensor_config.h"
#include "sensor_config_cli.h"
#include "baseband_task.h"
#include <stdlib.h>
#ifdef USE_IO_STREAM
#include "data_stream.h"
#endif

#include "flash.h"
#include "gpio_hal.h"

#include "console.h"
#include "can_cli.h"
#include "can_signal_interface.h"
#include "common_cli.h"
#include "spi_master.h"
#include "misc_tool.h"
#include "apb_lvds.h"
#ifdef CHIP_CASCADE
#include "cascade.h"
#endif

#ifdef SPIS_SERVER
#include "spis_server.h"
#endif

#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif

#ifdef CPU_CLOCK_LOCK_DETECTOR
#include "cpu_clock_lock_detector.h"
#endif

#ifdef CPU_STAT
#include "statistic.h"
#endif
#include "pmic_monitor_task.h"
#include "can_transceiver.h"
#include "tcan.h"

#define TSK_PRIOR_HI        (configMAX_PRIORITIES-1)
#define TSK_PRIOR_LO        (configMAX_PRIORITIES-2)


/* Task IDs */
static TaskHandle_t bb_handle = NULL;
static TaskHandle_t pmic_monitor_handle = NULL;
/* Task Communication */
QueueHandle_t queue_cas_sync;
QueueHandle_t queue_spi_cmd;
QueueHandle_t queue_fix_1p4;
QueueHandle_t queue_doa_finish;
QueueHandle_t queue_feed_wdt;
QueueHandle_t queue_fft1d_finish;
SemaphoreHandle_t mutex_frame_count;
SemaphoreHandle_t mutex_initial_flag;
SemaphoreHandle_t mutex_live_dection_flag;
SemaphoreHandle_t mutex_ldac_function;
EventGroupHandle_t bb_int_event_handle;

/* baseband */

/**
 * \brief  call FreeRTOS API, create and start tasks
 */
extern void Car_Indoor_Calib_Init(void);
int main(void)
{
	//chip_hw_mdelay(5000);
#if ACC_BB_BOOTUP == 0
        EMBARC_PRINTF("Benchmark CPU Frequency: %d Hz\r\n", get_current_cpu_freq());
#endif
        vTaskSuspendAll();

        /*when lvds is standby, lvds pad pin from DCK is not sure,it maybe cause wr skip;
          it make pad pin is sure when lvds is changed to test mode from nromal mode */
        fmcw_radio_lvds_mode_norm_to_test(NULL);
        lvds_status_init();
        common_cmd_init();

#ifndef PLAT_ENV_FPGA // flash init blocks firmware running on FPGA
        if (E_OK != flash_init()) {
            EMBARC_PRINTF("Error: externl flash initialize failed!\r\n");
        } else {
            flash_cli_register();
        }
#endif
        Car_Indoor_Calib_Init();
#ifdef CHIP_CASCADE
        cascade_init();
#endif

#ifdef SPIS_SERVER
        spis_server_init();
#endif

        queue_cas_sync = xQueueCreate(BB_ISR_QUEUE_LENGTH, sizeof(uint32_t));
        queue_spi_cmd = xQueueCreate(BB_ISR_QUEUE_LENGTH, sizeof(uint32_t));
        queue_fix_1p4 = xQueueCreate(BB_1P4_QUEUE_LENGTH, sizeof(uint32_t));
        queue_doa_finish = xQueueCreate(BB_DOA_QUEUE_LENGTH, sizeof(uint32_t));
        queue_feed_wdt = xQueueCreate(WDT_FEED_QUEUE_LENGTH, sizeof(uint32_t));
        queue_fft1d_finish = xQueueCreate(FFT1D_FINISH_QUEUE_LENGTH, sizeof(uint32_t));
        mutex_frame_count = xSemaphoreCreateMutex();
        mutex_initial_flag = xSemaphoreCreateMutex();
        mutex_live_dection_flag = xSemaphoreCreateMutex();
        mutex_ldac_function = xSemaphoreCreateMutex();
        bb_int_event_handle = xEventGroupCreate();

        /* initialization */
#ifdef FUNC_SAFETY
        /* print functional-safety error code */
        func_safety_check_emu_error_code();
#endif
        track_init(track_get_track_ctrl());
        baseband_clock_init();
#ifdef BASEBAND_CLI
        baseband_cli_commands();
#endif
#ifdef SENSOR_CONFIG_CLI
        sensor_config_cli_commands();
#endif
        EMBARC_ASSERT(NUM_FRAME_TYPE <= MAX_FRAME_TYPE);
        /* Multi-frame interleaving related configuration */
        /* bb and radio configuration for all frame types*/
        for (int i = 0; i < NUM_FRAME_TYPE; i++) {
#if ACC_BB_BOOTUP == 1
/* baseband pre acceleration stage */
        EMBARC_PRINTF("if(bb_hw->frame_type_id == %d){\n",i);
        sensor_config_t *cfg = sensor_config_get_config(i);
        baseband_t *bb = baseband_get_bb(i);
        if (cfg && bb)
                sensor_config_attach(cfg, bb, i);
        EMBARC_PRINTF("}\n\r");
#else
 /* baseband normal stage or acceleration stage */
        EMBARC_PRINTF("------------------------ init frame_type %d------------------------ \n", i);
        sensor_config_t *cfg = sensor_config_get_config(i);
        baseband_t *bb = baseband_get_bb(i);
		if (cfg && bb)
			sensor_config_attach(cfg, bb, i);
#endif
        }
        EMBARC_PRINTF("<EOF>\r\n");

        freertos_cli_init();
        /* initialize CAN device */
        can_cli_init();
        /* register CAN cli command */
        can_cli_commands();

#ifdef CPU_CLOCK_LOCK_DETECTOR
        /* initialize cpu clock lock detector */
        cpu_clock_lock_detector_init();
#endif

#ifdef FUNC_SAFETY
        /* initialize functional-safety */
        func_safety_init(NULL);
#endif

#ifdef SYSTEM_UART_OTA
        uart_ota_init();
#endif

#ifdef SYSTEM_WATCHDOG
        extern void watchdog_init(void);
        watchdog_init();
#endif

#ifdef CPU_STAT
        if(E_OK != stat_timer_init()){
                EMBARC_PRINTF("Fail to start cpu statistic timer\n");
        }
#endif

        /* initialize console interface */
        console_init();

        /* can transceiver init */
        //Tja1043_Can_Transceiver_init();
        //uint8_t testsend[8] = {11,22,33,44,55,66,77,88};
        //can_send_data(0,0x777,testsend,8);


        tcan_init(0,4000000);
        ads7029_spi_init(1,4000000);

        power_on_check();
        config_selective_wakeup();
        switch_tcan_mode(Normal_Mode_Cmd);
        /* can ota main */
        can_app_uds_main();
//        if (xTaskCreate(baseband_task, "baseband", 2048, (void *)0, configMAX_PRIORITIES-2, &bb_handle)
//            != pdPASS) {
//            EMBARC_PRINTF("create baseband error\r\n");
//            return -1;
//        }


        //init_extern_task();
        /* Set the interrupt threshold in the STATUS32 register to 0xf.
         * It is used to enable interrupt processing.
         * */
        cpu_unlock_restore(0xff);
        xTaskResumeAll();

        vTaskSuspend(NULL);
        vTaskSuspend(NULL);

        return 0;
}

/** @} */
