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
 * \defgroup	CHIP_ALPS_COMMON_TIMER	ALPS Common Timer Module
 * \ingroup	CHIP_ALPS_COMMON
 * \brief	provide basic chip timer-related functions
 * \details
 * 		Provide a 1 MS (default) timer interrupt,
 *	provide a 64-bit counter value (no clear) count in the timer interrupt,
 *	provide MS-precision delay, with OS enabled-support delay
 */

/**
 * \file
 * \ingroup	CHIP_ALPS_COMMON_TIMER
 * \brief	provide alps chip timer-related functions
 */

/**
 * \addtogroup	CHIP_ALPS_COMMON_TIMER
 * @{
 */
#include "arc_builtin.h"
#include "arc.h"
#include "arc_timer.h"
#include "arc_exception.h"

#include "chip.h"

#ifdef ENABLE_OS
#include "os_hal_inc.h"
#endif

#define MAX_SYS_COUNTER_VALUE		(0xffffffff)

#ifndef CHIP_SYS_TIMER_HZ
#define CHIP_SYS_TIMER_HZ		(1000)
#endif

/* current CPU frequency */
static uint32_t current_cpu_freq = 0;
/** alps chip timer counter in timer interrupt */
volatile uint64_t gl_alps_sys_hz_cnt = 0;
/** alps chip 1ms counter */
volatile uint32_t gl_alps_ms_cnt = 0;

#define HZ_COUNT_CONV(precision, base)	((precision)/(base))

/**
 * \brief	Update timer counter and other MS period operation
 * 	in cycling interrupt and must be called periodically. When the OS timer
 *	interrupt is in conflict with the bare-metal timer interrupt,
 *	put this function into the OS timer interrupt
 * \param[in]	precision	interrupt-period precision in Hz
 */
void chip_timer_update(uint32_t precision)
{
	static uint32_t sys_hz_update = 0;
	static uint32_t sys_ms_update = 0;
	uint32_t hz_conv = 0;

	/** count sys hz */
	hz_conv = HZ_COUNT_CONV(precision, CHIP_SYS_TIMER_HZ);
	sys_hz_update ++;
	if (sys_hz_update >= hz_conv) {
		sys_hz_update = 0;
		gl_alps_sys_hz_cnt ++;
	}

	/** count ms */
	hz_conv = HZ_COUNT_CONV(precision, CHIP_SYS_TIMER_MS_HZ);
	sys_ms_update ++;
	if (sys_ms_update >= hz_conv) {
		sys_ms_update = 0;
		gl_alps_ms_cnt ++;

#ifdef MID_FATFS
		alps_sdcard_1ms_update();
#endif
	}
}

/**
 * \brief	alps bare-metal timer interrupt.
 * 	the Interrupt frequency is based on the defined \ref CHIP_SYS_TIMER_HZ
 */
static void alps_timer_isr(void *ptr)
{
	timer_int_clear(CHIP_SYS_TIMER_ID);

	chip_timer_update(CHIP_SYS_TIMER_HZ);
}

/**
 * \brief	init bare-metal alps chip timer and interrupt
 * \details
 * 		This function is called in \ref chip_init, and
 * 	it initializes the 1-MS timer interrupt for bare-metal mode
 */
void system_tick_init(void)
{
	if (timer_present(CHIP_SYS_TIMER_ID)) {
		int_disable(CHIP_SYS_TIMER_INTNO); /* disable first then enable */
		int_handler_install(CHIP_SYS_TIMER_INTNO, alps_timer_isr);
		timer_start(CHIP_SYS_TIMER_ID, TIMER_CTRL_IE|TIMER_CTRL_NH, current_cpu_freq / CHIP_SYS_TIMER_HZ);  /* start 1ms timer interrupt */

		int_enable(CHIP_SYS_TIMER_INTNO);
	}
}

/**
 * \brief	get current cpu hardware ticks
 * \retval	hardware ticks count in 64bit format
 */
uint64_t chip_get_hwticks(void)
{
	uint32_t sub_ticks;
	uint64_t total_ticks;
	timer_current(TIMER_0, &sub_ticks);

	total_ticks = (uint64_t)OSP_GET_CUR_MS() * (current_cpu_freq/CHIP_SYS_TIMER_HZ);
	total_ticks += (uint64_t)sub_ticks;

	return total_ticks;
}

/**
 * \brief	get current passed us since timer init
 * \retval	us count in 64bit format
 */
uint64_t chip_get_cur_us(void)
{
	uint32_t sub_us;
	uint64_t total_us;
	timer_current(TIMER_0, &sub_us);

	sub_us = ((uint64_t)sub_us * 1000000) / current_cpu_freq;
	total_us = ((uint64_t)OSP_GET_CUR_MS()) * 1000 + (uint64_t)sub_us;

	return total_us;
}

/**
 * \brief	provide MS delay function
 * \details
 * 		this function needs a 1-MS timer interrupt to work.
 * 	For bare-metal, it is implemented in this file.
 * 	For OS, you must call \ref chip_timer_update in
 * 	the OS 1-MS timer interrupt when the bare-metal timer interrupt
 * 	is not ready
 * \param[in]	ms		MS to delay
 * \param[in]	os_compat	Enable or disable
 *	When this delay is enabled, use the OS delay function, if one is provided.
 *	See \ref OSP_DELAY_OS_COMPAT_ENABLE and
 *	\ref OSP_DELAY_OS_COMPAT_DISABLE
 */
void chip_delay_ms(uint32_t ms, uint8_t os_compat)
{
	uint64_t start_us, us_delayed;

#ifdef ENABLE_OS
	if (os_compat == OSP_DELAY_OS_COMPAT_ENABLE) {
		/** \todo add different os delay functions */
#ifdef OS_FREERTOS
		vTaskDelay(ms);
		return;
#endif
	}
#endif
	us_delayed = ((uint64_t)ms * 1000);
	start_us = chip_get_cur_us();
	while ((chip_get_cur_us() - start_us) < us_delayed);
}

/**
 * \brief	provide uS delay function
 * \details
 * this function needs a 1-uS timer interrupt to work.
 * For bare-metal, it is implemented in this file.
 * For OS, you must call \ref board_timer_update in
 * the OS 1-uS timer interrupt when the bare-metal timer interrupt
 * is not ready
 * \param[in]	us		uS to delay
 */
void chip_delay_us(uint32_t us)
{
	uint64_t start_us;

	start_us = chip_get_cur_us();
	while ((chip_get_cur_us() - start_us) < us);
}

/**
 * mS delay function based on ARC rtc counter.
 * note: No task reschedule, pure delay
 **/
void chip_hw_mdelay(uint32_t ms)
{
	uint32_t cur_cnt_high, s_cnt_high = 0;
        uint64_t cur_cnt, s_cnt = 0;

	uint32_t delta, ticks = (current_cpu_freq / 1000) * ms;

	timer_current_high(TIMER_RTC, (void *)&s_cnt_high);
	timer_current(TIMER_RTC, (void *)&s_cnt);
        s_cnt |= ((uint64_t)s_cnt_high) << 32;
	do {
		timer_current_high(TIMER_RTC, (void *)&cur_cnt_high);
		timer_current(TIMER_RTC, (void *)&cur_cnt);
                cur_cnt |= ((uint64_t)cur_cnt_high) << 32;
                delta = cur_cnt - s_cnt;
	} while (delta < ticks);
}

void chip_hw_udelay(uint32_t us)
{
        uint32_t cur_cnt_high, s_cnt_high = 0;
        uint64_t cur_cnt, s_cnt = 0;
        uint32_t delta, ticks = (current_cpu_freq / 1000000) * us;

        timer_current_high(TIMER_RTC, (void *)&s_cnt_high);
        timer_current(TIMER_RTC, (void *)&s_cnt);
        s_cnt |= ((uint64_t)s_cnt_high) << 32;
        do {
                timer_current_high(TIMER_RTC, (void *)&cur_cnt_high);
                timer_current(TIMER_RTC, (void *)&cur_cnt);
                cur_cnt |= ((uint64_t)cur_cnt_high) << 32;
                delta = cur_cnt - s_cnt;
        } while (delta < ticks);
}

uint32_t get_current_cpu_freq(void)
{
        return current_cpu_freq;
}

void set_current_cpu_freq(uint32_t Hz)
{
        current_cpu_freq = Hz;
}

/** @} end of group BOARD_ALPS_COMMON_TIMER */
