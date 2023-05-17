#include <string.h>
#include "embARC_toolchain.h"
#include "FreeRTOS.h"
#include "task.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "statistic.h"
#include "embARC_assert.h"
#include "FreeRTOS_CLI.h"
#include "arc_timer.h"

volatile uint64_t cpu_run_time = 0UL;

static void stat_timer_isr(void *params)
{
	timer_int_clear(STAT_TIMER_ID);
	cpu_run_time += 1;
}

uint32_t stat_timer_init()
{
    uint32_t result = E_OK;
    uint32_t current_cpu_freq = get_current_cpu_freq();

    if (timer_present(STAT_TIMER_ID)) {
        /* disable first then enable */
        int_disable(STAT_TIMER_INTNO);

        int_handler_install(STAT_TIMER_INTNO, stat_timer_isr);

        /* start 1us timer interrupt */
        timer_start(STAT_TIMER_ID, TIMER_CTRL_IE|TIMER_CTRL_NH, current_cpu_freq/1000000);
        int_enable(STAT_TIMER_INTNO);

    } else {
        result = E_DBUSY;
    }

    return result;
}