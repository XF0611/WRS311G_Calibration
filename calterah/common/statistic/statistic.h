#ifndef _STATISTIC_H_
#define _STATISTIC_H_

#define STAT_TIMER_ID               TIMER_1
#define STAT_TIMER_INTNO            INTNO_TIMER1

extern volatile uint64_t cpu_run_time;
uint32_t stat_timer_init();

#endif