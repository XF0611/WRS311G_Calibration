/*
 * max20430a.h
 *
 *  Created on: 2022定2埖17晩
 *      Author: fei.xu1
 */

#ifndef WEIFU_PMIC_MAX20430A_H_
#define WEIFU_PMIC_MAX20430A_H_

#include "stdint.h"

/* Diagnostic Status Register */
#define REST_ERR_OCCURRED (1 << 4)
#define POR_OCCURRED (1 << 3)
#define THERMAL_SHUTDOWN_OCCURRED (1 << 1)
#define INTERR_OCCURRED (1 << 0)
typedef enum {
	NO_POR_OCCURRED,
	POR_HAS_OCCURRED
}power_on_reset;

/* monitor voltage(OUT1、OUT2、OUT3、OUT4、IN5、IN6) status */
#define DIAG_VOLATGE_EXCEPTION_OCCURRED 1
#define DIAG_VOLTAGE_CHANNEL_NUMBER 6
#define MONITOR_VOLTAGE_EXCEPTION_OCCURRED 1

/* watchdog status register */
#define WD_LFSR_OCCURRED (1 << 2)
#define WD_UV_OCCURRED (1 << 1)
#define WD_EXP_OCCURRED (1 << 0)

/* can transfer error info */
#define max20430a_error_can_frame_id 0x131
typedef enum {
	WD_EXP = 0x11,
	WD_UV,
	WD_LFSR
}can_trans_wdt_error_info;

typedef enum {
	INTERR = 0x21,
	THSD,
	RSTERR
}can_trans_statd_error_info;
#define OUT_UV 0x31
#define OUT_OV 0x41
#define OUT_OFF_ERR 0x51

void max20430a_wdt_on(void);
uint8_t MAX20430A_LFSR(uint8_t iKey);
void max20430a_wdt_feed(void);
bool diag_max20430a_stat(max20430a_register diag_register);

#endif /* WEIFU_PMIC_MAX20430A_H_ */
