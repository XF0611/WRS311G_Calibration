/*
 * pmic_monitor_task.c
 *
 *  Created on: 2022ƒÍ2‘¬15»’
 *      Author: fei.xu1
 */
#include "embARC_error.h"
#include "embARC_debug.h"
#include "max20430a_operation.h"
#include "max20430a.h"
#include "can_hal.h"
#include "gpio_hal.h"
#include "FreeRTOS.h"

SemaphoreHandle_t mutex_pmic_status;
bool pmic_status = false;
static int entry_wdt_first_flag = 1;
static int max2340a_cid = 0x13;

void pmic_status_set(bool data)
{
    xSemaphoreTake(mutex_pmic_status, 0);
    pmic_status = data;
    xSemaphoreGive(mutex_pmic_status);
}

void pmic_monitor_task(void)
{
	uint8_t buf[1] = {0x00};
	bool pmic_final_result = false;
	uint8_t cid = 0;
	max20430a_register access_register = 0;
	bool statd,statwd,statuv,statov,statoff;
	mutex_pmic_status = xSemaphoreCreateMutex();
	while(1) {

		if (entry_wdt_first_flag == 1) {
				entry_wdt_first_flag = 2;
				/* ∂¡»°µÁ‘¥–æ∆¨cid */
				access_register = CID;
				read_max20430a(0,access_register,buf,1);
				cid = buf[0] & 0x3F;
				EMBARC_PRINTF("cid is %d\r\n",cid);
				/* ’Ô∂œmax20430a statd status */
				statd = diag_max20430a_stat(STATD);
				/* ’Ô∂œmax20430a wdt status */
				statwd = diag_max20430a_stat(STATWD);
				/* ’Ô∂œmax20430a UV status */
				statuv = diag_max20430a_stat(STATUV);
				/* ’Ô∂œmax20430a OV status */
				statov = diag_max20430a_stat(STATOV);
				/* ’Ô∂œmax20430a OFF status */
				statoff = diag_max20430a_stat(STATOFF);
				pmic_final_result = statd & statwd & statuv & statov & statoff;
				pmic_status_set(pmic_final_result);
				max20430a_wdt_on();

			} else if(cid == max2340a_cid && entry_wdt_first_flag == 2) {

				/* ’Ô∂œmax20430a statd status */
				statd = diag_max20430a_stat(STATD);
				/* ’Ô∂œmax20430a wdt status */
				statwd = diag_max20430a_stat(STATWD);
				/* ’Ô∂œmax20430a UV status */
				statuv = diag_max20430a_stat(STATUV);
				/* ’Ô∂œmax20430a OV status */
				statov = diag_max20430a_stat(STATOV);
				/* ’Ô∂œmax20430a OFF status */
				statoff = diag_max20430a_stat(STATOFF);
				pmic_final_result = statd & statwd & statuv & statov & statoff;
				pmic_status_set(pmic_final_result);
				/* ÷‹∆⁄–‘Œππ∑ */
				max20430a_wdt_feed();


			}
		vTaskDelay(1470);
	}
}


