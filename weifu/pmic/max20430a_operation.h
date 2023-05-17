/*
 * max20430a_operation.h
 *
 *  Created on: 2022Äê2ÔÂ17ÈÕ
 *      Author: fei.xu1
 */

#ifndef WEIFU_PMIC_MAX20430A_OPERATION_H_
#define WEIFU_PMIC_MAX20430A_OPERATION_H_

#include "stdint.h"
#define max20430a_iic_slave_addr 0x38
#define max20430a_register_len 1

typedef enum {
	CID = 0,
	CONFIG1,
	CONFIG2,
	CONFIGE,
	CONFIGM,
	FPSCFG1,
	PORRST,
	PINMAP1,
	STATUV,
	STATOV,
	STATOFF,
	STATD,
	STATM,
	STATWD,
	VOUT2,
	VOUT4,
	VIN5,
	VIN6,
	WDCDIV,
	WDCFG1,
	WDCFG2,
	WDKEY,
	WDPROT
}max20430a_register;

int32_t max20430a_iic_read(uint32_t addr_mode,uint32_t max20430a_register_addr,uint8_t *read_buf,uint32_t read_len);
int32_t max20430a_iic_write(uint32_t addr_mode,uint32_t max20430a_register_addr,uint8_t *write_buf,uint32_t write_len);

void read_max20430a(uint32_t addr_mode,max20430a_register read_register,uint8_t *read_buf,uint32_t read_len);
void write_max20430a(uint32_t addr_mode,max20430a_register write_register,uint8_t *write_buf,uint32_t write_len);
void iic_init(void);
#endif /* WEIFU_PMIC_MAX20430A_OPERATION_H_ */
