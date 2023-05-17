/*
 * max20430a_operation.c
 *
 *  Created on: 2022年2月15日
 *      Author: fei.xu1
 */
#include "embARC_error.h"
#include "embARC_debug.h"
#include "stdbool.h"
#include "i2c_hal.h"
#include "max20430a_operation.h"



int32_t max20430a_iic_read(uint32_t addr_mode,uint32_t max20430a_register_addr,uint8_t *read_buf,uint32_t read_len)
{
	int32_t read_result,transfer_config_result;

	transfer_config_result = i2c_transfer_config(addr_mode,max20430a_iic_slave_addr,max20430a_register_addr,max20430a_register_len);
	if(E_OK != transfer_config_result) {
		return transfer_config_result;
	}

	read_result = i2c_read(read_buf,read_len);
	if(E_OK != read_result) {
		return read_result;
	}

	return E_OK;
}


int32_t max20430a_iic_write(uint32_t addr_mode,uint32_t max20430a_register_addr,uint8_t *write_buf,uint32_t write_len)
{
	int32_t write_result,transfer_config_result;

	transfer_config_result = i2c_transfer_config(addr_mode,max20430a_iic_slave_addr,max20430a_register_addr,max20430a_register_len);
	if(E_OK != transfer_config_result) {
		return transfer_config_result;
	}

	write_result = i2c_write(write_buf,write_len);
	if(E_OK != write_result) {
		return write_result;
	}

	return E_OK;
}

void read_max20430a(uint32_t addr_mode,max20430a_register read_register,uint8_t *read_buf,uint32_t read_len)
{
	int32_t result;
	max20430a_register mr;
	mr = read_register;
	result = max20430a_iic_read(addr_mode,mr,read_buf,read_len);

	if(E_OK == result) {
		//EMBARC_PRINTF("read max20430a %d register success!\r\n",mr);
		//;
	}else {
		EMBARC_PRINTF("read max20430a %d register fail,error info is %d\r\n",mr,result);
		//can_send_data();
	}

}

void write_max20430a(uint32_t addr_mode,max20430a_register write_register,uint8_t *write_buf,uint32_t write_len)
{
	int32_t result;
	max20430a_register mr;
	mr = write_register;
	result = max20430a_iic_write(addr_mode,mr,write_buf,write_len);

	if(E_OK == result) {
		//EMBARC_PRINTF("write max20430a %d register success!\r\n",mr);
		//;
	}else {
		EMBARC_PRINTF("write max20430a %d register fail,error info is %d\r\n",mr,result);
		//can_send_data();
	}

}

i2c_io_timing_t i2c_timing_s =
{
    .scl_l_cnt   = 1980,
    .scl_h_cnt   = 1950,
    .sda_rx_hold = 0,
    .sda_tx_hold = 1,
    .spike_len   = 5,
};

i2c_io_timing_t i2c_timing_f =
{
    .scl_l_cnt   = 480,
    .scl_h_cnt   = 470,
    .sda_rx_hold = 0,
    .sda_tx_hold = 1,
    .spike_len   = 5,
};

void iic_init(void)
{
	int32_t result;

	i2c_params_t max20430a_i2c_params;

	/* 关闭i2c io端口多路复用功能 */
	io_mux_i2c_func_sel(0);

	/* 配置i2c初始化参数  speed_mode addr_mode restart_en rx_timeout i2c_io_timing */
	max20430a_i2c_params.addr_mode = 0;
	/* fast mode */
	max20430a_i2c_params.speed_mode = 1;
	max20430a_i2c_params.rx_timeout = 2000;
	max20430a_i2c_params.restart_en = 1;
	/* i2c时钟配置 */
	max20430a_i2c_params.timing = &i2c_timing_s;

	result = i2c_init(0,&max20430a_i2c_params);

	if (E_OK == result) {
		EMBARC_PRINTF("i2c init is ok!\r\n");
	} else {
		EMBARC_PRINTF("i2c init is error!\r\n");
	}

	/* disable watchdog */
	uint8_t wdt_dis = 0;
	write_max20430a(0,WDCFG2,&wdt_dis,1);

}


