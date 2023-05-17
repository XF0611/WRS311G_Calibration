/*
 * tcan.h
 *
 *  Created on: 2022Äê10ÔÂ18ÈÕ
 *      Author: fei.xu1
 */

#ifndef WEIFU_TCAN_TCAN_HAL_H_
#define WEIFU_TCAN_TCAN_HAL_H_
#include <stdint.h>

typedef enum {
	width_16bits,
	width_24bits,
	width_32bits
}sclk_nums_type;

typedef enum {
	tcan_read_cmd,
	tcan_write_cmd
};

#define calc_tcan_sendval(addr,cmd) ((uint16_t)addr << 9) | ((uint16_t)cmd << 8)

int32_t tcan_init(uint8_t spi_id,uint32_t baud);
uint32_t tcan_spi_read(uint8_t spi_id,uint8_t addr,uint32_t read_len);
int32_t tcan_spi_write(uint8_t spi_id,uint8_t addr,uint32_t writedata,uint32_t write_len);



#endif /* WEIFU_TCAN_TCAN_HAL_H_ */
