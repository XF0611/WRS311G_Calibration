/*
 * ads7029.h
 *
 *  Created on: 2022Äê5ÔÂ5ÈÕ
 *      Author: fei.xu1
 */

#ifndef WEIFU_ADS7029_ADS7029_H_
#define WEIFU_ADS7029_ADS7029_H_

#define ADS7029_SCLK_FREQUENCE 24000000

void ads7029_spi_init(uint8_t spi_id,uint32_t baud);
int32_t ads7029_spi_read(uint8_t spi_id,uint32_t *readdata,uint32_t read_len);
void spi_loopback_tests(void);
#endif /* WEIFU_ADS7029_ADS7029_H_ */
