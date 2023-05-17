/*
 * ads7029.c
 *
 *  Created on: 2022Äê5ÔÂ5ÈÕ
 *      Author: fei.xu1
 */
#include "embARC_error.h"
#include "embARC_debug.h"
#include "stdint.h"
#include "spi_hal.h"
#include "spi_master.h"
#include "ads7029.h"
#include "misc_tool.h"

static spi_xfer_desc_t spim_xfer_ads7029_desc =
{
    .clock_mode = SPI_CLK_MODE_0,
    .dfs = 12,
    .cfs = 0,
    .spi_frf = SPI_FRF_STANDARD,
    .rx_thres = 0,
    .tx_thres = 0,
};

static spi_xfer_desc_t spi_loopback_test =
{
    .clock_mode = SPI_CLK_MODE_0,
    .dfs = 32,
    .cfs = 0,
    .spi_frf = SPI_FRF_STANDARD,
    .rx_thres = 0,
    .tx_thres = 0,
};

static spi_xfer_desc_t spim_xfer_w550_desc =
{
    .clock_mode = SPI_CLK_MODE_0,
    .dfs = 32,
    .cfs = 0,
    .spi_frf = SPI_FRF_STANDARD,
    .rx_thres = 0,
    .tx_thres = 0,
};

void w550_spi_init(uint8_t spi_id,uint32_t baud)
{
    int32_t result = E_OK;

    if (spi_id == 0){
        io_mux_spi_m0_func_sel(0);
    } else if (spi_id == 1){
        io_mux_spi_m1_func_sel(0);
    }else{
        EMBARC_PRINTF("Invalid SPI Master ID. \r\n");
    }

    result = spi_open(spi_id, baud);
    if (E_OK != result){
        EMBARC_PRINTF("Error: w550 spi init failed %d\r\n",result);
    }
    result = spi_transfer_config(spi_id, &spim_xfer_w550_desc);

    if (E_OK != result){
        EMBARC_PRINTF("Error: spi master config failed %d.\r\n",result);
    }

}

void ads7029_spi_init(uint8_t spi_id,uint32_t baud)
{
    int32_t result = E_OK;

    if (spi_id == 0){
        io_mux_spi_m0_func_sel(0);
    } else if (spi_id == 1){
        io_mux_spi_m1_func_sel(0);
    }else{
        EMBARC_PRINTF("Invalid SPI Master ID. \r\n");
    }

    result = spi_open(spi_id, baud);
    if (E_OK != result){
        EMBARC_PRINTF("Error: ads7029 spi init failed %d\r\n",result);
    }
    result = spi_transfer_config(spi_id, &spim_xfer_ads7029_desc);

    if (E_OK != result){
        EMBARC_PRINTF("Error: spi master config failed %d.\r\n",result);
    }

}

uint8_t w550_spi_read(uint8_t spi_id,uint32_t *readdata,uint32_t read_len)
{
	int32_t result = E_OK;
	uint32_t w550_read_value;

	/* spi read */
	w550_read_value = spi_read(spi_id,readdata,read_len);

	if (w550_read_value != E_OK)
	{
		EMBARC_PRINTF("w550 spi read failed,reason is %d\r\n",w550_read_value);
	}else {
		EMBARC_PRINTF("w550 spi read result is %d\r\n",readdata[0]);
	}


	return result;
}

uint8_t w550_spi_write(uint8_t spi_id,uint32_t *writedata,uint32_t write_len)
{
	int32_t result = E_OK;
	uint32_t w550_write_value;


	/* spi read */
	w550_write_value = spi_write(spi_id,writedata,write_len);

	if (w550_write_value != E_OK)
	{
		EMBARC_PRINTF("w550 spi read failed,reason is %d\r\n",w550_write_value);
	}else {
		EMBARC_PRINTF("w550 spi read result is %d\r\n",writedata[0]);
	}


	return result;
}

int32_t ads7029_spi_read(uint8_t spi_id,uint32_t *readdata,uint32_t read_len)
{
	int32_t result = E_OK;
	uint32_t ads7029_read_value;
	//float monitor_voltage;
	float ads_voltage_collection = 0.0;

	/* spi read */
	ads7029_read_value = spi_read(spi_id,readdata,read_len);

	if (ads7029_read_value != E_OK)
	{
		EMBARC_PRINTF("ads spi read failed,reason is %d\r\n",ads7029_read_value);
		return ads7029_read_value;
	}else {
		EMBARC_PRINTF("ads spi read result is %d\r\n",readdata[0]);
	}

	if ((readdata[0] & 0xC00) != 0)
	{
		EMBARC_PRINTF("the first two bits of ads7029 conversion result is not zero!\r\n");
		result = -1;
	}else {
		EMBARC_PRINTF("the first two bits of ads7029 conversion result is success!\r\n");
		//monitor_voltage = (readdata[0] / 1024.0) * 3.3;
		//ads_voltage_collection = readdata[0] * ads_ref_voltage_scale * sample_coff;
		//EMBARC_PRINTF("monitor voltage is %f\r\n",ads_voltage_collection);
	}
	return result;
}


void spi_loopback_tests(void)
{
	int32_t result = E_OK;
	uint32_t m0_send_data[1] = {0x22334455};
	uint32_t slave_read[4] = {0x0};
	//uint32_t test_data[4] = {0x1234,0x2345,0x3456,0x4567};


	raw_writel(0xBA0000 + 0x320, 1);
	io_mux_spi_m0_func_sel(0);
	/* enable spi m0 */
	result = spi_open(0, 25000000);

	if (result != E_OK)
	{
		EMBARC_PRINTF("spi m0 open failed!\r\n");
	}

	result = spi_transfer_config(0, &spi_loopback_test);

	/* enable spi s1 */
	io_mux_spi_s1_func_sel(0);

	result = spi_open(2, 25000000);
	if (result != E_OK)
	{
		EMBARC_PRINTF("spi s1 open failed!\r\n");
	}

	spi_write(0,m0_send_data,1);

	spi_read(2,slave_read,1);

	io_mux_spi_m0_func_sel(4);
	io_mux_spi_s1_func_sel(4);


	//EMBARC_PRINTF("slave read result0 is %d\r\n",slave_read[0]);
	//EMBARC_PRINTF("slave read result1 is %d\r\n",slave_read[1]);
	//EMBARC_PRINTF("slave read result2 is %d\r\n",slave_read[2]);
	//EMBARC_PRINTF("slave read result3 is %d\r\n",slave_read[3]);
	//EMBARC_PRINTF("slave read result4 is %d\r\n",slave_read[4]);
	//EMBARC_PRINTF("slave read result5 is %d\r\n",slave_read[5]);
	//EMBARC_PRINTF("slave read result6 is %d\r\n",slave_read[6]);
	//EMBARC_PRINTF("slave read result7 is %d\r\n",slave_read[7]);

}
