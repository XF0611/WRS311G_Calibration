/*
 * tcan.c
 *
 *  Created on: 2022Äê10ÔÂ18ÈÕ
 *      Author: fei.xu1
 */

#include "embARC_error.h"
#include "embARC_debug.h"
#include "spi_hal.h"
#include "tcan_hal.h"
#include "alps_dmu.h"

uint8_t test[100];
//SPI MODE0(CPOL = 0,CPHA = 0)
static spi_xfer_desc_t spim_xfer_tcan_desc =
{
    .clock_mode = SPI_CLK_MODE_0,
    .dfs = 16,
    .cfs = 0,
    .spi_frf = SPI_FRF_STANDARD,
    .rx_thres = 0,
    .tx_thres = 0,
};

int32_t tcan_init(uint8_t spi_id,uint32_t baud)
{
	int32_t result = E_OK;

    if (spi_id == 0){
        io_mux_spi_m0_func_sel(0);
    } else if (spi_id == 1){
        io_mux_spi_m1_func_sel(0);
    }else{
        EMBARC_PRINTF("Invalid SPI Master ID. \r\n");
        return -1;
    }

    result = spi_open(spi_id, baud);
    if (E_OK != result){
        EMBARC_PRINTF("Error: tcan spi init failed %d\r\n",result);
        return result;
    }else
    {

        result = spi_transfer_config(spi_id, &spim_xfer_tcan_desc);
        if (E_OK != result){
            EMBARC_PRINTF("Error: tcan spi master config failed %d.\r\n",result);
        }
    }
        return result;
}


uint32_t tcan_spi_read(uint8_t spi_id,uint8_t addr,uint32_t read_len)
{
	int32_t result = E_OK;
	uint32_t send_tcan_value;
	uint32_t readdata;

	send_tcan_value = calc_tcan_sendval(addr,tcan_read_cmd);
	/* spi read */
	result = spi_xfer(spi_id,&send_tcan_value,&readdata,read_len);



	if (result != E_OK)
	{
        EMBARC_PRINTF("tcan spi read failed,reason is %d\r\n",result);
	}else {

        //EMBARC_PRINTF("tcan spi read result is %x\r\n",readdata & 0xFF);
	}


	return readdata & 0xFF;
}

int32_t tcan_spi_write(uint8_t spi_id,uint8_t addr,uint32_t writedata,uint32_t write_len)
{
	int32_t result = E_OK;
	uint32_t send_tcan_value;
	uint32_t write_data;

	send_tcan_value = calc_tcan_sendval(addr,tcan_write_cmd);

	write_data = send_tcan_value | writedata;


	/* spi write */
	result = spi_xfer(spi_id,&write_data,NULL,write_len);

	return result;
}



