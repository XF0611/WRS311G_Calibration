/*
 * can_transceiver.c
 *
 *  Created on: 2022Äê2ÔÂ24ÈÕ
 *      Author: fei.xu1
 */
#include "embARC_toolchain.h"
#include "embARC_assert.h"
#include "embARC.h"
#include "stdint.h"
#include "embARC.h"
#include "gpio_hal.h"
#include "can_hal.h"
#include "alps_hardware.h"
#include "clkgen.h"
#include "can_transceiver.h"


void Tja1043_Can_Transceiver_init(void)
{
    /* set can gpio mode */
    raw_writel(IO_MUX_SELECT,gpio_module);
    raw_writel(CAN_EN_IO_MUX,IO_MUX_GPIO_MODULE);
    raw_writel(CAN_STB_IO_MUX,IO_MUX_GPIO_MODULE);
    raw_writel(CAN_WAKE_IO_MUX,IO_MUX_GPIO_MODULE);

    /* set can gpio direction and level */
    gpio_set_direct(CAN_EN_PIN,output);
    gpio_set_direct(CAN_STB_PIN,output);
    //gpio_set_direct(CAN_LOCAL_WAKEUP_PIN,output);

    gpio_write(CAN_EN_PIN,high);
    gpio_write(CAN_STB_PIN,high);
    //gpio_write(CAN_LOCAL_WAKEUP_PIN,high);
}

void Enter_Tja1043_Sleep_Mode(void)
{
	gpio_write(CAN_STB_PIN,low);
}

void can_transfer_error_info(uint32_t can_error_frame_id,uint32_t can_error_info)
{
	//uint64_t data_time1,data_time2;
	//data_time1 = chip_get_cur_us();
	can_send_data(CAN_0_ID,can_error_frame_id,&can_error_info,1);
	//data_time2 = chip_get_cur_us();
	//EMBARC_PRINTF("data delata time is %d\r\n",data_time2 - data_time1);
	chip_hw_mdelay(2);
	Enter_Tja1043_Sleep_Mode();
}

void can_send_radar_status(radar_output_signal *sig)
{
	uint32_t can_send_radar_data;
	uint32_t radar_status;
	uint32_t alivecounter_dection;
	uint32_t send_data_checksum;
	radar_status = sig->radar_status & 0xF;
	alivecounter_dection = (sig->alivecounter << 5) & 0x1E0;
	send_data_checksum = (sig->checksum << 9) & 0x1FE00;

	can_send_radar_data = radar_status | alivecounter_dection | send_data_checksum;

	can_send_data(0,RADAR_CAN_ID,&can_send_radar_data,4);
}


