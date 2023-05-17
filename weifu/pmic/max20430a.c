/*
 * max20430.c
 *
 *  Created on: 2022年2月16日
 *      Author: fei.xu1
 */
#include "stdbool.h"
#include "embARC_debug.h"
#include "gpio_hal.h"
#include "max20430a_operation.h"
#include "max20430a.h"
#include "can_transceiver.h"


static uint8_t por_flag = 1;
void max20430a_wdt_on(void)
{
	uint8_t buf[1] = {0x00};
	max20430a_register mr;

	/* 先读取max20430 lock protection register */
	mr = CONFIG1;
	read_max20430a(0,mr,buf,1);

	/* watchdog configuration is writeable */
	if(((buf[0] >> 1) & 0x01) == 0) {
		/* configure pin map */
		mr = PINMAP1;
		uint8_t pinmap_value = 0x0F;
		write_max20430a(0,mr,&pinmap_value,1);

		/* configure reset hold time */
		mr = CONFIG2;
		uint8_t ret_hd = 0xE1;
		write_max20430a(0,mr,&ret_hd,1);

		/* configure wdt mode and tWDCLK */
		mr = WDCDIV;
		uint8_t wdt_mode = 0x3F;
		write_max20430a(0,mr,&wdt_mode,1);

		/* config twdl,twdh */
		mr = WDCFG1;
		uint8_t wdt_twd = 0xFF;
		write_max20430a(0,mr,&wdt_twd,1);

		/* enable wdt */
		mr = WDCFG2;
		uint8_t wdt_enable = 0x08;
		write_max20430a(0,mr,&wdt_enable,1);


	}else {
		/* configure max20430a all register is writeable */
		uint8_t prot = buf[0];
		prot &= 0x0D;
		write_max20430a(0,mr,&prot,1);

		/* configure pin map */
		mr = PINMAP1;
		uint8_t pinmap_value = 0x0F;
		write_max20430a(0,mr,&pinmap_value,1);

		/* configure reset hold time */
		mr = CONFIG2;
		uint8_t ret_hd = 0xE1;
		write_max20430a(0,mr,&ret_hd,1);

		/* configure wdt mode and tWDCLK */
		mr = WDCDIV;
		uint8_t wdt_mode = 0x3F;
		write_max20430a(0,mr,&wdt_mode,1);

		/* config twdl,twdh */
		mr = WDCFG1;
		uint8_t wdt_twd = 0xFF;
		write_max20430a(0,mr,&wdt_twd,1);

		/* enable wdt */
		mr = WDCFG2;
		uint8_t wdt_enable = 0x08;
		write_max20430a(0,mr,&wdt_enable,1);
	}
}

uint8_t MAX20430A_LFSR(uint8_t iKey) {
        uint8_t lfsr = iKey;

        uint8_t bit = ((lfsr >> 7) ^ (lfsr >> 5) ^ (lfsr >> 4) ^ (lfsr >> 3)) & 1;
        lfsr = (lfsr << 1) | bit;

        return lfsr;
}

void max20430a_wdt_feed(void)
{
	uint8_t buf[1] = {0x00};
	uint8_t old_wdtkey,new_wdtkey;
	max20430a_register mr;

	/* 读取wdt key register */
	mr = WDKEY;
	read_max20430a(0,mr,&buf[0],1);
	old_wdtkey = buf[0];
	//old_wdtkey = 222;
	EMBARC_PRINTF("old wdt key is %d\r\n",old_wdtkey);
	new_wdtkey = MAX20430A_LFSR(old_wdtkey);
	EMBARC_PRINTF("new wdt key is %d\r\n",new_wdtkey);
	/* write correct key to feed wdt */
	write_max20430a(0,mr,&new_wdtkey,1);
}

bool diag_max20430a_stat(max20430a_register diag_register)
{
	bool result = true;
	uint32_t can_trans_err_info;
	max20430a_register max20430a_diag_register;
	uint8_t buf[1] = {0x00};
	uint8_t diag_max20430a_result;
	max20430a_diag_register = diag_register;

	read_max20430a(0,max20430a_diag_register,buf,1);
	diag_max20430a_result = buf[0];

	switch(max20430a_diag_register) {
	case STATUV:
	case STATOV:
	case STATOFF:
		for(uint8_t i = 0;i < DIAG_VOLTAGE_CHANNEL_NUMBER;i++) {
			uint8_t monitor_status = diag_max20430a_result & (DIAG_VOLATGE_EXCEPTION_OCCURRED << i);
			if (monitor_status == MONITOR_VOLTAGE_EXCEPTION_OCCURRED && por_flag == NO_POR_OCCURRED) {
				result &= false;
				/* IN5 IN6需要连接上外部电源监控才可生效 暂时result定为true*/
				if(i == 4 || i == 5) {
					result = true;
					//EMBARC_PRINTF("IN[%d] above OV threshold!\r\n",i+1);
					//can_send_data();
					;

				} else if (diag_register == STATUV){
					EMBARC_PRINTF("OUT[%d] below UV threshold!\r\n",i+1);
					/* can报文发送电源芯片对应通道的欠压信息 */
					can_trans_err_info = OUT_UV + i;
					can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
				} else if (diag_register == STATOV){
					EMBARC_PRINTF("OUT[%d] above OV threshold!\r\n",i+1);
					/* can报文发送电源芯片对应通道的过压信息 */
					can_trans_err_info = OUT_OV + i;
					can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
				} else {
					EMBARC_PRINTF("OUT[%d] below OFF threshold!\r\n",i+1);
					/* can报文发送电源芯片对应通道的OFF comparator error信息  */
					can_trans_err_info = OUT_OFF_ERR + i;
					can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
				}
			}
		}
		break;
	case STATD:
		/* 判断是否为power on reset */
		por_flag = diag_max20430a_result & POR_OCCURRED;

		/* 判断RESETB是否在内部bist测试期间发生错误 */
		if(diag_max20430a_result & REST_ERR_OCCURRED) {
			result &= false;
			can_trans_err_info = RSTERR;
			can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
		}

		/* 判断电源芯片是否由于过热导致关机 */
		if(diag_max20430a_result & THERMAL_SHUTDOWN_OCCURRED) {
			result &= false;
			can_trans_err_info = THSD;
			can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);

		}

		/* 判断是否有内部错误发生(OTP CRC failure, OV/UV comparator test failure) */
		if(diag_max20430a_result & INTERR_OCCURRED) {
			result &= false;
			can_trans_err_info = INTERR;
			can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);

		}
		break;
	case STATWD:
		if(diag_max20430a_result & WD_LFSR_OCCURRED) {
			result &= false;
			EMBARC_PRINTF("LFSR Write Mismatch!\r\n");
			can_trans_err_info = WD_LFSR;
			can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
		}

		if(diag_max20430a_result & WD_UV_OCCURRED) {
			result &= false;
			can_trans_err_info = WD_UV;
			EMBARC_PRINTF("Watchdog Update Violation!\r\n");
			can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
		}

		if(diag_max20430a_result & WD_EXP_OCCURRED) {
			result &= false;
			can_trans_err_info = WD_EXP;
			EMBARC_PRINTF("Watchdog Open Window Expired!\r\n");
			can_transfer_error_info(max20430a_error_can_frame_id,can_trans_err_info);
		}
		break;
	default:
		break;
	}

	return result;
}









