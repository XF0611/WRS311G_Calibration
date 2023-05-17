#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "flash.h"
#include "ads7029.h"
#include "misc_tool.h"
#include "max20430a_operation.h"
#include "max20430a.h"
#include "ads7029.h"
#include "tcan.h"
#include "flash.h"
#include "flash_mmap.h"
#include "calterah_limits.h"
#include "uds_misc.h"


typedef struct {
	uint8_t state;
       uint32_t flash_base;
	uint32_t mem_base;
	uint32_t total_size;
	uint32_t received_size;
} can_uds_download_t;

#define CAN_UDS_XFER_BLOCK_LEN		(0x100 + 2)
static can_uds_download_t uds_download;
static uint32_t uds_mem_cur_addr;


void set_flash_addr(uint32_t addr)
{
	uds_download.flash_base = addr;
}

uint32_t get_flash_addr(void)
{
	return uds_download.flash_base;
}

static uint32_t busdata2word(uint8_t *data, uint8_t len)
{
	uint32_t word = 0;
	while (len) {
		word |= (*data++ << ((len - 1) << 3));
		len -= 1;
	}
	return word;
}



void uds_ecu_routine_ctrl(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{
	uint16_t routine_flag;
	uint32_t flash_addr;
	uint16_t write_times;
	txdata->data[0] = rxdata->data[0] + 0x40;
	txdata->data[1] = rxdata->data[1];
	txdata->data[2] = rxdata->data[2];
	txdata->data[3] = rxdata->data[3];
	txdata->length = 4;
	routine_flag = ((uint16_t)(rxdata->data[2]) << 8) | rxdata->data[3];

	switch(routine_flag)
	{
	case flash_addr_set:
		flash_addr = ((uint32_t)(rxdata->data[7]) << 24) | ((uint32_t)(rxdata->data[6]) << 16) | ((uint32_t)(rxdata->data[5]) << 8) | rxdata->data[4];
		set_flash_addr(flash_addr);
		break;
	case flash_erase:
		flash_memory_erase(uds_download.flash_base,uds_download.total_size);
		break;
	case flase_program:

		flash_memory_write(uds_download.flash_base,(uint8_t *)(uds_download.mem_base),uds_download.total_size);
		break;
	}
}

void uds_ecu_read_data_by_id(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{

	float temperature = 0.0f;
	float supply_vol = 0.0f;
	uint8_t sendbuffer[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
	uint32_t Vol_data[2];
	uint8_t buf[2];
	uint32_t RF_ERR_STA;
	uint32_t DIG_ERR_STA;
	uint32_t int_global_info;
	uint32_t borad_id;
	uint8_t read_type_low = rxdata->data[1];
	uint8_t read_type_high = rxdata->data[2];

	uint16_t read_type = (((uint16_t)read_type_high) << 8) | read_type_low;
	EMBARC_PRINTF("read_type_low is %x\r\n",read_type_low);
	EMBARC_PRINTF("read_type_high is %x\r\n",read_type_high);
	EMBARC_PRINTF("read_type is %x\r\n",read_type);
	memset(txdata->data,0,sizeof(txdata->length));
	switch(read_type)
	{
	case read_board_id:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		flash_memory_read(Board_Id_Addr,&borad_id,4);
		EMBARC_PRINTF("read board id is %d\r\n",borad_id);
		txdata->data[3] = borad_id & 0xFF;
		txdata->data[4] = (borad_id >> 8) & 0xFF;
		txdata->data[5] = (borad_id >> 16) & 0xFF;
		txdata->data[6] = (borad_id >> 24) & 0xFF;
		txdata->length = 7;
		break;
	case read_version_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		txdata->data[3] = 1;
		txdata->data[4] = 2;
		txdata->data[5] = 3;
		txdata->data[6] = 4;
		txdata->data[7] = 5;
		txdata->data[8] = 6;
		txdata->data[9] = 7;
		txdata->data[10] = 8;
		txdata->data[11] = 9;
		txdata->length = 12;
		break;
	case read_radar_temp_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		temperature = fmcw_radio_rf_comp_code(NULL);
		EMBARC_PRINTF("temperature is %f\r\n",temperature);
		sendbuffer[0] = (uint8_t)(temperature * 10.0 + 500.0);
		sendbuffer[1] = (uint8_t)((uint16_t)(temperature * 10.0 + 500.0)>>8u);

		txdata->data[3] = sendbuffer[0];
		txdata->data[4] = sendbuffer[1];
		txdata->length = 5;
		break;

	case read_supply_voltage_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		ads7029_spi_read(1,&Vol_data[0],1);
		supply_vol = Vol_data[0] * ads_ref_voltage_scale * sample_coff;
		EMBARC_PRINTF("supply_vol is %fV\r\n",supply_vol);

		sendbuffer[0] = (uint8_t)(supply_vol * 10.0 + 500.0);
		sendbuffer[1] = (uint8_t)((uint16_t)(supply_vol * 10.0 + 500.0)>>8u);

		txdata->data[3] = sendbuffer[0];
		txdata->data[4] = sendbuffer[1];
		txdata->length = 5;
		break;

	case read_statd_register_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		read_max20430a(0,STATD,buf,1);
		EMBARC_PRINTF("STATD is %x\r\n",buf[0]);
		txdata->data[3] = buf[0];
		txdata->length = 4;
		break;
	case read_int_global_register_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		int_global_info = tcan_spi_read(0,Int_Global,register_len);
		EMBARC_PRINTF("Int_Global is %x\r\n",int_global_info);
		uint32_t int1_info = tcan_spi_read(0,Int_1,register_len);
		EMBARC_PRINTF("int1_info is %x\r\n",int1_info);
		uint32_t int2_info = tcan_spi_read(0,Int_2,register_len);
		EMBARC_PRINTF("int2_info is %x\r\n",int2_info);
		uint32_t int3_info = tcan_spi_read(0,Int_3,register_len);
		EMBARC_PRINTF("int3_info is %x\r\n",int3_info);

		/* clear can slient flag */
		if(int1_info & 0x4)
		{
			uint32_t can_slient_clear = int1_info | (1 << 2);
			tcan_spi_write(0,Int_1,can_slient_clear,register_len);
			int_global_info = tcan_spi_read(0,Int_Global,register_len);
		}


		txdata->data[3] = int_global_info & 0xFF;
		txdata->data[4] = (int_global_info >> 8) & 0xFF;
		txdata->data[5] = (int_global_info >> 16) & 0xFF;
		txdata->data[6] = (int_global_info >> 24) & 0xFF;
		txdata->length = 7;
		break;
	case read_rf_err_register_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		RF_ERR_STA = raw_readl(0xB00000 + 0x32C);
		EMBARC_PRINTF("RF_ERR_STA is %x\r\n",RF_ERR_STA);
		txdata->data[3] = RF_ERR_STA & 0xFF;
		txdata->data[4] = (RF_ERR_STA >> 8) & 0xFF;
		txdata->data[5] = (RF_ERR_STA >> 16) & 0xFF;
		txdata->data[6] = (RF_ERR_STA >> 24) & 0xFF;
		txdata->length = 7;
		break;
	case read_dig_err_register_info:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = read_type_low;
		txdata->data[2] = read_type_high;
		DIG_ERR_STA = raw_readl(0xB00000 +  0x42C);
		EMBARC_PRINTF("DIG_ERR_STA is %x\r\n",DIG_ERR_STA);
		txdata->data[3] = DIG_ERR_STA & 0xFF;
		txdata->data[4] = (DIG_ERR_STA >> 8) & 0xFF;
		txdata->data[5] = (DIG_ERR_STA >> 16) & 0xFF;
		txdata->data[6] = (DIG_ERR_STA >> 24) & 0xFF;
		txdata->length = 7;
		break;

	default:
		txdata->length = 0;
		break;
	}
}

int32_t ang = 0;
float rng_min = 0.5;
float rng_max = 50.0;
uint32_t calib_cnt = 0;
uint32_t rng_index = 0;
uint32_t frame_period = 0;

typedef struct {
	float ang;
	float rng_min;
	float rng_max;
	uint32_t calib_cnt;
	uint32_t rng_index;
	uint32_t frame_period;
}calib_params;

calib_params params = {0,0.5,50.0,0,0,0};


static bool para_angle = false;
static bool para_rang_min = false;
static bool para_rang_max = false;
static bool para_calib_times = false;
static bool para_rng_index = false;
static bool para_frame_period = false;



extern uint32_t times_cnt;
extern float test[10];
extern float calib_data[4097];
extern float real_range_data;
void uds_ecu_read_mem_by_addr(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{
	sensor_config_t *cfg = sensor_config_get_cur_cfg();
	float *pos_data;
	float *comp_data;
	pdu_t pdu;
	//uint32_t oper_type = ((uint32_t)rxdata->data[5] << 24) | ((uint32_t)rxdata->data[4] << 16) | ((uint32_t)rxdata->data[3] << 8) | rxdata->data[2];
	uint32_t oper_type = ((uint32_t)rxdata->data[2] << 24) | ((uint32_t)rxdata->data[3] << 16) | ((uint32_t)rxdata->data[4] << 8) | rxdata->data[5];
	EMBARC_PRINTF("oper_type is %x\r\n",oper_type);
	EMBARC_PRINTF("rxdata0 is %d\r\n",rxdata->data[0]);

	switch(oper_type)
	{
	case ant_pos:
		memset(txdata->data,0,sizeof(MAX_ANT_ARRAY_SIZE * 4 + 1));
		txdata->data[0] = rxdata->data[0] + 0x40;
		memcpy(&txdata->data[1],cfg->ant_pos,MAX_ANT_ARRAY_SIZE * 4);
		txdata->length = MAX_ANT_ARRAY_SIZE * 4 + 1;

		break;
	case ant_comp:
		memset(txdata->data,0,sizeof(MAX_ANT_ARRAY_SIZE_SC * 4 + 1));
		txdata->data[0] = rxdata->data[0] + 0x40;
		memcpy(&txdata->data[1],cfg->ant_comps,MAX_ANT_ARRAY_SIZE_SC * 4);
		txdata->length = MAX_ANT_ARRAY_SIZE_SC * 4 + 1;
		break;
	case ant_ang:

		break;
	case ant_rng_min:

		break;
	case ant_rng_max:

		break;
	case calib_times:

		break;
	case ant_rng_inedx:

		break;
	case calib_frame_period:
		break;
	case calib_read_rng:
		txdata->data[0] = rxdata->data[0] + 0x40;
		baseband_t *bb = baseband_frame_interleave_recfg();
		real_range_data = params.rng_index*bb->sys_params.rng_delta;
		EMBARC_PRINTF("real_range_data is %f.\r\n",real_range_data);
		memcpy(&txdata->data[1],&real_range_data,4);
		txdata->length = 5;
		break;
	case calib_read_phase_pow:
		txdata->data[0] = rxdata->data[0] + 0x40;
		if(para_angle && para_rang_min && para_rang_max && para_calib_times && para_rng_index && para_frame_period)
		{
			EMBARC_PRINTF("parametes all ok\r\n");
			//memset(txdata->data,0,4097 * 4);
			//uds_ant_calib(params.ang,params.rng_min,params.rng_max,params.calib_cnt,params.rng_index,params.frame_period);
			ant_calib_uds(params.ang,params.rng_min,params.rng_max,params.calib_cnt,params.rng_index,params.frame_period);
			//memcpy((float *)&txdata->data[1],calib_data,times_cnt * 4);

			memmove((float *)&txdata->data[1], &calib_data[0], times_cnt * 4);
			txdata->length = times_cnt * 4 + 1;
			EMBARC_PRINTF("calib final send bytes number is %d\r\n", txdata->length);
			para_angle = false;
			para_rang_min = false;
			para_rang_max = false;
			para_calib_times = false;
			para_rng_index = false;
			para_frame_period = false;

		}else
		{
			EMBARC_PRINTF("parametes not ok\r\n");
			txdata->length = 0;
		}
		break;
	default:
		txdata->length = 0;
		break;
	}
}

void uds_ecu_write_mem_by_addr(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{
	sensor_config_t *cfg = sensor_config_get_cur_cfg();
	//uint32_t oper_type = ((uint32_t)rxdata->data[5] << 24) | ((uint32_t)rxdata->data[4] << 16) | ((uint32_t)rxdata->data[3] << 8) | rxdata->data[2];
	uint32_t oper_type = ((uint32_t)rxdata->data[2] << 24) | ((uint32_t)rxdata->data[3] << 16) | ((uint32_t)rxdata->data[4] << 8) | rxdata->data[5];
	EMBARC_PRINTF("oper_type is %x\r\n",oper_type);
	//EMBARC_PRINTF("enter uds_ecu_write_mem_by_addr\r\n");

	switch(oper_type)
	{
	case ant_pos:
		EMBARC_PRINTF("ant pos receive data len is %d\r\n",rxdata->length);
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		flash_memory_erase(cfg->ant_info_flash_addr,(MAX_ANT_ARRAY_SIZE + 1 ) * 4);
		uint32_t ant_pos_magicNum = ANTENNA_INFO_MAGICNUM;
		flash_memory_write(cfg->ant_info_flash_addr,&ant_pos_magicNum,4);
		flash_memory_write(cfg->ant_info_flash_addr + 4,&rxdata->data[10],MAX_ANT_ARRAY_SIZE * 4);
		txdata->length = 10;
		break;
	case ant_comp:
		EMBARC_PRINTF("ant comp receive data len is %d\r\n",rxdata->length);
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		flash_memory_erase(cfg->ant_info_flash_addr + ANTENNA_INFO_LEN,(MAX_ANT_ARRAY_SIZE_SC + 1 )* 4);
		uint32_t ant_comp_magicNum = ANTENNA_INFO_MAGICNUM;
		flash_memory_write(cfg->ant_info_flash_addr + ANTENNA_INFO_LEN,&ant_comp_magicNum,4);
		flash_memory_write(cfg->ant_info_flash_addr + ANTENNA_INFO_LEN + 4,&rxdata->data[10],MAX_ANT_ARRAY_SIZE_SC * 4);
		txdata->length = 10;
		break;
	case ant_ang:
		;
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		//params.ang = ((int32_t)rxdata->data[13] << 24) | ((int32_t)rxdata->data[12] << 16) | ((int32_t)rxdata->data[11] << 8) | (int32_t)rxdata->data[10];
		int angle = ((int)rxdata->data[13] << 24) | ((int)rxdata->data[12] << 16) | ((int)rxdata->data[11] << 8) | (int)rxdata->data[10];
		params.ang = *(float *)&angle;
		para_angle = true;
		EMBARC_PRINTF("calib angle is %f\r\n",params.ang);
		txdata->length = 10;
		break;
	case ant_rng_min:
		;
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		uint32_t range_min = ((uint32_t)rxdata->data[13] << 24) | ((uint32_t)rxdata->data[12] << 16) | ((uint32_t)rxdata->data[11] << 8) | (uint32_t)rxdata->data[10];
		params.rng_min = *(float *)&range_min;
		para_rang_min = true;
		EMBARC_PRINTF("calib range min is %f\r\n",params.rng_min);
		txdata->length = 10;
		break;
	case ant_rng_max:
		;
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		uint32_t range_max = ((uint32_t)rxdata->data[13] << 24) | ((uint32_t)rxdata->data[12] << 16) | ((uint32_t)rxdata->data[11] << 8) | (uint32_t)rxdata->data[10];
		params.rng_max = *(float *)&range_max;
		para_rang_max = true;
		EMBARC_PRINTF("calib range min is %f\r\n",params.rng_max);
		txdata->length = 10;
		break;
	case calib_times:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		params.calib_cnt = ((uint32_t)rxdata->data[13] << 24) | ((uint32_t)rxdata->data[12] << 16) | ((uint32_t)rxdata->data[11] << 8) | (uint32_t)rxdata->data[10];
		para_calib_times = true;
		EMBARC_PRINTF("calib_times is %d\r\n",params.calib_cnt);
		txdata->length = 10;
		break;
	case ant_rng_inedx:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		params.rng_index = ((uint32_t)rxdata->data[13] << 24) | ((uint32_t)rxdata->data[12] << 16) | ((uint32_t)rxdata->data[11] << 8) | (uint32_t)rxdata->data[10];
		para_rng_index = true;
		EMBARC_PRINTF("ant_rng_inedx is %d\r\n",params.rng_index);
		txdata->length = 10;
		break;
	case calib_frame_period:
		txdata->data[0] = rxdata->data[0] + 0x40;
		txdata->data[1] = rxdata->data[1];
		txdata->data[2] = rxdata->data[2];
		txdata->data[3] = rxdata->data[3];
		txdata->data[4] = rxdata->data[4];
		txdata->data[5] = rxdata->data[5];
		txdata->data[6] = rxdata->data[6];
		txdata->data[7] = rxdata->data[7];
		txdata->data[8] = rxdata->data[8];
		txdata->data[9] = rxdata->data[9];
		params.frame_period = ((uint32_t)rxdata->data[13] << 24) | ((uint32_t)rxdata->data[12] << 16) | ((uint32_t)rxdata->data[11] << 8) | (uint32_t)rxdata->data[10];
		para_frame_period = true;
		txdata->length = 10;
		break;
	default:
		txdata->length = 0;
		break;
	}
}

void uds_ecu_write_data_by_id(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{
	uint8_t type_low = rxdata->data[1];
	uint8_t type_high = rxdata->data[2];
	uint16_t type = (((uint16_t)type_high) << 8) | type_low;
	EMBARC_PRINTF("write_type is %x\r\n",type);
	uint32_t id_num;
	switch(type)
	{
	case write_board_id:
		id_num = rxdata->data[3] | ((uint16_t)rxdata->data[4]) << 8;
		EMBARC_PRINTF("id_num is %d\r\n",id_num);

		flash_memory_erase(Board_Id_Addr,4096);
		int32_t res = flash_memory_write(Board_Id_Addr,&id_num,4);
		EMBARC_PRINTF("write memory result is %d\r\n",res);
		txdata->length = 0;
		break;
	default:
		txdata->length = 0;
		break;
	}
}

void can_uds_request_download(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{
	uint8_t rsp_code = 0xFF;
	//uint8_t dfi = 0;
	uint8_t alfid = 0;
	uint8_t mem_addr_len, mem_size_len;
	uint32_t mem_addr, mem_size;

	do {
		if ((NULL == rxdata) || (NULL == txdata)) {
			/* what happened? */
			break;
		}

		//dfi = rxdata->data[1];
		alfid = rxdata->data[2];
		mem_addr_len = alfid & 0xF;
		mem_size_len = (alfid >> 4) & 0xF;

		if (mem_addr_len + mem_size_len + 3 < rxdata->length) {
			/* incorrectMessageLengthOrInvalidFormat */
			rsp_code = 0x13;
			break;
		}

		if ((mem_addr_len > 4) || (mem_size_len > 4)) {
			/* Error: return NRC. */
			rsp_code = 0x31;
			break;
		}

		mem_addr = busdata2word(&rxdata->data[3], mem_addr_len);
		mem_size = busdata2word(&rxdata->data[3 + mem_addr_len], mem_size_len);
		if ((mem_addr < 0x770000) || (mem_addr + mem_size >= 0x7f0000)) {
			/* Error: return NRC. */
			rsp_code = 0x31;
		}

		uds_download.mem_base = mem_addr;
		uds_download.total_size = mem_size;
		uds_download.received_size = 0;
		uds_download.state = 1;

		rsp_code = 0;

		/* filling response. */
		txdata->data[0] = 0x74;
		txdata->data[1] = 3;
		txdata->data[2] = (CAN_UDS_XFER_BLOCK_LEN >> 16) & 0xFF;
		txdata->data[3] = (CAN_UDS_XFER_BLOCK_LEN >> 8) & 0xFF;
		txdata->data[4] = CAN_UDS_XFER_BLOCK_LEN & 0xFF;
		txdata->length = 5;

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		txdata->data[0] = 0x7F;
		txdata->data[1] = 0x34;
		txdata->data[2] = rsp_code;
		txdata->length = 3;
	}
}

static void trace_info(uint32_t value)
{
	static uint32_t idx = 0;
	raw_writel(0x7d0000 + (idx++ << 2), value);

}

void uds_ecu_transfer_data(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
	uint32_t cur_xfer_size, remain_size;
	txdata->data[0] = rxdata->data[0] + 0x40;
	txdata->data[1] = rxdata->data[1];
	txdata->length = 2;

	remain_size = uds_download.total_size - uds_download.received_size;
	if (remain_size < CAN_UDS_XFER_BLOCK_LEN - 2) {
		cur_xfer_size = remain_size;
	} else {
		cur_xfer_size = CAN_UDS_XFER_BLOCK_LEN - 2;
	}

	uds_mem_cur_addr = uds_download.mem_base + uds_download.received_size;

	memmove((uint8_t *)uds_mem_cur_addr,&rxdata->data[2],cur_xfer_size);
	uds_download.received_size += cur_xfer_size;
}
void can_uds_transfer_data(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
	uint8_t rsp_code = 0xFF;

	static uint8_t bsc = 1;

	uint32_t cur_xfer_size, remain_size;
       /* like sequence number */
	uint8_t cur_bsc = bsc;
	uint8_t *dbuf = NULL;

	do {
		if ((NULL == rxdata) || (NULL == txdata)) {
			break;
		}

		remain_size = uds_download.total_size - uds_download.received_size;
		if (remain_size < CAN_UDS_XFER_BLOCK_LEN - 2) {
			cur_xfer_size = remain_size;
		} else {
			cur_xfer_size = CAN_UDS_XFER_BLOCK_LEN - 2;
		}

		if ((rxdata->length > CAN_UDS_XFER_BLOCK_LEN) || \
		    ((rxdata->length < CAN_UDS_XFER_BLOCK_LEN) && \
		     (rxdata->length != remain_size + 2))) {
			rsp_code = 0x13;
			break;
		}

		if ((uds_download.received_size >= uds_download.total_size) || \
		    (0 == uds_download.state)) {
			rsp_code = 0x24;
			break;
		}

		if (rxdata->data[1] != bsc) {
			rsp_code = 0x73;
			break;
		} else {
			bsc++;
		}
               /* specify the address in flash */
		dbuf = (uint8_t *)(uds_download.flash_base + uds_download.received_size);
		/*
		for (; idx < cur_xfer_size; idx++) {
			dbuf[idx] = rxdata->data[2 + idx];
		}
		*/
		memcpy(dbuf, &rxdata->data[2], cur_xfer_size);
		dcache_flush();

		if (0 == flash_memory_write((uint32_t)dbuf, &rxdata->data[2], cur_xfer_size)) {
			uds_download.received_size += cur_xfer_size;

			rsp_code = 0;
			txdata->data[0] = 0x76;
			txdata->data[1] = cur_bsc;
			txdata->length = 2;
		} else {
			rsp_code = 0x72;
			raw_writel(0xb30000, 0xde);
			raw_writel(0xb30000, 0xad);
			break;
		}
		/*After processing the last data, bsc set 1*/
               if(remain_size <= CAN_UDS_XFER_BLOCK_LEN - 2){
                      bsc =1;
		}
	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		txdata->data[0] = 0x7F;
		txdata->data[1] = 0x36;
		txdata->data[2] = rsp_code;
		txdata->length = 3;
	}
}

void uds_ecu_transfer_exit(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
	txdata->data[0] = rxdata->data[0] + 0x40;
	txdata->length = 1;
}

void can_uds_transfer_exit(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
	uint8_t rsp_code = 0xFF;

	do {
		if ((NULL == rxdata) || (NULL == txdata)) {
			break;
		}

		uds_download.state = 0;

		rsp_code = 0;

		txdata->data[0] = 0x77;
		txdata->length = 1;

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		txdata->data[0] = 0x7F;
		txdata->data[1] = 0x37;
		txdata->data[2] = rsp_code;
		txdata->length = 3;
	}
}
