/*
 * misc_tool.c
 *
 *  Created on: 2019Ae9OA20EO
 *      Author: xerxes
 */



#include "embARC_toolchain.h"
#include "embARC_assert.h"
#include "embARC.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"
#include "alps_hardware.h"
#include "clkgen.h"
#include "mux.h"
#include "sensor_config.h"
#include "baseband_reg.h"
#include "baseband_hw.h"
#include "baseband.h"
#include "radio_ctrl.h"
#include "calterah_limits.h"
#include "dbg_gpio_reg.h"
#include "baseband_task.h"

#include "misc_tool.h"
#include "rtwtypes.h"
#include "baseband_task.h"
#include "target_doa.h"
#include "CalVariable.h"
#include "can_transceiver.h"
#include "max20430a_operation.h"
#include "tcan.h"
#include "can_if.h"


//#include "TagtTrc_Walker.h"
//#include "BSD.h"
//#include "TagtTrc_dispIndex.h"

extern QueueHandle_t queue_doa_finish;
extern QueueHandle_t queue_fft1d_finish;
extern int32_t can_send_data(uint32_t bus_id, uint32_t frame_id, uint32_t *data, uint32_t len);

#define SYS_ENA(MASK, var) (var << SYS_ENABLE_##MASK##_SHIFT)
#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)
#define BB_READ_REG(bb_hw, RN) baseband_read_reg(bb_hw, BB_REG_##RN)



int16_t  Chirp_num=0;
int16_t  Chn_num=0;
int16_t  index_Start=0;
int16_t  index_End=0;
real32_T Ego_CarSpd;

uint8_t  RadarCfg_OutputType=0;//TRACKER_MODE;
uint8_t  Cluster_ID[MAX_DOA_CNT]={0};
uint8_t  Cluster_DynProp[MAX_DOA_CNT]={0};
uint8_t  Object_ID_60B[MAX_DOA_CNT]={0};
uint8_t  Object_ID_60D[MAX_DOA_CNT]={0};
uint8_t  Object_DynProp[MAX_DOA_CNT]={0};
uint8_t  RadarCfg_RadarPower=0;
uint8_t  Cluster_NofClustersNear=0;
uint8_t  Cluster_NofClustersFar=0;
uint16_t Cluster_MeasCounter=0;
uint8_t  Cluster_InterfaceVersion=0;
uint8_t  Object_NofObject=0;
uint16_t Object_MeasCounter=0;
uint8_t  Object_InterfaceVersion=1;

real32_T Object_ArelLat[MAX_DOA_CNT*2]={0};
uint8_t  Object_Class[MAX_DOA_CNT]={0};
uint8_t  Object_Length[MAX_DOA_CNT]={0};
uint8_t  Object_Width[MAX_DOA_CNT]={0};
uint8_t  RadarCfg_SendExtInfo=1;
uint8_t  RadarCfg_OutputType_vaild=0;
uint8_t  RadarCfg_SendExtInfo_vaild=0;
//uint16_t Ego_CarSpd=0;
uint8_t  radarcfg_flg=0;
uint8_t  ApolloSendSpeedflg=0;
extern int32_t frame_count;
extern SemaphoreHandle_t mutex_frame_count;
uint32_t ldac_flag = off;
extern SemaphoreHandle_t mutex_ldac_function;

int32_t can_send_data_TX(uint32_t frame_id, uint8_t *data, uint32_t len);
void CAN_Senddata_Supply_Vol(float Vol);
extern float calib_data[4097];
int32_t can_scan_data_RX(uint32_t frame_id, uint8_t *data, uint32_t len)
{

	int i;
	uint8_t buf[2] = {0};
	uint8_t err_data[8] = {0};
	bool result = true;
	uint32_t Vol_data[2] = {0};
	float supply_vol;
	pdu_t pdu;

	/* For FCT && EOL Test */
	int angle;
	if(frame_id == FUNC_SOFT_BUTTON)
	{
		if(data[0] == 1)
		{
			angle = 0 - data[1];
		}else
		{
			angle = data[1];
		}
		ant_calib_uds(angle,1,3,20,36,0);

		for(uint16_t j = 0; j < 20 * 8;j++)
		{
			EMBARC_PRINTF("receive calib data%d is %f\r\n",j,calib_data[j]);
		}
	}
	if (frame_id == FCT_EOL_RECEIVE)
	{
		uint8_t id_type = data[0];
		uint8_t id_cmd = data[1];

		switch(id_type)
		{
		case hand_shake:
			EMBARC_PRINTF("handshake success.\r\n");
			uint8_t hand_ack[8] = {0x00};
			hand_ack[0] = 1;
			pdu.data = hand_ack;
			pdu.length = 8;
			canif_transmit_new(0,&pdu,FCT_EOL_SEND);
			break;
		case bus_power_off:
			EMBARC_PRINTF("bus power off\r\n");
			switch_tcan_mode(Sleep_Mode_Cmd);
			break;
		case bus_power_wake:
			EMBARC_PRINTF("bus power wake\r\n");
			break;
		case read_radar_temp:
			EMBARC_PRINTF("enter fct temp get.\r\n");
			CAN_Senddata_Temperature_new();
			break;
		case read_supply_voltage:
			gpio_write(selfTestPin,low);
			ads7029_spi_read(1,&Vol_data[0],1);
			supply_vol = Vol_data[0] * ads_ref_voltage_scale * sample_coff;
			EMBARC_PRINTF("voltage is %f.\r\n",supply_vol);
			CAN_Senddata_Supply_Vol(supply_vol);
			break;
		case STATD_DIAG:
			EMBARC_PRINTF("enter STATD DIAG.\r\n");
			/* read max20430 statd*/
			read_max20430a(0,STATD,buf,1);
			err_data[0] = buf[0];
			pdu.data = err_data;
			pdu.length = 8;
			canif_transmit_new(0,&pdu,FCT_EOL_SEND);
			break;
		case INT_GLOBAL_DIAG:
			EMBARC_PRINTF("enter INT_GLOBAL DIAG.\r\n");
			/* read tcan INT Global */
			uint32_t int_global_info = tcan_spi_read(0,Int_Global,1);
			EMBARC_PRINTF("int_global_info is %d\r\n",int_global_info);
			uint32_t int1_info = tcan_spi_read(0,Int_1,register_len);
			EMBARC_PRINTF("int1_info is %x\r\n",int1_info);
			uint32_t int2_info = tcan_spi_read(0,Int_2,register_len);
			EMBARC_PRINTF("int2_info is %x\r\n",int2_info);
			uint32_t int3_info = tcan_spi_read(0,Int_3,register_len);
			EMBARC_PRINTF("int3_info is %x\r\n",int3_info);

			if(int1_info & 0x4)
			{
				uint32_t can_slient_clear = int1_info | (1 << 2);
				tcan_spi_write(0,Int_1,can_slient_clear,register_len);
				int_global_info = tcan_spi_read(0,Int_Global,register_len);
			}

			err_data[0] = int_global_info & 0xFF;
			pdu.data = err_data;
			pdu.length = 8;
			canif_transmit_new(0,&pdu,FCT_EOL_SEND);
			break;
		case RF_ERR_DIAG:
			EMBARC_PRINTF("enter RF ERR DIAG.\r\n");
			/* read radar EMU_RF_ERR_STA */
			uint32_t RF_ERR_STA = raw_readl(0xB00000 + 0x32C);
			EMBARC_PRINTF("RF_ERR_STA is %d\r\n",RF_ERR_STA);
			err_data[0] = RF_ERR_STA & 0xFF;
			err_data[1] = (RF_ERR_STA >> 8) & 0xFF;
			pdu.data = err_data;
			pdu.length = 8;
			canif_transmit_new(0,&pdu,FCT_EOL_SEND);
			break;
		case DIG_ERR_DIAG:
			EMBARC_PRINTF("enter DIG ERR DIAG.\r\n");
			/* read radar EMU_DIG_ERR_STA */
			uint32_t DIG_ERR_STA = raw_readl(0xB00000 + 0x42C);
			EMBARC_PRINTF("DIG_ERR_STA is %d\r\n",DIG_ERR_STA);
			err_data[0] = DIG_ERR_STA & 0xFF;
			err_data[1] = (DIG_ERR_STA >> 8) & 0xFF;
			err_data[2] = (DIG_ERR_STA >> 16) & 0xFF;
			pdu.data = err_data;
			pdu.length = 8;
			canif_transmit_new(0,&pdu,FCT_EOL_SEND);
			break;

		case read_selftest3V3:
			gpio_set_direct(fltReadPin,input);
			gpio_write(selfTestPin,high);
			chip_hw_mdelay(100);
			int32_t flt_readValue = gpio_read(fltReadPin);
			EMBARC_PRINTF("flt read value is %d\r\n",flt_readValue);
		    ads7029_spi_read(1,&Vol_data[0],1);
		    float selftest_value = Vol_data[0] * self_test_3V3_coff * ads_ref_voltage_scale;
		    EMBARC_PRINTF("selftest_3V3 is %fV\r\n",selftest_value);
		    CAN_Senddata_Supply_Vol(selftest_value);
		default:
			break;
		}
	}

	if (frame_id == ant_channel_switch)
	{
		sensor_config_t * cfg = sensor_config_get_cur_cfg();

		cfg->tx_groups[0] = data[0];
		cfg->tx_groups[1] = data[1];
		cfg->tx_groups[2] = data[2];
		cfg->tx_groups[3] = data[3];


	}

	if(frame_id == bb_task_ctrl)
	{
		if(data[0] == 1)
		{
	        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
	        frame_count = 0;
	        xSemaphoreGive(mutex_frame_count);
		}else if(data[0] == 0)
		{
	        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
	        frame_count = -1;
	        xSemaphoreGive(mutex_frame_count);
		}

	}

	if (frame_id == bb_interframe)
	{
		baseband_t* bb = baseband_get_rtl_frame_type();
		if (data[0] == 1)
		{
			EMBARC_PRINTF("bb_interframe single!\r\n");
			int_disable(INT_BB_SAM);
			baseband_interframe_power_save_enable(&bb->bb_hw, false);
		}else if (data[0] == 0)
		{
			EMBARC_PRINTF("bb_interframe block!\r\n");
			baseband_interframe_power_save_enable(&bb->bb_hw, true);
		}
	}

	if (frame_id == radio_single_tone)
	{
		double freq;
		baseband_t *bb = baseband_get_bb(0);

		EMBARC_PRINTF("data 0 is %d\r\n",data[0]);
		EMBARC_PRINTF("data 1 is %d\r\n",data[1]);
		EMBARC_PRINTF("data 2 is %d\r\n",data[2]);
		EMBARC_PRINTF("data 3 is %d\r\n",data[3]);
		if (data[0] == 1)
		{

			//freq = (double)(data[1] + data[2] / 255);
			freq = (double)data[1] + (double)(data[2] * 1.0 / 10 + (double)(data[3] * 1.00 / 100));
			EMBARC_PRINTF("freq is %f.\r\n",freq);
			fmcw_radio_single_tone(&bb->radio, freq, true);
		}else if (data[0] == 0)
		{
			fmcw_radio_single_tone(&bb->radio, freq, false);
		}
	}


	if (frame_id == FUNC_SOFT_BUTTON) {
		//EMBARC_PRINTF("ldac function!\r\n");
		/* change ldac function on/off */
		if (data[0] == SoftBtnSt) {
			//EMBARC_PRINTF("ldac function1!\r\n");
			xSemaphoreTake(mutex_ldac_function, portMAX_DELAY);
			ldac_flag = on;
			xSemaphoreGive(mutex_ldac_function);
		}
	}

	if(frame_id == FFT1D_ReadConfig)
	{
		index_End   = (data[7] << 8 | (data[6] )) & 0xFFFF;
		index_Start = (data[5] << 8 | (data[4] )) & 0xFFFF;
		Chn_num     = (data[3] << 8 | (data[2] )) & 0xFFFF;
		Chirp_num   = (data[1] << 8 | (data[0] )) & 0xFFFF;
//		EMBARC_PRINTF("FFT1D_ReadConfig=%d,%d,%d,%d,%d,%d,%d,%d\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
	}

//	EMBARC_PRINTF("Chirp_num = %d,%d,%d,%d \n", Chirp_num, Chn_num,index_Start,index_End);
	if(frame_id == RADARCFG)
	{
		RadarCfg_OutputType_vaild = (data[0]>>4) & 0x1;//bit[35:36] 32 33 34 35
		RadarCfg_SendExtInfo_vaild = (data[0]>>2) & 0x1;
		RadarCfg_OutputType =(data[4]>>3) & 0x3;
		RadarCfg_SendExtInfo= (data[5]>>4) & 0x1;

		radarcfg_flg=1;

//		EMBARC_PRINTF("RadarCfg= %d,%d,%d,%d\n",RadarCfg_OutputType_vaild,RadarCfg_SendExtInfo_vaild,RadarCfg_OutputType,RadarCfg_SendExtInfo);
	}

	if(frame_id == BH_DETE_1_CALI)
	{
		VS_uf_C[0] = data[0] & 0x3F;
		VS_uf_C[1] = data[1] & 0x3F;
		VS_uf_C[2] = data[2] & 0xFF;
		VS_uf_C[3] = data[3] & 0xFF;
		VS_uf_C[4] = data[4] & 0x7F;
		VS_uf_C[5] = data[5] & 0x7F;
		VS_uf_C[6] = data[6] & 0xF;
		VS_uf_C[7] = (data[6] >> 4) & 0xF;
		VS_uf_C[8] = data[7] & 0xFF;

//		EMBARC_PRINTF("VS_uf_C= %d,%d,%d,%d,%d,%d,%d,%d,%d\n",VS_uf_C[0],VS_uf_C[1],VS_uf_C[2],VS_uf_C[3],VS_uf_C[4],VS_uf_C[5],VS_uf_C[6],VS_uf_C[7],VS_uf_C[8]);
	}
	if(frame_id == BH_DETE_2_CALI)
	{
		VS_ff_C[0] = (float)(data[0] & 0x7F) / 10.0;
		VS_ff_C[1] = (float)(data[1] & 0x3F) / 10.0;
		VS_ff_C[2] = (float)(data[2] & 0x3F) / 10.0;
		VS_ff_C[3] = (float)(data[3] & 0x3F) / 10.0;
		VS_ff_C[4] = (float)(data[4] & 0x7F) / 10.0;
		VS_ff_C[5] = (float)((data[6] << 8 | data[5]) & 0x3FF) / 100.0;

//		EMBARC_PRINTF("VS_uf_C= %f,%f,%f,%f,%f,%f\n",VS_ff_C[0],VS_ff_C[1],VS_ff_C[2],VS_ff_C[3],VS_ff_C[4],VS_ff_C[5]);
	}
	if(frame_id == LO_DETE_CALI)
	{
		VOD_ThresData_M[0] = (float)(data[0] & 0x3F);
		VOD_ThresData_M[1] = (float)(data[1] & 0x3F);
		VOD_ThresData_M[2] = (float)(data[2] & 0xF);
		VOD_ThresData_M[3] = (float)((data[2] >> 4) & 0xF);
		VOD_ThresData_M[4] = (float)((data[4] << 8 | data[3]) & 0x1FF);
		VOD_ThresData_M[5] = (float)(data[5] & 0x7F);
		VOD_ThresData_M[6] = (float)(data[6] & 0x3F)/10.0;
//		VOD_ThresData_M[6] = (float)(data[6] & 0x3F) / 10.0;
		VOD_ThresData_M[7] = (float)(data[7] & 0xFF);
		EMBARC_PRINTF("ThresData_M= %f,%f,%f,%f,%f,%f,%f,%f\n",VOD_ThresData_M[0],VOD_ThresData_M[1],VOD_ThresData_M[2],VOD_ThresData_M[3],VOD_ThresData_M[4],VOD_ThresData_M[5],VOD_ThresData_M[6],VOD_ThresData_M[7]);
	}

	if(frame_id == CAR_INDOOR_CALIB)
	{
	  Car_Indoor_Calib[0] = (data[0] & 0xFF);
	  Car_Indoor_Calib[1] = (data[1] & 0xFF);
	  Car_Indoor_Calib[2] = (data[2] & 0xFF);
	  Car_Indoor_Calib[3] = (data[3] & 0xFF);
	  Car_Indoor_Calib[4] = (data[4] & 0xFF);
	  Car_Indoor_Calib[5] = (data[5] & 0xFF);
	  Car_Indoor_Calib[6] = (data[6] & 0xFF);
	  Car_Indoor_Calib[7] = (data[7] & 0xFF);
	  EMBARC_PRINTF("Car_Indoor_Calib = %x,%x,%x,%x,%x,%x,%x,%x\n",Car_Indoor_Calib[0],Car_Indoor_Calib[1],Car_Indoor_Calib[2],Car_Indoor_Calib[3],Car_Indoor_Calib[4],Car_Indoor_Calib[5],Car_Indoor_Calib[6],Car_Indoor_Calib[7]);
	}

	return 0;
}

int32_t can_send_data_TX(uint32_t frame_id, uint8_t *data, uint32_t len)
{
	int32_t result = 0;
	result = can_send_data(CAN_0_ID,frame_id, (uint32_t *)data,  len);

	return result;
}




static int16_t fft1d_senddata[4];
void Data_Convert(uint8_t index,complex_t *data_output)
{
	fft1d_senddata[0]=(int16_t)(data_output[index*2].i*10);
	fft1d_senddata[1]=(int16_t)(data_output[index*2].r*10);
	fft1d_senddata[2]=(int16_t)(data_output[index*2+1].i*10);
	fft1d_senddata[3]=(int16_t)(data_output[index*2+1].r*10);
}
extern uint64_t time1;
void CAN_Senddata_APP(complex_t *fft1d_data_output)
{
	uint8_t i=0;

	//can_send_data(FFT1D_OUTPUTVALUE0, &fft1d_senddata[0], 8);
	for(i=0;i<((index_End-index_Start+1+1)/2);i++)
	{

		Data_Convert(i,fft1d_data_output);
		can_send_data_TX(FFT1D_OUTPUTVALUE0+i, (uint8_t *)&fft1d_senddata[0], 8);
//		EMBARC_PRINTF("FFT1D_OUTPUTVALUE0\n");
	}

}
#define FMCW_TEMPERATURE  (0x6FFu)
void CAN_Senddata_Temperature(void)
{
  uint8_t sendbuffer[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  float temperature = 0.0f;
  temperature = fmcw_radio_rf_comp_code(NULL);
  sendbuffer[0] = (uint8_t)(temperature * 10.0 + 500.0);
  sendbuffer[1] = (uint8_t)((uint16_t)(temperature * 10.0 + 500.0)>>8u);
  EMBARC_PRINTF("FMCW Temperature is %f | sendbuffer[0] = %d\n", temperature, sendbuffer[0]);
  can_send_data_TX(FMCW_TEMPERATURE, &sendbuffer[0], 8);
}

void CAN_Senddata_Temperature_new(void)
{
	pdu_t pdu;
	uint8_t sendbuffer[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
	float temperature = 0.0f;
	temperature = fmcw_radio_rf_comp_code(NULL);
	sendbuffer[0] = (uint8_t)(temperature * 10.0 + 500.0);
	sendbuffer[1] = (uint8_t)((uint16_t)(temperature * 10.0 + 500.0)>>8u);
	EMBARC_PRINTF("FMCW Temperature is %f | sendbuffer[0] = %d sendbuffer[1] = %d\n", temperature, sendbuffer[0],sendbuffer[1]);


	pdu.data = sendbuffer;
	pdu.length = 8;
	canif_transmit_new(0,&pdu,FCT_EOL_SEND);
}

void CAN_Senddata_Supply_Vol(float Vol)
{
  pdu_t pdu;
  uint8_t sendbuffer[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  sendbuffer[0] = (uint8_t)(Vol * 10.0 + 500.0);
  sendbuffer[1] = (uint8_t)((uint16_t)(Vol * 10.0 + 500.0)>>8u);
  EMBARC_PRINTF("supply vol is %f | sendbuffer[0] = %d sendbuffer[1] = %d\n", Vol, sendbuffer[0],sendbuffer[1]);


	pdu.data = sendbuffer;
	pdu.length = 8;
	canif_transmit_new(0,&pdu,FCT_EOL_SEND);
}

void TxSetMsgValue_Conti_701(uint8_t num)
{
    //0x701 *****
	Cluster_ID[num] = num;

	if((DBF_datInfo_Vy[num] != 0) || (DBF_datInfo_Vx[num] != 0))
	{
		if(DBF_datInfo_Vy[num] < 0)
		{
			Cluster_DynProp[num] = 2;
		}
		else
		{
			Cluster_DynProp[num] = 0;
		}

	}
	else
	{
		Cluster_DynProp[num] = 1;
	}

	if(DBF_datInfo_Ry[num] < -250)//*0.1   -250 - 569.1
	{
		DBF_datInfo_Ry[num] = -250;
	}
	else if(DBF_datInfo_Ry[num] > 569.1)
	{
		DBF_datInfo_Ry[num] = 569.1;
	}
	else
	{
	  ;
	}

	if(DBF_datInfo_Rx[num] < -51.2)//*0.1   // -51.2 - 51.1
	{
		DBF_datInfo_Rx[num] = -51.2;
	}
	else if(DBF_datInfo_Rx[num] > 51.1)
	{
		DBF_datInfo_Rx[num] = 51.1;
	}
	else
	{
	  ;
	}

	if(DBF_datInfo_Vy[num] < -51.2)//*0.1    -51.2 - 51.1
	{
		DBF_datInfo_Vy[num] = -51.2;
	}
	else if(DBF_datInfo_Vy[num] > 51.1)
	{
		DBF_datInfo_Vy[num] = 51.1;
	}
	else
	{
	  ;
	}

	if(DBF_datInfo_Vx[num] < -25.6)//*0.1    -25.6 - 25.5
	{
		DBF_datInfo_Vx[num] = -25.6;
	}
	else if(DBF_datInfo_Vx[num] > 25.5)
	{
		DBF_datInfo_Vx[num] = 25.5;
	}
	else
	{
	  ;
	}

	if(DBF_datInfo_Amp[num] < -64)// 0.5    -64 - 63.5
	{
		DBF_datInfo_Amp[num] = -64;
	}
	else if(DBF_datInfo_Amp[num] > 63.5)
	{
		DBF_datInfo_Amp[num] = 63.5;
	}
	else
	{
	  ;
	}

}


void TagtTransmitTx(void)
{
    uint32_t Object_Information[6]   = {0};
    uint32_t RadarCfg_Information[2] = {0};
    //uint32_t Vehicle_SpeedRequest[2] = {0};
    uint32_t  BSD_Waringstate[2]     = {0};
    uint8_t  i = 0, j = 0;
    uint32_t Seat_Information[10]  = {0};
    uint32_t Phase_Information[10] = {0};
    uint32_t Live_Information = 0;

	if(RadarCfg_OutputType == DOA_MODE)//DOA_MODE)
	{
	   //0x600
		Object_Information[0] = (Cluster_NofClustersNear & 0xFF) | ((Cluster_NofClustersFar & 0xFF)<<8)
				                | ((Cluster_MeasCounter & 0xFFFF)<<16);
		Object_Information[1] = Cluster_InterfaceVersion & 0xF;

		can_send_data_TX(CLUSTER_0_STATUS, (uint8_t *)&Object_Information[0], 8);


		//0x701
		for(i = 0; i < MAX_DOA_CNT; i++)
		{
			if(DBF_datInfo_Ry[i] != 0)
			{
				TxSetMsgValue_Conti_701(i);

					Object_Information[2] = (Cluster_ID[i] & 0xFF)
									        | (((uint32_t)((DBF_datInfo_Ry[i] + CAN_Range_Long_Offset) / CAN_Range_Accuracy) & 0x1FFF) << 8)
									        | (((uint32_t)((DBF_datInfo_Rx[i] + CAN_Range_Lat_Offset) / CAN_Range_Accuracy) & 0x3FF) << 22);
					Object_Information[3] = ((uint32_t)((DBF_datInfo_Vy[i] + CAN_Velocity_Long_Offset) / CAN_Velocity_Accuracy) & 0x3FF)
									        | (((uint32_t)((DBF_datInfo_Vx[i] + CAN_Velocity_Lat_Offset) / CAN_Velocity_Accuracy) & 0x1FF) << 10)
									        | ((Cluster_DynProp[i] & 0x7) << 21)
									        | ((((uint32_t)((DBF_datInfo_Amp[i] + 64.0f) * 2) - RadarCfg_RadarPower * 3) & 0xFF) << 24);

					can_send_data_TX(CLUSTER_1_GENERAL, (uint8_t *)&Object_Information[2], 8);

		//			EMBARC_PRINTF("DBF_datInfo_Ry= %d , %f ,%f\n ", i, DBF_datInfo_Ry[i],DBF_datInfo_Rx[i]);
				}

		}
		//EMBARC_PRINTF("DBF_datInfo_Ry= %f, %f, %f\n ",DBF_datInfo_Ry[0],DBF_datInfo_Dst[0],DBF_datInfo_Ang[0]);
	}
	else if(RadarCfg_OutputType == TRACKER_MODE)
	{

	}
	if(radarcfg_flg)
	{
	    //0x201
	    RadarCfg_Information[1] = ((RadarCfg_OutputType & 0x3) << 12) | ((RadarCfg_SendExtInfo & 0x1) << 10);

	    can_send_data_TX(RADARSTATE, (uint8_t *)&RadarCfg_Information[0], 8);
	    radarcfg_flg = 0;
	}

	if(SingleCalc_Flag == 1)
	{
#if 0
		//0x640  �e�ƥ�?1�I�l�߸��H��
		Seat_Information[0] = (   VS_RspRateOut     & 0x3F)
							  | ((VS_HeartRateOut   & 0xFF ) << 8)
							  | ((VS_LeaveFlag      & 0xF  ) << 16)
							  | ((VS_stState        & 0xF  ) << 20);

		Seat_Information[1] = (uint32_t)(VS_lRange * 1000)  & 0x7FF;

		can_send_data_TX(FRONT_SEAT_1_INFO, (uint8_t *)&Seat_Information[0], 8);

	  //0x641  �e�ƥ�?1�ۦ�H��
		for(uint8_t i = 0; i < 13; i++ )
		{
			Phase_Information[0] = (((   VS_PhaseA_M[(i * 8)]     + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 1] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 2] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 3] + 5) * 10) & 0x7F) << 24);

			Phase_Information[1] = (((   VS_PhaseA_M[(i * 8) + 4] + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 5] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 6] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 7] + 5) * 10) & 0x7F) << 24);

			can_send_data_TX(FRONT_SEAT_1_PHASE, (uint8_t *)&Phase_Information[0], 8);
		}

	#if 0
		//0x642  �e�ƥ�?2�I�l�߸��H��
		Seat_Information[2] = (   VS_RspRateOut     & 0x3F)
							  | ((VS_HeartRateOut   & 0xFF ) << 8)
							  | ((VS_LeaveFlag      & 0xF  ) << 16)
							  | ((VS_stState        & 0xF  ) << 20);

		Seat_Information[3] = (uint32_t)(VS_lRange * 1000)  & 0x7FF;

		can_send_data_TX(FRONT_SEAT_2_INFO, (uint8_t *)&Seat_Information[2], 8);

		//0x643  �e�ƥ�?2�ۦ�H��
		for(uint8_t i = 0; i < 13; i++ )
		{
			Phase_Information[2] = (((   VS_PhaseA_M[(i * 8)]     + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 1] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 2] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 3] + 5) * 10) & 0x7F) << 24);

			Phase_Information[3] = (((   VS_PhaseA_M[(i * 8) + 4] + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 5] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 6] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 7] + 5) * 10) & 0x7F) << 24);

			can_send_data_TX(FRONT_SEAT_2_PHASE, (uint8_t *)&Phase_Information[2], 8);
		}

	   //0x644  �Z�ƥ�?3�I�l�߸��H��
		Seat_Information[4] = (   VS_RspRateOut     & 0x3F)
							  | ((VS_HeartRateOut   & 0xFF ) << 8)
							  | ((VS_LeaveFlag      & 0xF  ) << 16)
							  | ((VS_stState        & 0xF  ) << 20);

		Seat_Information[5] = (uint32_t)(VS_lRange * 1000)  & 0x7FF;

		can_send_data_TX(BACK_SEAT_3_INFO, (uint8_t *)&Seat_Information[4], 8);

		//0x645  �Z�ƥ�?3�ۦ�H��
		for(uint8_t i = 0; i < 13; i++ )
		{
			Phase_Information[4] = (((   VS_PhaseA_M[(i * 8)]     + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 1] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 2] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 3] + 5) * 10) & 0x7F) << 24);

			Phase_Information[5] = (((   VS_PhaseA_M[(i * 8) + 4] + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 5] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 6] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 7] + 5) * 10) & 0x7F) << 24);

			can_send_data_TX(BACK_SEAT_3_PHASE, (uint8_t *)&Phase_Information[4], 8);
		}

	  //0x646  �Z�ƥ�?4�I�l�߸��H��
		Seat_Information[6] = (   VS_RspRateOut     & 0x3F)
							  | ((VS_HeartRateOut   & 0xFF ) << 8)
							  | ((VS_LeaveFlag      & 0xF  ) << 16)
							  | ((VS_stState        & 0xF  ) << 20);

		Seat_Information[7] = (uint32_t)(VS_lRange * 1000)  & 0x7FF;

		can_send_data_TX(BACK_SEAT_4_INFO, (uint8_t *)&Seat_Information[6], 8);

		//0x647  �Z�ƥ�?4�ۦ�H��
		for(uint8_t i = 0; i < 13; i++ )
		{
			Phase_Information[6] = (((   VS_PhaseA_M[(i * 8)]     + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 1] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 2] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 3] + 5) * 10) & 0x7F) << 24);

			Phase_Information[7] = (((   VS_PhaseA_M[(i * 8) + 4] + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 5] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 6] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 7] + 5) * 10) & 0x7F) << 24);

			can_send_data_TX(BACK_SEAT_4_PHASE, (uint8_t *)&Phase_Information[6], 8);
		}

	 //0x648  �Z�ƥ�?5�I�l�߸��H��
		Seat_Information[8] = (   VS_RspRateOut     & 0x3F)
							  | ((VS_HeartRateOut   & 0xFF ) << 8)
							  | ((VS_LeaveFlag      & 0xF  ) << 16)
							  | ((VS_stState        & 0xF  ) << 20);

		Seat_Information[9] = (uint32_t)(VS_lRange * 1000)  & 0x7FF;

		can_send_data_TX(BACK_SEAT_5_INFO, (uint8_t *)&Seat_Information[8], 8);

		//0x649  �Z�ƥ�?5�ۦ�H��
		for(uint8_t i = 0; i < 13; i++ )
		{
			Phase_Information[8] = (((   VS_PhaseA_M[(i * 8)]     + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 1] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 2] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 3] + 5) * 10) & 0x7F) << 24);

			Phase_Information[9] = (((   VS_PhaseA_M[(i * 8) + 4] + 5) * 10) & 0x7F)
								   | ((((VS_PhaseA_M[(i * 8) + 5] + 5) * 10) & 0x7F) << 8)
								   | ((((VS_PhaseA_M[(i * 8) + 6] + 5) * 10) & 0x7F) << 16)
								   | ((((VS_PhaseA_M[(i * 8) + 7] + 5) * 10) & 0x7F) << 24);

			can_send_data_TX(BACK_SEAT_5_PHASE, (uint8_t *)&Phase_Information[8], 8);
		}
	#endif

#else
//		Live_Information= (   VOD_LivingPrescDet    & 0x1)
//						  | ((VOD_LivingPrescDet2   & 0x1 ) << 8);
//
//		can_send_data_TX(LIVE_OBJECT_INFO, (uint8_t *)&Live_Information, 8);
		EMBARC_PRINTF("misc_SingleCalc_Flag=%d\n",SingleCalc_Flag);
		Live_Information= (   VOD_LivingPrescDet    & 0x1)
						  | ((VOD_LivingPrescDet2   & 0x1 ) << 8)
						  | ((BudgetFlag   & 0xF ) << 16);
		EMBARC_PRINTF("budget flag is %d\r\n",BudgetFlag);
	    EMBARC_PRINTF("misc_Live_Information=%d\n",Live_Information);
		EMBARC_PRINTF("VOD output is:%d\n",Live_Information);
		//can_send_data_TX(LIVE_OBJECT_INFO, (uint8_t *)&Live_Information, 8);
#endif

		SingleCalc_Flag = 0;
	}
}

void power_on_check(void)
{
    uint32_t Vol_data[2];
    uint8_t buf[2];
    float selftest_3V3;

    ads7029_spi_read(1,&Vol_data[0],1);
    selftest_3V3 = Vol_data[0] * self_test_3V3_coff * ads_ref_voltage_scale;
    EMBARC_PRINTF("selftest_3V3 is %fV\r\n",selftest_3V3);

    read_tcan_device_id();
    read_max20430a(0,STATUV,buf,1);
    read_max20430a(0,STATOV,buf,1);
    read_max20430a(0,STATOFF,buf,1);
    read_max20430a(0,STATD,buf,1);

    /* set selftesthalf3v3 pin low */
    gpio_set_direct(selfTestPin,output);
    gpio_write(selfTestPin,low);

    /* set pwm1 io type is GPIO && input */
    uint32_t pwm1_io_type = raw_readl(0xBA0000 + 0x228);
    EMBARC_PRINTF("pwm1_io_type is %d\r\n",pwm1_io_type);
    raw_writel(0xBA0308,0);
    raw_writel(0xBA0228,0x4);
    gpio_set_direct(fltReadPin,input);



}



static TaskHandle_t monitor_task_handle = NULL;
static TaskHandle_t fft1d_temp_send_task_handle = NULL;
static TaskHandle_t radar_status_monitor_task_handle = NULL;
static TaskHandle_t test_task_handle = NULL;
extern int32_t initial_flag;
extern int32_t frame_count;
extern SemaphoreHandle_t mutex_frame_count;
extern SemaphoreHandle_t mutex_initial_flag;
void clr_doa_buffer(void);
void monitor_task(void *params)
{
	int ret;
	int buf[4];
	int cnt = 0;

    while(1)
    {

#if 0

		for( int i=0; i<64; i++)
		{
			buf[0] = buf[1] =i;
			(void)can_send_data(CAN_0_ID,0x601, buf, 8);
			(void)can_send_data(CAN_0_ID,0x602, buf, 8);
		}
		(void)can_send_data(CAN_0_ID,0x603, buf, 8);
		(void)can_send_data(CAN_0_ID,0x604, buf, 8);

#else
        if(cnt < 10)
        {
        	vTaskDelay(100);
        	cnt++;
        }
        if (cnt == 5)
		{
			baesband_frame_interleave_cnt_clr(); // clear recfg count in frame interleaving
			/* change initial flag */
	        xSemaphoreTake(mutex_initial_flag, portMAX_DELAY);
	        initial_flag = true;
	        xSemaphoreGive(mutex_initial_flag);
	        /* change frame number */
	        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
	        frame_count = -1;
	        xSemaphoreGive(mutex_frame_count);
	        EMBARC_PRINTF("-----------------start scan\r\n");
		}
        if(cnt > 8)//delay 8-5 frames to wait tracking work normal
        {
        	int finish_doa=0;
        	ret = xQueueReceive( queue_doa_finish, &finish_doa, portMAX_DELAY );
        	if(ret != pdPASS)
        	{
//        		EMBARC_PRINTF("r queue_doa_finish error!\n");
        	}
        	uint64_t time_1, time_2;
        	uint32_t time_delta;
        	time_1 = chip_get_cur_us();
//        	TagtTransmitTx();
        	clr_doa_buffer();
        	time_2 = chip_get_cur_us();
        	time_delta = time_2 - time_1;
//        	EMBARC_PRINTF("TagtTransmitTx costtime=%f us\n",time_delta);
//        	EMBARC_PRINTF("%s %d finish_doa=%d\n",__func__,__LINE__, finish_doa);
        }
#endif

        vTaskDelay(10);
    }
}

extern bool live_dection;
extern bool pmic_status;
#define temp_threshold 150
#define CRC_DIVSIOR 13
uint8_t get_checksum(uint8_t send_data)
{
	for(uint8_t i = 0; i < 8;i++) {

	}

	return 0;
}
bool radar_status(void)
{
	bool radar_failure = false;
	bool radar_overtemp = false;
	/* 获取温度 */
	float temp = fmcw_radio_get_temperature(NULL);
	//EMBARC_PRINTF("real temp is %f\r\n",temp);
	/* 与阈值作比较 */
	if (temp > (float)temp_threshold) {
		radar_overtemp = true;
		radar_failure = true;
	}

	/* 判断电源芯片电压情况 （过压、欠压）*/
	if (pmic_status == false) {
		radar_failure = true;
	}

	/* communction failure */

	/* TBD */

	return radar_failure;
}
void radar_status_monitor_task(void *params) {
	uint64_t can_send_time1,can_send_time2;
	uint32_t msg;
	uint32_t f_radar = 0;
	radar_output_signal *sig;
	bool radar_failure_dection = false;
	sig = (radar_output_signal *)&f_radar;
	while(1) {
		radar_failure_dection = radar_status();
		EMBARC_PRINTF("radar status is %d child status is %d\r\n",radar_failure_dection,live_dection);

		if (radar_failure_dection == true) {
			/* radar failure */
			sig->radar_status = 0x2;
			sig->alivecounter = 0;
			sig->checksum = 0;
			can_send_radar_status(sig);
		}else if (!radar_failure_dection && !live_dection) {
			/* radar init */
			sig->radar_status = 0x0;
			sig->alivecounter = 0;
			sig->checksum = 0;
			can_send_radar_status(sig);
		}else if (!radar_failure_dection && live_dection) {
			/* radar live dection */
			sig->radar_status = 0x1;
			sig->alivecounter = VOD_LivingPrescDet;
			sig->checksum = 0;
			can_send_radar_status(sig);
		}else {
			/* reserved situation */
			sig->radar_status = 0x3;
			sig->alivecounter = 0;
			sig->checksum = 0;
			can_send_radar_status(sig);
		}
		vTaskDelay(200);
	}

}
/*extern creal32_T fft1d_data[50];
extern QueueHandle_t queue_fft1d_temp_finish;
void fft1d_temp_send_task(void)
{
	uint32_t finish_fft1d_temp;
	while(1) {
		if (xQueueReceive(queue_fft1d_temp_finish, &finish_fft1d_temp, portMAX_DELAY) == pdTRUE) {
			EMBARC_PRINTF("enter can send fft1d!\r\n");
			CAN_Senddata_APP(fft1d_data);
			CAN_Senddata_Temperature();
		}
		vTaskDelay(10);
	}

}*/

void fft1d_temp_send_task(void)
{
	uint8_t fft1d_finish;
	while(1) {
		if (xQueueReceive( queue_fft1d_finish, &fft1d_finish, portMAX_DELAY ) == pdTRUE) {
			EMBARC_PRINTF("fft1d status is %d\r\n",fft1d_finish);
			CAN_Senddata_APP(fft1d_data);
		}

	}

}
int init_extern_task(void)
{

    if (xTaskCreate(monitor_task, "monitor_task", 1024, (void *)1, ((configMAX_PRIORITIES-4)), &monitor_task_handle)!= pdPASS) {
        EMBARC_PRINTF("create monitor_task error\r\n");
        return -1;
    }
    EMBARC_PRINTF("create monitor task ok\r\n");
/*    if (xTaskCreate(radar_status_monitor_task, "monitor radar status", 512, (void *)1, ((configMAX_PRIORITIES-2)), &radar_status_monitor_task_handle) != pdPASS) {
            EMBARC_PRINTF("create monitor radar status task error\r\n");
            return -1;
    }
    EMBARC_PRINTF("create radar status monitor task ok\r\n");*/
/*    if (xTaskCreate(fft1d_temp_send_task, "fft1d_temp_send_task", 1024, (void *)1, ((configMAX_PRIORITIES-2)), &fft1d_temp_send_task_handle)!= pdPASS) {
        EMBARC_PRINTF("create fft1d_temp_send_task error\r\n");
        return -1;
    }
    EMBARC_PRINTF("create fft1d_temp_send_task task ok\r\n");*/
    return 0;
}


