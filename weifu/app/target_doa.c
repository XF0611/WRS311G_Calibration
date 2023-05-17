/*
 * target_doa.c
 *
 *  Created on: 2019��9��24��
 *      Author: xerxes
 */

#include "target_doa.h"
#include "misc_tool.h"
#include "radar_sys_params.h"
#include "rtwtypes.h"
#include "CalVariable.h"
#include "VOD_LivingPrescDetGet_data.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_LivingPrescDetGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_ParameterBudget.h"
#include "baseband_cli.h"
#include "crc.h"

extern uint8_t  RadarCfg_OutputType;
extern QueueHandle_t queue_doa_finish;
doa_context_t doa_obj;

float DBF_datInfo_Ang[64];
float DBF_datInfo_Spd[64];
float DBF_datInfo_Dst[64];
float DBF_datInfo_Amp[64];
float DBF_datInfo_Rx[64];
float DBF_datInfo_Ry[64];
float DBF_datInfo_Vx[64];
float DBF_datInfo_Vy[64];
int   DBF_datInfo_Cnt;

t_None_u8_1 Bsd_Switch=1;         /* '<Root>/Bsd_Switch' */
t_None_u8_1 Ego_CarRghtlgt=0;     /* '<Root>/Ego_CarRghtlgt' */
t_None_u8_1 Ego_CarLeflgt=0;

uint8_t   SingleCalc_Flag = 0;
uint8_t   VOD_LivingPrescDet = 0;
uint8_t   VOD_LivingPrescDet2 = 0;
uint8_t   BudgetFlag=0;

void doa_init(baseband_t *bb)
{
    int index = bb->bb_hw.frame_type_id;
    doa_obj.radar_params[index] = &bb->sys_params;
}

void doa_sw_read(track_t *track)
{
    uint16_t i;
    void* sys_params = track->radar_params[baseband_get_cur_frame_type()];

    if (track->trk_update_header)
        track->trk_update_header(track->trk_data, sys_params);

    doa_obj.count = track->cdi_pkg.cdi_number;
    if(doa_obj.count >= MAX_DOA_CNT)
    {
//    	EMBARC_PRINTF("doa_obj.count >= MAX_DOA_CNT!\r\n");
    }
	
	EMBARC_PRINTF("DOA number is:%d\n", doa_obj.count);
    EMBARC_PRINTF("BK_START\n");
    for (i = 0; i < track->cdi_pkg.cdi_number; i++)
    {
        if(i >= MAX_DOA_CNT)
        {
        	doa_obj.count = i;
        	break;
        }

        doa_obj.target[i].sig = track->cdi_pkg.cdi[i].raw_z.sig;;
        doa_obj.target[i].noi = track->cdi_pkg.cdi[i].raw_z.noi;
#if (TRK_CONF_3D == 0)
        doa_obj.target[i].rng = track->cdi_pkg.cdi[i].raw_z.rng;
        doa_obj.target[i].vel = track->cdi_pkg.cdi[i].raw_z.vel;
        doa_obj.target[i].ang = track->cdi_pkg.cdi[i].raw_z.ang;
		if((bb_dump_data_info.doa_data_index != 0)&&(i < bb_dump_data_info.doa_data_index))
		{

		    EMBARC_PRINTF("\t%02d: P %3.2f, R %3.2f, V %3.2f, A %3.2f, E %3.2f\n",
                              i,
                              10*log10f(doa_obj.target[i].sig/doa_obj.target[i].noi),
                              doa_obj.target[i].rng,
                              doa_obj.target[i].vel,
                              doa_obj.target[i].ang,
                              0.0f);
		}
#else
        doa_obj.target[i].rng = track->cdi_pkg.cdi[i].raw_z.rng;
        doa_obj.target[i].vel = track->cdi_pkg.cdi[i].raw_z.vel;
        doa_obj.target[i].ang = track->cdi_pkg.cdi[i].raw_z.ang;
        doa_obj.target[i].ang_elv = track->cdi_pkg.cdi[i].raw_z.ang_elv;
		if((bb_dump_data_info.doa_data_index != 0)&&(i < bb_dump_data_info.doa_data_index))
		{
		    EMBARC_PRINTF("\t%02d: P %3.2f, R %3.2f, V %3.2f, A %3.2f, E %3.2f\n",
                              i,
                              10*log10f(doa_obj.target[i].sig/doa_obj.target[i].noi),
                              doa_obj.target[i].rng,
                              doa_obj.target[i].vel,
                              doa_obj.target[i].ang,
                              doa_obj.target[i].ang_elv);
		}
#endif

    }
    EMBARC_PRINTF("BK_END\n");

    for (i = 0; i < TRACK_NUM_TRK; i++) {
		if (track->trk_update_obj_info)
				track->trk_update_obj_info(track->trk_data, sys_params, i);
		if (track->output_obj.output) {
#if 0
#if (TRK_CONF_3D == 0)
			if(RadarCfg_OutputType == TRACKER_MODE)
			{
				TagtTrc_Trc_Dat_Amp[2*i+1] = track->output_obj.SNR;
				TagtTrc_Trc_Dat_Ry[2*i+1]  = track->output_obj.rng * cosf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Rx[2*i+1]  = track->output_obj.rng * sinf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Vy[2*i+1]  = track->output_obj.vel * cosf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Vx[2*i+1]  = track->output_obj.vel * sinf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Ang[2*i+1] = track->output_obj.ang;
			}
#else
			if(RadarCfg_OutputType == TRACKER_MODE)
			{
				TagtTrc_Trc_Dat_Amp[2*i+1] = track->output_obj.SNR;
				TagtTrc_Trc_Dat_Ry[2*i+1]  = track->output_obj.rng * cosf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Rx[2*i+1]  = track->output_obj.rng * sinf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Vy[2*i+1]  = track->output_obj.vel * cosf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Vx[2*i+1]  = track->output_obj.vel * sinf(track->output_obj.ang*3.14f/180.0f);
				TagtTrc_Trc_Dat_Ang[2*i+1] = track->output_obj.ang;
			}
#endif
#endif
		}
	}
}

unsigned char Frame_cnt2 = 0;
uint8_t VOD_windowstep_C = 10;
unsigned char  Frame_cnt_Parameter=0;
#define CAR_INDOOR_CALIB_INFO_BASE	(0x100000u + 0x40000u)
uint32_t Car_Indoor_Calib_mem_base = CAR_INDOOR_CALIB_INFO_BASE;
uint32_t Car_Indoor_Calib_mem_size = 0xBu;
int32_t Car_Indoor_Calib_Save(void)
{
  int32_t result = 0u;
  uint32_t ack   = 0u;
  float Car_Indoor_Calib_param[11];

  result = flash_memory_erase(Car_Indoor_Calib_mem_base, Car_Indoor_Calib_mem_size*4);
  if (0 != result)
  {
    ack = 0x0800U;
  }
  else
  {
    result = flash_memory_write(Car_Indoor_Calib_mem_base, (uint8_t *)(&VOD_ThresData_M[0]), Car_Indoor_Calib_mem_size*4);
    if (0 != result)
    {
	  ack = 0x0900U;
    }
	else
	{
	  result = flash_memory_read(Car_Indoor_Calib_mem_base, (uint8_t *)&Car_Indoor_Calib_param[0], 11*4);
	  /* Parament verify */
	  EMBARC_PRINTF("Car_Indoor_Calib_Done:Memory ROM = %f |%f |%f |%f |%f |%f |%f |%f |%f |%f |%f!\r\n",\
	  Car_Indoor_Calib_param[0], Car_Indoor_Calib_param[1], Car_Indoor_Calib_param[2],\
	  Car_Indoor_Calib_param[3], Car_Indoor_Calib_param[4], Car_Indoor_Calib_param[5],\
	  Car_Indoor_Calib_param[6], Car_Indoor_Calib_param[7], Car_Indoor_Calib_param[8],\
	  Car_Indoor_Calib_param[9], Car_Indoor_Calib_param[10]);
      //can send
	}
  }
  return ack;
}

typedef struct {
	uint8_t radar_monitor_status;
	uint8_t alive_counter;
	uint8_t radarEcu_Checksum;

}Radar2Domain_Message;
Radar2Domain_Message Rad2D_Message;

void Car_Indoor_Calib_Init(void)
{
  uint8_t i = 0u;
  int32_t result = 0;
  float Car_Indoor_Calib_param[11];

  result = flash_memory_read(Car_Indoor_Calib_mem_base, (uint8_t *)&Car_Indoor_Calib_param[0], 11*4);
  if(result != 0u)
  {
    EMBARC_PRINTF("Car Indoor Calib read flash is Error =%d \n",result);
  }
  else
  {
    if(*(uint32_t*)&Car_Indoor_Calib_param[0] != 0xFFFFFFFFu)
    {
	  /* Calibration Parament Read */
	  for(i=0; i < 11; i++)
	  {
	    VOD_ThresData_M[i] = Car_Indoor_Calib_param[i];
	  }
    }
    EMBARC_PRINTF("Car Indoor Calib read param is OK \n");
  }
  /* Calibration Parament Check */
  EMBARC_PRINTF("Car_Indoor_Calib_Init:VOD_ThresData_M = %f |%f |%f |%f |%f |%f |%f |%f |%f |%f |%f!\r\n",\
		                                                 VOD_ThresData_M[0],VOD_ThresData_M[1],VOD_ThresData_M[2],\
														 VOD_ThresData_M[3],VOD_ThresData_M[4],VOD_ThresData_M[5],\
														 VOD_ThresData_M[6],VOD_ThresData_M[7],VOD_ThresData_M[8],\
														 VOD_ThresData_M[9],VOD_ThresData_M[10]);
}

void Can_Send_Radar_Status(Radar2Domain_Message *message)
{
	uint8_t data[8] = {0x0};
	uint8_t radar_status = message->radar_monitor_status;
	uint8_t alive_counter = message->alive_counter;

	alive_counter = message->alive_counter;
	data[0] = (radar_status << 4) | (alive_counter & 0xF);
	data[1] = IOT_CRC8(&data[0],1);

	//Can_Send_Message(1,8,Alive_Dector_Info,&data[0]);
	can_send_data_TX(LIVE_OBJECT_INFO, (uint8_t *)&data[0], 8);
	message->alive_counter ++;
	if(message->alive_counter > 0x0Fu)
	{
		message->alive_counter = 0u;
	}
}

void target_doa_read(void)
{
	static uint64_t  time_last=0;
	uint64_t  time_current=0;
	uint32_t  time_delta;
	int32_t   ret              = 0;
	int doa_cnt = doa_obj.count;
    static uint8_t Frame_cnt = 0;
    static uint8_t diffFrame_cnt = 0;
    uint8_t  k = 1;
    float  VOD_DelFrm_C = VOD_ThresData_M[10];
//    uint8_t BudgetFlag=0;
//    EMBARC_PRINTF("06XXF =%d \n",Car_Indoor_Calib[0]);
    BudgetFlag=0;

  if(Car_Indoor_Calib[0] == 0x1u)
  { /* Car Indoor Calibration */
	   SingleCalc_Flag = 1;
	   VOD_LivingPrescDet = 0;
   	VOD_LivingPrescDet2 = 0;
	  BudgetFlag=VOD_ParameterBudget();
	  if (BudgetFlag==0)
	  {
	   EMBARC_PRINTF("wait&&&&&&&&&&&&&&&&&& =%.2f \n",VOD_ThresData_M);
	  }
	  if(BudgetFlag==1)
	  {
       EMBARC_PRINTF("VOD_ThresData_M update succeed  =%.2f \n",VOD_ThresData_M);
       /* Calibration Parament Save */
       ret = Car_Indoor_Calib_Save();
       if(ret != 0u)
       {
         EMBARC_PRINTF("Car Indoor Calib Save is Error =%d \n",ret);
       }
       else
       {
         EMBARC_PRINTF("Car Indoor Calib Save is Ok!\n");
       }
	   Car_Indoor_Calib[0] = 0x0u;
       EMBARC_PRINTF("111111106XXF =%d \n",Car_Indoor_Calib[0]);
       Frame_cnt_Parameter=0;
       Frame_cnt2 = 0;
       VOD_ParameterBudget_init();
	  }
	  if(BudgetFlag==2)
	  {
	   EMBARC_PRINTF("VOD_ThresData_M update failed =%.2f \n",VOD_ThresData_M);
	   Car_Indoor_Calib[0] = 0x0u;
	   EMBARC_PRINTF("222222206XXF =%d \n",Car_Indoor_Calib[0]);
	   Frame_cnt_Parameter=0;
	   Frame_cnt2 = 0;
	   VOD_ParameterBudget_init();
	  }
  }
  else
  {
#if 1
	if(doa_cnt >= MAX_DOA_CNT)
	{
		doa_cnt = MAX_DOA_CNT - 1;
	}
	BudgetFlag=3;
	time_current = chip_get_cur_us();
	 if  (diffFrame_cnt==1)
	 {
	        if (Frame_cnt2>=100+VOD_windowstep_C)
	        {
	            Frame_cnt2=VOD_windowstep_C;
	        }
	        VOD_D1FFTdataAbsDiffGet();
//	        EMBARC_PRINTF("diff_cnt=%d &&&&&&&&&&&&&&&7\n",Frame_cnt2);
	 }
	VOD_D1FFTDataGet();
//	EMBARC_PRINTF("Frame_cnt=%d &&&&&&&&&&&&&&&7\n",Frame_cnt);
	Frame_cnt++;
	Frame_cnt2++;
//	EMBARC_PRINTF("Frame_cnt=%d &&&&&&&&&&&&&&&7\n",Frame_cnt);
	if(Frame_cnt >= VOD_windowstep_C)
	{
		SingleCalc_Flag = 1;
		EMBARC_PRINTF("ThresData22= %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",VOD_ThresData_M[0],VOD_ThresData_M[1],VOD_ThresData_M[2],VOD_ThresData_M[3],VOD_ThresData_M[4],VOD_ThresData_M[5],VOD_ThresData_M[6],VOD_ThresData_M[7],VOD_ThresData_M[8],VOD_ThresData_M[9],VOD_ThresData_M[10]);
		VOD_LivingPrescDet=VOD_LivingPrescDetGet( VOD_ThresData_M);
		VOD_LivingPrescDet2=VOD_DetRsltShow(VOD_LivingPrescDet,VOD_DelFrm_C);
		Frame_cnt = 0;
		diffFrame_cnt = 1;
		VOD_LivingPrescDet = VOD_LivingPrescDet2;
		VOD_LivingPrescDet2 = 0;
		EMBARC_PRINTF("leaveDetection_convertC= %d,%d     ************\n",VOD_LivingPrescDet,VOD_LivingPrescDet2);
//		EMBARC_PRINTF("leaveLen= %d,%d     ************\n",leaveLen,convertLen);
	}
  }
	time_last = chip_get_cur_us();
	time_delta = time_last- time_current;
//	EMBARC_PRINTF("TagtTrc_step time=%d us\n",time_delta);

#endif
	Rad2D_Message.radar_monitor_status = VOD_LivingPrescDet;
	Can_Send_Radar_Status(&Rad2D_Message);
	TagtTransmitTx();
	int finish_doa = 1;
	ret = xQueueSendToBack( queue_doa_finish, &finish_doa, 0 );
	if(ret != pdPASS)
	{
//		EMBARC_PRINTF("s queue_doa_finish error!\n");
	}

	Object_MeasCounter++;
	if(Object_MeasCounter>=65535)
	{
		Object_MeasCounter=0;
	}
	EMBARC_PRINTF("SingleCalc_Flag=%d\n",SingleCalc_Flag);
}

void clr_doa_buffer(void)
{
	for(int i=0; i<MAX_DOA_CNT; i++)
	{
		DBF_datInfo_Dst[i] = 0;
		DBF_datInfo_Spd[i] = 0;
		DBF_datInfo_Ang[i] = 0;
		DBF_datInfo_Amp[i] = 0;
		DBF_datInfo_Rx[i] = 0;
		DBF_datInfo_Ry[i] = 0;
		DBF_datInfo_Vx[i] = 0;
		DBF_datInfo_Vy[i] = 0;
	}
}



