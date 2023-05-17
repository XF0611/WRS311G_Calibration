/*
 * target_doa.h
 *
 *  Created on: 2019Äê9ÔÂ24ÈÕ
 *      Author: x
 */

#ifndef _TARGET_DOA_H_
#define _TARGET_DOA_H_


#include "baseband.h"
#include "sensor_config.h"
#include "math.h"
#include "calterah_math.h"
#include "embARC_debug.h"
#include "calterah_limits.h"
#include "radio_ctrl.h"
#include <stdio.h>
#include "embARC.h"
#include "emu.h"
#include "clkgen.h"
#include "alps_dmu_reg.h"
#include "radio_reg.h"
#include "baseband_task.h"

#include <math.h>
#include <string.h>
#include "track.h"
#include "track_cli.h"
#include "ekf_track.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "can_signal_interface.h"
#include "EECU_Types.h"
//#include "DBSCAN_DotMerge.h"
//#include "DBSCAN_DotMerge_Cluster.h"
//#include "DBSCAN_DotMerge_Forklift.h"


enum CAN_PROTOCOL_MODE_{
	TRACKER_MODE = 0x1,
	DOA_MODE= 0x2,
};

#define CAN_Velocity_Accuracy		0.1
#define CAN_Range_Accuracy			0.1
#define CAN_Angle_Accuracy			0.5
#define CAN_RCS_Accuracy			0.33
#define CAN_Range_Lat_Offset	    51.2
#define CAN_Range_Long_Offset	    250.0
#define CAN_Velocity_Lat_Offset		25.6
#define CAN_Velocity_Long_Offset	51.2


/* use MAX_DOA_CNT in weifu code */
#define MAX_DOA_CNT		64

/* use doa_measu_t in weifu code */
typedef struct doa_measu_{
        float rng;       /* range    */
        float vel;       /* velocity */
        float ang;       /* angle    */
        float sig;       /* power    */
        float noi;
        float ang_elv;
        float sig_elv;
} doa_measu_t;

/* use doa_context_t in weifu code */
typedef struct doa_context_{
    radar_sys_params_t*      radar_params[MAX_FRAME_TYPE];
    int count;
    doa_measu_t target[MAX_DOA_CNT];
}doa_context_t;

/* tracker top, use target_doa_t in weifu code */
typedef struct target_doa_{
	doa_measu_t     *doa;
	int 			doa_cnt;
    track_t         *track;
} target_doa_t;

/* extern variable */
extern float DBF_datInfo_Ang[64];
extern float DBF_datInfo_Spd[64];
extern float DBF_datInfo_Dst[64];
extern float DBF_datInfo_Amp[64];
extern float DBF_datInfo_Rx[64];
extern float DBF_datInfo_Ry[64];
extern float DBF_datInfo_Vx[64];
extern float DBF_datInfo_Vy[64];
extern int   DBF_datInfo_Cnt;

extern t_None_u8_1 Bsd_Switch;         /* '<Root>/Bsd_Switch' */
extern t_None_u8_1 Ego_CarRghtlgt;     /* '<Root>/Ego_CarRghtlgt' */
extern t_None_u8_1 Ego_CarLeflgt;

//extern creal32_T VS_D1FFT_Combi[10000];
extern uint16_t  VS_LeaveFlag;
extern uint16_t  VS_RspRateOut;
extern uint16_t  VS_HeartRateOut;
extern uint16_t  VS_PhaseA_M[100];
extern float     VS_RspMagti;
extern float     VS_HeartMagti;
extern uint16_t  VS_stState;
extern float     VS_lRange;
extern uint8_t   SingleCalc_Flag;

extern uint8_t   VOD_LivingPrescDet;
extern uint8_t   VOD_LivingPrescDet2;
extern uint8_t   BudgetFlag;

extern unsigned char Frame_cnt2;
extern uint8_t VOD_windowstep_C;
extern unsigned char Frame_cnt_Parameter;
extern void target_doa_read(void);
extern void doa_sw_read(track_t *track);
extern void doa_init(baseband_t *bb);
extern void Car_Indoor_Calib_Init(void);

#endif /* _TARGET_DOA_H_ */
