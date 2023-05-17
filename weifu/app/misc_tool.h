/*
 * misc_tool.h
 *
 *  Created on: 2019Äê9ÔÂ20ÈÕ
 *      Author: xerxes
 */


#ifndef _MISC_TOOL_H_
#define _MISC_TOOL_H_

#include <string.h>
#include "embARC_toolchain.h"
#include "FreeRTOS.h"
#include "task.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_hal.h"
#include "can_cli.h"
#include "gpio_hal.h"
#include "baseband.h"
#include "dev_can.h"
#include "calterah_complex.h"
#include "target_doa.h"

#define CAN_TYPE_EXT

#if 1


#define CLUSTER_0_STATUS    0x600
#define CLUSTER_1_GENERAL   0x701
#define OBJECT_0_STATUS     0x60A
#define OBJECT_1_GENERAL    0x60B
#define OBJECT_1_EXTENDED   0x60D
#define READINGSPEED        0x7DF
#define FEEDBACKSPEED       0x7E8
#define RADARSTATE          0x201
#define RADARCFG            0x200
#define BSD_STATE           0x3B0
#define CARSPEED            0x390
#define CARSTATUS           0x391
#define Radar_AssemblyPos   0x411
#define SPEED_HC            0x422


//#define FFT1D_ReadConfig    0x61e
#define FFT1D_OUTPUTVALUE0  0x620
#define BH_RNG_CALI         0x672
#define BH_ANG_CALI         0x670
#define BH_RNG_CALI_REPLY   0x673
#define BH_ANG_CALI_REPLY   0x671

#define FRONT_SEAT_1_INFO   0x660
#define FRONT_SEAT_1_PHASE  0x661
#define FRONT_SEAT_2_INFO   0x662
#define FRONT_SEAT_2_PHASE  0x663
#define BACK_SEAT_3_INFO    0x664
#define BACK_SEAT_3_PHASE   0x665
#define BACK_SEAT_4_INFO    0x666
#define BACK_SEAT_4_PHASE   0x667
#define BACK_SEAT_5_INFO    0x668
#define BACK_SEAT_5_PHASE   0x669
#define BH_DETE_1_CALI      0x66A
#define BH_DETE_2_CALI      0x66B
#define LO_DETE_CALI        0x66C
//#define LIVE_OBJECT_INFO    0x66E
#define LIVE_OBJECT_INFO    0x291u
#define CAR_INDOOR_CALIB    (0x66Fu)
//#define LIVE_Parameter_INFO  0x66E
#endif

#ifdef CAN_TYPE_EXT
#define FFT1D_ReadConfig    (0x61e)
#define FFT1D_OUTPUTVALUE0  0x620
#if 0
#define CLUSTER_0_STATUS    0x18FF0260
#define CLUSTER_1_GENERAL   0x18FF0360
#define OBJECT_0_STATUS     0x18FF0460
#define OBJECT_1_GENERAL    0x18FF0560
#define OBJECT_1_EXTENDED   0x18FF0660
#define READINGSPEED        0x18DB33F1
#define FEEDBACKSPEED       0x18DAF100
#define RADARSTATE          0x18FF0160
#define RADARCFG            0x18FF0000
#define BSD_STATE           0x18FF0760
#define CARSPEED            0x18FF0860
#define CARSTATUS           0x18FF0960
#define Radar_AssemblyPos   0x18FF0A00
#define SPEED_HC            0x0CFE2020
#endif
#endif
#define DANGER              1
#define TARGETNUM           64

/* radar baseband enable can id */
#define FUNC_SOFT_BUTTON 0x53Bu /* information head unit private CAN */
#define SoftBtnSt 0x01
#define ant_channel_switch 0x532
#define bb_interframe 0x533
#define radio_single_tone 0x534
#define bb_task_ctrl  0x555

#define FCT_EOL_RECEIVE   0x765
#define FCT_EOL_SEND      0x766

#define ads_ref_voltage_scale 3.3 / 1024
#define sample_coff (1.0 *2300) / 300
#define self_test_3V3_coff (1.0 * 3 / 2)
#define selfTestPin 2
#define fltReadPin 1

typedef enum {
	hand_shake,
	get_sw_version,
	set_sw_version,
	read_sensor_cfg,
	ant_calib,
	ant_flash_erase,
	ant_prog_pos,
	ant_prog_comp,
	bus_power_off,
	bus_power_wake,
	bus_power_activate,
	read_radar_temp,
	read_supply_voltage,
	STATD_DIAG,
	INT_GLOBAL_DIAG,
	RF_ERR_DIAG,
	DIG_ERR_DIAG,
	read_selftest3V3,
	door_lock,
	door_unlock,
	read_veichle_gear
}can_oper;

typedef enum {
	off,
	on
}FUNC_SOFT_BUTTON_STATUS;

#define FFT1D_GETEN  0
extern int16_t  Chirp_num;
extern int16_t  Chn_num;
extern int16_t index_Start;
extern int16_t index_End;
extern uint16_t Object_MeasCounter;



//extern real32_T RadarPosXp;
//extern real32_T RadarPosXn;
//extern real32_T RadarPosYp;
//extern real32_T RadarPosYn;
//extern real32_T RadarPosAngle;

extern  int32_t can_scan_data_RX(uint32_t frame_id, uint8_t *data, uint32_t len);
//extern  int32_t can_send_data_TX(uint32_t frame_id, uint8_t *data, uint32_t len);
extern  void  CAN_Senddata_APP(complex_t *fft1d_data_output);
extern int init_extern_task(void);
extern void TagtTransmitTx(void);
#endif /* _MISC_TOOL_H_ */
