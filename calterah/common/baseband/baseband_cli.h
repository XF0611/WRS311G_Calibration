#ifndef BASEBAND_CLI_H
#define BASEBAND_CLI_H

#include "baseband.h"
/* This Marco is used to control all the Baseband related command-line,
   comment out this Marco can size RAM size but make all Baseband command-line can no longer be used */
#define BASEBAND_CLI

#define BB_DUMP_RAWDATA      0
#define BB_DUMP_D1FFT        1
#define BB_DUMP_D2FFT        2
#define BB_DUMP_DATA_MAX     10
#define BB_DUMP_CFAR         3
#define BB_DUMP_BFM          4
#define BB_DUMP_MICRD        5
#define BB_DUMP_DOA          6
#define BB_DUMP_DOPPLERV     7
#define BB_DUMP_DOPPLERR     8
typedef struct 
{
    uint16_t    curennt_all_index;
    uint16_t    T_start_index;
    uint16_t    T_stop_index;
    uint16_t    A_start_index;
    uint16_t    A_stop_index;
    uint16_t    V_start_index;
    uint16_t    V_stop_index;
    uint16_t    R_start_index;
    uint16_t    R_stop_index;
} bb_dump_data_index_t;

typedef struct 
{
    uint16_t     microdoppler_index;
    uint16_t     doa_data_index;
    uint16_t     bb_dump_datatype;
    bb_dump_data_index_t data_index[BB_DUMP_DATA_MAX];
} bb_dump_data_t;

typedef enum {
    eBB_DATDUMP_ADC   = 0,    /* Dump ADC Data */
    eBB_DATDUMP_1DFFT = 1,    /* Dump 1DFFT Data */
    eBB_DATDUMP_2DFFT = 2,    /* Dump 2DFFT Data */
    eBB_DATDUMP_CFAR  = 3,    /* Dump CFAR Data */
    eBB_DATDUMP_BFM   = 4,    /* Dump BFM Data */
    eBB_DATDUMP_MICRD = 5,    /* Dump microdoppler Data */
    eBB_DATDUMP_DOA   = 6,    /* Dump doa Data */
	eBB_DATDUMP_DOPV = 7, /* Dump doppler Data of V dimension */
	eBB_DATDUMP_DOPR = 8 /* Dump doppler Data of R dimension */
} eBB_DATDUMP_TYPE;

extern bb_dump_data_t bb_dump_data_info; 

void baseband_cli_commands( void );
bool baseband_stream_on_dmp_mid();
bool baseband_stream_on_dmp_fnl();
bool baseband_stream_on_fft1d();
void set_baseband_stream_on_dmp_mid(bool value);
void set_baseband_stream_on_dmp_fnl(bool value);
void set_baseband_stream_on_fft1d(bool value);
bool baseband_scan_stop_req();
bool baseband_stream_off_req();
void set_scan_stop_flag(bool value);
void set_stream_on_en(bool value);
bool get_stream_on_en();
extern int32_t bb_datdump_uart_print(baseband_t *bb, bb_dump_data_t * dump_info, eBB_DATDUMP_TYPE eBBDatdumpType);
extern int32_t bb_dumpdata_info_check(baseband_t* bb, bb_dump_data_t * dump_info);
extern int32_t bb_dumpdata_ctrl(baseband_t *bb);
//avoid DC interference and elevate precision
#define RNG_START_SEARCH_IDX                (4)

#endif
