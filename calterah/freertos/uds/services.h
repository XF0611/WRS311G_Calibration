#ifndef _SERVICES_H_
#define _SERVICES_H_

#define CAN_UDS_DIAG_SESSION_CTRL	(0x10)
#define CAN_UDS_ECU_RESET		(0x11)
#define CAN_UDS_SECURITY_ACCESS		(0x27)
#define CAN_UDS_COMM_CTRL		(0x28)
#define CAN_UDS_TESTER_PRESENT		(0x3E)
#define CAN_UDS_DTC_SETTING		(0x85)
#define CAN_UDS_CLEAR_DTC_INFO		(0x14)
#define CAN_UDS_READ_DTC_INFO		(0x19)
#define CAN_UDS_ROUTINE_CTRL		(0x31)
#define CAN_UDS_IOCTRL_BY_ID		(0x2F)
#define CAN_UDS_READ_DATA_BY_ID		(0x22)
#define CAN_UDS_READ_MEM_BY_ADDR	(0x23)
#define CAN_UDS_READ_SCALE_DATA_BY_ID	(0x24)
#define CAN_UDS_READ_DATA_BY_PERIODID	(0x2A)
#define CAN_UDS_WRITE_DATA_BY_ID	(0x2E)

#define CAN_UDS_REQUEST_DOWNLOAD	(0x34)
#define CAN_UDS_REQUEST_UPLOAD		(0x35)
#define CAN_UDS_TRANSFER_DATA		(0x36)
#define CAN_UDS_REQUEST_TRANSFER_EXIT	(0x37)
#define CAN_UDS_REQUEST_FILE_TRANSFER	(0x38)
#define CAN_UDS_WRITE_MEM_BY_ADDR   (0x3D)

typedef enum {
	flash_addr_set = 0xE300,
	flash_erase = 0xFF00,
	flase_program = 0xE301
}routine_oper_type;


typedef enum {
	ant_pos_flash_addr,
	ant_comp_flash_addr
};

typedef enum {
	read_board_id = 0xA000,
	read_version_info,
	read_radar_temp_info,
	read_supply_voltage_info,
	read_statd_register_info,
	read_int_global_register_info,
	read_rf_err_register_info,
	read_dig_err_register_info,
	read_door_lock_info,
	read_door_unlock_info,
	read_veichle_gear_info
}FCT_EOL_Read_Id_Type;

#define borad_id_addr 	0x550000
#define Board_Id_Addr	(0x030000+0x200000)
typedef enum {
	write_board_id = 0xB000
}FCT_EOL_Write_Id_Type;



typedef enum {
	ant_pos = 				0x00010000,
	ant_comp = 				0x00010080,
	ant_ang = 				0x00010100,
	ant_rng_min = 			0x00010104,
	ant_rng_max = 			0x00010108,
	calib_times = 			0x0001010C,
	ant_rng_inedx = 		0x00010110,
	calib_frame_period = 	0x00010114,
	calib_read_rng =        0x00010118,
	calib_read_phase_pow =  0x00010200
}calib_oper_type;

typedef void (*service_handler)(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
typedef void (*service_response_done)(uint8_t reserved, uint32_t result);
typedef struct {
	uint8_t sid;
	service_handler service;
	service_response_done done;
} can_service_t;


#define CAN_UDS_SERVICE(id, func, resp_func)	{.sid = id, .service = func, .done = resp_func}


void uds_ecu_reset(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id);
void uds_ecu_session_ctrl(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id);
void uds_ecu_routine_ctrl(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void uds_ecu_read_data_by_id(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void uds_ecu_read_mem_by_addr(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void uds_ecu_write_mem_by_addr(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void uds_ecu_write_data_by_id(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void can_uds_request_download(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void uds_ecu_transfer_data(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id);
void can_uds_transfer_data(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id);
void uds_ecu_transfer_exit(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id);
void can_uds_transfer_exit(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id);

void can_uds_routine_control(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id);
void set_flash_addr(uint32_t addr);

#endif
