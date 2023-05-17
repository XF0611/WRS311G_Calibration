/*
 * can_transceiver.h
 *
 *  Created on: 2022Äê2ÔÂ24ÈÕ
 *      Author: fei.xu1
 */

#ifndef WEIFU_CAN_TRANSCEIVER_CAN_TRANSCEIVER_H_
#define WEIFU_CAN_TRANSCEIVER_CAN_TRANSCEIVER_H_
/* can gpio pin module */
#define IO_MUX_SELECT 0xBA0308
typedef enum {
	gpio_module,
	debug_bus_data
}io_mux_select;
#define CAN_EN_IO_MUX 0xBA0248
#define CAN_STB_IO_MUX 0xBA0218
#define CAN_WAKE_IO_MUX 0xBA021C
#define IO_MUX_GPIO_MODULE 0x4

/* can gpio pin direction and logic level */
#define CAN_EN_PIN 9
#define CAN_STB_PIN 29
#define CAN_LOCAL_WAKEUP_PIN 27
typedef enum {
	low,
	high
}can_pin_logic_level;

typedef enum {
	input,
	output
}can_pin_io_direction;

/* define radar output signal */
typedef struct output_signal {
	uint32_t radar_status;
	uint32_t alivecounter;
	uint32_t checksum;
}radar_output_signal;

#define RADAR_CAN_ID 0x566
void Tja1043_Can_Transceiver_init(void);
void Enter_Tja1043_Sleep_Mode(void);
void can_transfer_error_info(uint32_t can_error_frame_id,uint32_t can_error_info);
void can_send_radar_status(radar_output_signal *sig);
#endif /* WEIFU_CAN_TRANSCEIVER_CAN_TRANSCEIVER_H_ */
