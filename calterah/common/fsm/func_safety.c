#include "embARC.h"
#include "embARC_debug.h"
#include "alps_emu_reg.h"
#include "func_safety.h"
#include "i2c_hal.h"
#include "stdbool.h"
#include "string.h"
#include "dbg_gpio_reg.h"
#include "alps_dmu_reg.h"
#include "alps_mmap.h"
#include "alps_interrupt.h"
#include "gpio_hal.h"
#include "dw_gpio.h"
#include "alps_clock.h"
#include "baseband_alps_FM_reg.h"
#include "alps_mp_radio_reg.h"
#include "calterah_error.h"
#include "baseband.h"
#include "cmd_reg.h"
#include "radio_reg.h"
#include "radio_ctrl.h"
#include "cascade.h"
#include <stdlib.h>
#include <math.h>
#include "sensor_config.h"
#include "uart_hal.h"
#include "baseband_cli.h"
#include "can_reg.h"
#include "dw_i2c_reg.h"
#include "dw_i2c.h"
#include "crc_hal.h"
#include "spi_hal.h"
#include "dw_ssi.h"
#include "can_signal_interface.h"
#include "can_hal.h"
#include "dw_ssi_reg.h"
#include "arc_wdg.h"
#include "baseband_dpc.h"
#include "track_cli.h"
#include "func_safety_config.h"

static uint8_t CRC8_ROHC(uint8_t array[], uint8_t length) ;
static void emu_rf_error_ss1_cpu_20_isr(void *params);
static void emu_digital_error_ss1_cpu_22_isr(void *params);
static void func_safety_set_dmu_irq_0_32_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss1_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss1_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss1_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss1_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss2_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss2_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss2_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss2_ena(uint8_t bit_shift);
static void func_safety_iic_isr(void *params);
static int32_t func_safety_crc16_update(uint32_t crc, uint16_t *data, uint32_t len);
static void func_safety_crc_isr(void *params);
static void func_safety_sm202_iic_init(void);
static void func_safety_sm_spi_loopback(uint8_t func_safety_error_type);
static void fsm_i2c_Periodic_Software_readback(uint32_t slave_addr, uint8_t addr,
                uint8_t *data, uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num);
static void fsm_i2c_readback_write(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num);
static void fsm_i2c_crc_write(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num);
static void fsm_i2c_crc_read(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num);
static void fsm_i2c_ack_write(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num) ;
static void fsm_on_time_error_pin_check(void);
static bool func_safety_sm_vga_consistence_check(void);
static void func_safety_sm_periodic_readback_configuration_registers(uint8_t func_safety_error_type);

safety_cycle_t FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_1_CYCLE;
static uint8_t can_send_status = CAN_SEND_STATUS_IDLE;

/*support function-safety configure list */
static const fsm_config_t fsmConfList[] =
{
        /* index                  sm number                error type        open flag  */
        {SM_INDEX_0,        FUNC_SAFETY_ITEM_SM1,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM1},
        {SM_INDEX_1,        FUNC_SAFETY_ITEM_SM2,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM2},
        {SM_INDEX_2,        FUNC_SAFETY_ITEM_SM3,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM3},
        {SM_INDEX_3,        FUNC_SAFETY_ITEM_SM4,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM4},
        {SM_INDEX_4,        FUNC_SAFETY_ITEM_SM5,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM5},
        {SM_INDEX_5,        FUNC_SAFETY_ITEM_SM6,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM6},
        {SM_INDEX_6,        FUNC_SAFETY_ITEM_SM8,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM8},
        {SM_INDEX_7,        FUNC_SAFETY_ITEM_SM11,       SAFE_STATE_IRQ,        SAFETY_FEATURE_SM11},
        {SM_INDEX_8,        FUNC_SAFETY_ITEM_SM12,       SAFE_STATE_IRQ,        SAFETY_FEATURE_SM12},
        {SM_INDEX_9,        FUNC_SAFETY_ITEM_SM13,       SAFE_STATE_IRQ,        SAFETY_FEATURE_SM13},
        {SM_INDEX_10,        FUNC_SAFETY_ITEM_SM14,       SAFE_STATE_IRQ,       SAFETY_FEATURE_SM14},
        {SM_INDEX_11,        FUNC_SAFETY_ITEM_SM201,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM201},
        {SM_INDEX_12,        FUNC_SAFETY_ITEM_SM101,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM101},
        {SM_INDEX_13,        FUNC_SAFETY_ITEM_SM102,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM102},
        {SM_INDEX_14,        FUNC_SAFETY_ITEM_SM103,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM103},
        {SM_INDEX_15,        FUNC_SAFETY_ITEM_SM104,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM104},
        {SM_INDEX_16,        FUNC_SAFETY_ITEM_SM105,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM105},
        {SM_INDEX_17,        FUNC_SAFETY_ITEM_SM106,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM106},
        {SM_INDEX_18,        FUNC_SAFETY_ITEM_SM107,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM107},
        {SM_INDEX_19,        FUNC_SAFETY_ITEM_SM108,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM108},
        {SM_INDEX_20,        FUNC_SAFETY_ITEM_SM109,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM109},
        {SM_INDEX_21,        FUNC_SAFETY_ITEM_SM120,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM120},
        {SM_INDEX_22,        FUNC_SAFETY_ITEM_SM121,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM121},
        {SM_INDEX_23,        FUNC_SAFETY_ITEM_SM122,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM122},
        {SM_INDEX_24,        FUNC_SAFETY_ITEM_SM123,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM123},
        {SM_INDEX_25,        FUNC_SAFETY_ITEM_SM124,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM124},
        {SM_INDEX_26,        FUNC_SAFETY_ITEM_SM125,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM125},
        {SM_INDEX_27,        FUNC_SAFETY_ITEM_SM126,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM126},
        {SM_INDEX_28,        FUNC_SAFETY_ITEM_SM129,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM129},
        {SM_INDEX_29,        FUNC_SAFETY_ITEM_SM130,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM130},
        {SM_INDEX_30,        FUNC_SAFETY_ITEM_SM133,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM133},
        {SM_INDEX_31,        FUNC_SAFETY_ITEM_SM202,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM202},
        {SM_INDEX_32,        FUNC_SAFETY_ITEM_SM206,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM206},
        {SM_INDEX_33,        FUNC_SAFETY_ITEM_SM805,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM805}
};

#define FSM_CONF_MAX (sizeof (fsmConfList) / sizeof (fsm_config_t))


#if (FUNC_SAFETY_CLI == 1)
static uint8_t fsm_test_wr_buf[MAX_FSM_TEST_BUF_LEN] = { 0x11, 0x22, 0x33, 0x44, 0x55,
                0x66, 0x77, 0x88, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x11,
                0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x11, 0x22, 0x33, 0x44, 0x55,
                0x66, 0x77, 0x88, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x11,
                0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x11, 0x22, 0x33, 0x44, 0x55,
                0x66, 0x77, 0x88, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
#endif
static uint8_t fsm_test_rd_buf[MAX_FSM_TEST_BUF_LEN] = { 0x00 };

//Look-up table
static const uint8_t REVERSE_TABLE[] = { 0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60,
                0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 0x08, 0x88, 0x48,
                0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78,
                0xF8, 0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54,
                0xD4, 0x34, 0xB4, 0x74, 0xF4, 0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C,
                0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 0x02, 0x82, 0x42,
                0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72,
                0xF2, 0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A,
                0xDA, 0x3A, 0xBA, 0x7A, 0xFA, 0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66,
                0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E, 0x4E,
                0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E,
                0xFE, 0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51,
                0xD1, 0x31, 0xB1, 0x71, 0xF1, 0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69,
                0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 0x05, 0x85, 0x45,
                0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75,
                0xF5, 0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D,
                0xDD, 0x3D, 0xBD, 0x7D, 0xFD, 0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63,
                0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 0x0B, 0x8B, 0x4B,
                0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B,
                0xFB, 0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57,
                0xD7, 0x37, 0xB7, 0x77, 0xF7, 0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F,
                0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF };

static const uint8_t CRC8_ROHC_TABLE[] = { 0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12,
                0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77, 0x7E,
                0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A,
                0x5D, 0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6,
                0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82,
                0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9,
                0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED,
                0xEA, 0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81,
                0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35,
                0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59,
                0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D,
                0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF,
                0xB8, 0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB,
                0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67,
                0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43,
                0x44, 0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F,
                0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C,
                0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30,
                0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14,
                0x13, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98,
                0x9F, 0x8A, 0x8D, 0x84, 0x83, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC,
                0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3 };

/* External Parameter */
bool external_wdt_on_flag = false;
bool external_wdt_test_flag = false;
uint32_t max20430_feed_dog_time_cnt;
bool bb_frame_start_flag = false;
bool ldo_set_part_flag = false;
uint8_t sm1_ldo_part_cnt = 0x00;
bool periodic_sm_finish_flag = false;
bool sample_adc_running_flag = false;


/****************************************************
 * NAME         : CRC8_ROHC X8+X2+X1+1
 * DESCRIPTIONS : accumulate the checksum, the result is the lowest byte
 * INPUT        : checksum data, length
 * OUTPUT       : checksum
 *****************************************************/
static uint8_t CRC8_ROHC(uint8_t array[], uint8_t length) {
        uint8_t i;
        uint8_t c = 0xFF;

        for (i = 0; i < length; i++) {
                c ^= REVERSE_TABLE[array[i]];
                c = CRC8_ROHC_TABLE[c];
        }
        c = REVERSE_TABLE[c];

        return c;
}

/****************************************************
 * NAME         : led_d1_init
 * DESCRIPTIONS : led d1 pin init, used to detect FDTI
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void led_d1_init(void) {
        int32_t ret = E_OK;

        ret = gpio_set_direct(LED_D1_NO, DW_GPIO_DIR_OUTPUT);
        if (ret != 0) {
                EMBARC_PRINTF("Dir gpio(%d) error! ret %d \n", LED_D1_NO, ret);
        }
}

/****************************************************
 * NAME         : clear_mem_mac_by_sw
 * DESCRIPTIONS : clear mem_mac
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void clear_mem_mac_by_sw(void)
{
        baesband_frame_interleave_cnt_clr();
        baseband_t* bb = baseband_get_rtl_frame_type();
        baseband_hw_t *bb_hw = &bb->bb_hw;

        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_MAC); //Store bank selected
        uint32_t *mem_mac_offset = 0; // offset = 0
        for (uint16_t loop= 0; loop< 16384; loop++) { //4x4096 loop
                 baseband_write_mem_table(bb_hw, *mem_mac_offset, 0);
                 (*mem_mac_offset)++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank
}

/****************************************************
 * NAME         : clear_all_bb_memory
 * DESCRIPTIONS : clear all bb memory
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void clear_all_bb_memory(baseband_hw_t *bb_hw)
{
        uint32_t mem_offset;
        uint32_t i;
        uint32_t old;
        uint32_t value;
        /*baesband_frame_interleave_cnt_clr();
        baseband_t* bb = baseband_get_cur_bb();
        baseband_hw_t *bb_hw = &bb->bb_hw;*/

        //clear BB MEM_WIN
        /*old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_WIN); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 8192; i++) { //32K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_NVE
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_NVE); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 4096; i++) { //16K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_BUF
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 524288; i++) { //2M
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_COE
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_COE); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 16384; i++) { //64K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank*/

        //clear BB MEM_MAC
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_MAC); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 16384; i++) { //64K
             baseband_write_mem_table(bb_hw, mem_offset, 0);
             mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_RLT_DOA
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_RLT); //Store bank selected
        memset((void *)BB_MEM_BASEADDR, 0, 1024 * sizeof(obj_info_t));
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_ANC
        /*old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_ANC); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 131072; i++) { //512K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_DML
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_DML); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 8192; i++) { //32K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank*/

        //clear BB MEM_WIN
        if (bb_hw->frame_type_id == 0) { // run one time
                //func_safety_enable_bb_mem_ecc();
                value = raw_readl(BB_REG_SYS_ECC_ENA);
                raw_writel(BB_REG_SYS_ECC_ENA, value | (1 << 1));
                value = raw_readl(BB_REG_SYS_ECC_ENA);

                old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_WIN); //Store bank selected
                mem_offset = 0;
                for (i = 0; i < 8192; i++) { //32K
                         baseband_write_mem_table(bb_hw, mem_offset, 0);
                         mem_offset++;
                }
                baseband_switch_mem_access(bb_hw, old); // Restore back to old bank
        }
}

/****************************************************
 * NAME         : emu_rf_error_irq_cpu_19_isr
 * DESCRIPTIONS : emu rf error irq cpu 19 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_rf_error_irq_cpu_19_isr(void *params) {
        uint32_t value;
        uint32_t value_clear;

        //disable rf error irq
        int_disable(INT_RF_ERROR_IRQ);

        portENTER_CRITICAL();
        //EMBARC_PRINTF("emu rf error irq cpu 19 isr.\n");
        // check EMU.RF_ERR_SS1_STA bit
        value = raw_readl(REG_EMU_RF_ERR_IRQ_STA);
        //EMBARC_PRINTF("check EMU.RF_ERR_SS1_STA_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_IRQ_STA, value);
        //clear the interrupt flag that lead to 20 isr
        //value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
        //EMBARC_PRINTF("check EMU.REG_EMU_RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value_clear);
        //raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
        //chip_hw_mdelay(2000);
        portEXIT_CRITICAL();
        if (value) {
                if (value & (1 << 0)) {        //LDO_MT bit
                        EMBARC_PRINTF("LDO_MT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 1)) {        //AVDD33_MT bit
                        EMBARC_PRINTF("AVDD33_MT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 2)) {        //DVDD11_MT bit
                        EMBARC_PRINTF("DVDD11_MT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 3)) {        //VBG_MT bit
                        EMBARC_PRINTF("VBG_MT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 4)) {        //PLL_LOCK_DT bit
                        EMBARC_PRINTF("PLL_LOCK_DT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 5)) {        //RF_POWER_DT bit
                        EMBARC_PRINTF("RF_POWER_DT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 6)) {        //TX_BALL_DT bit
                        EMBARC_PRINTF("TX_BALL_DT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 7)) {        //RX_SAT_DT bit
                        //EMBARC_PRINTF("RX_SAT_DT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                        EMBARC_PRINTF("RX_SAT_DT bit irq set.\n");
                } else if (value & (1 << 10)) {                        //IF_LOOPBACK bit
                        EMBARC_PRINTF("IF_LOOPBACK bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 11)) {                        //RX_LOOPBACK bit
                        EMBARC_PRINTF("RX_LOOPBACK bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 12)) { //GAIN_CK VGA gain consistence check bit
                        EMBARC_PRINTF("GAIN_CK bit set irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 8)) {                        //CHIRP_MT bit
                        EMBARC_PRINTF("CHIRP_MT bit set irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 9)) {                        //TEMP_OVER bit
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//end test
                        //set EMU.SS1_CTRL to 0x1
                        EMBARC_PRINTF("TEMP_OVER bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
                        raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
                        //value = raw_readl(REG_EMU_SS1_CTRL);
                        //EMBARC_PRINTF("set EMU.SS1_CTRL to 0x1_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_SS1_CTRL, value);
                } else {
                        //enable rf error irq
                        int_enable(INT_RF_ERROR_IRQ);
                }
        } else {
                //enable rf error irq
                int_enable(INT_RF_ERROR_IRQ);
        }

        //int_enable(INT_RF_ERROR_IRQ);
}

/****************************************************
 * NAME         : emu_rf_error_ss1_cpu_20_isr
 * DESCRIPTIONS : emu rf error ss1 cpu 20 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_rf_error_ss1_cpu_20_isr(void *params) {
        uint32_t value;
        //uint32_t value_clear;

        //disable rf error irq
        int_disable(INT_RF_ERROR_SS1);

        portENTER_CRITICAL();
        EMBARC_PRINTF("emu rf error ss1 cpu 20 isr.\n");
        // check EMU.RF_ERR_SS1_STA bit
        value = raw_readl(REG_EMU_RF_ERR_SS1_STA);
        EMBARC_PRINTF("check EMU.RF_ERR_SS1_STA_ADDR: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS1_STA, value);
        //clear the interrupt flag that lead to 20 isr
        //value_clear = raw_readl(REG_EMU_RF_ERR_CLR);
        //EMBARC_PRINTF("check EMU.REG_EMU_RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value_clear);
        //raw_writel(REG_EMU_RF_ERR_CLR, value | value_clear);
        //chip_hw_mdelay(2000);
        portEXIT_CRITICAL();
        if (value) {
                if (value & (1 << 0)) {        //LDO_MT bit
                        EMBARC_PRINTF("LDO_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 1)) {        //AVDD33_MT bit
                        EMBARC_PRINTF("AVDD33_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 2)) {        //DVDD11_MT bit
                        EMBARC_PRINTF("DVDD11_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 3)) {        //VBG_MT bit
                        EMBARC_PRINTF("VBG_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 4)) {        //PLL_LOCK_DT bit
                        EMBARC_PRINTF("PLL_LOCK_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 5)) {        //RF_POWER_DT bit
                        EMBARC_PRINTF("RF_POWER_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 6)) {        //TX_BALL_DT bit
                        EMBARC_PRINTF("TX_BALL_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 7)) {        //RX_SAT_DT bit
                        //clear EMU RF saturate detect irq
                        //value = raw_readl(REG_EMU_RF_ERR_CLR);
                        //raw_writel(REG_EMU_RF_ERR_CLR, value | (1 << 7));
                        EMBARC_PRINTF("RX_SAT_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 10)) {                        //IF_LOOPBACK bit
                        EMBARC_PRINTF("IF_LOOPBACK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 11)) {                        //RX_LOOPBACK bit
                        EMBARC_PRINTF("RX_LOOPBACK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 12)) { //GAIN_CK VGA gain consistence check bit
                        EMBARC_PRINTF("GAIN_CK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 8)) {                        //CHIRP_MT bit
                        EMBARC_PRINTF("CHIRP_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 9)) {                        //TEMP_OVER bit
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//end test
                        //set EMU.SS1_CTRL to 0x1
                        EMBARC_PRINTF("TEMP_OVER bit set EMU.SS1_CTRL to 0x1.\n");
                        value = 0x1;
                        raw_writel(REG_EMU_SS1_CTRL, value);
                        //value = raw_readl(REG_EMU_SS1_CTRL);
                        //EMBARC_PRINTF("set EMU.SS1_CTRL to 0x1_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_SS1_CTRL, value);
                } else {
                        //enable rf error irq
                        int_enable(INT_RF_ERROR_SS1);
                }
        } else {
                //enable rf error irq
                int_enable(INT_RF_ERROR_SS1);
        }
}

/****************************************************
 * NAME         : emu_digital_error_irq_cpu_21_isr
 * DESCRIPTIONS : emu digital error irq cpu 21 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_digital_error_irq_cpu_21_isr(void *params) {
        uint32_t value;
        uint32_t value_clear;

        //disable digital error irq
        int_disable(INT_DG_ERROR_IRQ);

        portENTER_CRITICAL();
        EMBARC_PRINTF("emu digital error irq cpu 21 isr.\n");
        // check EMU.DG_ERR_IRQ_STA bit
        value = raw_readl(REG_EMU_DIG_ERR_IRQ_STA);
        EMBARC_PRINTF("check EMU.DIG_ERR_IRQ_STA_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_IRQ_STA, value);

        portEXIT_CRITICAL();
        if (value) {
                if (value & (1 << 0)) {
                        EMBARC_PRINTF("BB_LBIST bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 3)) {
                        EMBARC_PRINTF("BB_SRAM_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 4)) {
                        EMBARC_PRINTF("BB_ROM_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 5)) {
                        EMBARC_PRINTF("CPU_ROM_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 6)) {
                        EMBARC_PRINTF("CPU_SRAM_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 7)) {
                        EMBARC_PRINTF("OTP_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 8)) {
                        EMBARC_PRINTF("CAN0_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 10)) {
                        EMBARC_PRINTF("CPU_WDT bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 11)) {
                        EMBARC_PRINTF("CAN0_ERR bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 16)) {
                        EMBARC_PRINTF("CAN0_LP bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 13)) {
                        EMBARC_PRINTF("I2C_ACK bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 14)) {
                        EMBARC_PRINTF("CHIRP_LS bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 15)) {
                        EMBARC_PRINTF("XIP_ECC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 18)) {
                        EMBARC_PRINTF("SPI_LP bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 19)) {
                        EMBARC_PRINTF("SPI_CRC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 20)) {
                        EMBARC_PRINTF("FLASH_CRC bit irq set.\n");
                        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
                        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else {
                        //enable digital error irq
                        int_enable(INT_DG_ERROR_IRQ);
                }
        } else {
                //enable digital error irq
                int_enable(INT_DG_ERROR_IRQ);
        }
}


/****************************************************
 * NAME         : emu_digital_error_ss1_cpu_22_isr
 * DESCRIPTIONS : emu digital error ss1 cpu 22 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_digital_error_ss1_cpu_22_isr(void *params) {
        uint32_t value;
        uint32_t value_clear;
        //uint32_t compare;

        //disable first then enable
        int_disable(INT_DG_ERROR_SS1);

        portENTER_CRITICAL();
        EMBARC_PRINTF("emu digital error ss1 cpu 22 isr.\n");
        // check EMU.DIG_ERR_SS1_STA.FLASH_CRC bit
        value = raw_readl(REG_EMU_DIG_ERR_SS1_STA);
        EMBARC_PRINTF("check EMU.DIG_ERR_SS1_STA_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_SS1_STA, value);
        //clear the interrupt flag that lead to 22 isr
        value_clear = raw_readl(REG_EMU_DIG_ERR_CLR);
        //EMBARC_PRINTF("check EMU.REG_EMU_DIG_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_CLR, value_clear);
        raw_writel(REG_EMU_DIG_ERR_CLR, value | value_clear);
        //chip_hw_mdelay(2000);
        portEXIT_CRITICAL();
        if (value) {
                if (value & (1 << 0)) {
                        //BB_LBIST bit
                        EMBARC_PRINTF(" BB_LBIST bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 3)) {
                        //BB_SRAM_ECC bit
                        EMBARC_PRINTF(" BB_SRAM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 4)) {
                        //BB_ROM_ECC bit
                        EMBARC_PRINTF(" BB_ROM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 5)) {
                        //CPU_ROM_ECC bit
                        EMBARC_PRINTF(" CPU_ROM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 6)) {
                        //CPU_SRAM_ECC bit
                        EMBARC_PRINTF(" CPU_SRAM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 7)) {
                        //OTP_ECC bit
                        EMBARC_PRINTF(" OTP_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 8)) {
                        //CAN0_ECC bit
                        EMBARC_PRINTF(" CAN0_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 10)) {
                        //CPU_WDT bit
                        EMBARC_PRINTF(" CPU WDT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 11)) {
                        //CAN0_ERR bit
                        EMBARC_PRINTF(" CAN0_ERR bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 16)) {
                        //CAN0_LP bit
                        EMBARC_PRINTF(" CAN0_LP bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 13)) {
                        //I2C_ACK bit
                        EMBARC_PRINTF(" I2C_ACK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 14)) {
                        //CHIRP_LS bit
                        EMBARC_PRINTF(" CHIRP_LS bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 15)) {
                        //XIP_ECC bit
                        EMBARC_PRINTF(" XIP_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 18)) {
                        //SPI_LP bit
                        EMBARC_PRINTF(" SPI_LP bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 19)) {
                        //SPI_CRC bit
                        EMBARC_PRINTF(" SPI_CRC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_writel(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 20)) {
                        //FLASH_CRC bit
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//end test
                        //set EMU.SS1_CTRL to 0x1
                        EMBARC_PRINTF(" FLASH_CRC bit set EMU.SS1_CTRL to 0x1.\n");
                        //chip_hw_mdelay(2000);
                        value = 0x1;
                        raw_writel(REG_EMU_SS1_CTRL, value);
                        //value = raw_readl(REG_EMU_SS1_CTRL);
                        //EMBARC_PRINTF("set EMU.SS1_CTRL to 0x1_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_SS1_CTRL, value);
                } else {
                        //enable digital error irq
                        int_enable(INT_DG_ERROR_SS1);
                }
        }else {
                //enable digital error irq
                int_enable(INT_DG_ERROR_SS1);
        }
}

/****************************************************
 * NAME         : func_safety_set_dmu_irq_0_32_ena
 * DESCRIPTIONS : set dmu irq 0_32 enable
 * INPUT        : bit_shift:bit 0_32 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_dmu_irq_0_32_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set DMU.IRQ_ENA_0_32.bit_shift bit to 0x1
        value = raw_readl(REL_REGBASE_DMU + REG_DMU_IRQ_ENA0_31_OFFSET);
        raw_writel(REL_REGBASE_DMU + REG_DMU_IRQ_ENA0_31_OFFSET, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_irq_mask
 * DESCRIPTIONS : set reg emu rf err irq mask enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_irq_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_IRQ_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_IRQ_MASK);
        raw_writel(REG_EMU_RF_ERR_IRQ_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_irq_ena
 * DESCRIPTIONS : set reg emu rf err irq enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_IRQ_ENA.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_IRQ_ENA);
        raw_writel(REG_EMU_RF_ERR_IRQ_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_irq_mask
 * DESCRIPTIONS : clear reg emu rf err irq mask
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_irq_mask(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_IRQ_MARK.bit_shift bit
        value = raw_readl(REG_EMU_RF_ERR_IRQ_MASK);
        //EMBARC_PRINTF("irq mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_writel(REG_EMU_RF_ERR_IRQ_MASK, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_irq_ena
 * DESCRIPTIONS : clear reg emu rf err irq
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_IRQ_ENA.bit_shift bit
        value = raw_readl(REG_EMU_RF_ERR_IRQ_ENA);
        //EMBARC_PRINTF("irq ena value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_writel(REG_EMU_RF_ERR_IRQ_ENA, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss1_mask
 * DESCRIPTIONS : set reg emu rf err ss1 mask enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss1_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS1_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_SS1_MASK);
        raw_writel(REG_EMU_RF_ERR_SS1_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss1_ena
 * DESCRIPTIONS : set reg emu rf err ss1 enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss1_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS1_ENA.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_SS1_ENA);
        raw_writel(REG_EMU_RF_ERR_SS1_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clrear_reg_emu_rf_err_ss1_mask
 * DESCRIPTIONS : clear reg emu rf err ss1 mask
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clrear_reg_emu_rf_err_ss1_mask(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS1_MARK.bit_shift bit
        value = raw_readl(REG_EMU_RF_ERR_SS1_MASK);
        //EMBARC_PRINTF("ss1 mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_writel(REG_EMU_RF_ERR_SS1_MASK, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_clrear_reg_emu_rf_err_ss1_ena
 * DESCRIPTIONS : clear reg emu rf err ss1
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clrear_reg_emu_rf_err_ss1_ena(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS1_ENA.bit_shift bit
        value = raw_readl(REG_EMU_RF_ERR_SS1_ENA);
        //EMBARC_PRINTF("ss1 ena value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_writel(REG_EMU_RF_ERR_SS1_ENA, value & (~(1 << bit_shift)));
}


/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_irq_mask
 * DESCRIPTIONS : set reg emu dig err irq mask enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_irq_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_IRQ_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_DIG_ERR_IRQ_MASK);
        raw_writel(REG_EMU_DIG_ERR_IRQ_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_irq_ena
 * DESCRIPTIONS : set reg emu dig err irq enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_IRQ_ENA.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_DIG_ERR_IRQ_ENA);
        raw_writel(REG_EMU_DIG_ERR_IRQ_ENA, value | (1 << bit_shift));
}


/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss1_mask
 * DESCRIPTIONS : set reg emu dig err ss1 mask enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss1_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS1_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_DIG_ERR_SS1_MASK);
        raw_writel(REG_EMU_DIG_ERR_SS1_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss1_ena
 * DESCRIPTIONS : set reg emu dig err ss1 enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss1_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS1_ENA.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_DIG_ERR_SS1_ENA);
        raw_writel(REG_EMU_DIG_ERR_SS1_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss2_mask
 * DESCRIPTIONS : set reg emu rf err ss2 mask enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss2_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS2_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_SS2_MASK);
        raw_writel(REG_EMU_RF_ERR_SS2_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss2_ena
 * DESCRIPTIONS : set reg emu rf err ss2 enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss2_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS2_ENA.bit_shift to 0x1
        value = raw_readl(REG_EMU_RF_ERR_SS2_ENA);
        raw_writel(REG_EMU_RF_ERR_SS2_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_ss2_mask
 * DESCRIPTIONS : clear reg emu rf err ss2 mask
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_ss2_mask(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS2_MARK.bit_shift bit
        value = raw_readl(REG_EMU_RF_ERR_SS2_MASK);
        //EMBARC_PRINTF("ss2 mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_writel(REG_EMU_RF_ERR_SS2_MASK, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_ss2_ena
 * DESCRIPTIONS : clear reg emu rf err ss2
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_ss2_ena(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS2_ENA.bit_shift
        value = raw_readl(REG_EMU_RF_ERR_SS2_ENA);
        //EMBARC_PRINTF("ss2 mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_writel(REG_EMU_RF_ERR_SS2_ENA, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss2_mask
 * DESCRIPTIONS : set reg emu dig err ss2 mask enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss2_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS2_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_DIG_ERR_SS2_MASK);
        raw_writel(REG_EMU_DIG_ERR_SS2_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss2_ena
 * DESCRIPTIONS : set reg emu dig err ss2 enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss2_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS2_ENA.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_DIG_ERR_SS2_ENA);
        raw_writel(REG_EMU_DIG_ERR_SS2_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_iic_isr
 * DESCRIPTIONS : func safety iic isr
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void func_safety_iic_isr(void *params) {
        uint32_t value;

        //disable first then enable
        int_disable(INT_I2C_M_IRQ);

        EMBARC_PRINTF("func_safety_iic_isr!\n");
        value = dw_i2c_int_status();
        EMBARC_PRINTF("dw_i2c_int_status: 0x%x \n", value);
        if (value & (1 << DW_I2C_TX_ABRT)) {
                value = dw_i2c_abort_source();
                EMBARC_PRINTF("dw_i2c_abort_source: 0x%x \n", value);
                if (value & (1 << DW_I2C_ABRT_7ADDR_NOACK)) {
                        func_safety_error_handler(FUNC_SAFETY_ITEM_SM128, SAFE_STATE_SS1);
                }
        }

        dw_i2c_all_int_clear();
        dw_i2c_tx_abort_clear();

        int_enable(INT_I2C_M_IRQ);
}

/****************************************************
 * NAME         : func_safety_crc16_update
 * DESCRIPTIONS : crc16 calculation
 * INPUT        : crc:initial value, data:data to calculate, len:data length
 * OUTPUT       : crc result
 *****************************************************/
static int32_t func_safety_crc16_update(uint32_t crc, uint16_t *data, uint32_t len) {
        int32_t result = E_OK;

        uint32_t idx, single_len = 0xFFFF;

        if ((NULL == data) || (0 == len)) {
                result = E_PAR;
        } else {
                //CRC_LOCK(crclock);
                while (len) {
                        if (len < single_len) {
                                single_len = len;
                        }

                        hw_crc_init_value(crc);
                        hw_crc_count(single_len);

                        for (idx = 0; idx < single_len; idx++) {
                                hw_crc_raw_data((uint32_t) *data++);
                        }

                        //HW_CRC_WAIT_COMPLETED();
                        crc = hw_crc_sector_value();
                        len -= single_len;
                }
                //CRC_UNLOCK(crclock);
        }

        return result;
}

/****************************************************
 * NAME         : func_safety_crc_isr
 * DESCRIPTIONS : func safety crc isr
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void func_safety_crc_isr(void *params) {
        uint32_t value;

        //disable first then enable
        int_disable(INT_CRC_IRQ1);

        EMBARC_PRINTF("func_safety_crc_isr!\n");

        value = hw_crc_interrupt_status();
        EMBARC_PRINTF("hw_crc_interrupt_status: 0x%x \n", value);
        if (value & (1 << INT_HW_CRC_FAIL)) {
                func_safety_error_handler(FUNC_SAFETY_ITEM_SM110, SAFE_STATE_SS1);
        }

        //hw_crc_interrupt_clear(INT_HW_CRC_COMPLETE);
        hw_crc_interrupt_clear(INT_HW_CRC_FAIL);
        //hw_crc_interrupt_clear(INT_HW_CRC_SUCCESS);

        int_enable(INT_CRC_IRQ1);
}

/****************************************************
 * NAME         : func_safety_sm202_iic_init
 * DESCRIPTIONS : SM202~SM205 IIC init
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void func_safety_sm202_iic_init(void) {
        io_mux_i2c_func_sel(0);

        static i2c_params_t xfer_params;

        uint32_t ref_clock = clock_frequency(I2C_CLOCK);
        xfer_params.addr_mode = 0;
        xfer_params.speed_mode = 1;
        xfer_params.timing->scl_h_cnt = ref_clock / 1000000;
        xfer_params.timing->scl_l_cnt = ref_clock / 1000000;
        xfer_params.timing->sda_rx_hold = 0;
        xfer_params.timing->sda_tx_hold = 1;
        xfer_params.timing->spike_len = 5;
        if (i2c_init(0, &xfer_params) == E_OK) {
                //EMBARC_PRINTF("iic initial ok.\n");
        } else {
                EMBARC_PRINTF("iic initial fail.\n");
        }
}

//spi loopback test parameter initial
static spi_xfer_desc_t fsm_spi_xfer_desc = { .clock_mode = SPI_CLK_MODE_0,
                .dfs = 32, .cfs = 0, .spi_frf = SPI_FRF_STANDARD, .rx_thres = 0,
                .tx_thres = 0, };

/****************************************************
 * NAME         : func_safety_sm_spi_loopback
 * DESCRIPTIONS : SM126 spi loop init and test
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void func_safety_sm_spi_loopback(uint8_t func_safety_error_type) {
        int32_t result = E_OK;
        uint32_t m0_send[1] = { 0x11223344 };
        uint32_t spi_loop_expect_rx[4] = { 0x11, 0x22, 0x33, 0x44 };
        uint32_t qspi_loop_expect_rx[8] = { 0x12345678, 0x1234567, 0x123456,
                        0x12345, 0x1234, 0x123, 0x12, 0x1 };
        uint32_t slave_read[4] = { 0 };
        uint32_t qspi_slave_read[8] = { 0 };
        uint32_t spi_slave_SCTRLR0_recover_value = 0;
        uint32_t recover_dmu_mux_spi_m1;
        uint32_t recover_dmu_mux_spi_s;
        uint8_t i;
        bool flag = false;

//SPI LOOPBACK TEST
        //set DMU_SYS_SPI_LOOP to 0x1
        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, 1);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET);
        //EMBARC_PRINTF("REG_DMU_SYS_SPI_LOOP_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, value);
        //spi master0 init
        io_mux_spi_m0_func_sel(0);
        if (spi_open(0, 25000000) != E_OK)
                EMBARC_PRINTF("spi_open DW_SPI_0_ID fail.\n");
        if (spi_transfer_config(0, &fsm_spi_xfer_desc) != E_OK)
                EMBARC_PRINTF("transfer_config DW_SPI_0_ID fail.\n");

        //spi slave init
        result = spi_open(2, 25000000);
        if (result != E_OK)
                EMBARC_PRINTF("spi_open DW_SPI_2_ID fail.\n");

        //master0 wtite data
        spi_write(0, m0_send, 1);
        //slave read data
        spi_read(2, slave_read, 4);

        //print data
        //EMBARC_PRINTF("m0_send: 0x%x.\n", m0_send[0]);
        //EMBARC_PRINTF("slave_read: 0x%x  0x%x  0x%x  0x%x.\n", slave_read[0], slave_read[1], slave_read[2], slave_read[3]);
        io_mux_spi_m0_func_sel(4);
        //EMBARC_PRINTF("SPI Loop Test Done!\n", m0_send[0]);
        spi_slave_SCTRLR0_recover_value = raw_readl(SPI_S_ADDR + CTRLR0);
        //EMBARC_PRINTF("SPI_S_ADDR+CTRLR0_ADDR_READ: 0x%x, value: 0x%x.\n", SPI_S_ADDR+CTRLR0, spi_slave_SCTRLR0_recover_value);
        for (i = 0; i < 4; i++) {
                if (spi_loop_expect_rx[i] != slave_read[i]) {
                        flag = true;
                        break;
                }
        }

//QSPI LOOPBACK TEST
        //PLL enable
        //value = raw_readl(CLKGEN_ADDR + 0x110);
        //EMBARC_PRINTF("CLKGEN_DIV_APB_REF_ADDR_READ: 0x%x, value: 0x%x.\n", CLKGEN_ADDR + 0x110, value);

        //set DMU_SYS_SPI_LOOP to 0x1
        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, 1);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET);
        //EMBARC_PRINTF("REG_DMU_SYS_SPI_LOOP_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, value);
        //set DMU_MUX_SPI_S to 0x2
        recover_dmu_mux_spi_s = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, 2);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        //EMBARC_PRINTF("REG_DMU_MUX_SPI_S_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, value);
        //set DMU_MUX_SPI_S1_MOSI to 0x2
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, 2);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET);
        //EMBARC_PRINTF("REG_DMU_MUX_SPI_S1_MOSI_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, value);
        //set DMU_MUX_SPI_S1_MISO to 0x2
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, 2);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET);
        //EMBARC_PRINTF("REG_DMU_MUX_SPI_S1_MISO_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, value);

        // config qspi
        raw_writel(REG_CLKGEN_ENA_QSPI, 1);        //enable QSPI clk
        raw_writel(REG_CLKGEN_RSTN_QSPI, 0);        //CLKGEN_RSTN_QSPI
        raw_writel(REG_CLKGEN_RSTN_QSPI, 1);        //CLKGEN_RSTN_QSPI
        raw_writel(REG_CLKGEN_RSTN_SPI_S, 0);        //CLKGEN_RSTN_SPI_S
        raw_writel(REG_CLKGEN_RSTN_SPI_S, 1);        //CLKGEN_RSTN_SPI_S

        int SCKDV = 400 / 25;

        recover_dmu_mux_spi_m1 = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET);
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET, 2); //set DMU_MUX_SPI_M1 to QSPI_M1
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_SSIENR_OFFSET, 0); // disables the QSPI
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_BAUDR_OFFSET, SCKDV); //Baud Rate divider
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_CTRLR0_OFFSET,
                        0x005f0000 | ((1 & 0x3) << 8));        //quad spi 32bit transmit
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_CTRLR1_OFFSET, 0x7ff); //number of data frames
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_SPI_CTRLR0_OFFSET, 0x00004202); // 8-bit instruction
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_SSIENR_OFFSET, 1); // enables the QSPI

        // config slave
        raw_writel(REG_CLKGEN_ENA_SPI_S, 1);        //enable slave clk
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0000); // disables the SPI slave
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_CTRLR0_OFFSET,
                        0x005f0000 | ((2 & 0x3) << 8)); //32 bit, receive , (SCPOL,SCPH)=(0,0)
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_TXFTLR_OFFSET, 0x0001); //tx_num <=1 -> irq on
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_RXFTLR_OFFSET, 0x0001); //rx_num > 0 -> irq on
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_IMR_OFFSET, 0x0000); //all mask  receive fifo full and transmit fifo empty
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0001); // enables the SPI slave

        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_DR_OFFSET(0), 0x80); // write 8-bit instruction
        for (i = 0; i < 8; i++) {
                raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_DR_OFFSET(0),
                                (TEST_QSPI_DAT >> (i * 4)) & 0xffffffff);        //write data
        }

        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_SER_OFFSET, (1 << 0)); //Slave is selected
        //wait QSPI transmit finish
        for (i = 0; i < 2; i++) {
                while ((raw_readl(REL_REGBASE_QSPI + REG_DW_SSI_SR_OFFSET) & 0x4) != 0x4) { //transmit fifo empty
                        ;
                }
        }

        //wait slave receive data finish
        while (raw_readl(REL_REGBASE_SPI2 + REG_DW_SSI_RXFLR_OFFSET) != 0x08) {
                ;
        }
        //chip_hw_mdelay(10);
        raw_writel(REL_REGBASE_QSPI + REG_DW_SSI_SER_OFFSET, 0); //No slave selected
        //read the data
        for (i = 0; i < 8; i++) {
                qspi_slave_read[i] = raw_readl(
                REL_REGBASE_SPI2 + REG_DW_SSI_DR_OFFSET(0));
        }

        for (i = 0; i < 8; i++) {
                if (qspi_loop_expect_rx[i] != qspi_slave_read[i]) {
                        flag = true;
                        break;
                }
        }

        //recover register
        //set DMU_SYS_SPI_LOOP to 0x0
        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, 0);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET);
        //EMBARC_PRINTF("REG_DMU_SYS_SPI_LOOP_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, value);
        //set DMU_MUX_SPI_S to 0x0
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, recover_dmu_mux_spi_s);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        //EMBARC_PRINTF("REG_DMU_MUX_SPI_S_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, value);
        //set DMU_MUX_SPI_S1_MOSI to 0x0
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, 0);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET);
        //EMBARC_PRINTF("REG_DMU_MUX_SPI_S1_MOSI_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, value);
        //set DMU_MUX_SPI_S1_MISO to 0x0
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, 0);
        //value = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET);
        //EMBARC_PRINTF("REG_DMU_MUX_SPI_S1_MISO_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, value);
        //recover REG_DMU_MUX_SPI_M1
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET, recover_dmu_mux_spi_m1);

        //recover spi_slave_SCTRLR0 parameter.
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0000); // disables the SPI slave
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_CTRLR0_OFFSET,
                        spi_slave_SCTRLR0_recover_value); // 8 bit, tx&rx , (SCPOL,SCPH)=(0,0)
        raw_writel(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0001); // enables the SPI slave
        //value = raw_readl(SPI_S_ADDR+CTRLR0);
        //EMBARC_PRINTF("SPI_S_ADDR+CTRLR0_ADDR_READ: 0x%x, value: 0x%x.\n", SPI_S_ADDR+CTRLR0, value);

        if (flag == true)
                func_safety_error_handler(fsmConfList[SM_INDEX_27].sm_num, fsmConfList[SM_INDEX_27].error_type);
        else {
                //EMBARC_PRINTF("SPI loop and QSPI loop test ok!\n");
        }
}

/****************************************************
 * NAME         : func_safety_sm_can_loopback_init
 * DESCRIPTIONS : can loopback initial
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_can_loopback_init(void) {
        uint32_t value;
        ;
        uint32_t can_send_buf[2] = { 0x04030201, 0x08070605 };

        //read MCR
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //EMBARC_PRINTF("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //set MCR.CFG = 1
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //EMBARC_PRINTF("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //set MCR.LBACK = 1
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_LBACK);
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //EMBARC_PRINTF("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //set MCR.CFG = 0
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //EMBARC_PRINTF("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);

        //send data
        //EMBARC_PRINTF("func_safety_can_send_data: 0x%x  0x%x.\n", can_send_buf[0], can_send_buf[1]);
        //func_safety_can_send_data(FUNC_SAFETY_FRAME_ID, can_send_buf, eDATA_LEN_8);
        can_send_data(CAN_0_ID, FUNC_SAFETY_FRAME_ID, can_send_buf, eDATA_LEN_8);
}

/****************************************************
 * NAME         : func_safety_sm_can_loopback_rx_handler
 * DESCRIPTIONS : can loopback rx handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_can_loopback_rx_handler(uint8_t *data, uint32_t len) {
        uint8_t func_safety_can_expect_rx_data[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
        bool flag = false;
        //uint32_t value;

        //recover can to normal mode
        //config mode
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, BIT_MODE_CTRL_CFG);
        //value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //EMBARC_PRINTF("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //normal mode
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, 0);
        //value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //EMBARC_PRINTF("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);

        /*EMBARC_PRINTF("can_read: ");
        for (uint8_t i = 0; i < 8; i++) {
                EMBARC_PRINTF("  0x%x", data[i]);
        }
        EMBARC_PRINTF("\r\n");*/

        for (uint8_t i = 0; i < 8; i++) {
                if (data[i] != func_safety_can_expect_rx_data[i]) {
                        flag = true;
                        break;
                }
        }

        if (flag == true) {
                func_safety_error_handler(fsmConfList[SM_INDEX_21].sm_num, fsmConfList[SM_INDEX_21].error_type);
        } else {
                //EMBARC_PRINTF("CAN loopback test ok!\n");
        }
}

/****************************************************
 * NAME         : func_safety_sm_cpu_wdt
 * DESCRIPTIONS : arc cpu watchdog handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
int32_t func_safety_sm_cpu_wdt(void) {
        int32_t result = E_OK;

        /* setting the timeout is 200ms */
        uint32_t period = (200 * PERIOD_TO_MS);

        uint32_t time_out = 0;

        time_out = (_arc_aux_read(AUX_WDG_CTRL) >> 3);
        if (time_out) {
                uint8_t root_cnt = raw_readl(REG_EMU_REBOOT_CNT);

                EMBARC_PRINTF(
                                "[%s] [%d] timeout!!! root_cnt:%d, wdg_ctrl:0x%x wdg_count:0x%x!\r\n",
                                __func__, __LINE__, root_cnt, _arc_aux_read(AUX_WDG_CTRL),
                                _arc_aux_read(AUX_WDG_COUNT));

                raw_writel(EMU_SAVE_WDG_TO_MEM_ADDR, _arc_aux_read(AUX_WDG_CTRL));
                raw_writel(EMU_SAVE_WDG_TO_MEM_ADDR + 0x4,
                                _arc_aux_read(AUX_WDG_COUNT));
                raw_writel(EMU_SAVE_WDG_TO_MEM_ADDR + 0x8, root_cnt);

                arc_wdg_status_clear();
        }

        arc_wdg_update(period);

        return result;
}

/****************************************************
 * NAME         : func_safety_sm8_enable
 * DESCRIPTIONS : enable/disable sm8
 * INPUT        : func_safety_error_type: irq or ss1 or ss2
                  enable_flag: true/false, true: enable, false: disable
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm8_enable(uint8_t func_safety_error_type, bool enable_flag)
{
        uint32_t value;

        switch (func_safety_error_type) {
                case SAFE_STATE_IRQ:
                        if (enable_flag == false) {
                                value = raw_readl(REG_EMU_RF_ERR_STA);
                                //EMBARC_PRINTF("RF_ERR_STA front: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                                func_safety_clear_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        } else {
                                value = raw_readl(REG_EMU_RF_ERR_CLR);
                                //EMBARC_PRINTF("after read RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value);
                                raw_writel(REG_EMU_RF_ERR_CLR, value | (1 << FUNC_SAFETY_BIT_7_SHIFT));

                                value = raw_readl(REG_EMU_RF_ERR_STA);
                                //EMBARC_PRINTF("RF_ERR_STA last: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);

                                func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        }
                        break;
                case SAFE_STATE_SS1:
                        if (enable_flag == false) {
                                value = raw_readl(REG_EMU_RF_ERR_STA);
                                //EMBARC_PRINTF("RF_ERR_STA front: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                                func_safety_clrear_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_clrear_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        } else {
                                value = raw_readl(REG_EMU_RF_ERR_CLR);
                                //EMBARC_PRINTF("after read RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value);
                                raw_writel(REG_EMU_RF_ERR_CLR, value | (1 << FUNC_SAFETY_BIT_7_SHIFT));

                                value = raw_readl(REG_EMU_RF_ERR_STA);
                                //EMBARC_PRINTF("RF_ERR_STA last: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);

                                func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        }
                        break;
                case SAFE_STATE_SS2:
                        if (enable_flag == false) {
                                value = raw_readl(REG_EMU_RF_ERR_STA);
                                //EMBARC_PRINTF("RF_ERR_STA front: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                                func_safety_clear_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_clear_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        } else {
                                value = raw_readl(REG_EMU_RF_ERR_CLR);
                                //EMBARC_PRINTF("after read RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value);
                                raw_writel(REG_EMU_RF_ERR_CLR, value | (1 << FUNC_SAFETY_BIT_7_SHIFT));

                                value = raw_readl(REG_EMU_RF_ERR_STA);
                                //EMBARC_PRINTF("RF_ERR_STA last: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);

                                func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        }
                        break;
                default:
                        break;
        }
}

/****************************************************
 * NAME         : func_safety_sm_init
 * DESCRIPTIONS : func safety sm init
 * INPUT        : func_safety_sm_num: sm number
 *                func_safety_error_type: ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_init(uint16_t func_safety_sm_num, uint8_t func_safety_error_type)
{
        uint32_t value;
        uint32_t IRQ_value;
        baseband_t *bb = baseband_get_cur_bb();

        if (func_safety_error_type == SAFE_STATE_IRQ) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM1:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_TEST_ENA.LDO_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 0));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM2:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_TEST_ENA.AVDD33_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 1));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM3:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_TEST_ENA.DVDD11_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 2));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM4:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 3));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM5:
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //configure radio
                        fmcw_radio_reg_write(NULL, 0, 0);
                        fmcw_radio_reg_write(NULL, 125, 0x15);
                        fmcw_radio_reg_write(NULL, 0, 0xa);
                        fmcw_radio_reg_write(NULL, 110, 0x2);
                        //set EMU.RF_TEST_ENA.PLL_LOCK_DT bit to 0x1
                        //value = raw_readl(REG_EMU_RF_TEST_ENA);
                        //raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 4));
                        //wait error status
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM6:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_TEST_ENA.RF_POWER_DT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 5));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM7:
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.TX_BALL_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.TX_BALL_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //configure radio

                        //set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 6));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM8:
                case FUNC_SAFETY_ITEM_SM9:
                case FUNC_SAFETY_ITEM_SM10:
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RX_SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RX_SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //configure radio & bb

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM11:
                        //configure radio & bb
                        IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        //configure radio & bb
                        IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM13:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_TEST_ENA.CHIRP_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 8));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM14:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_TEST_ENA.TEMP_OVER bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 9));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_12_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_12_SHIFT);
                        //configure radio & bb

                        //enable test and check results

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM101:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //configure radio

                        /*//set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set EMU.BB_LBIST_ENA to 0x1
                        raw_writel(REG_EMU_BB_LBIST_ENA, 0x1);*/
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM102:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM103:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM104:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_writel(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM105:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_writel(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM106:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set DMU.MEMINI_ENA to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM107:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //set DMU.MEMRUN_ENA to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM108:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //set DMU.OTP_ECC_EN to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM109:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //set CAN_0.MCR.CFG to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
                        //set CAN_0.MCR.ECCENA to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_ECCENA);
                        // set CAN_0.IE. set CAN_0.IE.BEUE and CAN_0.IE.BECE to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET, value | BIT_INTERRUPT_BEU | BIT_INTERRUPT_BEC);
                        //set CAN_0.ILS0R.BEULS and CAN_0.ILS0R.BECLS to 0x3
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET, value | (BITS_INTERRUPT_LINE_SELECT_ELOLS_MASK << BITS_INTERRUPT_LINE_SELECT_BEULS_SHIFT) | (BITS_INTERRUPT_LINE_SELECT_BEULS_MASK << BITS_INTERRUPT_LINE_SELECT_BECLS_SHIFT));
                        //clear CAN0.IR register
                        value = raw_readl(0xBB0000 + 0x0034);
                        raw_writel(0xBB0000 + 0x0034, value | (1 << 18));
                        //set CAN_0.ILE.EINT3 to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET, value | BIT_INTERRUPT_LINE_ENABLE_EINT3);

                        //set CAN_0.MCR.CFG to 0x0
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_16_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_16_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //configure can

                        //enable test and check results
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        // initialize can, done in main()

                        // set CAN_0.IE.PEDE to 0x1
                        value = raw_readl(0xBB0000 + 0x0038);
                        raw_writel(0xBB0000 + 0x0038, value | (1 << 23));
                        //set CAN_0.ILS0R.PEDELS to 0x2
                        value = raw_readl(0xBB0000 + 0x003C);
                        raw_writel(0xBB0000 + 0x003C, value | (1 << 17));
                        //set CAN_0.ILE.EINT2 to 0x1
                        value = raw_readl(0xBB0000 + 0x0044);
                        raw_writel(0xBB0000 + 0x0044, value | (1 << 2));
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_18_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_18_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_14_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_14_SHIFT);
                        // configure radio

                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        value = raw_readl(REG_EMU_DIG_TEST_ENA);
                        raw_writel(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_15_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_15_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                        //config iic
                        func_safety_sm202_iic_init();
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //config iic
                        MAX20430A_wdt_init();
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        MAX20430A_wdt_on(SAFE_STATE_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                default:
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS1) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM1:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio,
                        false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1,
                                        emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n",
                                                INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_TEST_ENA.LDO_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 0));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM2:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_TEST_ENA.AVDD33_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 1));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM3:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_TEST_ENA.DVDD11_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 2));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM4:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_TEST_ENA.VBG_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 3));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM5:
                        //set EMU.RF_ERR_SS1_MARK.PLL_LOCK_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.PLL_LOCK_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //configure radio
                        fmcw_radio_reg_write(NULL, 0, 0);
                        fmcw_radio_reg_write(NULL, 125, 0x15);
                        fmcw_radio_reg_write(NULL, 0, 0xa);
                        fmcw_radio_reg_write(NULL, 110, 0x2);
                        //set EMU.RF_TEST_ENA.PLL_LOCK_DT bit to 0x1
                        //value = raw_readl(REG_EMU_RF_TEST_ENA);
                        //raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 4));
                        //wait error status
                        break;
                case FUNC_SAFETY_ITEM_SM6:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RF_POWER_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RF_POWER_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_TEST_ENA.RF_POWER_DT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 5));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM7:
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.TX_BALL_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.TX_BALL_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //configure radio

                        //set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 6));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM8:
                case FUNC_SAFETY_ITEM_SM9:
                case FUNC_SAFETY_ITEM_SM10:
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //configure radio & bb

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM11:
                        //configure radio & bb
                        IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        //configure radio & bb
                        IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM13:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_TEST_ENA.CHIRP_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 8));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM14:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_TEST_ENA.TEMP_OVER bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 9));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                                EMBARC_PRINTF("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_12_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_12_SHIFT);
                        //configure radio & bb

                        //enable test and check results

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM101:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //configure radio

                        /*//set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set EMU.BB_LBIST_ENA to 0x1
                        raw_writel(REG_EMU_BB_LBIST_ENA, 0x1);*/
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM102:
                        int_disable(INT_DG_ERROR_SS1);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM103:
                        int_disable(INT_DG_ERROR_SS1);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM104:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_writel(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM105:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_writel(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM106:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set DMU.MEMINI_ENA to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM107:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //set DMU.MEMRUN_ENA to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM108:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //set DMU.OTP_ECC_EN to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM109:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //set CAN_0.MCR.CFG to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
                        //set CAN_0.MCR.ECCENA to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_ECCENA);
                        // set CAN_0.IE. set CAN_0.IE.BEUE and CAN_0.IE.BECE to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET, value | BIT_INTERRUPT_BEU | BIT_INTERRUPT_BEC);
                        //set CAN_0.ILS0R.BEULS and CAN_0.ILS0R.BECLS to 0x3
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET, value | (BITS_INTERRUPT_LINE_SELECT_ELOLS_MASK << BITS_INTERRUPT_LINE_SELECT_BEULS_SHIFT) | (BITS_INTERRUPT_LINE_SELECT_BEULS_MASK << BITS_INTERRUPT_LINE_SELECT_BECLS_SHIFT));
                        //clear CAN0.IR register
                        value = raw_readl(0xBB0000 + 0x0034);
                        raw_writel(0xBB0000 + 0x0034, value | (1 << 18));
                        //set CAN_0.ILE.EINT3 to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET, value | BIT_INTERRUPT_LINE_ENABLE_EINT3);

                        //set CAN_0.MCR.CFG to 0x0
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM110:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);

                        int_disable(INT_CRC_IRQ1);
                        //crc_16_ccitt(0x1021), crc_success or crc_fail interrupt.
                        if (crc_init(2, 3) != E_OK) {
                                EMBARC_PRINTF("crc init fail锛乗n");
                        }
                        if (int_handler_install(INT_CRC_IRQ1, func_safety_crc_isr) != E_OK)
                                EMBARC_PRINTF("INT_CRC_IRQ1_init_error: %d.\n", INT_CRC_IRQ1);
                        //set DMU.IRQ_ENA_64_95.65 bit to 0x1
                        value = raw_readl(REL_REGBASE_DMU + REG_DMU_IRQ_ENA64_95_OFFSET);
                        raw_writel(REL_REGBASE_DMU + REG_DMU_IRQ_ENA64_95_OFFSET, value | (1 << 1));
                        hw_crc_interrupt_en(INT_HW_CRC_COMPLETE, 0);
                        hw_crc_interrupt_en(INT_HW_CRC_FAIL, 1);
                        hw_crc_interrupt_en(INT_HW_CRC_SUCCESS, 0);

                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_13_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_13_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        int_enable(INT_CRC_IRQ1);
                        break;
                case FUNC_SAFETY_ITEM_SM113:
                        //set EMU.DIG_ERR_SS1_MARK.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable cpu watchdog timer, period and event_timeou

                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_16_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_16_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //configure can

                        //enable test and check results
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        // initialize can, done in main()

                        // set CAN_0.IE.PEDE to 0x1
                        value = raw_readl(0xBB0000 + 0x0038);
                        raw_writel(0xBB0000 + 0x0038, value | (1 << 23));
                        //set CAN_0.ILS0R.PEDELS to 0x2
                        value = raw_readl(0xBB0000 + 0x003C);
                        raw_writel(0xBB0000 + 0x003C, value | (1 << 17));
                        //set CAN_0.ILE.EINT2 to 0x1
                        value = raw_readl(0xBB0000 + 0x0044);
                        raw_writel(0xBB0000 + 0x0044, value | (1 << 2));
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_18_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_18_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //config iic
                        func_safety_sm202_iic_init();
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);

                        int_disable(INT_I2C_M_IRQ);
                        if (int_handler_install(INT_I2C_M_IRQ, func_safety_iic_isr) != E_OK)
                                EMBARC_PRINTF("INT_I2C_M_IRQ_init_error: %d.\n", INT_I2C_M_IRQ);
                        //set DMU.IRQ_ENA_64_95.81 bit to 0x1
                        value = raw_readl(REL_REGBASE_DMU + REG_DMU_IRQ_ENA64_95_OFFSET);
                        raw_writel(REL_REGBASE_DMU + REG_DMU_IRQ_ENA64_95_OFFSET,
                                        value | (1 << 17));
                        dw_i2c_int_mask(1 << DW_I2C_TX_ABRT);

                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_13_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_13_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        int_enable(INT_I2C_M_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        int_enable(INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_14_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_14_SHIFT);
                        // configure radio

                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        value = raw_readl(REG_EMU_DIG_TEST_ENA);
                        raw_writel(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        int_enable(INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_15_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_15_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                        //config iic
                        func_safety_sm202_iic_init();
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //config iic
                        MAX20430A_wdt_init();
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        MAX20430A_wdt_on(SAFE_STATE_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK)
                                EMBARC_PRINTF("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                default:
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS2) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM1:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio, false);
                        if (IRQ_value) {
                                EMBARC_PRINTF("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);
                        }
                        //set EMU.RF_ERR_SS2_MARK.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.LDO_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_TEST_ENA.LDO_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 0));
                        break;
                case FUNC_SAFETY_ITEM_SM2:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, false);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM2 AVDD33 Monitor IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.AVDD33_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_TEST_ENA.AVDD33_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 1));
                        break;
                case FUNC_SAFETY_ITEM_SM3:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, false);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM3 DVDD11 Monitor IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.DVDD11_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_TEST_ENA.DVDD11_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 2));
                        break;
                case FUNC_SAFETY_ITEM_SM4:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, false);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM4 BG Monitor IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.VBG_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_TEST_ENA.VBG_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 3));
                        break;
                case FUNC_SAFETY_ITEM_SM5:
                        //set EMU.RF_ERR_SS2_MARK.PLL_LOCK_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.PLL_LOCK_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //configure radio
                        fmcw_radio_reg_write(NULL, 0, 0);
                        fmcw_radio_reg_write(NULL, 125, 0x15);
                        fmcw_radio_reg_write(NULL, 0, 0xa);
                        fmcw_radio_reg_write(NULL, 110, 0x2);
                        //set EMU.RF_TEST_ENA.PLL_LOCK_DT bit to 0x1
                        //value = raw_readl(REG_EMU_RF_TEST_ENA);
                        //raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 4));
                        break;
                case FUNC_SAFETY_ITEM_SM6:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(
                                        &bb->radio);
                        if (IRQ_value) {
                                EMBARC_PRINTF("SM6 RF Power Detector IRQ_Value =%d\n", IRQ_value);
                        }
                        //set EMU.RF_ERR_SS2_MARK.RF_POWER_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.RF_POWER_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_TEST_ENA.RF_POWER_DT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 5));
                        break;
                case FUNC_SAFETY_ITEM_SM7:
                        //set EMU.RF_ERR_SS2_MARK.TX_BALL_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.TX_BALL_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //configure radio

                        //set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 6));
                        break;
                case FUNC_SAFETY_ITEM_SM8:
                case FUNC_SAFETY_ITEM_SM9:
                case FUNC_SAFETY_ITEM_SM10:
                        //set EMU.RF_ERR_SS2_MARK.SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.SAT_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //configure radio & bb

                        break;
                case FUNC_SAFETY_ITEM_SM11:
                        //configure radio & bb
                        IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, false);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM11 IF Loopback IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.IF_LP to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, false);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM12 RF Loopback IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.RX_LP to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM13:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM13 Chirp Monitor IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.CHIRP_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_TEST_ENA.CHIRP_MT bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 8));
                        break;
                case FUNC_SAFETY_ITEM_SM14:
                        //configure radio
                        IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, false);
                        if (IRQ_value)
                                EMBARC_PRINTF("SM14 Over Temp Detector IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.TEMP_OVERbit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_TEST_ENA.TEMP_OVER bit to 0x1
                        value = raw_readl(REG_EMU_RF_TEST_ENA);
                        raw_writel(REG_EMU_RF_TEST_ENA, value | (1 << 9));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        //set EMU.RF_ERR_SS2_MARK.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_12_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.GAIN_CK to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_12_SHIFT);
                        //configure radio & bb

                        //enable test and check results

                        break;
                case FUNC_SAFETY_ITEM_SM101:
                        //set EMU.DIG_ERR_SS2_MARK.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //configure radio

                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set EMU.BB_LBIST_ENA to 0x1
                        raw_writel(REG_EMU_BB_LBIST_ENA, 0x1);
                        break;
                case FUNC_SAFETY_ITEM_SM102:
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS2_MARK.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM103:
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS2_MARK.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM104:
                        //set EMU.DIG_ERR_SS2_MARK.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_writel(BB_REG_SYS_ECC_ENA, 0x7fff);
                        break;
                case FUNC_SAFETY_ITEM_SM105:
                        //set EMU.DIG_ERR_SS2_MARK.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_writel(BB_REG_SYS_ECC_ENA, 0x7fff);
                        break;
                case FUNC_SAFETY_ITEM_SM106:
                        //set EMU.DIG_ERR_SS2_MARK.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set DMU.MEMINI_ENA to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET, 0x01);
                        break;
                case FUNC_SAFETY_ITEM_SM107:
                        //set EMU.DIG_ERR_SS2_MARK.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //set DMU.MEMRUN_ENA to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET, 0x01);
                        break;
                case FUNC_SAFETY_ITEM_SM108:
                        //set EMU.DIG_ERR_SS2_MARK.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //set DMU.OTP_ECC_EN to 0x1
                        raw_writel(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET, 0x01);
                        break;
                case FUNC_SAFETY_ITEM_SM109:
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //set CAN_0.MCR.CFG to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
                        //set CAN_0.MCR.ECCENA to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_ECCENA);
                        // set CAN_0.IE. set CAN_0.IE.BEUE and CAN_0.IE.BECE to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET, value | BIT_INTERRUPT_BEU | BIT_INTERRUPT_BEC);
                        //set CAN_0.ILS0R.BEULS and CAN_0.ILS0R.BECLS to 0x3
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET, value | (BITS_INTERRUPT_LINE_SELECT_ELOLS_MASK << BITS_INTERRUPT_LINE_SELECT_BEULS_SHIFT) | (BITS_INTERRUPT_LINE_SELECT_BEULS_MASK << BITS_INTERRUPT_LINE_SELECT_BECLS_SHIFT));
                        //clear CAN0.IR register
                        value = raw_readl(0xBB0000 + 0x0034);
                        raw_writel(0xBB0000 + 0x0034, value | (1 << 18));
                        //set CAN_0.ILE.EINT3 to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET, value | BIT_INTERRUPT_LINE_ENABLE_EINT3);

                        //set CAN_0.MCR.CFG to 0x0
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));
                        break;
                case FUNC_SAFETY_ITEM_SM113:
                        //set EMU.DIG_ERR_SS2_MARK.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable cpu watchdog timer, period and event_timeout

                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_16_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_16_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //configure can
                        //enable test and check results
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        // initialize can, done in main()
                        // set CAN_0.IE.PEDE to 0x1
                        value = raw_readl(0xBB0000 + 0x0038);
                        raw_writel(0xBB0000 + 0x0038, value | (1 << 23));
                        //set CAN_0.ILS0R.PEDELS to 0x2
                        value = raw_readl(0xBB0000 + 0x003C);
                        raw_writel(0xBB0000 + 0x003C, value | (1 << 17));
                        //set CAN_0.ILE.EINT2 to 0x1
                        value = raw_readl(0xBB0000 + 0x0044);
                        raw_writel(0xBB0000 + 0x0044, value | (1 << 2));
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        //set EMU.DIG_ERR_SS2_MARK.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_18_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_18_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        //set EMU.DIG_ERR_SS2_MARK.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_19_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //set EMU.DIG_ERR_SS2_MARK.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_13_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_13_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //set EMU.DIG_ERR_SS2_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        //set EMU.DIG_ERR_SS2_MARK.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_14_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_14_SHIFT);
                        // configure radio

                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        value = raw_readl(REG_EMU_DIG_TEST_ENA);
                        raw_writel(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //set EMU.DIG_ERR_SS2_MARK.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_15_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_15_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                        //config iic
                        func_safety_sm202_iic_init();
                        //set EMU.DIG_ERR_SS2_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //config iic
                        MAX20430A_wdt_init();
                        //set EMU.DIG_ERR_SS2_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        MAX20430A_wdt_on(SAFE_STATE_SS2);
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //set EMU.DIG_ERR_SS2_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        break;
                default:
                        break;
                }
        }
}

/****************************************************
 * NAME         : func_safety_error_handler
 * DESCRIPTIONS : func safety error handler
 * INPUT        : func_safety_sm_num, test num
 *                func_safety_error_type, ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_error_handler(uint16_t func_safety_sm_num,
                uint8_t func_safety_error_type) {
        uint32_t value;
        //uint8_t IRQ_value;
        //baseband_t *bb = baseband_get_cur_bb();

        if (func_safety_error_type == SAFE_STATE_IRQ) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM11:
                        // set EMU.RF_ERR_STA.IF_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.IF_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 10));
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // set EMU.RF_ERR_STA.RX_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.RX_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 11));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        // set EMU.RF_ERR_STA.GAIN_CK bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.GAIN_CK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 12));
                        break;
                case FUNC_SAFETY_ITEM_SM110:
                        //write SM110 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 7));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        // set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 16));
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        // set EMU.DIG_ERR_STA.SPI_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.SPI_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 18));
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        // set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 19));
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //write SM128 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 9));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //write SM129 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 10));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        // set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 15));
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                        //write SM202 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 1));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM203:
                        //write SM203 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 2));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM204:
                        //write SM204 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 3));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM205:
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //write SM206 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 5));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM207:
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //EMBARC_PRINTF("EMU.DIG_ERR_STA_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //value = raw_readl(REG_EMU_DIG_ERR_STA);
                        //EMBARC_PRINTF("EMU.DIG_ERR_STA_READ: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value);
                        //wait cpu interrupt 22
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //write SM805 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 11));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                default:
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS1) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM11:
                        // set EMU.RF_ERR_STA.IF_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.IF_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 10));
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // set EMU.RF_ERR_STA.RX_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.RX_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 11));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        // set EMU.RF_ERR_STA.GAIN_CK bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.GAIN_CK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 12));
                        break;
                case FUNC_SAFETY_ITEM_SM110:
                        //write SM110 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 7));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        // set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 16));
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        // set EMU.DIG_ERR_STA.SPI_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.SPI_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 18));
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        // set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 19));
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //write SM128 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 9));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //write SM129 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 10));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        // set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 15));
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                        //write SM202 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 1));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM203:
                        //write SM203 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 2));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM204:
                        //write SM204 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 3));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM205:
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //write SM206 ss1 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 5));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM207:
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //EMBARC_PRINTF("EMU.DIG_ERR_STA_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //value = raw_readl(REG_EMU_DIG_ERR_STA);
                        //EMBARC_PRINTF("EMU.DIG_ERR_STA_READ: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value);
                        //wait cpu interrupt 22
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //write SM805 error code bit
                        value = raw_readl(REG_EMU_SPARED_0);
                        raw_writel(REG_EMU_SPARED_0, value | (1 << 11));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                default:
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS2) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM11:
                        // set EMU.RF_ERR_STA.IF_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.IF_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 10));
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // set EMU.RF_ERR_STA.RX_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.RX_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 11));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        // set EMU.RF_ERR_STA.GAIN_CK bit to 0x1
                        EMBARC_PRINTF("set EMU.RF_ERR_STA.GAIN_CK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_RF_ERR_STA);
                        raw_writel(REG_EMU_RF_ERR_STA, value | (1 << 12));
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        // set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 16));
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        // set EMU.DIG_ERR_STA.SPI_LP bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.SPI_LP bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 18));
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        // set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 19));
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //wait enter SS2 state
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_TEST_ENA);
                        raw_writel(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1
                        EMBARC_PRINTF("set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1.\n");
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 15));
                        //enter SS2 state
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                case FUNC_SAFETY_ITEM_SM206:
                case FUNC_SAFETY_ITEM_SM207:
                case FUNC_SAFETY_ITEM_SM805:
                        //set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        value = raw_readl(REG_EMU_DIG_ERR_STA);
                        raw_writel(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //wait enter SS2 state
                        break;
                default:
                        break;
                }
        }
}

/****************************************************
 * NAME         : func_safety_check_emu_error_code
 * DESCRIPTIONS : check reboot cnt and ss1 error code
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_check_emu_error_code(void) {
        uint32_t value;

        //check emu reboot cnt
        EMBARC_PRINTF("\r\n");
        value = raw_readl(REG_EMU_REBOOT_CNT);
        EMBARC_PRINTF("EMU_REBOOT_CNT_READ: 0x%x, value: %d.\n", REG_EMU_REBOOT_CNT,
                        value);
        EMBARC_PRINTF("\r\n");
        //check emu rf SS1 error code
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE0);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE0: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS1_CODE0, value);
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE1);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE1: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS1_CODE1, value);
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE2);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE2: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS1_CODE2, value);
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE3);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE3: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS1_CODE3, value);
        EMBARC_PRINTF("\r\n");
        //check emu digital SS1 error code
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE0);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE0: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE0, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE1);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE1: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE1, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE2);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE2: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE2, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE3);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE3: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE3, value);
        EMBARC_PRINTF("\r\n");
        //check emu rf SS2 error code
        value = raw_readl(REG_EMU_RF_ERR_SS2_CODE);
        EMBARC_PRINTF("EMU_RF_ERR_SS2_CODE: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS2_CODE, value);
        //check emu digital SS2 error code
        value = raw_readl(REG_EMU_DIG_ERR_SS2_CODE);
        EMBARC_PRINTF("EMU_DIG_ERR_SS2_CODE: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS2_CODE, value);
        EMBARC_PRINTF("\r\n");
        //check SM202 SM203 SM204 SM205 SM206 error code
        value = raw_readl(REG_EMU_SPARED_0);
        EMBARC_PRINTF("EMU_SOFTWARE_ERR_SS1_CODE: 0x%x, value: 0x%x.\n",
        REG_EMU_SPARED_0, value);
        EMBARC_PRINTF("\r\n");
}

/****************************************************
 * NAME         : fsm_i2c_Periodic_Software_readback
 * DESCRIPTIONS : SM202 IIC Periodic Software Readback of Writen,
 *                Readback of I2C Writen data and confirm
 * INPUT        : slave_addr, register addr, written data ptr, len, error_type, safety test num
 * OUTPUT       : None
 *****************************************************/
static void fsm_i2c_Periodic_Software_readback(uint32_t slave_addr, uint8_t addr,
                uint8_t *data, uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num) {
        int32_t result;
        uint32_t i;
        bool error_flag = false;

        //gpio_write(LED_D1_NO, LED_D1_ON);//start test
        //result = i2c_read(slave_addr, addr, fsm_test_rd_buf, len);
        i2c_transfer_config(0, slave_addr, addr, 1);
        result = i2c_read(fsm_test_rd_buf, len);
        for (i = 0; i < len; i++) {
                if (fsm_test_rd_buf[i] != data[i]) {
                        error_flag = true;
                        break;
                }
        }

        if (error_flag == false) {
                EMBARC_PRINTF("readbackdata:");
                for (i = 0; i < len; i++) {
                        EMBARC_PRINTF(" %x", fsm_test_rd_buf[i]);
                }
                EMBARC_PRINTF("\n");
                EMBARC_PRINTF("iic read ok.\n");
                EMBARC_PRINTF("I2C_PERIODIC_SOFTWARE_READBACK_SAFETY_PASS!\n");
        } else {
                EMBARC_PRINTF("result: %f \n", result);
                EMBARC_PRINTF("iic read fail.\n");
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                EMBARC_PRINTF("I2C_PERIODIC_SOFTWARE_READBACK_SAFETY_ERROR!\n");
        }
}

/****************************************************
 * NAME         : fsm_i2c_readback_write
 * DESCRIPTIONS : SM203 IIC Readback of Writen,
 *                Readback of I2C Writen data and confirm
 * INPUT        : slave_addr, register addr, written data ptr, len, error_type, safety test num
 * OUTPUT       : None
 *****************************************************/
static void fsm_i2c_readback_write(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num) {
        int32_t result;
        uint32_t i;
        bool error_flag = false;

        EMBARC_PRINTF("writedata:");
        for (i = 0; i < len; i++) {
                EMBARC_PRINTF(" %x", data[i]);
        }
        EMBARC_PRINTF("\n");

        //result = i2c_write(slave_addr, addr, data, len);
        i2c_transfer_config(0, slave_addr, addr, 1);
        result = i2c_write(data, len);
        if (result == E_OK) {
                EMBARC_PRINTF("iic write ok.\n");
        } else {
                EMBARC_PRINTF("iic write fail.\n");
                EMBARC_PRINTF("result: %f \n", result);
        }

        //gpio_write(LED_D1_NO, LED_D1_ON);//start test
        //result = i2c_read(slave_addr, addr, fsm_test_rd_buf, len);
        i2c_transfer_config(0, slave_addr, addr, 1);
        result = i2c_read(fsm_test_rd_buf, len);
        for (i = 0; i < len; i++) {
                if (fsm_test_rd_buf[i] != data[i]) {
                        error_flag = true;
                        break;
                }
        }

        if (error_flag == false) {
                EMBARC_PRINTF("readbackdata:");
                for (i = 0; i < len; i++) {
                        EMBARC_PRINTF(" %x", fsm_test_rd_buf[i]);
                }
                EMBARC_PRINTF("\n");
                EMBARC_PRINTF("iic read ok.\n");
                EMBARC_PRINTF("I2C_READBACK_WRITEN_SAFETY_PASS!\n");
        } else {
                EMBARC_PRINTF("result: %f \n", result);
                EMBARC_PRINTF("iic read fail.\n");
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                EMBARC_PRINTF("I2C_READBACK_WRITEN_SAFETY_ERROR!\n");
        }
}

/****************************************************
 * NAME         : fsm_i2c_crc_write
 * DESCRIPTIONS : SM204 IIC CRC X8+X2+X1+1,
 *                write data and add one crc byte after data,readback and confirm
 * INPUT        : slave_addr, register addr, written data ptr, len, error_type
 * OUTPUT       : None
 *****************************************************/
static void fsm_i2c_crc_write(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num) {
        int32_t result;
        uint8_t *ptr_write;
        uint8_t crc;

        ptr_write = data;
        crc = CRC8_ROHC(data, len);
        ptr_write[len] = crc;
        //result = i2c_write(slave_addr, addr, ptr_write, len + 1);
        i2c_transfer_config(0, slave_addr, addr, 1);
        result = i2c_write(ptr_write, len + 1);

        if (result == E_OK) {
                EMBARC_PRINTF("iic write ok.\n");
        } else {
                EMBARC_PRINTF("iic write fail.\n");
                EMBARC_PRINTF("result: %f \n", result);
        }
}

/****************************************************
 * NAME         : fsm_i2c_crc_read
 * DESCRIPTIONS : SM204 IIC CRC X8+X2+X1+1,
 *                readback data and crc byte, crc data and confirm
 * INPUT        : slave_addr, register addr, read data ptr, len, error_type, safety test num
 * OUTPUT       : None
 *****************************************************/
static void fsm_i2c_crc_read(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num) {
        int32_t result;
        uint8_t crc;

        //gpio_write(LED_D1_NO, LED_D1_ON);//start test
        //result = i2c_read(slave_addr, addr, data, len + 1);
        i2c_transfer_config(0, slave_addr, addr, 1);
        result = i2c_read(data, len + 1);
        crc = CRC8_ROHC(data, len);
        if (crc == data[len]) {
                EMBARC_PRINTF("iic read ok.\n");
                EMBARC_PRINTF("readback: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", data[0],
                                data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                                data[8]);
                EMBARC_PRINTF("I2C_CRC_READ_SAFETY_PASS!\n");
        } else {
                EMBARC_PRINTF("iic read fail.\n");
                EMBARC_PRINTF("result: %f \n", result);
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                EMBARC_PRINTF("I2C_CRC_READ_SAFETY_ERROR!\n");
        }
}

/****************************************************
 * NAME         : fsm_i2c_ack_write
 * DESCRIPTIONS : SM128 IIC ACK
 *                write data to iic, wait no ACK
 * INPUT        : slave_addr, register addr, read data ptr, len, error_type, safety test num
 * OUTPUT       : None
 *****************************************************/
static void fsm_i2c_ack_write(uint32_t slave_addr, uint8_t addr, uint8_t *data,
                uint32_t len, uint8_t func_safety_error_type,
                uint16_t func_safety_sm_num) {
        int32_t result;

        //result = i2c_write(slave_addr, addr, data, len);
        i2c_transfer_config(0, slave_addr, addr, 1);
        result = i2c_write(data, len);
        if (result == E_OK) {
                EMBARC_PRINTF("iic write ok.\n");
        } else {
                EMBARC_PRINTF("iic write fail.\n");
                EMBARC_PRINTF("result: %f \n", result);
        }
}

/****************************************************
 * NAME         : fsm_on_time_error_pin_check
 * DESCRIPTIONS : SM205 On Time Error Pin Check,
 *                write 0 or 1 to EMU_ERROR_OUT and EMU_SAFE_STATE,
 *                then confirm the output level
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void fsm_on_time_error_pin_check(void) {
        static uint8_t runcount = 1;

        raw_writel(REG_EMU_SAFETY_KEY1, 1); //Set the register EMU_SAFETY_KEY1 to a non-zero value.
        raw_writel(REG_EMU_SAFETY_KEY2, 1); //Set the register EMU_SAFETY_KEY2 to the same value with EMU_SAFETY_KEY1.
        raw_writel(REG_EMU_SAFETY_PAD_CTRL, 1); //Set the register EMU_SAFETY_PAD_CTRL to 1.

        if (runcount % 2) {
                raw_writel(REG_EMU_ERROR_OUT, 0); //Change the ERROR pin output value by setting the register EMU_ERROR_OUT_O to 0 or 1.
                raw_writel(REG_EMU_SAFE_STATE, 0); //Change the SAFE_STATE pin output value by setting the register EMU_SAFE_STATE_O to 0 or 1.
        } else {
                raw_writel(REG_EMU_ERROR_OUT, 1);
                raw_writel(REG_EMU_SAFE_STATE, 1);
        }

        runcount++;
}

/****************************************************
 * NAME         : MAX20430A_wdt_init
 * DESCRIPTIONS : PMU MAX20430A wdt init
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void MAX20430A_wdt_init(void) {
        i2c_params_t xfer_params_max20430a;

        uint32_t ref_clock = clock_frequency(I2C_CLOCK);

        xfer_params_max20430a.addr_mode = 0;
        xfer_params_max20430a.speed_mode = 1;
        xfer_params_max20430a.timing->scl_h_cnt = ref_clock / 1000000;
        xfer_params_max20430a.timing->scl_l_cnt = ref_clock / 1000000;
        xfer_params_max20430a.timing->sda_rx_hold = 0;
        xfer_params_max20430a.timing->sda_tx_hold = 1;
        xfer_params_max20430a.timing->spike_len = 5;

        //i2c init
        if (i2c_init(0, &xfer_params_max20430a) == E_OK) {
                EMBARC_PRINTF("max20430a iic initial ok.\n");
        } else {
                EMBARC_PRINTF("max20430a iic initial fail.\n");
        }
}

/****************************************************
 * NAME         : MAX20430A_LFSR
 * DESCRIPTIONS : PMU MAX20430A LFSR/CRC Code: x^8 + x^6 + x^5 + x^4 + 1
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
uint8_t MAX20430A_LFSR(uint8_t iKey) {
        uint8_t lfsr = iKey;

        uint8_t bit = ((lfsr >> 7) ^ (lfsr >> 5) ^ (lfsr >> 4) ^ (lfsr >> 3)) & 1;
        lfsr = (lfsr << 1) | bit;

        return lfsr;
}

/****************************************************
 * NAME         : MAX20430A_wdt_read_status
 * DESCRIPTIONS : PMU MAX20430A wdt read status
 * INPUT        : safety test num
 * OUTPUT       : None
 *****************************************************/
void MAX20430A_wdt_read_status(uint8_t func_safety_error_type, uint16_t func_safety_sm_num) {
        int32_t result;
        uint8_t rd_buf[1] = { 0x00 };

        //result = i2c_read(0x38, 0x0D, rd_buf, 1);                //read wdt register
        i2c_transfer_config(0, 0x38, 0x0D, 1);
        result = i2c_read(rd_buf, 1);                //read wdt register
        if (result == E_OK) {
                //EMBARC_PRINTF("STATWD: 0x%x \n", rd_buf[0]);
                if (external_wdt_test_flag == true) {
                        if ((rd_buf[0] & 0x10) == 0x00) {
                                //wdt timeout
                                EMBARC_PRINTF("WDT TIMEOUT! \n");
                                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                        }
                }
        } else {
                EMBARC_PRINTF("read STATWD fail.\n");
        }
}

/****************************************************
 * NAME         : MAX20430A_wdt_feed
 * DESCRIPTIONS : PMU MAX20430A wdt feed periodic
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void MAX20430A_wdt_feed(void) {
        int32_t result;
        uint8_t rd_buf[9] = { 0x00 };
        uint8_t value = 0;

        //result = i2c_read(0x38, 0x015, &rd_buf[2], 1);         //read wdkey register
        i2c_transfer_config(0, 0x38, 0x15, 1);
        result = i2c_read(&rd_buf[2], 1);         //read wdkey register
        if (result == E_OK) {
                //EMBARC_PRINTF("WDKEY: 0x%x \n", rd_buf[2]);
                value = rd_buf[2];
        } else {
                EMBARC_PRINTF("read WDKEY fail.\n");
        }
        value = MAX20430A_LFSR(value);
        //EMBARC_PRINTF("FEED VALUE: 0x%x \n", value);
        //result = i2c_write(0x38, 0x15, &value, 1); //write correct value to the WDKEY will result in a valid watchdog refresh signal
        result = i2c_write(&value, 1); //write correct value to the WDKEY will result in a valid watchdog refresh signal
        //MAX20430A_wdt_read_status();
}

/****************************************************
 * NAME         : MAX20430A_wdt_on
 * DESCRIPTIONS : PMU MAX20430A wdt on
 * INPUT        : error_type
 * OUTPUT       : None
 *****************************************************/
void MAX20430A_wdt_on(uint8_t func_safety_error_type)
{
        int32_t result;
        uint8_t rd_buf[9] = { 0x00 };
        uint8_t value;

        //config OV[x] and UV[x] mapped to RESETB pin
        value = 0x0F;
        //result = i2c_write(0x38, 0x07, &value, 1);
        //result = i2c_read(0x38, 0x07, &rd_buf[0], 1);
        i2c_transfer_config(0, 0x38, 0x07, 1);
        result = i2c_write(&value, 1);
        result = i2c_read(&rd_buf[0], 1);
        if (result == E_OK) {
                //EMBARC_PRINTF("PINMAP1: 0x%x \n", rd_buf[0]);
        } else {
                EMBARC_PRINTF("read PINMAP1 fail.\n");
        }
        //set watchdog mode and tWDCLK
        value = 0x04;        //Challenge/response watchdog enabled
        //value = 0x01;//Challenge/response watchdog enabled
        //value = 0x41;//Standard windowed watchdog enabled
        //result = i2c_write(0x38, 0x12, &value, 1);
        //result = i2c_read(0x38, 0x012, &rd_buf[0], 1);
        i2c_transfer_config(0, 0x38, 0x12, 1);
        result = i2c_write(&value, 1);
        result = i2c_read(&rd_buf[0], 1);
        if (result == E_OK) {
                //EMBARC_PRINTF("WDCDIV: 0x%x \n", rd_buf[0]);
        } else {
                EMBARC_PRINTF("read WDCDIV fail.\n");
        }
        //config tWD1 and tWD2
        value = 0x0F;
        //result = i2c_write(0x38, 0x13, &value, 1);
        //result = i2c_read(0x38, 0x013, &rd_buf[1], 1);
        i2c_transfer_config(0, 0x38, 0x13, 1);
        result = i2c_write(&value, 1);
        result = i2c_read(&rd_buf[1], 1);
        if (result == E_OK) {
                //EMBARC_PRINTF("WDCFG1: 0x%x \n", rd_buf[1]);
        } else {
                EMBARC_PRINTF("read WDCFG1 fail.\n");
        }
        //watchdog enable and start
        value = 0x08;
        //result = i2c_write(0x38, 0x14, &value, 1);        //wdt on
        //result = i2c_read(0x38, 0x014, &rd_buf[2], 1);        //read wdt register
        i2c_transfer_config(0, 0x38, 0x14, 1);
        result = i2c_write(&value, 1);        //wdt on
        result = i2c_read(&rd_buf[2], 1);        //read wdt register
        if (result == E_OK) {
                //EMBARC_PRINTF("WDCFG2: 0x%x \n", rd_buf[2]);
        } else {
                EMBARC_PRINTF("read WDCFG2 fail.\n");
        }

        MAX20430A_wdt_feed();
        external_wdt_on_flag = true;
        max20430_feed_dog_time_cnt = 0x00;
}

/****************************************************
 * NAME         : func_safety_init
 * DESCRIPTIONS : fucntional safety test item(sm) initial
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_init(func_safety_t *fsm)
{
        uint8_t i;
#if (INTER_FRAME_POWER_SAVE == 1)
        baseband_t *bb = baseband_get_bb(0);
#endif

        //used to detect Pulse Width by toggle gpio led_d1.
        led_d1_init();
        gpio_write(LED_D1_NO, LED_D1_OFF);        //output low level

        //enable reboot limit
        raw_writel(REG_EMU_REBOOT_LIMIT, 0x3);

#if (INTER_FRAME_POWER_SAVE == 1)
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        for (i = 0; i < FSM_CONF_MAX; ++i)
        {
                if (fsmConfList[i].open_flag == true)
                        func_safety_sm_init(fsmConfList[i].sm_num, fsmConfList[i].error_type);
        }

        /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM1, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM2, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM3, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM4, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM8, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM11, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM12, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM13, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM201, SAFE_STATE_SS1);

        func_safety_sm_init(FUNC_SAFETY_ITEM_SM101, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM102, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM103, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM104, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM105, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM106, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM107, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM108, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM109, SAFE_STATE_SS1);

        func_safety_sm_init(FUNC_SAFETY_ITEM_SM120, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM121, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM122, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM123, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM124, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM125, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM126, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM129, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM130, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM133, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM202, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM805, SAFE_STATE_SS1);*/

#if (INTER_FRAME_POWER_SAVE == 1)
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        EMBARC_PRINTF("/*** func_safety_sm_init done! ***/\n\r");

        /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM1, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM2, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM3, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM4, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM8, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM11, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM12, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM13, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_SS1);*/

        /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM1, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM2, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM3, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM4, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM8, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM11, SAFE_STATE_SS1);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM12, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM13, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_SS1);*/

        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM1, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM2, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM3, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM4, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM8, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM11, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM12, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM13, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM101, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM102, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM103, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM104, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM105, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM106, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM107, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM108, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM109, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM120, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM121, SAFE_STATE_IRQ);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM126, SAFE_STATE_IRQ);

        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_SS1);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_SS1);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_SS1);
         /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM11, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM12, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM1, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM2, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM3, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM4, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM13, SAFE_STATE_SS1);
         //func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM201, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM8, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM9, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM10, SAFE_STATE_SS1);*/

         /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM6, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM11, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM12, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM1, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM2, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM3, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM4, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM5, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM13, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM14, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM201, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM8, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM9, SAFE_STATE_SS2);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM10, SAFE_STATE_SS2);*/

        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM101, SAFE_STATE_SS1);
        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM201, SAFE_STATE_SS1);
        /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM102, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM103, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM104, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM105, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM106, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM107, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM108, SAFE_STATE_SS1);
        func_safety_sm_init(FUNC_SAFETY_ITEM_SM109, SAFE_STATE_SS1);*/

        //func_safety_sm_init(FUNC_SAFETY_ITEM_SM120, SAFE_STATE_SS1);
         /*func_safety_sm_init(FUNC_SAFETY_ITEM_SM113, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM120, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM121, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM122, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM123, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM124, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM125, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM126, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM130, SAFE_STATE_SS1);
         func_safety_sm_init(FUNC_SAFETY_ITEM_SM133, SAFE_STATE_SS1);*/

//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM129, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM202, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM128, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM110, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM120, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM126, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM113, SAFE_STATE_SS1);
//      func_safety_sm_init(FUNC_SAFETY_ITEM_SM206, SAFE_STATE_SS1);
}

/****************************************************
 * NAME         : func_safety_test_handler
 * DESCRIPTIONS : used to start one item test manually
 * INPUT        : func_safety_sm_num: sm number, func_safety_error_type: error type, ss1 or ss2.
 * OUTPUT       : None
 *****************************************************/
void func_safety_test_handler(int32_t func_safety_sm_num, uint8_t func_safety_error_type)
{
#if (FUNC_SAFETY_CLI == 1)
        uint32_t value;
        uint32_t IRQ_value;
        bool ret;
        baseband_t *bb = baseband_get_cur_bb();
        sensor_config_t *cfg = sensor_config_get_cur_cfg();
        //static uint8_t cnt = 1;
        //runtime test
        /*float time_calc_t1 = 0.0;
        float time_calc_t2 = 0.0;
        float time_calc_t12 = 0.0;*/

        switch (func_safety_sm_num) {
        case FUNC_SAFETY_ITEM_SM1:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_ldo_monitor_fault_injection_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm1_runtime = %6.4f\n", time_calc_t12);*/

                /*if(cnt%2){
                 IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio,true);
                 EMBARC_PRINTF("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);
                 }else{
                 IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio,false);
                 EMBARC_PRINTF("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);
                 }
                 cnt++;*/
                break;
        case FUNC_SAFETY_ITEM_SM2:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM2 AVDD33 Monitor IRQ = %d\n", IRQ_value);
                break;
        case FUNC_SAFETY_ITEM_SM3:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM3 DVDD11 Monitor IRQ = %d\n", IRQ_value);
                break;
        case FUNC_SAFETY_ITEM_SM4:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM4 BG Monitor IRQ = %d\n", IRQ_value);
                break;
        case FUNC_SAFETY_ITEM_SM5:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                fmcw_radio_reg_write(NULL, 0, 0);
                fmcw_radio_reg_write(NULL, 125, 0x14);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                break;
        case FUNC_SAFETY_ITEM_SM6:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rfpower_detector_fault_injection_IRQ(&bb->radio, 5);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM6 RF Power Detector IRQ_Value =%d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm6_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM7:
                {
                        uint32_t ECC_ENABLE = raw_readl(BB_REG_SYS_ECC_ENA);
                        uint32_t ECC_SB_STAT = raw_readl(BB_REG_SYS_ECC_SB_STATUS);
                        uint32_t ECC_DB_STAT =  raw_readl(BB_REG_SYS_ECC_DB_STATUS);
                        EMBARC_PRINTF("ECC_ENABLE_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_ENA, ECC_ENABLE);
                        EMBARC_PRINTF("ECC_SB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_SB_STATUS, ECC_SB_STAT);
                        EMBARC_PRINTF("ECC_DB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_DB_STATUS, ECC_DB_STAT);
                }
                break;
        case FUNC_SAFETY_ITEM_SM8:
        case FUNC_SAFETY_ITEM_SM9:
        case FUNC_SAFETY_ITEM_SM10:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                fmcw_radio_sm_saturation_detector_fault_injection_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                break;
        case FUNC_SAFETY_ITEM_SM11:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM11 IF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm11_runtime = %6.4f\n", time_calc_t12);
                EMBARC_PRINTF("SM11 IF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);*/
                break;
        case FUNC_SAFETY_ITEM_SM12:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM12 RF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm12_runtime = %6.4f\n", time_calc_t12);
                EMBARC_PRINTF("SM12 RF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);*/
                break;
        case FUNC_SAFETY_ITEM_SM13:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM13 Chirp Monitor IRQ = %d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm13_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM14:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                EMBARC_PRINTF("SM14 Over Temp Detector IRQ = %d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm14_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM201:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                ret = func_safety_sm_vga_consistence_check();
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                if (ret == true)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                func_safety_sm_vga_consistence_check();
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm201_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM101:
                //disable sm6
                func_safety_sm8_enable(fsmConfList[SM_INDEX_6].error_type, false);

                baseband_bist_ctrl(cfg->bb, false);

                //clear sm6 err status and enable sm6
                //value = raw_readl(REG_EMU_RF_ERR_STA);
                //EMBARC_PRINTF("RF_ERR_STA after: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                func_safety_sm8_enable(fsmConfList[SM_INDEX_6].error_type, true);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                baseband_bist_ctrl(cfg->bb, false);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm101_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM102:
                //set CPU.ERP_CTRL to 0x00 to disable cpu ecc
                _arc_aux_write(REG_CPU_ERP_CTRL, 0x00);
                EMBARC_PRINTF("REG_CPU_ERP_CTRL_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, 0x00);
                value = _arc_aux_read(REG_CPU_ERP_CTRL);
                EMBARC_PRINTF("REG_CPU_ERP_CTRL_ADDR_READ: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, value);

                //write data 0x55AA66BB to address 0xA07FFC
                raw_writel(0x107FFC, 0x55AA66BB);
                EMBARC_PRINTF("WRITE_DATA_TO_0xA07FFC_ADDR: 0x%x, value: 0x%x.\n", 0x107FFC, 0x55AA66BB);

                //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                _arc_aux_write(REG_CPU_ERP_CTRL, 0xf);
                EMBARC_PRINTF("REG_CPU_ERP_CTRL_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, 0xf);
                value = _arc_aux_read(REG_CPU_ERP_CTRL);
                EMBARC_PRINTF("REG_CPU_ERP_CTRL_ADDR_READ: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, value);

                //read address 0xA07FFC data
                value = raw_readl(0x107FFC);
                EMBARC_PRINTF("REG_0xA07FFC_ADDR_READ: 0x%x, value: 0x%x.\n", 0x107FFC, value);

                //wait fault
                break;
        case FUNC_SAFETY_ITEM_SM110:
                func_safety_crc_acceleration(func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM113:
                arc_wdt_on_flag = false;
                //enable cpu watchdog timer, period and event_timeout
                break;
        case FUNC_SAFETY_ITEM_SM120:
                //can_loopback_test_flag = true;
                func_safety_sm_can_loopback_init();
                //func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_can_loopback_init();
                //gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm120_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM121:
        case FUNC_SAFETY_ITEM_SM122:
        case FUNC_SAFETY_ITEM_SM123:
        case FUNC_SAFETY_ITEM_SM124:
        case FUNC_SAFETY_ITEM_SM125:
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM126:
                func_safety_sm_spi_loopback(func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_spi_loopback(func_safety_error_type);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm126_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM128:
                fsm_i2c_ack_write(0x50, 0x00, fsm_test_wr_buf, 8, func_safety_error_type, func_safety_sm_num);
                //func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM129:
                func_safety_sm_can_config_reg_protection(func_safety_error_type);
                //func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_can_config_reg_protection(func_safety_error_type);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm129_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM130:
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM133:
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM202:
                fsm_i2c_Periodic_Software_readback(0x50, 0x00, fsm_test_wr_buf, 1, func_safety_error_type, func_safety_sm_num);
                break;
        case FUNC_SAFETY_ITEM_SM203:
                fsm_i2c_readback_write(0x50, 0x00, fsm_test_wr_buf, 8, func_safety_error_type, func_safety_sm_num);
                break;
        case FUNC_SAFETY_ITEM_SM204:
                fsm_i2c_crc_write(0x50, 0x00, fsm_test_wr_buf, 8, func_safety_error_type, func_safety_sm_num);
                fsm_i2c_crc_read(0x50, 0x00, fsm_test_rd_buf, 8, func_safety_error_type, func_safety_sm_num);
                break;
        case FUNC_SAFETY_ITEM_SM205:
                fsm_on_time_error_pin_check();
                break;
        case FUNC_SAFETY_ITEM_SM206:
                external_wdt_test_flag = true;
                break;
        case FUNC_SAFETY_ITEM_SM805:
                func_safety_sm_periodic_readback_configuration_registers(func_safety_error_type);

                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_periodic_readback_configuration_registers(func_safety_error_type);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                EMBARC_PRINTF("\t sm805_runtime = %6.4f\n", time_calc_t12);*/
                break;
        default:
                break;
        }
#endif
}

/****************************************************
 * NAME         : func_safety_sm_vga_consistence_check
 * DESCRIPTIONS : sm201, vga consistence check test handler
 * INPUT        : None
 * OUTPUT       : true or false
 *****************************************************/
static bool func_safety_sm_vga_consistence_check(void)
{
        bool ret = false;
        baseband_t* bb = baseband_get_rtl_frame_type(); /* read back the bank selected in RTL */
        //sensor_config_t* cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;

        int calib_cnt = 1;      // calib times
        //gpio_write(LED_D1_NO, LED_D1_ON);//output low level

        // baseband dump init
        uint32_t old_buf_store = baseband_switch_buf_store(bb_hw,
        SYS_BUF_STORE_ADC);
        uint8_t old_bnk = BB_READ_REG(bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
        uint16_t old_status_en = BB_READ_REG(bb_hw, SYS_ENABLE);

        /* turn off zero doppler removel */
        //bool old_zer_dpl = BB_READ_REG(bb_hw, FFT_ZER_DPL_ENB);
        //BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB  , 0);
        //dmu_adc_reset(); // ADC should reset in cascade
        //close all tx
        fmcw_radio_tx_ch_on(&bb->radio, -1, false);

        while (1) {
                if (calib_cnt > 0) {
                        /* start baseband */
                        baseband_start_with_params(bb, true, false,
                                        (1 << SYS_ENABLE_SAM_SHIFT), true, BB_IRQ_ENABLE_SAM_DONE,
                                        false);
                        /* wait done */
                        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);
                        // Search test target peak in 2D-FFT plane
                        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//output low level

                        uint32_t fft_mem;
                        int ch_index;
                        int tmp_rng_ind;
                        int rng_ind_end;
                        int count;
                        //int16_t *BufPtr;
                        int16_t temp_value1, temp_value2;
                        int32_t rx_sum_src_value_buf[4] = { 0, 0, 0, 0 };
                        float rx_average_src_value_buf[4] = { 0, 0, 0, 0 };
                        uint32_t rx_sum_square_value_buf[4] = { 0, 0, 0, 0 };
                        float rx_sum_diff[4] = { 0, 0, 0, 0 };

                        for (ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {

                                count = 0;

                                //for (int j = 0; j < 10; j++) {
                                tmp_rng_ind = 0;
                                rng_ind_end = 200;
                                for (; tmp_rng_ind < rng_ind_end; tmp_rng_ind++) {
                                        fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index,
                                                        tmp_rng_ind, 0, 0);
                                        temp_value1 = ((int16_t) ((fft_mem >> 16) & 0xFFFF)) / 4;
                                        temp_value2 = ((int16_t) (fft_mem & 0xFFFF)) / 4;
                                        //EMBARC_PRINTF("%d ", temp_value1); // Print int16_t to UART
                                        //EMBARC_PRINTF("%d ", temp_value2); // Print int16_t  to UART

                                        rx_sum_src_value_buf[ch_index] +=
                                                        (temp_value1 + temp_value2);
                                        rx_sum_square_value_buf[ch_index] += (temp_value1
                                                        * temp_value1 + temp_value2 * temp_value2);

                                        count++;
                                        //gpio_write(LED_D1_NO, LED_D1_OFF);
                                }
                                //}
                                //EMBARC_PRINTF("\n");
                                rx_average_src_value_buf[ch_index] =
                                                rx_sum_src_value_buf[ch_index] / 512.0;
                        }
                        //EMBARC_PRINTF("one frame done!\n");
                        //EMBARC_PRINTF("rx_sum_src_value_buf:  %d  %d  %d  %d  \n", rx_sum_src_value_buf[0], rx_sum_src_value_buf[1], rx_sum_src_value_buf[2], rx_sum_src_value_buf[3]);
                        //EMBARC_PRINTF("rx_average_src_value_buf:  %.4f  %.4f  %.4f  %.4f  \n", rx_average_src_value_buf[0], rx_average_src_value_buf[1], rx_average_src_value_buf[2], rx_average_src_value_buf[3]);
                        //EMBARC_PRINTF("rx_sum_square_value_buf:  %d  %d  %d  %d  \n", rx_sum_square_value_buf[0], rx_sum_square_value_buf[1], rx_sum_square_value_buf[2], rx_sum_square_value_buf[3]);

                        for (int k = 0; k < MAX_NUM_RX; k++) {
                                rx_sum_diff[k] = (rx_sum_square_value_buf[k]
                                                - 512 * rx_average_src_value_buf[k]
                                                                * rx_average_src_value_buf[k]);
                        }
                        //EMBARC_PRINTF("rx_sum_diff:  %.4f  %.4f  %.4f  %.4f  \n", rx_sum_diff[0], rx_sum_diff[1], rx_sum_diff[2], rx_sum_diff[3]);

                        //gpio_write(LED_D1_NO, LED_D1_OFF);//output low level
                        //EMBARC_PRINTF("\r\n");
                        float diff1, diff2, diff3, diff4, diff5, diff6;
                        float logrx1, logrx2, logrx3, logrx4;

                        logrx1 = 10 * log10f(rx_sum_diff[0]);
                        logrx2 = 10 * log10f(rx_sum_diff[1]);
                        logrx3 = 10 * log10f(rx_sum_diff[2]);
                        logrx4 = 10 * log10f(rx_sum_diff[3]);
                        //EMBARC_PRINTF("logrx:  %.4f  %.4f  %.4f  %.4f  \n", logrx1, logrx2, logrx3, logrx4);

                        if (logrx1 >= logrx2)
                                diff1 = (logrx1 - logrx2);
                        else
                                diff1 = (logrx2 - logrx1);

                        if (logrx1 >= logrx3)
                                diff2 = (logrx1 - logrx3);
                        else
                                diff2 = (logrx3 - logrx1);

                        if (logrx1 >= logrx4)
                                diff3 = (logrx1 - logrx4);
                        else
                                diff3 = (logrx4 - logrx1);

                        if (logrx2 >= logrx3)
                                diff4 = (logrx2 - logrx3);
                        else
                                diff4 = (logrx3 - logrx2);

                        if (logrx2 >= logrx4)
                                diff5 = (logrx2 - logrx4);
                        else
                                diff5 = (logrx4 - logrx2);

                        if (logrx3 >= logrx4)
                                diff6 = (logrx3 - logrx4);
                        else
                                diff6 = (logrx4 - logrx3);

                        //gpio_write(LED_D1_NO, LED_D1_OFF);//output low level
                        /*EMBARC_PRINTF(
                                        "SM201 dif1: %.2f dif2: %.2f dif3: %.2f dif4: %.2f dif5: %.2f dif6: %.2f\n",
                                        diff1, diff2, diff3, diff4, diff5, diff6);*/

                        //restore tx parameter
                        fmcw_radio_tx_restore(&bb->radio);

                        if ((diff1 > VGA_DIFF_VALUE) || (diff2 > VGA_DIFF_VALUE)
                                        || (diff3 > VGA_DIFF_VALUE) || (diff4 > VGA_DIFF_VALUE)
                                        || (diff5 > VGA_DIFF_VALUE) || (diff6 > VGA_DIFF_VALUE)) {
                                ret = true;
                        } else {
                                ret = false;
                        }

                        baseband_switch_mem_access(bb_hw, old);

                        calib_cnt--;
                } else if (calib_cnt == 0) {
                        break;
                } else {
                        taskYIELD();
                }
        }

        /* restore zero doppler removel */
        //BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB, old_zer_dpl);
        BB_WRITE_REG(bb_hw, SYS_BNK_ACT, old_bnk);
        BB_WRITE_REG(bb_hw, SYS_ENABLE, old_status_en);
        baseband_switch_buf_store(bb_hw, old_buf_store);

        return ret;
}

/****************************************************
 * NAME         : func_safety_sm_can_config_reg_protection
 * DESCRIPTIONS : sm129, can config re protection handler
 * INPUT        : func_safety_error_type: error type ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_can_config_reg_protection(uint8_t func_safety_error_type)
{
        uint32_t value1[9];
        uint32_t value2[9];
        uint8_t i;
        bool flag = false;

        value1[0] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);    //MCR
        //EMBARC_PRINTF("MCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value1[0]);
        value1[1] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET); //PCR
        //EMBARC_PRINTF("PCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET, value1[1]);
        value1[2] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET);    //TSCCR
        //EMBARC_PRINTF("TSCCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET, value1[2]);
        value1[3] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET); //DBTCR
        //EMBARC_PRINTF("DBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET, value1[3]);
        value1[4] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_NOMINAL_BIT_TIME_CTRL_OFFSET);    //NBTCR
        //EMBARC_PRINTF("NBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_NOMINAL_BIT_TIME_CTRL_OFFSET, value1[4]);
        value1[5] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET); //IDFCR
        //EMBARC_PRINTF("IDFCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET, value1[5]);
        value1[6] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET); //XIDAMR
        //EMBARC_PRINTF("XIDAMR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET, value1[6]);
        value1[7] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_RX_ELEMENT_SIZE_CFG_OFFSET);     //RXESCR
        //EMBARC_PRINTF("RXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_RX_ELEMENT_SIZE_CFG_OFFSET, value1[7]);
        value1[8] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_TX_ELEMENT_SIZE_CFG_OFFSET);     //TXESCR
        //EMBARC_PRINTF("TXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TX_ELEMENT_SIZE_CFG_OFFSET, value1[8]);

        //EMBARC_PRINTF("write 0xAAAAAAAA to MCR PCR TSCCR DBTCR NBTCR IDFCR XIDAMR RXESCR TXESCR.\n");
        //raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, 0xAAAAAAAA);//MCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET, 0xAAAAAAAA); //PCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET,
                        0xAAAAAAAA);        //TSCCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET,
                        0xAAAAAAAA);        //DBTCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_NOMINAL_BIT_TIME_CTRL_OFFSET,
                        0xAAAAAAAA);        //NBTCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET, 0xAAAAAAAA); //IDFCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET,
                        0xAAAAAAAA);        //XIDAMR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_RX_ELEMENT_SIZE_CFG_OFFSET,
                        0xAAAAAAAA);        //RXESCR
        raw_writel(REL_REGBASE_CAN0 + REG_CAN_TX_ELEMENT_SIZE_CFG_OFFSET,
                        0xAAAAAAAA);        //TXESCR

        value2[0] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);    //MCR
        //EMBARC_PRINTF("MCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value2[0]);
        value2[1] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET); //PCR
        //EMBARC_PRINTF("PCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET, value2[1]);
        value2[2] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET);    //TSCCR
        //EMBARC_PRINTF("TSCCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET, value2[2]);
        value2[3] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET); //DBTCR
        //EMBARC_PRINTF("DBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET, value2[3]);
        value2[4] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_NOMINAL_BIT_TIME_CTRL_OFFSET);    //NBTCR
        //EMBARC_PRINTF("NBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_NOMINAL_BIT_TIME_CTRL_OFFSET, value2[4]);
        value2[5] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET); //IDFCR
        //EMBARC_PRINTF("IDFCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET, value2[5]);
        value2[6] = raw_readl(REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET); //XIDAMR
        //EMBARC_PRINTF("XIDAMR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET, value2[6]);
        value2[7] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_RX_ELEMENT_SIZE_CFG_OFFSET);     //RXESCR
        //EMBARC_PRINTF("RXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_RX_ELEMENT_SIZE_CFG_OFFSET, value2[7]);
        value2[8] = raw_readl(
        REL_REGBASE_CAN0 + REG_CAN_TX_ELEMENT_SIZE_CFG_OFFSET);     //TXESCR
        //EMBARC_PRINTF("TXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TX_ELEMENT_SIZE_CFG_OFFSET, value2[8]);

        for (i = 0; i < 9; i++) {
                if (value1[i] != value2[i]) {
                        flag = true;
                        break;
                }
        }

        if(flag == true)
                func_safety_error_handler(fsmConfList[SM_INDEX_28].sm_num, fsmConfList[SM_INDEX_28].error_type);
        else {
                //EMBARC_PRINTF("func_safety_sm_can_config_reg_protection test ok.\n");
        }
}

/****************************************************
 * NAME         : func_safety_sm_periodic_readback_configuration_registers
 * DESCRIPTIONS : sm805, periodic software readback of configuration registers
 * INPUT        : func_safety_error_type: error type IRQ or ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
static void func_safety_sm_periodic_readback_configuration_registers(uint8_t func_safety_error_type)
{
        uint32_t reg_value[64];
        uint8_t i;
        static uint8_t act_flag = 0x00;
        static uint32_t reg_value_crc_surce,reg_value_crc_curret;

        //read Clock and Reset Register
        reg_value[0] = raw_readl(REG_CLKGEN_READY_50M);
        reg_value[1] = raw_readl(REG_CLKGEN_READY_PLL);
        reg_value[2] = raw_readl(REG_CLKGEN_SEL_300M);
        reg_value[3] = raw_readl(REG_CLKGEN_SEL_400M);
        reg_value[4] = raw_readl(REG_CLKGEN_DIV_CPU);
        reg_value[5] = raw_readl(REG_CLKGEN_DIV_MEM);
        reg_value[6] = raw_readl(REG_CLKGEN_DIV_AHB);
        reg_value[7] = raw_readl(REG_CLKGEN_DIV_APB);
        reg_value[8] = raw_readl(REG_CLKGEN_ENA_ROM);
        reg_value[9] = raw_readl(REG_CLKGEN_ENA_RAM);
        reg_value[10] = raw_readl(REG_CLKGEN_ENA_BB_TOP);
        reg_value[11] = raw_readl(REG_CLKGEN_ENA_BB_CORE);
        reg_value[12] = raw_readl(REG_CLKGEN_ENA_FLASH_CTRL);
        reg_value[13] = raw_readl(REG_CLKGEN_ENA_DMU);
        reg_value[14] = raw_readl(REG_CLKGEN_RSTN_BB_TOP);
        reg_value[15] = raw_readl(REG_CLKGEN_RSTN_BB_CORE);
        reg_value[16] = raw_readl(REG_CLKGEN_RSTN_FLASH_CTRL);
        reg_value[17] = raw_readl(REG_CLKGEN_RSTN_DMU);

        //read DMU Registers
        reg_value[18] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_QSPI_OFFSET);
        reg_value[19] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET);
        reg_value[20] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_UART0_OFFSET);
        reg_value[21] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_UART1_OFFSET);
        reg_value[22] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_CAN0_OFFSET);
        reg_value[23] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_CAN1_OFFSET);
        reg_value[24] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_RESET_OFFSET);
        reg_value[25] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SYNC_OFFSET);
        reg_value[26] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_I2C_OFFSET);
        reg_value[27] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_PWM0_OFFSET);
        reg_value[28] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_PWM1_OFFSET);
        reg_value[29] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_ADC_CLK_OFFSET);
        reg_value[30] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_CAN_CLK_OFFSET);
        reg_value[31] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M0_OFFSET);
        reg_value[32] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        reg_value[33] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_CLK_OFFSET);
        reg_value[34] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_SEL_OFFSET);
        reg_value[35] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET);
        reg_value[36] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET);
        reg_value[37] = raw_readl(REL_REGBASE_DMU + REG_DMU_MUX_JTAG_OFFSET);
        reg_value[38] = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET);
        reg_value[39] = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET);
        reg_value[40] = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_OTP_PRGM_EN_OFFSET);
        reg_value[41] = raw_readl(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET);

        //read EMU Registers
        reg_value[42] = raw_readl(REG_EMU_SEC_STA);
        reg_value[43] = raw_readl(REG_EMU_SEC_CTRL);
        reg_value[44] = raw_readl(REG_EMU_SAFETY_PAD_CTRL);
        reg_value[45] = raw_readl(REG_EMU_FSM_STA);
        reg_value[46] = raw_readl(REG_EMU_RF_ERR_IRQ_ENA);
        reg_value[47] = raw_readl(REG_EMU_RF_ERR_IRQ_MASK);
        reg_value[48] = raw_readl(REG_EMU_RF_ERR_SS1_ENA);
        reg_value[49] = raw_readl(REG_EMU_RF_ERR_SS1_MASK);
        reg_value[50] = raw_readl(REG_EMU_RF_ERR_SS2_ENA);
        reg_value[51] = raw_readl(REG_EMU_RF_ERR_SS2_MASK);
        reg_value[52] = raw_readl(REG_EMU_RF_TEST_DIV);
        reg_value[53] = raw_readl(REG_EMU_DIG_ERR_IRQ_ENA);
        reg_value[54] = raw_readl(REG_EMU_DIG_ERR_IRQ_MASK);
        reg_value[55] = raw_readl(REG_EMU_DIG_ERR_SS1_ENA);
        reg_value[56] = raw_readl(REG_EMU_DIG_ERR_SS1_MASK);
        reg_value[57] = raw_readl(REG_EMU_DIG_ERR_SS2_ENA);
        reg_value[58] = raw_readl(REG_EMU_DIG_ERR_SS2_MASK);
        reg_value[59] = raw_readl(REG_EMU_DIG_TEST_DIV);
        reg_value[60] = raw_readl(REG_EMU_LBIST_STA);
        reg_value[61] = raw_readl(REG_EMU_REBOOT_CNT);
        reg_value[62] = raw_readl(REG_EMU_REBOOT_LIMIT);

        //read RF registers
        uint8_t old_bank = fmcw_radio_switch_bank(NULL, 0);
        reg_value[63] = RADIO_READ_BANK_REG(0, CBC_EN);
        reg_value[63] &= 0x07;
        fmcw_radio_switch_bank(NULL, old_bank);
        /*EMBARC_PRINTF("\r\n");
        for (i = 0; i < 64; i++) {
                EMBARC_PRINTF("0x%x \n", reg_value[i]);
        }
        EMBARC_PRINTF("\r\n");*/

        if (act_flag == 0) {
                reg_value_crc_surce = 0x00000000;
                for (i = 0; i < 64; i++) {
                        reg_value_crc_surce += reg_value[i];
                }
                act_flag = 0xff;
                reg_value_crc_curret = reg_value_crc_surce;
        } else {
                reg_value_crc_curret = 0x00000000;
                for (i = 0; i < 64; i++) {
                        reg_value_crc_curret += reg_value[i];
                }
        }

        if(reg_value_crc_curret == reg_value_crc_surce) {
                //EMBARC_PRINTF("SM805 func_safety_sm_periodic_readback_configuration_registers test ok.\n");
        } else
                func_safety_error_handler(fsmConfList[SM_INDEX_33].sm_num, fsmConfList[SM_INDEX_33].error_type);
}


/****************************************************
 * NAME         : func_safety_crc_acceleration
 * DESCRIPTIONS : sm110, crc acceleration handler
 * INPUT        : func_safety_error_type: error type ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_crc_acceleration(uint8_t func_safety_error_type)
{
        uint16_t data_to_cal[5] = { 0x1122, 0x3344, 0x5566, 0x7788, 0x6C8B };
        uint16_t crc16_result;

        //hw_crc_pre(data_to_cal[4]);

        if (func_safety_crc16_update(0, data_to_cal, 4) == E_OK) {
                crc16_result = crc_output();
                EMBARC_PRINTF("data_to_cal: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x.\n", data_to_cal[0], data_to_cal[1], data_to_cal[2], data_to_cal[3], crc16_result);
        }
}

/* Start Baseband once to implement read/write process on Baseband memory */
/* Supposedly, better this to be done by hardware */
void clear_bb_memory_by_scan_start_bb_one_time(void)
{
        uint16_t bb_status_en = 0;
        baseband_t* bb = baseband_get_rtl_frame_type(); /* read back the bank selected in RTL */
        baseband_hw_t* bb_hw = &bb->bb_hw;

        bb_status_en = ((SYS_ENABLE_SAM_MASK << SYS_ENABLE_SAM_SHIFT)
                      | (SYS_ENABLE_FFT_2D_MASK << SYS_ENABLE_FFT_2D_SHIFT)
                      | (SYS_ENABLE_CFR_MASK << SYS_ENABLE_CFR_SHIFT)
                      | (SYS_ENABLE_BFM_MASK << SYS_ENABLE_BFM_SHIFT));

        /* Clear event bit before bb start */
        uint32_t event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
        if( event_bits != E_OK)
        {
                EMBARC_PRINTF("Event bit set to %d before start @%s", event_bits, __func__);
        }

        /* start baseband */
        baseband_start_with_params(bb, true, true, bb_status_en, true, BB_IRQ_DISABLE_ALL, false);

        /* wait done */
        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);

        EMBARC_PRINTF("/*** clear_bb_memory_by_scan_start_bb_one_time:done! ***/\n\r");

        /* stop baseband */
        baseband_stop(bb);

}

void clear_bb_memory_ecc_status_bit(void)
{
        //uint32_t value_sb,value_db;

        if (fsmConfList[SM_INDEX_12].open_flag == true) {
                /*value_sb = raw_readl(BB_REG_SYS_ECC_SB_STATUS);
                EMBARC_PRINTF("ECC_SB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_SB_STATUS, value_sb);
                value_db =  raw_readl(BB_REG_SYS_ECC_DB_STATUS);
                EMBARC_PRINTF("ECC_DB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_DB_STATUS, value_db);*/

                baseband_write_reg(NULL, BB_REG_SYS_ECC_SB_CLR, 0x2000);
                baseband_write_reg(NULL, BB_REG_SYS_ECC_DB_CLR, 0x2000);

                /*value_sb = raw_readl(BB_REG_SYS_ECC_SB_STATUS);
                EMBARC_PRINTF("ECC_SB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_SB_STATUS, value_sb);
                value_db =  raw_readl(BB_REG_SYS_ECC_DB_STATUS);
                EMBARC_PRINTF("ECC_DB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_DB_STATUS, value_db);*/
        }
}

void func_safety_sm_fault_injection(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t IRQ_value;
        uint8_t IRQ_all = 0xFF;

        EMBARC_PRINTF("FEI start!\r\n"); //just for autotest use
        /* sm_rfpower_detector fault injection */
        if (fsmConfList[SM_INDEX_5].open_flag == true) {
                IRQ_value = fmcw_radio_sm_rfpower_detector_fault_injection_IRQ(radio,5);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm RF Power Detector IRQ = %d\n",IRQ_value);
        }

        /* sm_if_loopback fault injection */
        if (fsmConfList[SM_INDEX_7].open_flag == true) {
                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(radio,true);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm IF Loopback IRQ = %d\n",IRQ_value);
        }

        /* sm_rf_loopback fault injection */
        if (fsmConfList[SM_INDEX_8].open_flag == true) {
                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(radio,true);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm RF Loopback IRQ = %d\n",IRQ_value);
        }

        /* sm_ldo_monitor fault injection */
        if (fsmConfList[SM_INDEX_0].open_flag == true) {
                IRQ_value = fmcw_radio_sm_ldo_monitor_fault_injection_IRQ(radio);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm LDO Monitor IRQ = %d\n",IRQ_value);
        }

        /* sm_avdd33_monitor fault injection */
        if (fsmConfList[SM_INDEX_1].open_flag == true) {
                IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(radio,true);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm AVDD33 Monitor IRQ = %d\n",IRQ_value);
        }

        /* sm_dvdd11_monitor fault injection */
        if (fsmConfList[SM_INDEX_2].open_flag == true) {
                IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(radio,true);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm DVDD11 Monitor IRQ = %d\n",IRQ_value);
        }

        /* sm_bg_monitor fault injection */
        if (fsmConfList[SM_INDEX_3].open_flag == true) {
                IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(radio,true);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm BG Monitor IRQ = %d\n",IRQ_value);
        }

        /* sm_chirp_monitor fault injection */
        if (fsmConfList[SM_INDEX_9].open_flag == true) {
                IRQ_value = fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(radio);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm Chirp Monitor IRQ = %d\n",IRQ_value);
        }

        /* sm_over_temp_detector fault injection */
        if (fsmConfList[SM_INDEX_10].open_flag == true) {
                IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(radio,true);
                IRQ_all &= IRQ_value;
                EMBARC_PRINTF("sm Over Temp Detector IRQ = %d\n",IRQ_value);
        }

        /* sm_saturation_detector fault injection */
        if (fsmConfList[SM_INDEX_6].open_flag == true) {
                IRQ_value = fmcw_radio_sm_saturation_detector_fault_injection_IRQ(radio);
                if (IRQ_value)
                        IRQ_all &= 0x01;
                else
                        IRQ_all &= 0x00;
                EMBARC_PRINTF("sm Saturation Detector IRQ = %d\n",IRQ_value);
        }

        fmcw_radio_switch_bank(radio, old_bank);

        if (IRQ_all)
                EMBARC_PRINTF("/*** sm self-check done: IRQ_all = %d  success!***/\n\r", IRQ_all);
        else
                EMBARC_PRINTF("/*** sm self-check done: IRQ_all = %d  failed!***/\n\r", IRQ_all);

        EMBARC_PRINTF("FEI end!\r\n"); //just for autotest use
}

/****************************************************
 * NAME         : get_can_send_status
 * DESCRIPTIONS : get can intr send status
 * INPUT        : None
 * OUTPUT       : CAN_SEND_STATUS_IDLE: can idle.
 *                CAN_SEND_STATUS_SENDING: can intr send all data finish
 *****************************************************/
uint8_t get_can_send_status(void)
{
        return can_send_status;
}

/****************************************************
 * NAME         : set_can_send_status
 * DESCRIPTIONS : set can intr send status
 * INPUT        : CAN_SEND_STATUS_IDLE: can idle.
 *                CAN_SEND_STATUS_SENDING: can intr send all data finish
 * OUTPUT       : None
 *****************************************************/
void set_can_send_status(uint8_t value)
{
        can_send_status = value;
}

/****************************************************
 * NAME         : func_safety_sm_ldo_monitor_set_part
 * DESCRIPTIONS : sm1 set part handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_ldo_monitor_set_part(fmcw_radio_t *radio)
{
        if (fsmConfList[SM_INDEX_0].open_flag == true) {
                if (bb_frame_start_flag == true) {
                        bb_frame_start_flag = false;
                        ldo_set_part_flag = true;
                        fmcw_radio_sm_ldo_monitor_setting(radio, sm1_ldo_part_cnt);
                }
        }
}

/****************************************************
 * NAME         : func_safety_sm_ldo_monitor_read_part
 * DESCRIPTIONS : sm1 read part handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_ldo_monitor_read_part(void)
{
        static uint8_t ldo_read_part_time_cnt = 0x00;
        baseband_t *bb = baseband_get_cur_bb();

        if (fsmConfList[SM_INDEX_0].open_flag == true) {
                if (ldo_set_part_flag == true) {
                        ldo_read_part_time_cnt++;
                        if (ldo_read_part_time_cnt > 35) {
                                ldo_read_part_time_cnt = 0x00;
                                ldo_set_part_flag = false;
                                fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio,sm1_ldo_part_cnt);
                                sm1_ldo_part_cnt++;
                                if (sm1_ldo_part_cnt > 3)
                                        sm1_ldo_part_cnt = 0x00;
                        }
                }
        }
}

/****************************************************
 * NAME         : func_safety_sm_periodic_run_handler
 * DESCRIPTIONS : periodic sm run handler: SM120 SM126 SM1
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_periodic_run_handler(void)
{
        if (get_track_cfg() == CAN_INT) {
                if ((get_can_send_status() == CAN_SEND_STATUS_IDLE) && (periodic_sm_finish_flag == true)) {
                        periodic_sm_finish_flag = false;

                        if (fsmConfList[SM_INDEX_21].open_flag == true) {
                                func_safety_sm_can_loopback_init();
                        }

                        if (fsmConfList[SM_INDEX_27].open_flag == true) {
                                func_safety_sm_spi_loopback(fsmConfList[SM_INDEX_27].error_type);
                        }
                }
        }

        func_safety_sm_ldo_monitor_read_part();
}

/****************************************************
 * NAME         : func_safety_enable_can_ecc
 * DESCRIPTIONS : enable CAN ECC
 * INPUT        : id: can id
 * OUTPUT       : None
 *****************************************************/
void func_safety_enable_can_ecc(uint32_t id)
{
        uint32_t value;

        if (fsmConfList[SM_INDEX_20].open_flag == true) {
                if (id == CAN_0_ID) {
                        //set CAN_0.MCR.CFG to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
                        //set CAN_0.MCR.ECCENA to 0x1
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_ECCENA);
                        //set CAN_0.MCR.CFG to 0x0
                        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
                        raw_writel(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));
                }
        }
}

void get_sm109_init_para(void)
{
        uint32_t value;

        value = raw_readl(REL_REGBASE_DMU + REG_DMU_IRQ_ENA0_31_OFFSET);
        EMBARC_PRINTF("IRQ_ENA_0_32: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_IRQ_ENA0_31_OFFSET, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_MASK);
        EMBARC_PRINTF("DIG_ERR_SS1_MARK: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_SS1_MASK, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_ENA);
        EMBARC_PRINTF("DIG_ERR_SS1_ENA: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_SS1_ENA, value);
        //value = raw_readl(REG_CLKGEN_ENA_CAN_0);
        //EMBARC_PRINTF("CLKGEN.ENA_CAN_0: 0x%x, value: 0x%x.\n", REG_CLKGEN_ENA_CAN_0, value);

        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        EMBARC_PRINTF("CAN_0.MCR: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_OFFSET);
        EMBARC_PRINTF("CAN_0.IR: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_OFFSET, value);
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET);
        EMBARC_PRINTF("CAN_0.IE: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_ENABLE_OFFSET, value);
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET);
        EMBARC_PRINTF("CAN_0.ILS0R: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_SELECT0_OFFSET, value);
        value = raw_readl(REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET);
        EMBARC_PRINTF("CAN_0.ILE: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_INTERRUPT_LINE_ENABLE_OFFSET, value);
}

/****************************************************
 * NAME         : func_safety_process
 * DESCRIPTIONS : used to run the items periodically
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_process(void *params)
{
        uint32_t IRQ_value = 0;
        bool test_result;
        baseband_t *bb = baseband_get_cur_bb();
        sensor_config_t *cfg = sensor_config_get_cur_cfg();

        periodic_sm_finish_flag = false;
        switch(FUNC_SAFETY_PERIODIC_SM_CLCLE) {
        case PERIODIC_SM_1_CYCLE:
                clear_bb_memory_ecc_status_bit();
                /* SM6 and SM201 */
                if ((fsmConfList[SM_INDEX_5].open_flag == true) || (fsmConfList[SM_INDEX_11].open_flag == true)) {
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                        if (fsmConfList[SM_INDEX_5].open_flag == true)
                                IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
                        if (fsmConfList[SM_INDEX_11].open_flag == true) {
                                test_result = func_safety_sm_vga_consistence_check();
                                if (test_result == true)
                                        func_safety_error_handler(fsmConfList[SM_INDEX_5].sm_num, fsmConfList[SM_INDEX_5].error_type);
                        }
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                }
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_2_CYCLE;
                break;
        case PERIODIC_SM_2_CYCLE:
                /* SM11 and SM13 */
                if ((fsmConfList[SM_INDEX_7].open_flag == true) || (fsmConfList[SM_INDEX_9].open_flag == true)) {
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                        if ((fsmConfList[SM_INDEX_7].open_flag == true) && (sample_adc_running_flag == false)) {
                                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio,false);
                                if (IRQ_value)
                                        func_safety_error_handler(fsmConfList[SM_INDEX_7].sm_num, fsmConfList[SM_INDEX_7].error_type);
                        }
                        if (fsmConfList[SM_INDEX_9].open_flag == true)
                                fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                }
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_3_CYCLE;
                break;
        case PERIODIC_SM_3_CYCLE:
                /* SM12 and SM14 */
                if ((fsmConfList[SM_INDEX_8].open_flag == true) || (fsmConfList[SM_INDEX_10].open_flag == true)) {
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                        if ((fsmConfList[SM_INDEX_8].open_flag == true) && (sample_adc_running_flag == false)) {
                                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio,false);
                                if (IRQ_value)
                                        func_safety_error_handler(fsmConfList[SM_INDEX_8].sm_num, fsmConfList[SM_INDEX_8].error_type);
                        }
                        if (fsmConfList[SM_INDEX_10].open_flag == true)
                                fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                }
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_4_CYCLE;
                break;
        case PERIODIC_SM_4_CYCLE:
                /* SM101 Run Time LBIST for Baseband */
                if (fsmConfList[SM_INDEX_12].open_flag == true) {
                        //disable sm8
                        func_safety_sm8_enable(fsmConfList[SM_INDEX_6].error_type, false);
                        test_result = baseband_bist_ctrl(cfg->bb, false);
                        //clear sm6 err status and enable sm8
                        func_safety_sm8_enable(fsmConfList[SM_INDEX_6].error_type, true);
                }

                if (get_track_cfg() != CAN_INT) {
                        /* SM120 CAN loopback test */
                        if (fsmConfList[SM_INDEX_21].open_flag == true) {
                                func_safety_sm_can_loopback_init();
                        }

                        /* SM126 SPI loopback test */
                        if (fsmConfList[SM_INDEX_27].open_flag == true) {
                                func_safety_sm_spi_loopback(fsmConfList[SM_INDEX_27].error_type);
                        }
                }

                /* SM129 Configuration Registers Protection */
                if (fsmConfList[SM_INDEX_28].open_flag == true) {
                        func_safety_sm_can_config_reg_protection(fsmConfList[SM_INDEX_28].error_type);
                }

                /* SM805 VGA Gain Consistence Check */
                if (fsmConfList[SM_INDEX_33].open_flag == true) {
                        func_safety_sm_periodic_readback_configuration_registers(fsmConfList[SM_INDEX_33].error_type);
                }

                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_1_CYCLE;
                periodic_sm_finish_flag = true;
                break;
        default:
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_1_CYCLE;
                break;
        }
}



