#ifndef _ALPS_DMU_REG_H_
#define _ALPS_DMU_REG_H_

#define REG_DMU_CMD_SRC_SEL_OFFSET	(0x0000)
#define REG_DMU_CMD_OUT_OFFSET		(0x0004)
#define REG_DMU_CMD_IN_OFFSET		(0x0008)

#define REG_DMU_ADC_RSTN_OFFSET		(0x000C)
#define REG_DMU_ADC_CNT_OFFSET		(0x0010)

#define REG_DMU_FMCW_START_SRC		(0x0014)
#define REG_DMU_FMCW_START		(0x0018)
#define REG_DMU_FMCW_STATUS		(0x001C)

#define REG_DMU_DBG_SRC_OFFSET		(0x0100)
#define REG_DMU_DBG_VAL_OEN_OFFSET	(0x0104)
#define REG_DMU_DBG_DAT_OEN_OFFSET	(0x0108)
#define REG_DMU_DBG_DOUT_OFFSET		(0x010C)
#define REG_DMU_DBG_DIN_OFFSET		(0x0110)
#define REG_DMU_HIL_ENA_OFFSET		(0x0114)
#define REG_DMU_HIL_DAT_OFFSET		(0x0118)


/* IO MUX registers. */
#define REG_DMU_MUX_QSPI_OFFSET		(0x0200)
#define REG_DMU_MUX_SPI_M1_OFFSET	(0x0204)
#define REG_DMU_MUX_UART0_OFFSET	(0x0208)
#define REG_DMU_MUX_UART1_OFFSET	(0x020C)
#define REG_DMU_MUX_CAN0_OFFSET		(0x0210)
#define REG_DMU_MUX_CAN1_OFFSET		(0x0214)
#define REG_DMU_MUX_RESET_OFFSET	(0x0218)
#define REG_DMU_MUX_SYNC_OFFSET		(0x021C)
#define REG_DMU_MUX_I2C_OFFSET		(0x0220)
#define REG_DMU_MUX_PWM0_OFFSET		(0x0224)
#define REG_DMU_MUX_PWM1_OFFSET		(0x0228)
#define REG_DMU_MUX_ADC_CLK_OFFSET	(0x022C)
#define REG_DMU_MUX_CAN_CLK_OFFSET	(0x0230)
#define REG_DMU_MUX_SPI_M0_OFFSET	(0x0234)
#define REG_DMU_MUX_SPI_S_OFFSET	(0x0238)
#define REG_DMU_MUX_SPI_S1_CLK_OFFSET	(0x023C)
#define REG_DMU_MUX_SPI_S1_SEL_OFFSET	(0x0240)
#define REG_DMU_MUX_SPI_S1_MOSI_OFFSET	(0x0244)
#define REG_DMU_MUX_SPI_S1_MISO_OFFSET	(0x0248)
#define REG_DMU_MUX_JTAG_OFFSET		(0x024C)

/* system. */
#define REG_DMU_SYS_DMA_ENDIAN_OFFSET	(0x0300)
#define REG_DMU_SYS_SHSEL_NP_OFFSET	(0x0304)
#define REG_DMU_SYS_DMU_SEL_OFFSET	(0x0308)
#define REG_DMU_SYS_ICM0_FIX_P_OFFSET	(0x030C)
#define REG_DMU_SYS_ICM1_FIX_P_OFFSET	(0x0310)
#define REG_DMU_SYS_PWM0_ENA_OFFSET	(0x0314)
#define REG_DMU_SYS_MEMRUN_ENA_OFFSET	(0x0318)
#define REG_DMU_SYS_MEMINI_ENA_OFFSET	(0x031C)
#define REG_DMU_SYS_SPI_LOOP_OFFSET	(0x0320)
#define REG_DMU_SYS_OTP_PRGM_EN_OFFSET	(0x0324)
#define REG_DMU_SYS_OTP_ECC_EN_OFFSET	(0x0328)
#define REG_DMU_SYS_MEM_CLR_OFFSET	(0x032C)
#define REG_DMU_SYS_MEM_CLR_DONE_OFFSET	(0x0330)
//#define REG_DMU_SYS_PWM1_ENA_OFFSET	(45 << 2)
//#define REG_DMU_SYS_DMA_REQ_S_OFFSET	(0x0304)
//#define REG_DMU_SYS_SHSEL_NP_OFFSET	(40 << 2)


#define REG_DMU_IRQ_TRIG_OFFSET		(0x0400)
#define REG_DMU_IRQ_STA_OFFSET		(0x0404)

#define REG_DMU_IRQ_ENA0_31_OFFSET	(0x0500)
#define REG_DMU_IRQ_ENA32_63_OFFSET	(0x0504)
#define REG_DMU_IRQ_ENA64_95_OFFSET	(0x0508)
#define REG_DMU_IRQ_ENA96_127_OFFSET	(0x050C)
#define REG_DMU_IRQ25_SEL_OFFSET	(0x0510)
#define REG_DMU_IRQ26_SEL_OFFSET	(0x0514)
#define REG_DMU_IRQ27_SEL_OFFSET	(0x0518)
#define REG_DMU_IRQ28_SEL_OFFSET	(0x051C)
#define REG_DMU_IRQ29_SEL_OFFSET	(0x0520)
#define REG_DMU_IRQ30_SEL_OFFSET	(0x0524)
#define REG_DMU_IRQ31_SEL_OFFSET	(0x0528)


/* cmd out bits fields define. */
#define BIT_REG_DMU_CMD_WR		(1 << 15)
#define BITS_REG_DMU_CMD_ADDR_MASK	(0x7F)
#define BITS_REG_DMU_CMD_ADDR_SHIFT	(8)
#define BITS_REG_DMU_CMD_WR_DAT_MASK	(0xFF)
#define BITS_REG_DMU_CMD_WR_DAT_SHIFT	(0)

/* dmu select. */
#define SYS_DMU_SEL_DBG			(1)
#define SYS_DMU_SEL_GPIO		(0)


/* for debug bus. */
#define DBGBUS_OUTPUT_ENABLE  0
#define DBGBUS_OUTPUT_DISABLE 1

#define DBGBUS_DAT_ENABLE	0xffff0000
#define DBGBUS_DAT_DISABLE	0xffffffff

#define DBG_SRC_CPU                   0
#define DBG_SRC_SAM                   1
#define DBG_SRC_CFR                   2
#define DBG_SRC_BFM                   3
#define DBG_SRC_DUMP_W_SYNC           4
#define DBG_SRC_DUMP_WO_SYNC          5

#define DBGBUS_DAT_RESET     0x00000000
#define DBGBUS_DAT_0_OEN     0xfffffffe
#define DBGBUS_DAT_1_OEN     0xfffffffd
#define DBGBUS_DAT_2_OEN     0xfffffffb
#define DBGBUS_DAT_3_OEN     0xfffffff7
#define DBGBUS_DAT_4_OEN     0xffffffef
#define DBGBUS_DAT_5_OEN     0xffffffdf
#define DBGBUS_DAT_6_OEN     0xffffffbf
#define DBGBUS_DAT_7_OEN     0xffffff7f
#define DBGBUS_DAT_8_OEN     0xfffffeff
#define DBGBUS_DAT_9_OEN     0xfffffdff
#define DBGBUS_DAT_10_OEN    0xfffffbff
#define DBGBUS_DAT_11_OEN    0xfffff7ff
#define DBGBUS_DAT_12_OEN    0xffffefff
#define DBGBUS_DAT_13_OEN    0xffffdfff
#define DBGBUS_DAT_14_OEN    0xffffbfff
#define DBGBUS_DAT_15_OEN    0xffff7fff
#define DBGBUS_DAT_16_OEN    0xfffeffff
#define DBGBUS_DAT_17_OEN    0xfffdffff
#define DBGBUS_DAT_18_OEN    0xfffbffff
#define DBGBUS_DAT_19_OEN    0xfff7ffff
#define DBGBUS_DAT_20_OEN    0xffefffff
#define DBGBUS_DAT_21_OEN    0xffdfffff
#define DBGBUS_DAT_22_OEN    0xffbfffff
#define DBGBUS_DAT_23_OEN    0xff7fffff
#define DBGBUS_DAT_24_OEN    0xfeffffff
#define DBGBUS_DAT_25_OEN    0xfdffffff
#define DBGBUS_DAT_26_OEN    0xfbffffff
#define DBGBUS_DAT_27_OEN    0xf7ffffff
#define DBGBUS_DAT_28_OEN    0xefffffff
#define DBGBUS_DAT_29_OEN    0xdfffffff
#define DBGBUS_DAT_30_OEN    0xbfffffff
#define DBGBUS_DAT_31_OEN    0x7fffffff

#define DBGBUS_DAT_0_MASK    0x00000001
#define DBGBUS_DAT_1_MASK    0x00000002
#define DBGBUS_DAT_2_MASK    0x00000004
#define DBGBUS_DAT_3_MASK    0x00000008
#define DBGBUS_DAT_4_MASK    0x00000010
#define DBGBUS_DAT_5_MASK    0x00000020
#define DBGBUS_DAT_6_MASK    0x00000040
#define DBGBUS_DAT_7_MASK    0x00000080
#define DBGBUS_DAT_8_MASK    0x00000100
#define DBGBUS_DAT_9_MASK    0x00000200
#define DBGBUS_DAT_10_MASK   0x00000400
#define DBGBUS_DAT_11_MASK   0x00000800
#define DBGBUS_DAT_12_MASK   0x00001000
#define DBGBUS_DAT_13_MASK   0x00002000
#define DBGBUS_DAT_14_MASK   0x00004000
#define DBGBUS_DAT_15_MASK   0x00008000
#define DBGBUS_DAT_16_MASK   0x00010000
#define DBGBUS_DAT_17_MASK   0x00020000
#define DBGBUS_DAT_18_MASK   0x00040000
#define DBGBUS_DAT_19_MASK   0x00080000
#define DBGBUS_DAT_20_MASK   0x00100000
#define DBGBUS_DAT_21_MASK   0x00200000
#define DBGBUS_DAT_22_MASK   0x00400000
#define DBGBUS_DAT_23_MASK   0x00800000
#define DBGBUS_DAT_24_MASK   0x01000000
#define DBGBUS_DAT_25_MASK   0x02000000
#define DBGBUS_DAT_26_MASK   0x04000000
#define DBGBUS_DAT_27_MASK   0x08000000
#define DBGBUS_DAT_28_MASK   0x10000000
#define DBGBUS_DAT_29_MASK   0x20000000
#define DBGBUS_DAT_30_MASK   0x40000000
#define DBGBUS_DAT_31_MASK   0x80000000

#endif
