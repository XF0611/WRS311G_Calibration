DEVICE_IP_ROOTDIR = $(EMBARC_ROOT)/device/ip

BOOT_INCDIRS += $(EMBARC_ROOT)/device/inc $(EMBARC_ROOT)/device/ip/ip_hal/inc

DEVICE_IP_DIR = $(DEVICE_IP_ROOTDIR)
DEVICE_HAL_DIR = $(DEVICE_IP_ROOTDIR)/ip_hal
DEVICE_BAREMETAL_DRV_DIR = ./device

DEVICE_HAL_COBJS ?=
DEVICE_IP_COBJS ?=

DEVICE_BAREMETAL_DRV_CSRCS ?=
DEVICE_BAREMETAL_DRV_COBJS ?=

ifneq ($(USE_CAN),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/can_r2p0

DEVICE_HAL_CSRCS += $(DEVICE_IP_ROOTDIR)/ip_hal/can_hal.c
DEVICE_CAN_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/can_r2p0)
DEVICE_CAN_COBJS += $(call get_relobjs, $(DEVICE_CAN_CSRCS))

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/can_r2p0
else
DEVICE_CAN_COBJS ?=
endif

ifneq ($(USE_OTP),)
DEVICE_HAL_CSRCS += $(DEVICE_IP_ROOTDIR)/ip_hal/otp_hal.c
endif

ifneq ($(USE_UART),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/designware/dw_uart

DEVICE_UART_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/designware/dw_uart)
DEVICE_UART_COBJS = $(call get_relobjs, $(DEVICE_UART_CSRCS))

ifneq ($(UART_OTA),)
DEVICE_BAREMETAL_DRV_CSRCS += device/uart.c
else
ifneq ($(UART_LOG_EN),)
DEVICE_BAREMETAL_DRV_CSRCS += device/uart.c
endif
endif

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/designware/dw_uart

CHIP_CONSOLE_UART_BAUD ?= 3000000
else
DEVICE_UART_COBJS ?=

CHIP_CONSOLE_UART_BAUD ?= 3000000
endif

ifneq ($(USE_GPIO),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/designware/gpio

DEVICE_GPIO_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/designware/gpio)
DEVICE_GPIO_COBJS = $(call get_relobjs, $(DEVICE_GPIO_CSRCS))

DEVICE_HAL_CSRCS += $(DEVICE_IP_ROOTDIR)/ip_hal/gpio_hal.c
else
DEVICE_GPIO_COBJS ?=
endif

ifneq ($(USE_DW_TIMER),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/designware/dw_timer

DEVICE_HAL_CSRCS += $(DEVICE_IP_ROOTDIR)/ip_hal/timer_hal.c
DEVICE_DW_TIMER_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/designware/dw_timer)
DEVICE_DW_TIMER_COBJS = $(call get_relobjs, $(DEVICE_DW_TIMER_CSRCS))

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/designware/dw_timer
else
DEVICE_DW_TIMER_COBJS ?=
endif

ifneq ($(USE_SSI),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/designware/ssi

DEVICE_SSI_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/designware/ssi)
DEVICE_SSI_COBJS = $(call get_relobjs, $(DEVICE_SSI_CSRCS))

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/designware/ssi
else
DEVICE_SSI_COBJS ?=
endif

ifneq ($(USE_QSPI),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/designware/dw_qspi

DEVICE_QSPI_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/designware/dw_qspi)
DEVICE_QSPI_COBJS = $(call get_relobjs, $(DEVICE_QSPI_CSRCS))

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/designware/dw_qspi
else
DEVICE_QSPI_COBJS ?=
endif

ifneq ($(USE_XIP),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/xip

DEVICE_HAL_CSRCS += $(DEVICE_IP_ROOTDIR)/ip_hal/xip_hal.c

DEVICE_XIP_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/xip)
DEVICE_XIP_COBJS = $(call get_relobjs, $(DEVICE_XIP_CSRCS))

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/xip
else
DEVICE_XIP_COBJS ?=
endif

ifneq ($(BOOT_USE_HW_CRC),)
BOOT_INCDIRS += $(DEVICE_IP_ROOTDIR)/crc

DEVICE_HAL_CSRCS += $(DEVICE_IP_ROOTDIR)/ip_hal/crc_hal.c

DEVICE_CRC_CSRCS = $(call get_csrcs, $(DEVICE_IP_ROOTDIR)/crc)
DEVICE_CRC_COBJS = $(call get_relobjs, $(DEVICE_CRC_CSRCS))

DEVICE_IP_DIR += $(DEVICE_IP_ROOTDIR)/crc
else
DEVICE_CRC_COBJS ?=
endif

DEVICE_BAREMETAL_DRV_COBJS = $(call get_relobjs, $(DEVICE_BAREMETAL_DRV_CSRCS))
DEVICE_BAREMETAL_DRV_OBJS = $(DEVICE_BAREMETAL_DRV_COBJS)

DEVICE_IP_COBJS = $(DEVICE_CAN_COBJS) $(DEVICE_SSI_COBJS) $(DEVICE_QSPI_COBJS) $(DEVICE_UART_COBJS) $(DEVICE_XIP_COBJS) $(DEVICE_CRC_COBJS) $(DEVICE_GPIO_COBJS) $(DEVICE_TIMER_COBJS) $(DEVICE_DW_TIMER_COBJS)
DEVICE_HAL_COBJS = $(call get_relobjs, $(DEVICE_HAL_CSRCS))

DEVICE_COBJS = $(DEVICE_IP_COBJS) $(DEVICE_HAL_COBJS)
DEVICE_OBJS = $(DEVICE_COBJS)

DEVICE_IP_OUT_DIR = $(OUT_DIR)/device/ip
DEVICE_HAL_OUT_DIR = $(OUT_DIR)/device/ip/ip_hal
DEVICE_BAREMETAL_DRV_OUT_DIR = $(OUT_DIR)/device


################# external devices ################
EXT_DEV_ROOT_DIR = $(EMBARC_ROOT)/device/peripheral

ifneq ($(USE_NOR_FLASH),)
NOR_FLASH_ROOT_DIR = $(EXT_DEV_ROOT_DIR)/nor_flash


VENDOR_ROOT = $(NOR_FLASH_ROOT_DIR)/vendor

SUPPORTED_FLASH_TYPE = $(basename $(notdir $(wildcard $(VENDOR_ROOT)/*/*.c)))

VALID_FLASH_TYPE = $(call check_item_exist, $(FLASH_TYPE), $(SUPPORTED_FLASH_TYPE))
ifeq ($(VALID_FLASH_TYPE), )
$(info FLASH - $(SUPPORTED_FLASH_TYPE) are supported)
$(error FLASH $(FLASH_TYPE) is not supported, please check it!)
endif

NOR_FLASH_CSRCS = $(NOR_FLASH_ROOT_DIR)/flash.c \
		  $(VENDOR_ROOT)/$(VALID_FLASH_TYPE)/$(VALID_FLASH_TYPE).c
ifeq ($(NOR_FLASH_STATIC_PARAM),)
NOR_FLASH_CSRCS += $(NOR_FLASH_ROOT_DIR)/sfdp.c \
		   $(NOR_FLASH_ROOT_DIR)/cfi.c
endif

NOR_FLASH_COBJS = $(call get_relobjs, $(NOR_FLASH_CSRCS))

NOR_FLASH_OUT_DIR = $(OUT_DIR)/device/peripheral/nor_flash

BOOT_INCDIRS += $(NOR_FLASH_ROOT_DIR) $(VENDOR_ROOT)/$(VALID_FLASH_TYPE)
BOOT_INCDIRS += $(EMBARC_ROOT)/device/ip/xip
BOOT_INCDIRS += $(NOR_FLASH_ROOT_DIR) $(NOR_FLASH_ROOT_DIR)/vendor
else
NOR_FLASH_ROOT_DIR ?=
endif


EXT_DEV_COBJS = $(NOR_FLASH_COBJS)
EXT_DEV_OBJS = $(EXT_DEV_COBJS)

EXT_DEV_DEFINES ?=

ifneq ($(NOR_FLASH_STATIC_PARAM),)
EXT_DEV_DEFINES += -DSTATIC_EXT_FLASH_PARAM
endif

EXT_DEV_DEFINES += -DCHIP_CONSOLE_UART_BAUD=$(CHIP_CONSOLE_UART_BAUD)
