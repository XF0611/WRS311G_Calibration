# author: Xerxes 2022-2-15

WEIFU_ROOT ?= $(FIRMWARE_ROOT)/weifu


include $(WEIFU_ROOT)/app/app.mk
include $(WEIFU_ROOT)/baseband_patch/baseband_patch.mk
include $(WEIFU_ROOT)/pmic/pmic.mk
include $(WEIFU_ROOT)/can_transceiver/can_transceiver.mk
include $(WEIFU_ROOT)/ads7029/ads7029.mk
include $(WEIFU_ROOT)/tcan/tcan.mk





