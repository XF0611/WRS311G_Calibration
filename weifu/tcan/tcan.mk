# author: Xerxes 2022-2-15

WEIFU_TCAN_ROOT = $(WEIFU_ROOT)/tcan

WEIFU_TCAN_CSRCDIR = $(WEIFU_TCAN_ROOT)
WEIFU_TCAN_ASMSRCDIR = $(WEIFU_TCAN_ROOT)

# find all the source files in the target directories
WEIFU_TCAN_CSRCS = $(call get_csrcs, $(WEIFU_TCAN_CSRCDIR))
WEIFU_TCAN_ASMSRCS = $(call get_asmsrcs, $(WEIFU_TCAN_ASMSRCDIR))

# get object files
WEIFU_TCAN_COBJS = $(call get_relobjs, $(WEIFU_TCAN_CSRCS))
WEIFU_TCAN_ASMOBJS = $(call get_relobjs, $(WEIFU_TCAN_ASMSRCS))
WEIFU_TCAN_OBJS = $(WEIFU_TCAN_COBJS) $(WEIFU_TCAN_ASMOBJS)

# get dependency files
WEIFU_TCAN_DEPS = $(call get_deps, $(WEIFU_TCAN_OBJS))


# genearte library
WEIFU_TCAN_LIB = $(OUT_DIR)/lib_weifu_tcan.a

COMMON_COMPILE_PREREQUISITES += $(WEIFU_TCAN_ROOT)/tcan.mk

# library generation rule
$(WEIFU_TCAN_LIB): $(WEIFU_TCAN_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(WEIFU_TCAN_OBJS)
	

WF_TCAN_CFLAG ?= -Wno-unused-variable 
	
# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(WEIFU_TCAN_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $(COMPILE_HW_OPT) $(WF_TCAN_CFLAG) $< -o $@
	

.SECONDEXPANSION:
$(WEIFU_TCAN_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $(COMPILE_HW_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(WEIFU_BASEBAND_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(WEIFU_BASEBAND_ASMSRCS)

CALTERAH_COMMON_COBJS += $(WEIFU_TCAN_COBJS)
CALTERAH_COMMON_ASMOBJS += $(WEIFU_TCAN_ASMOBJS)

CALTERAH_COMMON_LIBS += $(WEIFU_TCAN_LIB)
