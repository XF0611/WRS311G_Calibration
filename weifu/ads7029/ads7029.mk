# author: Xerxes 2022-2-15

WEIFU_ADS7029_ROOT = $(WEIFU_ROOT)/ads7029

WEIFU_ADS7029_CSRCDIR = $(WEIFU_ADS7029_ROOT)
WEIFU_ADS7029_ASMSRCDIR = $(WEIFU_ADS7029_ROOT)

# find all the source files in the target directories
WEIFU_ADS7029_CSRCS = $(call get_csrcs, $(WEIFU_ADS7029_CSRCDIR))
WEIFU_ADS7029_ASMSRCS = $(call get_asmsrcs, $(WEIFU_ADS7029_ASMSRCDIR))

# get object files
WEIFU_ADS7029_COBJS = $(call get_relobjs, $(WEIFU_ADS7029_CSRCS))
WEIFU_ADS7029_ASMOBJS = $(call get_relobjs, $(WEIFU_ADS7029_ASMSRCS))
WEIFU_ADS7029_OBJS = $(WEIFU_ADS7029_COBJS) $(WEIFU_ADS7029_ASMOBJS)

# get dependency files
WEIFU_ADS7029_DEPS = $(call get_deps, $(WEIFU_ADS7029_OBJS))


# genearte library
WEIFU_ADS7029_LIB = $(OUT_DIR)/lib_weifu_ads7029.a

COMMON_COMPILE_PREREQUISITES += $(WEIFU_ADS7029_ROOT)/ads7029.mk

# library generation rule
$(WEIFU_ADS7029_LIB): $(WEIFU_ADS7029_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(WEIFU_ADS7029_OBJS)
	

WF_ADS7029_CFLAG ?= -Wno-unused-variable 
	
# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(WEIFU_ADS7029_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $(COMPILE_HW_OPT) $(WF_ADS7029_CFLAG) $< -o $@
	

.SECONDEXPANSION:
$(WEIFU_ADS7029_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $(COMPILE_HW_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(WEIFU_BASEBAND_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(WEIFU_BASEBAND_ASMSRCS)

CALTERAH_COMMON_COBJS += $(WEIFU_ADS7029_COBJS)
CALTERAH_COMMON_ASMOBJS += $(WEIFU_ADS7029_ASMOBJS)

CALTERAH_COMMON_LIBS += $(WEIFU_ADS7029_LIB)
