# author: Xerxes 2019-9-20

WEIFU_APP_ROOT = $(WEIFU_ROOT)/app

WEIFU_APP_CSRCDIR = $(WEIFU_APP_ROOT)
WEIFU_APP_ASMSRCDIR = $(WEIFU_APP_ROOT)

# find all the source files in the target directories
WEIFU_APP_CSRCS = $(call get_csrcs, $(WEIFU_APP_CSRCDIR))
WEIFU_APP_ASMSRCS = $(call get_asmsrcs, $(WEIFU_APP_ASMSRCDIR))

# get object files
WEIFU_APP_COBJS = $(call get_relobjs, $(WEIFU_APP_CSRCS))
WEIFU_APP_ASMOBJS = $(call get_relobjs, $(WEIFU_APP_ASMSRCS))
WEIFU_APP_OBJS = $(WEIFU_APP_COBJS) $(WEIFU_APP_ASMOBJS)

# get dependency files
WEIFU_APP_DEPS = $(call get_deps, $(WEIFU_APP_OBJS))


# genearte library
WEIFU_APP_LIB = $(OUT_DIR)/lib_weifu_app.a

COMMON_COMPILE_PREREQUISITES += $(WEIFU_APP_ROOT)/app.mk

# library generation rule
$(WEIFU_APP_LIB): $(WEIFU_APP_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(WEIFU_APP_OBJS)
	

WF_APP_CFLAG ?= -Wno-unused-variable 
	
# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(WEIFU_APP_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $(COMPILE_HW_OPT) $(WF_APP_CFLAG) $< -o $@
	

.SECONDEXPANSION:
$(WEIFU_APP_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $(COMPILE_HW_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(WEIFU_BASEBAND_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(WEIFU_BASEBAND_ASMSRCS)

CALTERAH_COMMON_COBJS += $(WEIFU_APP_COBJS)
CALTERAH_COMMON_ASMOBJS += $(WEIFU_APP_ASMOBJS)

CALTERAH_COMMON_LIBS += $(WEIFU_APP_LIB)
