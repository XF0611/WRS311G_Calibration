# author: Xerxes 2022-2-15

WEIFU_PMIC_ROOT = $(WEIFU_ROOT)/pmic

WEIFU_PMIC_CSRCDIR = $(WEIFU_PMIC_ROOT)
WEIFU_PMIC_ASMSRCDIR = $(WEIFU_PMIC_ROOT)

# find all the source files in the target directories
WEIFU_PMIC_CSRCS = $(call get_csrcs, $(WEIFU_PMIC_CSRCDIR))
WEIFU_PMIC_ASMSRCS = $(call get_asmsrcs, $(WEIFU_PMIC_ASMSRCDIR))

# get object files
WEIFU_PMIC_COBJS = $(call get_relobjs, $(WEIFU_PMIC_CSRCS))
WEIFU_PMIC_ASMOBJS = $(call get_relobjs, $(WEIFU_PMIC_ASMSRCS))
WEIFU_PMIC_OBJS = $(WEIFU_PMIC_COBJS) $(WEIFU_PMIC_ASMOBJS)

# get dependency files
WEIFU_PMIC_DEPS = $(call get_deps, $(WEIFU_PMIC_OBJS))


# genearte library
WEIFU_PMIC_LIB = $(OUT_DIR)/lib_weifu_pmic.a

COMMON_COMPILE_PREREQUISITES += $(WEIFU_PMIC_ROOT)/pmic.mk

# library generation rule
$(WEIFU_PMIC_LIB): $(WEIFU_PMIC_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(WEIFU_PMIC_OBJS)
	

WF_PMIC_CFLAG ?= -Wno-unused-variable 
	
# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(WEIFU_PMIC_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $(COMPILE_HW_OPT) $(WF_PMIC_CFLAG) $< -o $@
	

.SECONDEXPANSION:
$(WEIFU_PMIC_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $(COMPILE_HW_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(WEIFU_BASEBAND_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(WEIFU_BASEBAND_ASMSRCS)

CALTERAH_COMMON_COBJS += $(WEIFU_PMIC_COBJS)
CALTERAH_COMMON_ASMOBJS += $(WEIFU_PMIC_ASMOBJS)

CALTERAH_COMMON_LIBS += $(WEIFU_PMIC_LIB)
