FREERTOS_UDS_ROOT = $(CALTERAH_ROOT)/freertos/uds

FREERTOS_UDS_CSRCDIR += $(FREERTOS_UDS_ROOT)

# find all the source files in the target directories
FREERTOS_UDS_CSRCS = $(call get_csrcs, $(FREERTOS_UDS_CSRCDIR))
FREERTOS_UDS_ASMSRCS = $(call get_asmsrcs, $(FREERTOS_UDS_ASMSRCDIR))

# get object files
FREERTOS_UDS_COBJS = $(call get_relobjs, $(FREERTOS_UDS_CSRCS))
FREERTOS_UDS_ASMOBJS = $(call get_relobjs, $(FREERTOS_UDS_ASMSRCS))
FREERTOS_UDS_OBJS = $(FREERTOS_UDS_COBJS) $(FREERTOS_UDS_ASMOBJS)

# get dependency files
FREERTOS_UDS_DEPS = $(call get_deps, $(FREERTOS_UDS_OBJS))


# genearte library
FREERTOS_UDS_LIB = $(OUT_DIR)/lib_freertos_uds.a

COMMON_COMPILE_PREREQUISITES += $(FREERTOS_UDS_ROOT)/uds.mk

# library generation rule
$(FREERTOS_UDS_LIB): $(FREERTOS_UDS_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(FREERTOS_UDS_OBJS)

# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(FREERTOS_UDS_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $< -o $@

.SECONDEXPANSION:
$(FREERTOS_UDS_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(FREERTOS_UDS_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(FREERTOS_UDS_ASMSRCS)

CALTERAH_COMMON_COBJS += $(FREERTOS_UDS_COBJS)
CALTERAH_COMMON_ASMOBJS += $(FREERTOS_UDS_ASMOBJS)

CALTERAH_COMMON_LIBS += $(FREERTOS_UDS_LIB)
CALTERAH_INC_DIR += $(FREERTOS_UDS_CSRCDIR)
