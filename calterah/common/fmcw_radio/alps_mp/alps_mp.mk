CALTERAH_FMCW_RADIO_ROOT = $(CALTERAH_COMMON_ROOT)/fmcw_radio/alps_mp


CALTERAH_FMCW_RADIO_CSRCDIR = $(CALTERAH_FMCW_RADIO_ROOT)
CALTERAH_FMCW_RADIO_ASMSRCDIR = $(CALTERAH_FMCW_RADIO_ROOT)

# find all the source files in the target directories
CALTERAH_FMCW_RADIO_CSRCS = $(call get_csrcs, $(CALTERAH_FMCW_RADIO_CSRCDIR))
CALTERAH_FMCW_RADIO_ASMSRCS = $(call get_asmsrcs, $(CALTERAH_FMCW_RADIO_ASMSRCDIR))

# get object files
CALTERAH_FMCW_RADIO_COBJS = $(call get_relobjs, $(CALTERAH_FMCW_RADIO_CSRCS))
CALTERAH_FMCW_RADIO_ASMOBJS = $(call get_relobjs, $(CALTERAH_FMCW_RADIO_ASMSRCS))
CALTERAH_FMCW_RADIO_OBJS = $(CALTERAH_FMCW_RADIO_COBJS) $(CALTERAH_FMCW_RADIO_ASMOBJS)

# get dependency files
CALTERAH_FMCW_RADIO_DEPS = $(call get_deps, $(CALTERAH_FMCW_RADIO_OBJS))


# genearte library
CALTERAH_FMCW_RADIO_LIB = $(OUT_DIR)/lib_calterah_fmcw_radio.a

COMMON_COMPILE_PREREQUISITES += $(CALTERAH_FMCW_RADIO_ROOT)/alps_mp.mk

# library generation rule
$(CALTERAH_FMCW_RADIO_LIB): $(CALTERAH_FMCW_RADIO_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(CALTERAH_FMCW_RADIO_OBJS)

# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(CALTERAH_FMCW_RADIO_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $(COMPILE_HW_OPT) $< -o $@

.SECONDEXPANSION:
$(CALTERAH_FMCW_RADIO_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $(COMPILE_HW_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(CALTERAH_FMCW_RADIO_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(CALTERAH_FMCW_RADIO_ASMSRCS)

CALTERAH_COMMON_COBJS += $(CALTERAH_FMCW_RADIO_COBJS)
CALTERAH_COMMON_ASMOBJS += $(CALTERAH_FMCW_RADIO_ASMOBJS)

CALTERAH_COMMON_LIBS += $(CALTERAH_FMCW_RADIO_LIB)
