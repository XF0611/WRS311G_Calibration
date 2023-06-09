CALTERAH_BB_FLOW_ROOT = $(CALTERAH_COMMON_ROOT)/bb_flow

CALTERAH_BB_FLOW_CSRCDIR = $(CALTERAH_BB_FLOW_ROOT)
CALTERAH_BB_FLOW_ASMSRCDIR = $(CALTERAH_BB_FLOW_ROOT)

# find all the source files in the target directories
CALTERAH_BB_FLOW_CSRCS = $(call get_csrcs, $(CALTERAH_BB_FLOW_CSRCDIR))
CALTERAH_BB_FLOW_ASMSRCS = $(call get_asmsrcs, $(CALTERAH_BB_FLOW_ASMSRCDIR))

# get object files
CALTERAH_BB_FLOW_COBJS = $(call get_relobjs, $(CALTERAH_BB_FLOW_CSRCS))
CALTERAH_BB_FLOW_ASMOBJS = $(call get_relobjs, $(CALTERAH_BB_FLOW_ASMSRCS))
CALTERAH_BB_FLOW_OBJS = $(CALTERAH_BB_FLOW_COBJS) $(CALTERAH_BB_FLOW_ASMOBJS)

# get dependency files
CALTERAH_BB_FLOW_DEPS = $(call get_deps, $(CALTERAH_BB_FLOW_OBJS))


# genearte library
CALTERAH_BB_FLOW_LIB = $(OUT_DIR)/lib_calterah_bb_flow.a

COMMON_COMPILE_PREREQUISITES += $(CALTERAH_BB_FLOW_ROOT)/bb_flow.mk

# library generation rule
$(CALTERAH_BB_FLOW_LIB): $(CALTERAH_BB_FLOW_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(CALTERAH_BB_FLOW_OBJS)

# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(CALTERAH_BB_FLOW_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $(COMPILE_HW_OPT) $< -o $@

.SECONDEXPANSION:
$(CALTERAH_BB_FLOW_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $(COMPILE_HW_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(CALTERAH_BB_FLOW_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(CALTERAH_BB_FLOW_ASMSRCS)

CALTERAH_COMMON_COBJS += $(CALTERAH_BB_FLOW_COBJS)
CALTERAH_COMMON_ASMOBJS += $(CALTERAH_BB_FLOW_ASMOBJS)

CALTERAH_COMMON_LIBS += $(CALTERAH_BB_FLOW_LIB)
