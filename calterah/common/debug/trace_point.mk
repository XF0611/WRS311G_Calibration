CALTERAH_TRACE_ROOT = $(CALTERAH_COMMON_ROOT)/debug

CALTERAH_TRACE_CSRCDIR = $(CALTERAH_TRACE_ROOT)
CALTERAH_TRACE_ASMSRCDIR = $(CALTERAH_TRACE_ROOT)

# find all the source files in the target directories
CALTERAH_TRACE_CSRCS = $(call get_csrcs, $(CALTERAH_TRACE_CSRCDIR))
CALTERAH_TRACE_ASMSRCS = $(call get_asmsrcs, $(CALTERAH_TRACE_ASMSRCDIR))

# get object files
CALTERAH_TRACE_COBJS = $(call get_relobjs, $(CALTERAH_TRACE_CSRCS))
CALTERAH_TRACE_ASMOBJS = $(call get_relobjs, $(CALTERAH_TRACE_ASMSRCS))
CALTERAH_TRACE_OBJS = $(CALTERAH_TRACE_COBJS) $(CALTERAH_TRACE_ASMOBJS)

# get dependency files
CALTERAH_TRACE_DEPS = $(call get_deps, $(CALTERAH_TRACE_OBJS))


# genearte library
CALTERAH_TRACE_LIB = $(OUT_DIR)/lib_calterah_trace.a

COMMON_COMPILE_PREREQUISITES += $(CALTERAH_TRACE_ROOT)/trace_point.mk

# library generation rule
$(CALTERAH_TRACE_LIB): $(CALTERAH_TRACE_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(CALTERAH_TRACE_OBJS)

# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(CALTERAH_TRACE_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $< -o $@

.SECONDEXPANSION:
$(CALTERAH_TRACE_ASMOBJS): $(OUT_DIR)/%.o : %.s $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(ASM_OPT) $< -o $@

CALTERAH_COMMON_CSRC += $(CALTERAH_TRACE_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(CALTERAH_TRACE_ASMSRCS)

CALTERAH_COMMON_COBJS += $(CALTERAH_TRACE_COBJS)
CALTERAH_COMMON_ASMOBJS += $(CALTERAH_TRACE_ASMOBJS)

CALTERAH_COMMON_LIBS += $(CALTERAH_TRACE_LIB)
