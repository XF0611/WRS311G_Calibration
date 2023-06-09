CALTERAH_SPI_S_SERVER_ROOT = $(CALTERAH_COMMON_ROOT)/spi_slave_server

CALTERAH_SPI_S_SERVER_CSRCDIR = $(CALTERAH_SPI_S_SERVER_ROOT)
CALTERAH_SPI_S_SERVER_ASMSRCDIR = $(CALTERAH_SPI_S_SERVER_ROOT)

# find all the source files in the target directories
CALTERAH_SPI_S_SERVER_CSRCS = $(call get_csrcs, $(CALTERAH_SPI_S_SERVER_CSRCDIR))
CALTERAH_SPI_S_SERVER_ASMSRCS = $(call get_asmsrcs, $(CALTERAH_SPI_S_SERVER_ASMSRCDIR))

# get object files
CALTERAH_SPI_S_SERVER_COBJS = $(call get_relobjs, $(CALTERAH_SPI_S_SERVER_CSRCS))
CALTERAH_SPI_S_SERVER_ASMOBJS = $(call get_relobjs, $(CALTERAH_SPI_S_SERVER_ASMSRCS))
CALTERAH_SPI_S_SERVER_OBJS = $(CALTERAH_SPI_S_SERVER_COBJS) $(CALTERAH_SPI_S_SERVER_ASMOBJS)

# get dependency files
CALTERAH_SPI_S_SERVER_DEPS = $(call get_deps, $(CALTERAH_SPI_S_SERVER_OBJS))


# genearte library
CALTERAH_SPI_S_SERVER_LIB = $(OUT_DIR)/lib_spi_slave_server.a

COMMON_COMPILE_PREREQUISITES += $(CALTERAH_SPI_S_SERVER_ROOT)/spi_slave_server.mk

# library generation rule
$(CALTERAH_SPI_S_SERVER_LIB): $(CALTERAH_SPI_S_SERVER_OBJS)
	$(TRACE_ARCHIVE)
	$(Q)$(AR) $(AR_OPT) $@ $(CALTERAH_SPI_S_SERVER_OBJS)

# specific compile rules
# user can add rules to compile this library
# if not rules specified to this library, it will use default compiling rules
.SECONDEXPANSION:
$(CALTERAH_SPI_S_SERVER_COBJS): $(OUT_DIR)/%.o : %.c $$(COMMON_COMPILE_PREREQUISITES)
	$(TRACE_COMPILE)
	$(Q)$(CC) -c $(COMPILE_OPT) $< -o $@


CALTERAH_COMMON_CSRC += $(CALTERAH_SPI_S_SERVER_CSRCS)
CALTERAH_COMMON_ASMSRCS += $(CALTERAH_SPI_S_SERVER_ASMSRCS)

CALTERAH_COMMON_COBJS += $(CALTERAH_SPI_S_SERVER_COBJS)
CALTERAH_COMMON_ASMOBJS += $(CALTERAH_SPI_S_SERVER_ASMOBJS)

CALTERAH_COMMON_LIBS += $(CALTERAH_SPI_S_SERVER_LIB)
