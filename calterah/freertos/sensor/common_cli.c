#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "command.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include "flash_header.h"
#include "common_cli.h"
#include "radio_ctrl.h"

#include <stdlib.h>

#ifdef CPU_STAT
#include <string.h>
#include "task.h"
static uint8_t cpu_usage_info[128];
#endif

/* uart_ota_cli */
static BaseType_t uart_ota_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t ota_command =
{
        "uart_ota",
        "\r\nuart_ota:\r\n reprogram firmare through uart interface\r\n\r\n",
        uart_ota_handler,
        0
};

static BaseType_t uart_ota_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
        fmcw_radio_reboot_cause_set(ECU_REBOOT_UART_OTA);

        chip_hw_mdelay(1);

        fmcw_radio_reset();

        return 0;
}

void uart_ota_init(void)
{
        FreeRTOS_CLIRegisterCommand(&ota_command);
}


/* firmware_version_cli */
static bool common_cli_registered = false;
static BaseType_t fw_ver_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t fw_ver_cmd = {
        "fw_ver",
        "fw_ver \n\r"
        "\tGet firmware software version. \n\r"
        "\tUsage: fw_ver \n\r",
        fw_ver_handler,
        -1
};

static BaseType_t fw_ver_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
#if (defined SYSTEM_3_STAGES_BOOT) && (defined __FIRMWARE__)
        image_header_t *flash_header = (image_header_t *)flash_header_get();
#else
        flash_header_t *flash_header = (flash_header_t *)flash_header_get();
#endif

#if FLASH_XIP_EN == 1
        snprintf(pcWriteBuffer, xWriteBufferLen, \
                "On XIP, Version: %d.%d.%d \n\rBuild Data: %s\n\rBuild Time: %s \n\rSystem Name:%s\n\rBuild commit ID: %s\n\r", \
                flash_header->sw_version.major_ver_id, \
                flash_header->sw_version.minor_ver_id, \
                flash_header->sw_version.stage_ver_id, \
                flash_header->sw_version.date, \
                flash_header->sw_version.time, \
                flash_header->sw_version.info, \
                BUILD_COMMIT_ID
        );
#else
        snprintf(pcWriteBuffer, xWriteBufferLen, \
                "On SRAM, Version: %d.%d.%d \n\rBuild Data: %s\n\rBuild Time: %s \n\rSystem Name:%s\n\rBuild commit ID: %s\n\r", \
                flash_header->sw_version.major_ver_id, \
                flash_header->sw_version.minor_ver_id, \
                flash_header->sw_version.stage_ver_id, \
                flash_header->sw_version.date, \
                flash_header->sw_version.time, \
                flash_header->sw_version.info, \
                BUILD_COMMIT_ID
        );
#endif

        return pdFALSE;
}

static uint32_t mem_read(const uint32_t addr)
{
        volatile uint32_t *ptr = (uint32_t *)addr;
        return *ptr;
}

static BaseType_t mem_dump_command_handle(char *pcWriteBuffer, \
		size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1, len2;
	uint32_t addr, length;
	uint32_t buffer;

	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	addr = (uint32_t)strtol(param1, NULL, 0);
	length = (uint32_t)strtol(param2, NULL, 0);

	for (uint32_t i = 0; i < length; i++) {
		if ((i == 0) || (i % 4 == 0)) {
			EMBARC_PRINTF("%08x", (addr + (i * 4)));
		}

		buffer = mem_read(addr + i*4);
		if ((i+1) % 4 == 0) {
			EMBARC_PRINTF("\t %08x \r\n", buffer);
		} else {
			EMBARC_PRINTF("\t %08x", buffer);
		}
	}

	return pdFALSE;
}

static const CLI_Command_Definition_t mem_dump_command =
{
	"mem_dump",
	"\rmem_dump:"
	"\r\n\tmem_dump [address] [length(word)]\r\n\r\n",
	mem_dump_command_handle,
	-1
};


static BaseType_t chip_reset_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t chip_reset_command = {
	"chip_reset",
	"chip_reset \n\r"
	"\tChip reset. \n\r"
	"\tUsage: chip_reset \n\r",
	chip_reset_command_handler,
	-1
};


static BaseType_t chip_reset_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
        //fmcw_radio_reset();
		chip_reset();
	return pdFALSE;
}

#ifdef CPU_STAT
static BaseType_t cpu_usage_stat_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
        memset(cpu_usage_info, 0, 128);
        vTaskGetRunTimeStats((char *)&cpu_usage_info);
        EMBARC_PRINTF("Task\t\tTime(us)\tUsage(%%)\r\n");
        EMBARC_PRINTF("%s", cpu_usage_info);
        return pdFALSE;
}

static const CLI_Command_Definition_t cpu_usage_stat_command = {
        "cpu_usage",
        "cpu_usage statistics\n\r",
        cpu_usage_stat_command_handler,
        0
};
#endif

void common_cmd_init(void)
{
        if (common_cli_registered)
                return;

        FreeRTOS_CLIRegisterCommand(&fw_ver_cmd);
        FreeRTOS_CLIRegisterCommand(&mem_dump_command);
        FreeRTOS_CLIRegisterCommand(&chip_reset_command);
#ifdef CPU_STAT
        FreeRTOS_CLIRegisterCommand(&cpu_usage_stat_command);
#endif
        common_cli_registered = true;

        return;
}
