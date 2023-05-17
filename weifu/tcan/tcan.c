/*
 * tcan.c
 *
 *  Created on: 2022Äê10ÔÂ18ÈÕ
 *      Author: fei.xu1
 */
#include "tcan_hal.h"
#include "embARC_error.h"
#include "embARC_debug.h"
#include "tcan.h"
uint8_t test[100];
void read_tcan_device_id(void)
{
	int32_t res = E_OK;
	uint32_t read_data;


	for(uint16_t j = 0; j < device_id_num;j++)
	{
		read_data = tcan_spi_read(0,Device_Id + j,register_len);
		EMBARC_PRINTF("addr:%x read value is %x\r\n",Device_Id + j,read_data);
	}

}

void switch_tcan_mode(tcan_mode Mode)
{
	uint16_t read_data;
	uint16_t write_data;
	read_data = tcan_spi_read(0,Mode_Cntrl,register_len);
	read_data &= ~(7 << 0);
	read_data |= Mode;

	write_data = read_data;

	uint8_t test[100];
	EMBARC_PRINTF("mode final value is %x\r\n",write_data);
	tcan_spi_write(0,Mode_Cntrl,write_data,register_len);
}

void config_selective_wakeup(void)
{
	uint16_t read_before;
	uint16_t read_back;
	uint16_t write_value;

	/* sw config4 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Config4,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Config4,register_len);
	EMBARC_PRINTF("read Sw_Config4 is %x\r\n",read_back);

	/* set can wake id:0x765 */
	/* sw id1 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Id1,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id1,register_len);
	EMBARC_PRINTF("read Sw_Id1 is %x\r\n",read_back);

	/* sw id2 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Id2,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id2,register_len);
	EMBARC_PRINTF("read Sw_Id2 is %x\r\n",read_back);


	/* sw id3 */
	//write_value = 0x04;
	write_value = 0x1D;
	tcan_spi_write(0,Sw_Id3,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id3,register_len);
	EMBARC_PRINTF("read Sw_Id3 is %x\r\n",read_back);


	/* sw id4 */
	//write_value = 0x8C;
	write_value = 0x94;
	tcan_spi_write(0,Sw_Id4,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id4,register_len);
	EMBARC_PRINTF("read Sw_Id4 is %x\r\n",read_back);


	/* sw id mask1 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Id_Mask1,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id_Mask1,register_len);
	EMBARC_PRINTF("read Sw_Id_Mask1 is %x\r\n",read_back);


	/* sw id mask2 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Id_Mask2,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id_Mask2,register_len);
	EMBARC_PRINTF("read Sw_Id_Mask2 is %x\r\n",read_back);


	/* sw id mask3 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Id_Mask3,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id_Mask3,register_len);
	EMBARC_PRINTF("read Sw_Id_Mask3 is %x\r\n",read_back);


	/* sw id mask4 */
	write_value = 0x00;
	tcan_spi_write(0,Sw_Id_Mask4,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id_Mask4,register_len);
	EMBARC_PRINTF("read Sw_Id_Mask4 is %x\r\n",read_back);


	/* sw id maskdlc */
	//write_value = 0x00;
	write_value = 0x11;
	tcan_spi_write(0,Sw_Id_Mask_Dlc,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Id_Mask_Dlc,register_len);
	EMBARC_PRINTF("read Sw_Id_Mask_Dlc is %x\r\n",read_back);

	/* config data 0x08*/
	write_value = 0x09;
	tcan_spi_write(0,Data_Y + 0x07,write_value,register_len);
	read_back = tcan_spi_read(0,Data_Y,register_len);
	EMBARC_PRINTF("Data_Y is %x\r\n",read_back);

	/* sw config1 */
	write_value = 0xD0;
	tcan_spi_write(0,Sw_Config1,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Config1,register_len);
	EMBARC_PRINTF("read Sw_Config1 is %x\r\n",read_back);


	/* sw config3 */
	write_value = 0xFE;
	tcan_spi_write(0,Sw_Config3,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Config3,register_len);
	EMBARC_PRINTF("read Sw_Config3 is %x\r\n",read_back);


	/* sw config4 */
	write_value = 0x80;
	tcan_spi_write(0,Sw_Config4,write_value,register_len);
	read_back = tcan_spi_read(0,Sw_Config4,register_len);
	EMBARC_PRINTF("read Sw_Config4 is %x\r\n",read_back);


	/* set swe disable */
	write_value = 0x84;
	tcan_spi_write(0,Swe_Dis,write_value,register_len);
	read_back = tcan_spi_read(0,Swe_Dis,register_len);
	EMBARC_PRINTF("read Swe_Dis is %x\r\n",read_back);

	/* mode cntl default mode is standby mode*/
	write_value = 0x84;
	tcan_spi_write(0,Mode_Cntrl,write_value,register_len);
	read_back = tcan_spi_read(0,Mode_Cntrl,register_len);
	EMBARC_PRINTF("read Mode_Cntrl is %x\r\n",read_back);

	/* clear pwon flag */
    uint32_t Int2_sta = tcan_spi_read(0,Int_2,register_len);
    EMBARC_PRINTF("Int2_sta is %x\r\n",Int2_sta);
    uint32_t Pwon_clear = Int2_sta | (1 << 6);
    tcan_spi_write(0,Int_2,Pwon_clear,register_len);
}





