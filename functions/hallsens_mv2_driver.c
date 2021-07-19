/*
 * hallsens_mv2_driver.c
 *
 *  Created on: Jul 16, 2021
 *      Author: dave
 */

#include "hallsens_mv2_driver.h"
#include "general.h"
#include "avalon_spi.h"
#include <socal/socal.h>
#include <stdint.h>
#include <stdio.h>

extern void *h2p_ctrl_out_addr;
extern void *h2p_ctrl_in_addr;
extern uint32_t ctrl_out;
extern unsigned int hs_config;

void hs_init(volatile unsigned int * hs_addr, unsigned char en_mesg)
{
	// unsigned int hs_config;
	unsigned char R00, R01, R10;

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// set inversion to 0 and enable the conversion
	ctrl_out &= ~(MV2_INV | MV2_INIT | MV2_DA);

	// read the current configuration
	hs_config = hs_read_settings(hs_addr, en_mesg);
	R00 = (unsigned char) (hs_config & 0xFF);
	R01 = (unsigned char) ((hs_config >> 8) & 0xFF);
	R10 = (unsigned char) ((hs_config >> 16) & 0xFF);

	if (en_mesg)
	{
		printf("\tHall sensor initial config error checking:\n");
		if ((R00 >> MV2_R00_MA_ofst) & MV2_R00_MA_msk)
		{
			printf("\t\tMA register is not default or altered.\n");
		}
		if ((R00 >> MV2_R00_RE_ofst) & MV2_R00_RE_msk)
		{
			printf("\t\tRE register is not default or altered.\n");
		}
		if ((R00 >> MV2_R00_RA_ofst) & MV2_R00_RA_msk)
		{
			printf("\t\tRA register is not default or altered.\n");
		}
		if ((R00) & MV2_R00_OS_msk)
		{
			printf("\t\tOS register is not default or altered.\n");
		}

		if (R01 & MV2_R01_LMR_msk)
		{
			printf("\t\tLMR register is not default or altered.\n");
		}
		if (R01 & MV2_R01_SC_msk)
		{
			printf("\t\tSC register is not default or altered.\n");
		}
		if (R01 & MV2_R01_EMR_msk)
		{
			printf("\t\tEMR register is not default or altered.\n");
		}
		if (R01 & MV2_R01_HC_msk)
		{
			printf("\t\tHC register is not default or altered.\n");
		}
		if (R01 & MV2_R01_INV_msk)
		{
			printf("\t\tINV register is not default or altered.\n");
		}
		if (R01 & MV2_R01_LP_msk)
		{
			printf("\t\tLP register is not default or altered.\n");
		}
		if (R01 & MV2_R01_PO_msk)
		{
			printf("\t\tPO register is not default or altered.\n");
		}
		if (R01 & MV2_R01_SP_msk)
		{
			printf("\t\tSP register is not default or altered.\n");
		}

		if (R10 & MV2_R10_DSB)
		{
			printf("\t\tERROR! DSB register must be zero!\n");
		}
		if (((R10 >> MV2_R10_TC_ofst) & MV2_R10_TC_msk) != 1)
		{
			printf("\t\tTC register is not default or altered.\n");
		}

	}

	// re-write the register with the default value
	R00 = MV2_R00_default;
	R01 = MV2_R01_default;
	R10 = MV2_R10_default;

	hs_config = R00 | R01 << 8 | R10 << 16;

	hs_wr_settings(hs_addr, hs_config);
	printf("\tHall sensor default config is written to R00, R01, R10.\n");

	// read back the current configuration
	hs_config = hs_read_settings(hs_addr, en_mesg);
	R00 = (unsigned char) (hs_config & 0xFF);
	R01 = (unsigned char) ((hs_config >> 8) & 0xFF);
	R10 = (unsigned char) ((hs_config >> 16) & 0xFF);

	if ((R00 != MV2_R00_default) | (R01 != MV2_R01_default)
			| (R10 != MV2_R10_default))
	{
		printf("ERROR! Hall sensor spi writing is malfunctioning!\n");
	}
}

unsigned int hs_read_settings(volatile unsigned int * hs_addr,
		unsigned char en_mesg)
{
	unsigned char R00, R01, R10;

	R00 = hs_rd_reg(hs_addr, MV2_R00);
	R01 = hs_rd_reg(hs_addr, MV2_R01);
	R10 = hs_rd_reg(hs_addr, MV2_R10);

	if (en_mesg)
	{
		printf("\tHall sensor data registers:\n");
		printf("\t\tMA : %d (measurement axis --analog)\n",
				(R00 >> MV2_R00_MA_ofst) & MV2_R00_MA_msk);
		printf("\t\tRE : %d (resolution)\n",
				(R00 >> MV2_R00_RE_ofst) & MV2_R00_RE_msk);
		printf("\t\tRA : %d (range)\n",
				(R00 >> MV2_R00_RA_ofst) & MV2_R00_RA_msk);
		printf("\t\tOS : %d (output selection --digital)\n",
				(R00) & MV2_R00_OS_msk);

		printf("\t\tLMR : %d (large measurement range)\n", (R01 >> 7) & 0x01);
		printf("\t\tSC : %d (spinning current)\n", (R01 >> 6) & 0x01);
		printf("\t\tEMR : %d (extended measurement range)\n",
				(R01 >> 5) & 0x01);
		printf("\t\tHC : %d (high clock)\n", (R01 >> 4) & 0x01);
		printf("\t\tINV : %d (invert)\n", (R01 >> 3) & 0x01);
		printf("\t\tLP : %d (low power)\n", (R01 >> 2) & 0x01);
		printf("\t\tPO : %d (permanent output)\n", (R01 >> 1) & 0x01);
		printf("\t\tSP : %d (status position)\n", (R01) & 0x01);

		printf("\t\tDSB : %d (disable separate bias)\n", (R10 >> 7) & 0x01);
		printf("\t\tTC : %d (temperature compensation)\n",
				(R10 >> MV2_R10_TC_ofst) & MV2_R10_TC_msk);
	}

	return R00 | R01 << 8 | R10 << 16;

}

void hs_wr_settings(volatile unsigned int * hs_addr, unsigned int hs_settings)
{
	unsigned char R00, R01, R10;

	R00 = (unsigned char) (hs_settings & 0xFF);
	R01 = (unsigned char) ((hs_settings >> 8) & 0xFF);
	R10 = (unsigned char) ((hs_settings >> 16) & 0xFF);

	hs_wr_reg(hs_addr, MV2_R00, R00);
	hs_wr_reg(hs_addr, MV2_R01, R01);
	hs_wr_reg(hs_addr, MV2_R10, R10);

}

unsigned int hs_wr_reg(volatile unsigned int * hs_addr, unsigned char reg_addr,
		unsigned char data)
{

// send the config out
	alt_write_word((hs_addr + SPI_TXDATA_offst),
			MV2_SPI_DEFAULT | MV2_WR_msk
					| ((reg_addr & MV2_REG_msk) << MV2_REG_ofst)
					| ((data & MV2_DATA_msk) << MV2_DATA_ofst));
	while (!(alt_read_word(hs_addr + SPI_STATUS_offst) & (1 << status_TMT_bit)))
		;						// wait for the spi command to finish

// read the data from SPI operation
	return (alt_read_word(hs_addr + SPI_RXDATA_offst) & 0xFFFF);// read the data

}

unsigned char hs_rd_reg(volatile unsigned int * hs_addr, unsigned char reg_addr)
{
// read the register. data is present just at the half-end of SPI
	alt_write_word((hs_addr + SPI_TXDATA_offst),
			MV2_SPI_DEFAULT | MV2_RD_msk
					| ((reg_addr & MV2_REG_msk) << MV2_REG_ofst)
					| ((0x00 & MV2_DATA_msk) << MV2_DATA_ofst));
	while (!(alt_read_word(hs_addr + SPI_STATUS_offst) & (1 << status_TMT_bit)))
		;						// wait for the spi command to finish

	return (alt_read_word(hs_addr + SPI_RXDATA_offst) & MV2_DATA_msk);// read the data

}

void hs_rd_xyz(volatile unsigned int * hs_addr, unsigned int * x,
		unsigned int * y, unsigned int *z, unsigned int no_samples)
{
	unsigned int ii;

	hs_wr_reg(hs_addr, MV2_R00, MV2_R00_OS_Zaxis); // set the Z axis data to be output for the next SPI cycle

	for (ii = 0; ii < no_samples; ii++)
	{
		while (!(alt_read_word(h2p_ctrl_in_addr) & MV2_DR))
			;
		z[ii] = hs_wr_reg(hs_addr, MV2_R00, MV2_R00_OS_Xaxis); // get the Z axis data from previous SPI, and send X axis request for the next SPI

		while (!(alt_read_word(h2p_ctrl_in_addr) & MV2_DR))
			;
		x[ii] = hs_wr_reg(hs_addr, MV2_R00, MV2_R00_OS_Yaxis); // get the X axis data from previous SPI, and send Y axis request for the next SPI

		while (!(alt_read_word(h2p_ctrl_in_addr) & MV2_DR))
			;
		y[ii] = hs_wr_reg(hs_addr, MV2_R00, MV2_R00_OS_Zaxis); // get the Y axis data from previous SPI, and send Z axis request for the next SPI
	}
}

float conv_to_Tesla(unsigned int data)
{
	unsigned char R00, R01; //, R10;
	float EMR_mult, RA_mult, LMR_mult;

	R00 = (unsigned char) (hs_config & 0xFF);
	R01 = (unsigned char) ((hs_config >> 8) & 0xFF);
	// R10 = (unsigned char) ((hs_config >> 16) & 0xFF);

	if (R01 & MV2_R01_EMR_msk)
	{
		EMR_mult = 0.333;
	}
	else
	{
		EMR_mult = 1;
	}

	if (R01 & MV2_R01_LMR_msk)
	{
		LMR_mult = 10;
	}
	else
	{
		LMR_mult = 1;
	}

	switch ((R00 >> MV2_R00_RA_ofst) & MV2_R00_RA_msk)
	{
	case 0:
		RA_mult = 0.1;
		break;
	case 1:
		RA_mult = 0.3;
		break;
	case 3:
		RA_mult = 1;
		break;
	case 4:
		RA_mult = 3;
		break;
	}

	int offst = 32768;
	int signed_data = (int) data;

	return (((float) (signed_data - offst)) * (RA_mult * 1.2) * LMR_mult
			* EMR_mult / offst); // 1.2 factor is because the ADC saturates at field 20% larger than the range

}
