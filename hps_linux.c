// if compilation gives error of "errno not found" or "EXIT_FAILURE not found", it is a bug.
// comment #include <stdlib.h> and compile, it'll fail. And then comment it out again, it should work.
// any functions except cpmg_iterate needs to use GET_RAW_DATA. cpmg_iterate can either use GET_RAW_DATA or GET_DCONV_DATA

#include "hps_linux.h"

void open_physical_memory_device()
{
	fd_dev_mem = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd_dev_mem == -1)
	{
		printf("ERROR: could not open \"/dev/mem\".\n");
		printf("    errno = %s\n", strerror(errno));
		exit (EXIT_FAILURE);
	}
}

void close_physical_memory_device()
{
	close (fd_dev_mem);
}

void mmap_hps_peripherals()
{
	hps_gpio = mmap(NULL, hps_gpio_span, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd_dev_mem, hps_gpio_ofst);
	if (hps_gpio == MAP_FAILED)
	{
		printf("Error: hps_gpio mmap() failed.\n");
		printf("    errno = %s\n", strerror(errno));
		close (fd_dev_mem);
		exit (EXIT_FAILURE);
	}
}

void munmap_hps_peripherals()
{
	if (munmap(hps_gpio, hps_gpio_span) != 0)
	{
		printf("Error: hps_gpio munmap() failed\n");
		printf("    errno = %s\n", strerror(errno));
		close (fd_dev_mem);
		exit (EXIT_FAILURE);
	}

	hps_gpio = NULL;
}

void mmap_fpga_peripherals()
{
	// IMPORTANT: If you try to only mmap the fpga leds, it is possible for the
	// operation to fail, and you will get "Invalid argument" as errno. The
	// mmap() manual page says that you can only map a file from an offset which
	// is a multiple of the system's page size.

	// In our specific case, our fpga leds are located at address 0xFF200000,
	// which is a multiple of the page size, however this is due to luck because
	// the fpga leds are the only peripheral connected to the h2f_lw_axi_master.
	// The typical page size in Linux is 0x1000 bytes.

	// So, generally speaking, you will have to mmap() the closest address which
	// is a multiple of your page size and access your peripheral by a specific
	// offset from the mapped address.

	lwaxi_base = mmap(NULL, h2f_lw_axi_master_span, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd_dev_mem, h2f_lw_axi_master_ofst);
	if (lwaxi_base == MAP_FAILED)
	{
		printf("Error: h2f_lw_axi_master mmap() failed.\n");
		printf("    errno = %s\n", strerror(errno));
		close (fd_dev_mem);
		exit (EXIT_FAILURE);
	}

	axi_base = mmap(NULL, h2f_axi_master_span, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd_dev_mem, h2f_axi_master_ofst);
	if (axi_base == MAP_FAILED)
	{
		printf("Error: h2f_axi_master mmap() failed.\n");
		printf("    errno = %s\n", strerror(errno));
		close (fd_dev_mem);
		exit (EXIT_FAILURE);
	}

	h2p_ctrl_out_addr = lwaxi_base + CTRL_OUT_BASE;
	h2p_ctrl_in_addr = lwaxi_base + CTRL_IN_BASE;
	h2p_pulse1_addr = lwaxi_base + NMR_PARAMETERS_PULSE_90DEG_BASE;
	h2p_pulse2_addr = lwaxi_base + NMR_PARAMETERS_PULSE_180DEG_BASE;
	h2p_delay1_addr = lwaxi_base + NMR_PARAMETERS_DELAY_NOSIG_BASE;
	h2p_delay2_addr = lwaxi_base + NMR_PARAMETERS_DELAY_SIG_BASE;
	h2p_nmr_sys_pll_addr = lwaxi_base + NMR_SYS_PLL_RECONFIG_BASE;
	h2p_echo_per_scan_addr = lwaxi_base + NMR_PARAMETERS_ECHOES_PER_SCAN_BASE;
	// h2p_i2c_ext_addr				= h2f_lw_axi_master + I2C_EXT_BASE;
	h2p_i2c_int_addr = lwaxi_base + I2C_INT_BASE;
	h2p_adc_fifo_addr = lwaxi_base + ADC_FIFO_MEM_OUT_BASE;
	h2p_adc_fifo_status_addr = lwaxi_base + ADC_FIFO_MEM_IN_CSR_BASE;
	h2p_adc_samples_per_echo_addr = lwaxi_base
			+ NMR_PARAMETERS_SAMPLES_PER_ECHO_BASE;
	h2p_init_adc_delay_addr = lwaxi_base + NMR_PARAMETERS_INIT_DELAY_BASE;
	h2p_rx_delay_addr = lwaxi_base + NMR_PARAMETERS_RX_DELAY_BASE;

	h2p_dac_preamp_addr = lwaxi_base + DAC_PREAMP_BASE;
	h2p_dac_grad_addr = lwaxi_base + DAC_GRAD_BASE;

	h2p_spi_mtch_ntwrk_addr = lwaxi_base + SPI_MTCH_NTWRK_BASE;
	h2p_spi_afe_relays_addr = lwaxi_base + SPI_AFE_RELAYS_BASE;
	h2p_analyzer_pll_addr = lwaxi_base + ANALYZER_PLL_RECONFIG_BASE;
	h2p_t1_pulse = lwaxi_base + NMR_PARAMETERS_PULSE_T1_BASE;
	h2p_t1_delay = lwaxi_base + NMR_PARAMETERS_DELAY_T1_BASE;
	h2p_dma_addr = lwaxi_base + DMA_FIFO_BASE;
	h2p_dconvi_dma_addr = lwaxi_base + DMA_DCONVI_BASE;
	//h2p_dconvq_dma_addr				= h2f_lw_axi_master + DMA_DCONVQ_BASE;
	h2p_dconvi_addr = lwaxi_base + DCONV_FIFO_MEM_OUT_BASE;
	//h2p_dconvq_addr					= h2f_lw_axi_master + DCONV_FIFO_MEM_Q_OUT_BASE;
	h2p_dconvi_csr_addr = lwaxi_base + DCONV_FIFO_MEM_IN_CSR_BASE;
	//h2p_dconvq_csr_addr				= h2f_lw_axi_master + DCONV_FIFO_MEM_Q_IN_CSR_BASE;
	h2p_adc_val_sub = lwaxi_base + NMR_PARAMETERS_ADC_VAL_SUB_BASE;
	h2p_dec_fact_addr = lwaxi_base + NMR_PARAMETERS_DEC_FACT_BASE;
	h2p_echo_skip_hw_addr = lwaxi_base + NMR_PARAMETERS_ECHO_SKIP_BASE;

	h2p_dconv_firI_addr = lwaxi_base + DCONV_FIR_BASE;
	h2p_dconv_firQ_addr = lwaxi_base + DCONV_FIR_Q_BASE;

	h2p_sdram_addr = axi_base + SDRAM_BASE;
	h2p_switches_addr = axi_base + SWITCHES_BASE;

	// magnet memory map magnet addresses
	h2p_mgnt_chg_plen_addr = lwaxi_base + NMR_PARAMETERS_MGNT_CHG_PLEN_BASE;
	h2p_mgnt_chg_dlen_addr = lwaxi_base + NMR_PARAMETERS_MGNT_CHG_DLEN_BASE;
	h2p_mgnt_dchg_plen_addr = lwaxi_base + NMR_PARAMETERS_MGNT_DCHG_PLEN_BASE;
	h2p_mgnt_dchg_dlen_addr = lwaxi_base + NMR_PARAMETERS_MGNT_DCHG_DLEN_BASE;
	h2p_mgnt_n_addr = lwaxi_base + NMR_PARAMETERS_MGNT_N_BASE;
	h2p_mgnt_d_addr = lwaxi_base + NMR_PARAMETERS_MGNT_D_BASE;

	// hall-effect sensor
	h2p_mgnt_hs_addr = lwaxi_base + HS_SPI_BASE;

	// dummy code
	//h2p_dmadummy_addr				= h2f_lw_axi_master + DMA_DUMMY_BASE;
	//h2p_fifoin_dummy_addr			= h2f_axi_master + FIFO_DUMMY_IN_BASE;
	//h2p_fifoout_dummy_addr			= h2f_axi_master + FIFO_DUMMY_OUT_BASE;
	//h2p_fifoincsr_dummy_addr		= h2f_axi_master + FIFO_DUMMY_IN_CSR_BASE;

	//h2p_fifoin64dummy_addr			= h2f_axi_master + FIFO_DUMMY64_IN_IN_BASE;
	//h2p_fifoout64dummy_addr			= h2f_axi_master + FIFO_DUMMY64_OUT_OUT_BASE;
	//h2p_fifoout64csrdummy_addr		= h2f_axi_master + FIFO_DUMMY64_OUT_IN_CSR_BASE;

}

void munmap_fpga_peripherals()
{

	if (munmap(lwaxi_base, h2f_lw_axi_master_span) != 0)
	{
		printf("Error: h2f_lw_axi_master munmap() failed\n");
		printf("    errno = %s\n", strerror(errno));
		close (fd_dev_mem);
		exit (EXIT_FAILURE);
	}

	lwaxi_base = NULL;
	fpga_leds = NULL;
	fpga_switches = NULL;

}

void mmap_peripherals()
{
	mmap_hps_peripherals();
	mmap_fpga_peripherals();
}

void munmap_peripherals()
{
	munmap_hps_peripherals();
	munmap_fpga_peripherals();
}

void create_measurement_folder(char * foldertype)
{
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	char command[60];
	sprintf(foldername, "%04d_%02d_%02d_%02d_%02d_%02d_%s", tm.tm_year + 1900,
			tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
			foldertype);
	sprintf(command, "mkdir %s", foldername);
	system(command);

	// copy the executable file to the folder
	// sprintf(command,"cp ./thesis_nmr_de1soc_hdl2.0 %s/execfile",foldername);
	// system(command);
}

void write_relay_cnt(uint16_t c_shunt, uint16_t c_series, uint8_t en_mesg)
{

	// data is transfered LSB first, but the data needs to be sent in following sequence:
	// of cseries_lsb, cshunt_lsb+cseries_msb, cshunt_msb
	uint8_t cser_lsb, cshunt_lsb_cser_msb, cshunt_msb;

	cser_lsb = (uint8_t)(c_series & 0xFF);
	cshunt_lsb_cser_msb = (uint8_t)(
			((c_shunt & 0x0F) << 4) | ((c_series >> 8) & 0x0F));
	cshunt_msb = (uint8_t)((c_shunt >> 4) & 0xFF);

	alt_write_word((h2p_spi_mtch_ntwrk_addr + SPI_TXDATA_offst),
			((uint32_t) cshunt_msb) << 16
					| ((uint32_t) cshunt_lsb_cser_msb) << 8 | cser_lsb); // set the matching network
	while (!(alt_read_word(h2p_spi_mtch_ntwrk_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		; // wait for the spi command to finish
}

void write_pamprelay_cnt(uint32_t val, uint8_t en_mesg)
{
	alt_write_word((h2p_spi_afe_relays_addr + SPI_TXDATA_offst), val);
	while (!(alt_read_word(h2p_spi_afe_relays_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		; // wait for the spi command to finish
	if (en_mesg)
	{
		printf("\tpamprelay control via spi (PCB v5 only!)...\n");
	}

}

void check_i2c_isr_stat(volatile unsigned long * i2c_addr, uint8_t en_mesg)
{
	uint32_t isr_status;
	isr_status = alt_read_word(i2c_addr + ISR_OFST);
	if (isr_status & RX_OVER_MSK)
	{
		printf(
				"\t[ERROR] Receive data FIFO has overrun condition, new data is lost\n");
		alt_write_word((i2c_addr + ISR_OFST), RX_OVER_MSK); // clears receive overrun
	}
	else
	{
		if (en_mesg)
			printf("\t[NORMAL] No receive overrun\n");
	}
	if (isr_status & ARBLOST_DET_MSK)
	{
		printf("\t[ERROR] Core has lost bus arbitration\n");
		alt_write_word((i2c_addr + ISR_OFST), ARBLOST_DET_MSK); // clears receive overrun
	}
	else
	{
		if (en_mesg)
			printf("\t[NORMAL] No arbitration lost\n");
	}
	if (isr_status & NACK_DET_MSK)
	{
		printf("\t[ERROR] NACK is received by the core\n");
		alt_write_word((i2c_addr + ISR_OFST), NACK_DET_MSK); // clears receive overrun
	}
	else
	{
		if (en_mesg)
			printf("\t[NORMAL] ACK has been received\n");
	}
	if (isr_status & RX_READY_MSK)
	{
		printf(
				"\t[WARNING] RX_DATA_FIFO level is equal or more than its threshold\n");
	}
	else
	{
		if (en_mesg)
			printf(
					"\t[NORMAL] RX_DATA_FIFO level is less than its threshold\n");
	}
	if (isr_status & TX_READY_MSK)
	{
		printf(
				"\t[WARNING] TFR_CMD level is equal or more than its threshold\n");
	}
	else
	{
		if (en_mesg)
			printf("\t[NORMAL] TFR_CMD level is less than its threshold\n");
	}

	if (en_mesg)
		printf("\t --- \n");
}

void write_i2c_int_cnt(uint32_t en, uint32_t addr_msk0, uint32_t addr_msk1,
		uint8_t en_mesg)
{

	// en		: 1 for enable the address mask given, 0 for disable the address mask given
	// addr_msk	: give 1 to the desired address mask that needs to be changed, and 0 to the one that doesn't need to be changed
	// en_msg	: enable error message

	uint8_t i2c_addr_cnt0, i2c_addr_cnt1;
	i2c_addr_cnt0 = 0x40;	// i2c address for TCA9555PWR used by the relay
	i2c_addr_cnt1 = 0x42;

	addr_msk0 = addr_msk0 & 0xFFFF;
	addr_msk1 = addr_msk1 & 0xFFFF;

	i2c_addr_cnt0 >>= 1;// shift by one because the LSB address is not used as an address (controlled by the Altera I2C IP)
	i2c_addr_cnt1 >>= 1;// shift by one because the LSB address is not used as an address (controlled by the Altera I2C IP)

	uint8_t i2c0_port0, i2c0_port1;
	uint8_t i2c1_port0, i2c1_port1;

	if (en)
	{
		ctrl_i2c0 = ctrl_i2c0 | addr_msk0;
		ctrl_i2c1 = ctrl_i2c1 | addr_msk1;
	}
	else
	{
		ctrl_i2c0 = ctrl_i2c0 & ~(addr_msk0);
		ctrl_i2c1 = ctrl_i2c1 & ~(addr_msk1);
	}

	i2c0_port0 = ctrl_i2c0 & 0xFF;
	i2c0_port1 = (ctrl_i2c0 >> 8) & 0xFF;
	i2c1_port0 = ctrl_i2c1 & 0xFF;
	i2c1_port1 = (ctrl_i2c1 >> 8) & 0xFF;

	alt_write_word((h2p_i2c_int_addr + ISR_OFST),
			RX_OVER_MSK | ARBLOST_DET_MSK | NACK_DET_MSK); // RESET THE I2C FROM PREVIOUS ERRORS
	alt_write_word((h2p_i2c_int_addr + CTRL_OFST), 1 << CORE_EN_SHFT); // enable i2c core

	alt_write_word((h2p_i2c_int_addr + SCL_LOW_OFST), 250); // set the SCL_LOW_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word((h2p_i2c_int_addr + SCL_HIGH_OFST), 250); // set the SCL_HIGH_OFST to 250 for 100 kHz with 50 MHz clock
	alt_write_word((h2p_i2c_int_addr + SDA_HOLD_OFST), 1); // set the SDA_HOLD_OFST to 1 as the default (datasheet requires min 0 ns hold time)

	// set values for i2c_addr_cnt0
	// set port 0 as output
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(1 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt0 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_CONF_PORT0 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((0x00) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);

	// set port 1 as output
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(1 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt0 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_CONF_PORT1 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((0x0F) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg); // configure first 4 bits as input

	// set output on port 0
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt0 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_OUT_PORT0 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((i2c0_port0) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);

	// set output on port 1
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt0 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_OUT_PORT1 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((i2c0_port1) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);

	//if (en_mesg) {
	//	printf("Status for i2c transactions:\n");
	//	printf("\ti2c0_port0 : %x\n", i2c0_port0);
	//	printf("\ti2c0_port1 : %x\n", i2c0_port1);
	//}

	// set values for i2c_addr_cnt0
	// set port 0 as output
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(1 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt1 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_CONF_PORT0 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((0xFF) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);// configure all bits as input
	// set port 1 as output
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(1 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt1 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_CONF_PORT1 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((0x00) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	// set output on port 0
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt1 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_OUT_PORT0 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((i2c1_port0) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	// set output on port 1
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT) | (i2c_addr_cnt1 << AD_SHFT)
					| (WR_I2C << RW_D_SHFT));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (0 << STO_SHFT)
					| (CNT_REG_OUT_PORT1 & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);
	alt_write_word((h2p_i2c_int_addr + TFR_CMD_OFST),
			(0 << STA_SHFT) | (1 << STO_SHFT) | ((i2c1_port1) & I2C_DATA_MSK));
	check_i2c_isr_stat(h2p_i2c_int_addr, en_mesg);

	//if (en_mesg) {
	//	printf("Status for i2c transactions:\n");
	//	printf("\ti2c1_port0 : %x\n", i2c1_port0);
	//	printf("\ti2c1_port1 : %x\n", i2c1_port1);
	//}

	alt_write_word((h2p_i2c_int_addr + CTRL_OFST), 0 << CORE_EN_SHFT); // disable i2c core

	usleep(10000); // delay to finish i2c operation

}

void sweep_matching_network()
{
	uint8_t c_sw = 0;
	uint8_t c_sta = 36;
	uint8_t c_sto = 40;
	uint8_t c_spa = 1;
	while (1)
	{
		for (c_sw = c_sta; c_sw <= c_sto; c_sw += c_spa)
		{
			// put number between 1 and 255 (0 is when nothing is connected)
			write_relay_cnt(c_sw, 75, DISABLE_MESSAGE); //(c_shunt, c_series)
			printf("c_match_ntwrk = %d\n", c_sw);
			usleep(2000000);
		}
	}
}

void init_dac_ad5724r()
{
	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	alt_write_word((h2p_dac_preamp_addr + SPI_TXDATA_offst),
			WR_DAC | PWR_CNT_REG | DAC_A_PU | DAC_B_PU | DAC_C_PU | DAC_D_PU
					| REF_PU); // power up reference voltage, dac A, dac B, dac C, and DAC D
	while (!(alt_read_word(h2p_dac_preamp_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		;					// wait for the spi command to finish
	alt_write_word((h2p_dac_preamp_addr + SPI_TXDATA_offst),
			WR_DAC | OUT_RANGE_SEL_REG | DAC_ALL | PN50);// set range voltage to +/- 5.0V
	while (!(alt_read_word(h2p_dac_preamp_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		;					// wait for the spi command to finish
	alt_write_word((h2p_dac_preamp_addr + SPI_TXDATA_offst),
			WR_DAC | CNT_REG | Other_opt | Clamp_en);// enable the current limit clamp
	while (!(alt_read_word(h2p_dac_preamp_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		;					// wait for the spi command to finish

	// clear the DAC output
	ctrl_out = ctrl_out & (~DAC_CLR);
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

	// release the clear pin
	ctrl_out = ctrl_out | DAC_CLR;
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

}

void print_warning_ad5724r(uint8_t en_mesg)
{
	int dataread;

	alt_write_word((h2p_dac_preamp_addr + SPI_TXDATA_offst),
			RD_DAC | PWR_CNT_REG);			// read the power control register
	while (!(alt_read_word(h2p_dac_preamp_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		;			// wait for the spi command to finish
	alt_write_word((h2p_dac_preamp_addr + SPI_TXDATA_offst),
			WR_DAC | CNT_REG | NOP);					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_preamp_addr + SPI_STATUS_offst)
			& (1 << status_TMT_bit)))
		;			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_preamp_addr + SPI_STATUS_offst)
			& (1 << status_RRDY_bit)))
		;			// wait for the data to be ready
	dataread = alt_read_word(h2p_dac_preamp_addr + SPI_RXDATA_offst);// read the data
	if (dataread & (TSD))
	{
		printf("\t\nDevice is in thermal shutdown (TSD) mode!\n");
	}
	if (dataread & OC_A)
	{
		printf("\tDAC A overcurrent alert (OCa)!\n");
		usleep(10);
	}
	if (dataread & OC_B)
	{
		printf("\tDAC B overcurrent alert (OCb)!\n");
		usleep(10);
	}
	if (dataread & OC_C)
	{
		printf("\tDAC C overcurrent alert (OCc)!\n");
		usleep(10);
	}
	if (dataread & OC_D)
	{
		printf("\tDAC D overcurrent alert (OCd)!\n");
		usleep(10);
	}

	if (en_mesg)
	{
		if (dataread & DAC_A_PU)
		{
			printf("\tDAC A is up");
		}
		else
		{
			printf("\tDAC A is down");
		}

		if (dataread & DAC_B_PU)
		{
			printf("\tDAC B is up");
		}
		else
		{
			printf("\tDAC B is down");
		}

		if (dataread & DAC_C_PU)
		{
			printf("\tDAC C is up");
		}
		else
		{
			printf("\tDAC C is down");
		}

		if (dataread & DAC_D_PU)
		{
			printf("\tDAC D is up");
		}
		else
		{
			printf("\tDAC D is down");
		}

		if (dataread & REF_PU)
		{
			printf("\tVREF is up");
		}
		else
		{
			printf("\tVREF is down");
		}
	}

}

void wr_dac_ad5724r(volatile unsigned int * dac_addr, unsigned int dac_id,
		double volt, uint8_t en_mesg)
{
	int16_t volt_int;

	uint8_t ldac_is_wired = 0; // if LDAC pin is wired to the FPGA
	uint8_t sdo_is_wired = 0; // if the SDO pin is wired to the FPGA

	volt_int = (int16_t)((volt / 5) * 2048);
	if (volt_int > 2047)
	{
		volt_int = 2047;
	}
	else
	{
		if (volt_int < -2048)
		{
			volt_int = -2048;
		}
	}

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	alt_write_word((dac_addr + SPI_TXDATA_offst),
			WR_DAC | DAC_REG | dac_id | ((volt_int & 0x0FFF) << 4)); // set the voltage
	while (!(alt_read_word(dac_addr + SPI_STATUS_offst) & (1 << status_TMT_bit)))
		;						// wait for the spi command to finish

	// use this only if SDO pin is connected to the FPGA
	// read the value of the DAC, check warning, and redo the writing if necessary
	if (sdo_is_wired)
	{
		alt_write_word((dac_addr + SPI_TXDATA_offst),
				RD_DAC | DAC_REG | dac_id | 0x00);			// read DAC value
		while (!(alt_read_word(dac_addr + SPI_STATUS_offst)
				& (1 << status_TMT_bit)))
			;			// wait for the spi command to finish
		alt_write_word((dac_addr + SPI_TXDATA_offst), WR_DAC | CNT_REG | NOP);// no operation (NOP)
		while (!(alt_read_word(dac_addr + SPI_STATUS_offst)
				& (1 << status_TMT_bit)))
			;			// wait for the spi command to finish
		while (!(alt_read_word(dac_addr + SPI_STATUS_offst)
				& (1 << status_RRDY_bit)))
			;			// wait for read data to be ready

		int dataread;
		dataread = alt_read_word(dac_addr + SPI_RXDATA_offst);// read the data at the dac register
		if (en_mesg)
		{
			printf("\tV_in: %4.3f V ", (double) volt_int / 2048 * 5); // print the voltage desired
			printf("\t(w:0x%04x)", (volt_int & 0x0FFF)); // print the integer dac_varac value, truncate to 12-bit signed integer value
			printf("\t(r:0x%04x)\n", dataread >> 4);	// print the read value
		}
		usleep(100);
		print_warning_ad5724r(en_mesg); // find out if warning has been detected

		// recursion to make sure it works
		if ((volt_int & 0x0FFF) != (dataread >> 4))
		{
			wr_dac_ad5724r(dac_addr, dac_id, volt, en_mesg);
		}
	}

	// write data register to DAC output (ONLY IF LDAC IS WIRED)
	if (ldac_is_wired)
	{
		ctrl_out = ctrl_out & ~DAC_LDAC_en;
		alt_write_word((h2p_ctrl_out_addr), ctrl_out);
		usleep(50);

		// disable LDAC one more time
		ctrl_out = ctrl_out | DAC_LDAC_en;
		alt_write_word((h2p_ctrl_out_addr), ctrl_out);
		usleep(50);
	}
}

void check_dma(volatile unsigned int * dma_addr, uint8_t en_mesg)
{
	// this function waits until the dma addressed finishes its operation
	unsigned int dma_status;
	do
	{
		dma_status = alt_read_word(dma_addr + DMA_STATUS_OFST);
		if (en_mesg)
		{
			printf("\tDMA Status reg: 0x%x\n", dma_status);
			if (!(dma_status & DMA_STAT_DONE_MSK))
			{
				printf("\tDMA transaction is not done.\n");
			}
			if (dma_status & DMA_STAT_BUSY_MSK)
			{
				printf("\tDMA is busy.\n");

				// print length register
				printf("\t--> DMA length register: %d\n",
						alt_read_word(dma_addr + DMA_LENGTH_OFST));	// set transfer length (in byte, so multiply by 4 to get word-addressing));

				// wait time in case of DMA busy. Only valid under ENABLE_MESSAGE because the loop needs to be fast otherwise
				unsigned int wait_time_ms = 500;
				usleep(wait_time_ms * 1000); // wait time to prevent overloading the DMA bus arbitration request only if the dma is busy
				printf("\t---> waiting for %d ms ...\n", wait_time_ms);
			}
		}
	} while (!(dma_status & DMA_STAT_DONE_MSK)
			|| (dma_status & DMA_STAT_BUSY_MSK)); // keep in the loop when the 'DONE' bit is '0' and 'BUSY' bit is '1'
	if (en_mesg)
	{
		if (dma_status & DMA_STAT_REOP_MSK)
		{
			printf(
					"\tDMA transaction completed due to end-of-packet on read side.\n");
		}
		if (dma_status & DMA_STAT_WEOP_MSK)
		{
			printf(
					"\tDMA transaction completed due to end-of-packet on write side.\n");
		}
		if (dma_status & DMA_STAT_LEN_MSK)
		{
			printf(
					"\tDMA transaction completed due to length-register decrements to 0.\n");
		}
	}
}

void fifo_to_sdram_dma_trf(volatile unsigned int * dma_addr, uint32_t rd_addr,
		uint32_t wr_addr, uint32_t transfer_length)
{
	// the original conventional code
	alt_write_word(dma_addr + DMA_CONTROL_OFST, DMA_CTRL_SWRST_MSK); // write twice to do software reset
	alt_write_word(dma_addr + DMA_CONTROL_OFST, DMA_CTRL_SWRST_MSK); // software resetted
	alt_write_word(dma_addr + DMA_STATUS_OFST, 0x0); 	// clear the DONE bit
	alt_write_word(dma_addr + DMA_READADDR_OFST, rd_addr); // set DMA read address
	alt_write_word(dma_addr + DMA_WRITEADDR_OFST, wr_addr);	// set DMA write address
	alt_write_word(dma_addr + DMA_LENGTH_OFST, transfer_length * 4);// set transfer length (in byte, so multiply by 4 to get word-addressing)
	alt_write_word(dma_addr + DMA_CONTROL_OFST,
			(DMA_CTRL_WORD_MSK | DMA_CTRL_LEEN_MSK | DMA_CTRL_RCON_MSK)); // set settings for transfer
	alt_write_word(dma_addr + DMA_CONTROL_OFST,
			(DMA_CTRL_WORD_MSK | DMA_CTRL_LEEN_MSK | DMA_CTRL_RCON_MSK
					| DMA_CTRL_GO_MSK)); // set settings & also enable transfer
	//

	/* burst method (tested. Doesn't really work with large data due to slow reinitialization. Generally burst mode only works and faster when the amount of data is less than max burst size. Otherwise It doesn't work)
	 uint32_t burst_size = 1024; // the max burst size is set in QSYS in DMA section
	 uint32_t transfer_remain = transfer_length*4; // multiplied by 4 to get word-addressing
	 alt_write_word(dma_addr+DMA_CONTROL_OFST,	DMA_CTRL_SWRST_MSK); 	// write twice to do software reset
	 alt_write_word(dma_addr+DMA_CONTROL_OFST,	DMA_CTRL_SWRST_MSK); 	// software resetted
	 alt_write_word(dma_addr+DMA_READADDR_OFST,	rd_addr); 				// set DMA read address (fifo)
	 uint32_t iaddr = 0;
	 while (transfer_remain>burst_size) { // transfer with burst_size when remaining data is larger than burst_size
	 alt_write_word(dma_addr+DMA_WRITEADDR_OFST,	wr_addr + iaddr);		// set DMA write address
	 alt_write_word(dma_addr+DMA_LENGTH_OFST, burst_size);				// set transfer length (in byte, so multiply by 4 to get word-addressing)
	 alt_write_word(dma_addr+DMA_CONTROL_OFST,	(DMA_CTRL_WORD_MSK|DMA_CTRL_LEEN_MSK|DMA_CTRL_RCON_MSK)); // set settings for transfer
	 alt_write_word(dma_addr+DMA_CONTROL_OFST,	(DMA_CTRL_WORD_MSK|DMA_CTRL_LEEN_MSK|DMA_CTRL_RCON_MSK|DMA_CTRL_GO_MSK)); // set settings & also enable transfer
	 transfer_remain -= burst_size;
	 iaddr += (burst_size/4);
	 check_dma(dma_addr, DISABLE_MESSAGE); // wait for the dma operation to complete
	 }
	 if (transfer_remain>0) { // transfer with the remaining data
	 alt_write_word(dma_addr+DMA_WRITEADDR_OFST,	wr_addr + iaddr);		// set DMA write address
	 alt_write_word(dma_addr+DMA_LENGTH_OFST, transfer_remain);			// set transfer length (in byte, so multiply by 4 to get word-addressing)
	 alt_write_word(dma_addr+DMA_CONTROL_OFST,	(DMA_CTRL_WORD_MSK|DMA_CTRL_LEEN_MSK|DMA_CTRL_RCON_MSK)); // set settings for transfer
	 alt_write_word(dma_addr+DMA_CONTROL_OFST,	(DMA_CTRL_WORD_MSK|DMA_CTRL_LEEN_MSK|DMA_CTRL_RCON_MSK|DMA_CTRL_GO_MSK)); // set settings & also enable transfer
	 check_dma(dma_addr, DISABLE_MESSAGE); // wait for the dma operation to complete
	 }
	 */
}

void reset_dma(volatile unsigned int * dma_addr)
{
	alt_write_word(dma_addr + DMA_CONTROL_OFST, DMA_CTRL_SWRST_MSK); // write twice to do software reset
	alt_write_word(dma_addr + DMA_CONTROL_OFST, DMA_CTRL_SWRST_MSK); // software resetted
}

#ifdef GET_RAW_DATA
void datawrite_with_dma (uint32_t transfer_length, uint8_t en_mesg)
{
	fifo_to_sdram_dma_trf (h2p_dma_addr, ADC_FIFO_MEM_OUT_BASE, SDRAM_BASE, transfer_length);
	check_dma(h2p_dma_addr, DISABLE_MESSAGE); // wait for the dma operation to complete. POSSIBLE_ISSUES: DEPENDING ON THE LENGTH OF CPMG, THIS SCRIPT MIGHT BREAK IF ENABLE_MESSAGE

	/*
	 int i_sd = 0;
	 unsigned int fifo_data_read;
	 for (i_sd=0; i_sd < transfer_length; i_sd++)
	 {
	 fifo_data_read = alt_read_word(h2p_sdram_addr+i_sd);

	 // the data is 2 symbols-per-beat in the fifo.
	 // And the symbol arrangement can be found in Altera Embedded Peripherals pdf.
	 // The 32-bit data per beat is transfered from FIFO to the SDRAM with the same
	 // format so this formatting should follow the FIFO format.

	 rddata_16[i_sd*2] = fifo_data_read & 0x3FFF;
	 rddata_16[i_sd*2+1] = (fifo_data_read>>16) & 0x3FFF;
	 }
	 */

	memcpy(rddata,(int*)h2p_sdram_addr,transfer_length*sizeof(int));
	// buf32_to_buf16 (rddata, rddata_16, transfer_length );// transfer data from 32-bit buffer to 16-bit buffer

}
#endif

#ifdef GET_DCONV_DATA
void data_dconv_write_with_dma(uint32_t transfer_length, uint8_t en_mesg)
{
	int i_sd = 0;
	int fifo_data_read;

	reset_dma(h2p_dconvi_dma_addr);
	fifo_to_sdram_dma_trf (h2p_dconvi_dma_addr, DCONV_FIFO_MEM_OUT_BASE, SDRAM_BASE, transfer_length); // add data_len offset due to raw data before. (*4 factor is due to byte-addressing)

	// process dconvi
	check_dma(h2p_dconvi_dma_addr, DISABLE_MESSAGE);// check and wait until dma is done
	//for (i_sd = 0; i_sd < transfer_length; i_sd++)
	//{
	//	fifo_data_read = alt_read_word(h2p_sdram_addr + i_sd);
	//	dconv[i_sd] = fifo_data_read;
	//}

	memcpy(dconv,(int*)h2p_sdram_addr,transfer_length*sizeof(int));

}
#endif

void runFSM(double nmr_fsm_clkfreq, uint32_t ph_cycl_en,
		unsigned int acq_length, char * filename, uint8_t sav_indv_scan,
		uint8_t store_to_sdram_noread, uint8_t rd_sdram_OR_n_rd_fifo)
{

	// read settings
	// uint8_t store_to_sdram_noread = 0; // do not write the data from fifo to text file (external reading mechanism should be implemented)
	// uint8_t rd_sdram_OR_rd_fifo = 1; // store data to sdram (increasing memory limit). Or else the program reads data directly from the fifo

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// set pll for CPMG
	Set_PLL(h2p_nmr_sys_pll_addr, 0, nmr_fsm_clkfreq, 0.5, DISABLE_MESSAGE);
	Reset_PLL(h2p_ctrl_out_addr, PLL_NMR_SYS_RST_ofst, ctrl_out);
	Set_DPS(h2p_nmr_sys_pll_addr, 0, 0, DISABLE_MESSAGE);
	Wait_PLL_To_Lock(h2p_ctrl_in_addr, PLL_NMR_SYS_lock_ofst);

	// cycle phase for CPMG measurement (in the case of fix phase_cycle state, this code will just generate the negation of it.
	if (ph_cycl_en == ENABLE)
	{
		if (ctrl_out & PHASE_CYCLING)
		{
			ctrl_out &= ~PHASE_CYCLING;
		}
		else
		{
			ctrl_out |= PHASE_CYCLING;
		}
		alt_write_word((h2p_ctrl_out_addr), ctrl_out);
		usleep(1);
	}

	// reset buffer
	ctrl_out |= (0x01 << ADC_FIFO_RST_ofst);
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);
	ctrl_out &= ~(0x01 << ADC_FIFO_RST_ofst);
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

	// start fsm
	// it will reset the pll as well, so it's important to set the phase
	// the pll_rst_dly should be longer than the delay coming from changing the phase
	// otherwise, the fsm will start with wrong relationship between 4 pll output clocks (1/2 pi difference between clock)
	// alt_write_word( (h2p_nmr_pll_rst_dly_addr) , 1000000 );	// set the amount of delay for pll reset (with 50MHz system clock, every tick means 20ns) -> default: 100000
	alt_write_word((h2p_ctrl_out_addr), ctrl_out | (0x01 << FSM_START_ofst)); // POTENTIAL ISSUE: this line and DMA needs to be almost atomic
	alt_write_word((h2p_ctrl_out_addr), ctrl_out & ~(0x01 << FSM_START_ofst));

#ifdef GET_RAW_DATA
	// WARNING: PUT ATTENTION TO DMA DELAY IF both raw data and dconv data are processed at the same time
	// DMA should be started as fast as possible after FSM is started
	// process raw data
	if (store_to_sdram_noread)
	{ // do not write data to text with C programming: external mechanism should be implemented
	  // fifo_to_sdram_dma_trf (h2p_dma_addr, ADC_FIFO_MEM_OUT_BASE, SDRAM_BASE, acq_length/2);// start DMA process
		fifo_to_sdram_dma_trf (h2p_dma_addr, ADC_FIFO_MEM_OUT_BASE, SDRAM_BASE, acq_length);// start DMA process
		//while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) ); // might not be needed as the system will wait until data is available anyway
	}
	else
	{ // write data to text via c-programming
		if (rd_sdram_OR_n_rd_fifo)
		{ // if read with dma is intended.
		  // datawrite_with_dma(acq_length/2,DISABLE_MESSAGE);
			datawrite_with_dma(acq_length,DISABLE_MESSAGE);
			while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );// wait until fsm stops, just in case the DMA is too fast.
		}
		else
		{ // if read from fifo is intended
			while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );// wait until fsm stops
			usleep(300);
			unsigned int datacaptured = rd_FIFO (h2p_adc_fifo_status_addr, h2p_adc_fifo_addr, rddata);
			// if ((datacaptured<<1) != acq_length)
			if ((datacaptured) != acq_length)
			{
				// printf("[ERROR] number of data in the FIFO (%d) and data ordered (%d): NOT MATCHED\nData are flushed!\nReconfigure the FPGA immediately\n", datacaptured<<1, acq_length);
				printf("[ERROR] number of data in the FIFO (%d) and data ordered (%d): NOT MATCHED\nData are flushed!\nReconfigure the FPGA immediately\n", datacaptured, acq_length);
				return;
			}
			// buf32_to_buf16 (rddata, rddata_16, acq_length>>1 ); // transfer data from 32-bit buffer to 16-bit buffer
		}

		if (sav_indv_scan)
		{ // put the individual scan data into a file

			// save as binary file or ascii file (remember to change the corresponding Python code to process the right type of data
			sprintf(pathname,"%s/%s",foldername,filename);// create a filename
			wr_File (pathname, acq_length, rddata, SAV_BINARY);// write the data to the filename

			// save as ascii file
			// sprintf(pathname,"%s/%s_ascii",foldername,filename);// create a filename
			// wr_File (pathname, acq_length, (int*)rddata_16, SAV_ASCII);// write the data to the filename
			// printf("rddata_16 datatype above is changed to int*, check if this is a correct implementation!!!");
		}
	}
#endif

#ifdef GET_DCONV_DATA

	if (store_to_sdram_noread)
	{ // do not write data to text with C programming: external mechanism should be implemented
		fifo_to_sdram_dma_trf (h2p_dconvi_dma_addr, DCONV_FIFO_MEM_OUT_BASE, SDRAM_BASE, acq_length);// add data_len offset due to raw data before. (*4 factor is due to byte-addressing)
		//while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) ); // might not be needed as the system will wait until data is available anyway
	}

	else // process downconverted data
	{
		if (rd_sdram_OR_n_rd_fifo)
		{ // read from sdram
			data_dconv_write_with_dma (acq_length, DISABLE_MESSAGE);
		}
		else
		{ // read directly from fifo
			while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst) );// wait until the scan is done
			usleep(1);
			unsigned int datacaptured = rd_FIFO (h2p_dconvi_csr_addr, h2p_dconvi_addr, dconv);
			if (datacaptured != acq_length)
			{
				printf("[ERROR] number of data in the FIFO (%d) and data ordered (%d): NOT MATCHED\nData are flushed!\nReconfigure the FPGA immediately\n", datacaptured, acq_length);
				return;
			}
		}

		if (sav_indv_scan)
		{ // put the individual scan data into an individual file
			{ // put the individual scan data into a file
				sprintf(pathname,"%s/%s",foldername,filename);// create a filename
				wr_File (pathname, acq_length, dconv, SAV_BINARY);// write the data to the filename
			}
		}
	}
#endif

}

// duty cycle is not functioning anymore
void CPMG_Sequence(double cpmg_freq, double pulse1_us, double pulse2_us,
		double pulse1_dtcl, double pulse2_dtcl, double echo_spacing_us,
		long unsigned scan_spacing_us, unsigned int samples_per_echo,
		unsigned int echoes_per_scan, double init_adc_delay_compensation,
		uint32_t ph_cycl_en, char * filename, char * avgname,
		uint32_t enable_message)
{

	clock_t start, end;
	double net_acq_time, net_elapsed_time, net_scan_time;

	// measure the start time
	start = clock(); // measure time

	unsigned int cpmg_param[5];
	double adc_ltc1746_freq = cpmg_freq * 4;
	double nmr_fsm_clkfreq = cpmg_freq * 16;

	double init_delay_inherent = 2.25; // inherehent delay factor from the HDL structure, in ADC clock cycles
	double acq_window_safety_fact = (1 / cpmg_freq) * 10; // safety factor for acquisition window in clock cycles
	double rx_dly_us = 0; // set the rx delay to generate RX_EN or DUP_EN

	// set delay for the RX_EN or duplexer enable. Also serves as the Qswitch enable if available
	unsigned int rx_dly = (unsigned int) (lround(rx_dly_us * adc_ltc1746_freq));
	alt_write_word(h2p_rx_delay_addr, rx_dly);
	double rx_dly_us_achieved = (double) rx_dly / adc_ltc1746_freq;

	// printf("nilai = %d\n", samples_per_echo % dconv_fact);
	if (samples_per_echo % dconv_fact != 0)
	{
		printf("\t[ERROR] (samples_per_echo/dconv_fact) is not an integer!\n");
		if (samples_per_echo < dconv_fact)
		{
			printf("\t[ERROR] samples_per_echo is less than dconv_fact!\n");
		}
		return;
	}

	// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	// usleep(100);

	cpmg_param_calculator_ltc1746(cpmg_param, nmr_fsm_clkfreq, cpmg_freq,
			adc_ltc1746_freq, init_adc_delay_compensation, pulse1_us, pulse2_us,
			echo_spacing_us, samples_per_echo);

	alt_write_word((h2p_pulse1_addr), cpmg_param[PULSE1_OFFST]);
	alt_write_word((h2p_delay1_addr), cpmg_param[DELAY1_OFFST]);
	alt_write_word((h2p_pulse2_addr), cpmg_param[PULSE2_OFFST]);
	alt_write_word((h2p_delay2_addr), cpmg_param[DELAY2_OFFST]);
	alt_write_word((h2p_init_adc_delay_addr), cpmg_param[INIT_DELAY_ADC_OFFST]);
	alt_write_word((h2p_echo_per_scan_addr), echoes_per_scan);
	alt_write_word((h2p_adc_samples_per_echo_addr), samples_per_echo);

	// this is added because of the delay added in the state machine is minimum 2.25 ADC clock cycles for init_delay of 2 or less, and inherent 0.25 clock cycles for anything more than 2
	if (cpmg_param[INIT_DELAY_ADC_OFFST] <= 2)
	{
		init_delay_inherent = 2.25 - cpmg_param[INIT_DELAY_ADC_OFFST];
	}
	else
	{
		init_delay_inherent = 0.25;
	}

	if (enable_message)
	{
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tPulse 1\t\t\t: %7.3f us (%d)\n",
				(double) cpmg_param[PULSE1_OFFST] / nmr_fsm_clkfreq,
				cpmg_param[PULSE1_OFFST]);
		printf("\tDelay 1\t\t\t: %7.3f us (%d)\n",
				(double) cpmg_param[DELAY1_OFFST] / nmr_fsm_clkfreq,
				cpmg_param[DELAY1_OFFST]);
		printf("\tPulse 2\t\t\t: %7.3f us (%d)\n",
				(double) cpmg_param[PULSE2_OFFST] / nmr_fsm_clkfreq,
				cpmg_param[PULSE2_OFFST]);
		printf("\tDelay 2\t\t\t: %7.3f us (%d)\n",
				(double) cpmg_param[DELAY2_OFFST] / nmr_fsm_clkfreq,
				cpmg_param[DELAY2_OFFST]);
		printf("\tADC init delay\t: %7.3f us (%d) -not-precise\n",
				((double) cpmg_param[INIT_DELAY_ADC_OFFST] + init_delay_inherent)
						/ adc_ltc1746_freq, cpmg_param[INIT_DELAY_ADC_OFFST]); // not precise due to the clock uncertainties between the main clock and ADC clock
		printf("\tADC acq window\t: %7.3f us (%d)\n",
				((double) samples_per_echo) / adc_ltc1746_freq,
				samples_per_echo);
		printf("\tRX_EN or DUP_EN delay\t: %7.3f us (%d)\n", rx_dly_us_achieved,
				rx_dly);
	}
	if (cpmg_param[INIT_DELAY_ADC_OFFST] < 2)
	{
		printf("\t[WARNING] Computed ADC_init_delay < 2 clks\n");
		printf("\t[WARNING] ADC_init_delay is forced to 2 clks in FPGA HDL!\n");
	}
	if (((double) samples_per_echo) / adc_ltc1746_freq
			> (echo_spacing_us - pulse2_us))
	{
		printf("\t[ERROR] acq.window (%.1fus) >> tE-p180 (%.1fus).\n",
				(((double) samples_per_echo) / adc_ltc1746_freq),
				echo_spacing_us - pulse2_us);
		printf("\t[ERROR] Increase tE or reduce SpE or reduce p180.\n");
		return;
	}
	double excess_acq = (echo_spacing_us
			- ((double) samples_per_echo) / adc_ltc1746_freq) / 2
			- init_adc_delay_compensation - acq_window_safety_fact
			- rx_dly_us_achieved;
	if (excess_acq < 0)
	{
		printf("\t[ERROR] (acq.window) exceeds (delay180.window) by %.1fus.\n",
				-excess_acq);
		printf(
				"\t[ERROR] Increase tE or reduce SpE or reduce p180 or adjust echo_shift or carefully adjust rx_delay\n");
		return;
	}

#ifdef GET_RAW_DATA
	runFSM(nmr_fsm_clkfreq, ph_cycl_en, dsize,
			filename, NO_SAV_INDV_SCAN, RD_DATA_VIA_SDRAM_OR_FIFO, RD_SDRAM);
#endif
#ifdef GET_DCONV_DATA
	runFSM(nmr_fsm_clkfreq, ph_cycl_en, dconv_size,
			filename, NO_SAV_INDV_SCAN, RD_DATA_VIA_SDRAM_OR_FIFO, RD_SDRAM);
#endif

	// measure elapsed time after acquisition
	end = clock(); // measure time
	net_acq_time = ((double) (end - start)) * 1000000 / CLOCKS_PER_SEC; // measure time in us
	if (enable_message)
	{
		printf("\t Elapsed time after data acquisition is %ld us.\n",
				(unsigned long) net_acq_time);
	}

	// add delay according to the given scan_spacing_us
	end = clock(); // measure time
	net_elapsed_time = ((double) (end - start)) * 1000000 / CLOCKS_PER_SEC; // measure time in us
	if ((unsigned long) net_elapsed_time < scan_spacing_us)
	{
		while ((unsigned long) net_elapsed_time < scan_spacing_us)
		{
			end = clock(); // measure time
			net_elapsed_time = ((double) (end - start)) * 1000000
					/ CLOCKS_PER_SEC; // measure time in us
		}

		if (enable_message)
		{
			printf(
					"\t Added %0.0f us at the end of the scan to account for scan_spacing_us.\n",
					net_elapsed_time - net_acq_time);
		}
	}

	else // scan duration is already longer than the scan_spacing_us parameter
	{
		printf(
				"\t[WARNING] One scan duration is longer than scan_spacing_us parameter (%ld us) and is measured to be approx. %ld us\n",
				scan_spacing_us, (unsigned long) net_acq_time);
	}

	// Check and wait until the FSM stop running
	if (alt_read_word(h2p_ctrl_in_addr) & (0x01 << NMR_SEQ_run_ofst))
	{

		while (alt_read_word(h2p_ctrl_in_addr) & (0x01 << NMR_SEQ_run_ofst))
			;

		// measure time after waiting for the FSM to be done
		end = clock(); // measure time
		net_scan_time = (double) (end - start) * 1000000 / CLOCKS_PER_SEC; // measure time in us
		printf(
				"\t[WARNING] Added additional delay as FSM was still running after scan_spacing_us time had elapsed. Net scan duration is approx. %ld us\n",
				(unsigned long) net_scan_time);

	}

}

void CPMG_iterate(double cpmg_freq, double pulse1_us, double pulse2_us,
		double pulse1_dtcl, double pulse2_dtcl, double echo_spacing_us,
		long unsigned scan_spacing_us, unsigned int samples_per_echo,
		unsigned int echoes_per_scan, double init_adc_delay_compensation,
		unsigned int number_of_iteration, uint32_t ph_cycl_en)
{

// settings
	char progress_verbose = 1; // print progress
	char binary_OR_ascii = 1; // save binary output into the text file (1). Otherwise, it'll be ASCII output (0)

	double nmr_fsm_clkfreq = 16 * cpmg_freq;
	double adc_ltc1746_freq = 4 * cpmg_freq;

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	create_measurement_folder("cpmg");
// printf("Approximated measurement time : %.2f mins\n",( scan_spacing_us*(double)number_of_iteration) *1e-6/60);

	unsigned int cpmg_param[5];
	cpmg_param_calculator_ltc1746(cpmg_param, nmr_fsm_clkfreq, cpmg_freq,
			adc_ltc1746_freq, init_adc_delay_compensation, pulse1_us, pulse2_us,
			echo_spacing_us, samples_per_echo);
// print general measurement settings
	sprintf(pathname, "%s/acqu.par", foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr, "b1Freq = %4.3f\n", cpmg_freq);
	fprintf(fptr, "p90LengthGiven = %4.3f\n", pulse1_us);
	fprintf(fptr, "p90LengthRun = %4.3f\n",
			(double) cpmg_param[PULSE1_OFFST] / nmr_fsm_clkfreq);
	fprintf(fptr, "p90LengthCnt = %d @ %4.3f MHz\n", cpmg_param[PULSE1_OFFST],
			nmr_fsm_clkfreq);
	fprintf(fptr, "d90LengthRun = %4.3f\n",
			(double) cpmg_param[DELAY1_OFFST] / nmr_fsm_clkfreq);
	fprintf(fptr, "d90LengthCnt = %d @ %4.3f MHz\n", cpmg_param[DELAY1_OFFST],
			nmr_fsm_clkfreq);
	fprintf(fptr, "p180LengthGiven = %4.3f\n", pulse2_us);
	fprintf(fptr, "p180LengthRun = %4.3f\n",
			(double) cpmg_param[PULSE2_OFFST] / nmr_fsm_clkfreq);
	fprintf(fptr, "p180LengthCnt =  %d @ %4.3f MHz\n", cpmg_param[PULSE2_OFFST],
			nmr_fsm_clkfreq);
	fprintf(fptr, "d180LengthRun = %4.3f\n",
			(double) cpmg_param[DELAY2_OFFST] / nmr_fsm_clkfreq);
	fprintf(fptr, "d180LengthCnt = %d @ %4.3f MHz\n", cpmg_param[DELAY2_OFFST],
			nmr_fsm_clkfreq);
//fprintf(fptr,"p90_dtcl = %4.3f\n", pulse1_dtcl);
//fprintf(fptr,"p180_dtcl = %4.3f\n", pulse2_dtcl);
	fprintf(fptr, "echoTimeRun = %4.3f\n",
			(double) (cpmg_param[PULSE2_OFFST] + cpmg_param[DELAY2_OFFST])
					/ nmr_fsm_clkfreq);
	fprintf(fptr, "echoTimeGiven = %4.3f\n", echo_spacing_us);
	fprintf(fptr, "ieTime = %lu\n", scan_spacing_us / 1000);
	fprintf(fptr, "nrPnts = %d\n", samples_per_echo);
	fprintf(fptr, "nrEchoes = %d\n", echoes_per_scan);
	fprintf(fptr, "echoShift = %4.3f\n", init_adc_delay_compensation);
	fprintf(fptr, "nrIterations = %d\n", number_of_iteration);
	fprintf(fptr, "dummyEchoes = 0\n");
	fprintf(fptr, "adcFreq = %4.3f\n", adc_ltc1746_freq);
	fprintf(fptr, "usePhaseCycle = %d\n", ph_cycl_en);
	fprintf(fptr, "echoSkipHw = %d\n", echo_skip_hw);
#ifdef GET_RAW_DATA
	fprintf(fptr, "dwellTime = %4.3f\n", 1 / adc_ltc1746_freq);
	fprintf(fptr, "fpgaDconv = 0\n");
	fprintf(fptr,"dconvFact = 1\n");
#endif
#ifdef GET_DCONV_DATA
	fprintf(fptr, "dwellTime = %4.3f\n", 1 / adc_ltc1746_freq*dconv_fact);
	fprintf(fptr, "fpgaDconv = 1\n");
	fprintf(fptr,"dconvFact = %d\n", dconv_fact);
#endif
	fclose (fptr);

// print matlab script to analyze datas
	sprintf(pathname, "measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr, "compute_iterate([data_folder,'%s']);\n", foldername);
	fclose(fptr);

// print matlab script to analyze datas
	sprintf(pathname, "current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr, "%s\n", foldername);
	fclose(fptr);

	int iterate = 1;

	int FILENAME_LENGTH = 100;
	char *name;
	name = (char*) malloc(FILENAME_LENGTH * sizeof(char));
	char *nameavg;
	nameavg = (char*) malloc(FILENAME_LENGTH * sizeof(char));

// amplitude sum
#ifdef GET_RAW_DATA
	float Asum[dsize];
	for (i=0; i<dsize; i++)
	{
		Asum[i] = 0;
	}
#endif

#ifdef GET_DCONV_DATA
// downconverted sum
// printf ("dconv_size = %d\n", dconv_size); // print the buffer size
	float dconv_sum[dconv_size];
	for (i=0; i < dconv_size; i++) dconv_sum[i] = 0;
#endif

	if (progress_verbose)
	{
		printf("\tPROGRESS: \n");
	}

	for (iterate = 1; iterate <= number_of_iteration; iterate++)
	{
		// printf("\n*** RUN %d ***\n",iterate);
		if (progress_verbose)
			print_progress(iterate, number_of_iteration);

		snprintf(name, FILENAME_LENGTH, "dat_%03d", iterate);
		snprintf(nameavg, FILENAME_LENGTH, "avg_%03d", iterate);

		CPMG_Sequence(cpmg_freq, //cpmg_freq
				pulse1_us, //pulse1_us
				pulse2_us, //pulse2_us
				pulse1_dtcl, //pulse1_dtcl
				pulse2_dtcl, //pulse2_dtcl
				echo_spacing_us, //echo_spacing_us
				scan_spacing_us, //scan_spacing_us
				samples_per_echo, //samples_per_echo
				echoes_per_scan, //echoes_per_scan
				init_adc_delay_compensation, //compensation delay number (counted by the adc base clock)
				ph_cycl_en, //phase cycle enable/disable
				name, //filename for data
				nameavg, //filename for average data
				DISABLE_MESSAGE);

#ifdef GET_RAW_DATA
		// process the data
		if (ph_cycl_en)
		{
			if (iterate % 2 == 0)
			{
				for (i = 0; i < dsize; i++)
				// Asum[i] -= (float)rddata_16[i]/(float)number_of_iteration;
				Asum[i] -= (float)rddata[i]/(float)number_of_iteration;
			}
			else
			{
				for (i = 0; i < dsize; i++)
				// Asum[i] += (float)rddata_16[i]/(float)number_of_iteration;
				Asum[i] += (float)rddata[i]/(float)number_of_iteration;
			}
		}
		else
		{
			for (i = 0; i < dsize; i++)
			// Asum[i] += (float)rddata_16[i]/(float)number_of_iteration;
			Asum[i] += (float)rddata[i]/(float)number_of_iteration;
		}

#endif

#ifdef GET_DCONV_DATA
		if (ph_cycl_en)
		{
			if (iterate % 2 == 0)
			{
				for (i = 0; i < dconv_size; i++)
				dconv_sum[i] -= (float)dconv[i] / (float)number_of_iteration;
			}
			else
			{
				for (i = 0; i < dconv_size; i++)
				dconv_sum[i] += (float)dconv[i] / (float)number_of_iteration;
			}
		}
		else
		{
			for (i = 0; i < dconv_size; i++)
			dconv_sum[i] += (float)dconv[i] / (float)number_of_iteration;
		}
#endif
	}

#ifdef GET_RAW_DATA
// write raw data sum
	sprintf(pathname, "%s/%s", foldername, "asum");// put the data into the data folder
	fptr = fopen(pathname, "w");
	if (binary_OR_ascii)
	{ // binary output
		fwrite(&Asum, sizeof(float), dsize, fptr);
	}
	else
	{ // ascii output
		for (i = 0; i < dsize; i++)
		fprintf(fptr, "%d\n", (int)Asum[i]);
	}
	fclose(fptr);
#endif

#ifdef GET_DCONV_DATA
// write downconverted data sum in-phase
	sprintf(pathname, "%s/%s", foldername, "dconv");// put the data into the data folder
	fptr = fopen(pathname, "w");
	if (binary_OR_ascii)
	{ // binary output
		fwrite(&dconv_sum, sizeof(float), dconv_size, fptr);
	}
	else
	{ // ascii output
		for (i = 0; i < dconv_size; i++) fprintf(fptr, "%d\n", (int)dconv_sum[i]);
	}
	fclose(fptr);
#endif

	if (progress_verbose)
	{
		printf("\t done!\n");
	}

	free(name);
	free(nameavg);

}

void FID(double cpmg_freq, double pulse2_us, double pulse2_dtcl,
		long unsigned scan_spacing_us, unsigned int samples_per_echo,
		char * filename, uint32_t enable_message)
{
	double adc_ltc1746_freq = cpmg_freq * 4;
	double nmr_fsm_clkfreq = cpmg_freq * 16;
	uint8_t ph_cycl_en = 0;

	usleep(scan_spacing_us);

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	unsigned int pulse2_int =
			(unsigned int) (round(pulse2_us * nmr_fsm_clkfreq)); // the number of 180 deg pulse in the multiplication of cpmg pulse period (discrete value, no continuous number supported)
	unsigned int delay2_int = (unsigned int) (round(
			samples_per_echo * (nmr_fsm_clkfreq / adc_ltc1746_freq) * 10)); // the number of delay after 180 deg pulse. It is simply samples_per_echo multiplied by (nmr_fsm_clkfreq/adc_ltc1746_freq) factor, as the delay2_int is counted by nmr_fsm_clkfreq, not by adc_ltc1746_freq. It is also multiplied by a constant 10 as safety factor to make sure the ADC acquisition is inside FSMSTAT (refer to HDL) 'on' window.
	unsigned int fixed_init_adc_delay = 3; // set to the minimum delay values, which is 3 (limited by HDL structure).
	unsigned int fixed_echo_per_scan = 1; // it must be 1, otherwise the HDL will go to undefined state.
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2)
	{
		init_delay_inherent = 2.25;
	}
	else
	{ // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	alt_write_word((h2p_pulse1_addr), 0);
	alt_write_word((h2p_delay1_addr), 0);
	alt_write_word((h2p_pulse2_addr), pulse2_int);
	alt_write_word((h2p_delay2_addr), delay2_int);
	alt_write_word((h2p_init_adc_delay_addr), fixed_init_adc_delay);
	alt_write_word((h2p_echo_per_scan_addr), fixed_echo_per_scan);
	alt_write_word((h2p_adc_samples_per_echo_addr), samples_per_echo);

	if (enable_message)
	{
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tPulse 2\t\t\t: %7.3f us (%d)\n",
				(double) pulse2_int / nmr_fsm_clkfreq, pulse2_int);
		printf("\tDelay 2\t\t\t: %7.3f us (%d)\n",
				(double) delay2_int / nmr_fsm_clkfreq, delay2_int);
		printf("\tADC init delay\t: %7.3f us (%d) --imprecise\n",
				init_delay_inherent / adc_ltc1746_freq, fixed_init_adc_delay);
		printf("\tADC acq window\t: %7.3f us (%d)\n",
				((double) samples_per_echo) / adc_ltc1746_freq,
				samples_per_echo);
	}
	if (fixed_init_adc_delay < 2)
	{
		printf(
				"\tWARNING: Computed ADC_init_delay is less than 2, ADC_init_delay is force driven to 2 inside the HDL!");
	}

	runFSM(nmr_fsm_clkfreq, ph_cycl_en, samples_per_echo, filename,
			NO_SAV_INDV_SCAN, RD_DATA_VIA_SDRAM_OR_FIFO, RD_SDRAM);

}

void FID_iterate(double cpmg_freq, double pulse2_us, double pulse2_dtcl,
		long unsigned scan_spacing_us, unsigned int samples_per_echo,
		unsigned int number_of_iteration, uint32_t enable_message)
{
	double nmr_fsm_clkfreq = 16 * cpmg_freq;
	double adc_ltc1746_freq = 4 * cpmg_freq;

	unsigned int fixed_init_adc_delay = 2; // set to the minimum delay values, which is 2 (limited by HDL structure).
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2)
	{
		init_delay_inherent = 2.25;
	}
	else
	{ // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	double init_adc_delay_compensation = init_delay_inherent / adc_ltc1746_freq;
	unsigned int pulse2_int =
			(unsigned int) (round(pulse2_us * nmr_fsm_clkfreq)); // the number of 180 deg pulse in the multiplication of cpmg pulse period (discrete value, no continuous number supported)
	unsigned int delay2_int = (unsigned int) (samples_per_echo
			* (nmr_fsm_clkfreq / adc_ltc1746_freq) * 10); // the number of delay after 180 deg pulse. It is simply samples_per_echo multiplied by (nmr_fsm_clkfreq/adc_ltc1746_freq) factor, as the delay2_int is counted by nmr_fsm_clkfreq, not by adc_ltc1746_freq. It is also multiplied by a constant 2 as safety factor to make sure the ADC acquisition is inside FSMSTAT (refer to HDL) 'on' window.

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	create_measurement_folder("fid");
// printf("Approximated measurement time : %.2f mins\n",( scan_spacing_us*(double)number_of_iteration)*1e-6/60);

// print general measurement settings
	sprintf(pathname, "%s/acqu.par", foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr, "b1Freq = %4.3f\n", cpmg_freq);
	fprintf(fptr, "p180LengthGiven = %4.3f\n", pulse2_us);
	fprintf(fptr, "p180LengthRun = %4.3f\n",
			(double) pulse2_int / nmr_fsm_clkfreq);
	fprintf(fptr, "p180LengthCnt =  %d @ %4.3f MHz\n", pulse2_int,
			nmr_fsm_clkfreq);
	fprintf(fptr, "d180LengthRun = %4.3f\n",
			(double) delay2_int / nmr_fsm_clkfreq);
	fprintf(fptr, "d180LengthCnt = %d @ %4.3f MHz\n", delay2_int,
			nmr_fsm_clkfreq);
//fprintf(fptr,"p90_dtcl = %4.3f\n", pulse1_dtcl);
//fprintf(fptr,"p180_dtcl = %4.3f\n", pulse2_dtcl);
	fprintf(fptr, "ieTime = %lu\n", scan_spacing_us / 1000);
	fprintf(fptr, "nrPnts = %d\n", samples_per_echo);
	fprintf(fptr, "echoShift = %4.3f --imprecise\n",
			init_adc_delay_compensation);
	fprintf(fptr, "nrIterations = %d\n", number_of_iteration);
	fprintf(fptr, "dummyEchoes = 0\n");
	fprintf(fptr, "adcFreq = %4.3f\n", adc_ltc1746_freq);
	fprintf(fptr, "dwellTime = %4.3f\n", 1 / adc_ltc1746_freq);
	fclose (fptr);

// print matlab script to analyze datas
	sprintf(pathname, "measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr, "fid_iterate([data_folder,'%s']);\n", foldername);
	fclose(fptr);

// print matlab script to analyze datas
	sprintf(pathname, "current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr, "%s\n", foldername);
	fclose(fptr);

	int FILENAME_LENGTH = 100;
	char *name;
	name = (char*) malloc(FILENAME_LENGTH * sizeof(char));

// initialize sum data
	int Asum[samples_per_echo];
	for (i = 0; i < samples_per_echo; i++)
		Asum[i] = 0;

	int iterate = 1;
	for (iterate = 1; iterate <= number_of_iteration; iterate++)
	{
		// printf("\n*** RUN %d ***\n",iterate);
		print_progress(iterate, number_of_iteration);

		snprintf(name, FILENAME_LENGTH, "dat_%03d", iterate);

		FID(cpmg_freq, //cpmg_freq
				pulse2_us, //pulse2_us
				pulse2_dtcl, //pulse2_dtcl
				scan_spacing_us, //scan_spacing_us
				samples_per_echo, //samples_per_echo
				name, //filename for data
				enable_message);

#ifdef GET_RAW_DATA
		for (i = 0; i < samples_per_echo; i++)
		// Asum[i] += rddata_16[i];
		Asum[i] += rddata[i];
#endif

	}

// write raw data sum
	sprintf(pathname, "%s/%s", foldername, "asum"); // put the data into the data folder
	fptr = fopen(pathname, "w");
	for (i = 0; i < samples_per_echo; i++)
		fprintf(fptr, "%d\n", Asum[i]);
	fclose(fptr);

	free(name);

}

void noise(double cpmg_freq, long unsigned scan_spacing_us,
		unsigned int samples_per_echo, char * filename, uint32_t enable_message)
{
	double adc_ltc1746_freq = cpmg_freq * 4;
	double nmr_fsm_clkfreq = cpmg_freq * 16;
	uint8_t ph_cycl_en = 0;

	usleep(scan_spacing_us);

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	unsigned int delay2_int = (unsigned int) (round(
			samples_per_echo * (nmr_fsm_clkfreq / adc_ltc1746_freq) * 10));
	unsigned int fixed_init_adc_delay = (delay2_int >> 4); // set to the minimum delay values, which is 3 (limited by HDL structure). Or set to the middle of the acquisition to avoid additional noise coming due to switching control signal at the beginning. It is simply (delay2_int/4) to get 1/4 length of the window and another 1/4 because adc_clock is 1/4 the control clock.
	unsigned int fixed_echo_per_scan = 1; // it must be 1, otherwise the HDL will go to undefined state.
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2)
	{
		init_delay_inherent = 2.25;
	}
	else
	{ // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	alt_write_word((h2p_pulse1_addr), 0);
	alt_write_word((h2p_delay1_addr), 0);
	alt_write_word((h2p_pulse2_addr), 0);
	alt_write_word((h2p_delay2_addr), delay2_int);
	alt_write_word((h2p_init_adc_delay_addr), fixed_init_adc_delay);
	alt_write_word((h2p_echo_per_scan_addr), fixed_echo_per_scan);
	alt_write_word((h2p_adc_samples_per_echo_addr), samples_per_echo);

	if (enable_message)
	{
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tDelay 2\t\t\t: %7.3f us (%d)\n",
				(double) delay2_int / nmr_fsm_clkfreq, delay2_int);
		printf("\tADC init delay\t: %7.3f us (%d) --imprecise\n",
				init_delay_inherent / adc_ltc1746_freq, fixed_init_adc_delay);
		printf("\tADC acq window\t: %7.3f us (%d)\n",
				((double) samples_per_echo) / adc_ltc1746_freq,
				samples_per_echo);
	}
	if (fixed_init_adc_delay < 2)
	{
		printf(
				"\tWARNING: Computed ADC_init_delay is less than 2, ADC_init_delay is force driven to 2 inside the HDL!");
	}

	runFSM(nmr_fsm_clkfreq, ph_cycl_en, samples_per_echo, filename,
			NO_SAV_INDV_SCAN, RD_DATA_VIA_SDRAM_OR_FIFO, RD_SDRAM);

}

void noise_iterate(double cpmg_freq, long unsigned scan_spacing_us,
		unsigned int samples_per_echo, unsigned int number_of_iteration,
		uint32_t enable_message)
{
	double nmr_fsm_clkfreq = 16 * cpmg_freq;
	double adc_ltc1746_freq = 4 * cpmg_freq;

	unsigned int fixed_init_adc_delay = 2; // set to the minimum delay values, which is 2 (limited by HDL structure).
	double init_delay_inherent; // inherehent delay factor from the HDL structure. The minimum is 2.25 no matter how small the delay is set. Look ERRATA
	if (fixed_init_adc_delay <= 2)
	{
		init_delay_inherent = 2.25;
	}
	else
	{ // if fixed_init_adc_delay is more than 2
		init_delay_inherent = (double) fixed_init_adc_delay + 0.25; // look at ERRATA from the HDL to get 0.25
	}

	double init_adc_delay_compensation = init_delay_inherent / adc_ltc1746_freq;
	unsigned int delay2_int = (unsigned int) (samples_per_echo
			* (nmr_fsm_clkfreq / adc_ltc1746_freq) * 10); // the number of delay after 180 deg pulse. It is simply samples_per_echo multiplied by (nmr_fsm_clkfreq/adc_ltc1746_freq) factor, as the delay2_int is counted by nmr_fsm_clkfreq, not by adc_ltc1746_freq. It is also multiplied by a constant 2 as safety factor to make sure the ADC acquisition is inside FSMSTAT (refer to HDL) 'on' window.

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

	create_measurement_folder("noise");
// printf("Approximated measurement time : %.2f mins\n",( scan_spacing_us*(double)number_of_iteration)*1e-6/60);

// print general measurement settings
	sprintf(pathname, "%s/acqu.par", foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr, "b1Freq = %4.3f\n", cpmg_freq);
	fprintf(fptr, "d180LengthRun = %4.3f\n",
			(double) delay2_int / nmr_fsm_clkfreq);
	fprintf(fptr, "d180LengthCnt = %d @ %4.3f MHz\n", delay2_int,
			nmr_fsm_clkfreq);
	fprintf(fptr, "ieTime = %lu\n", scan_spacing_us / 1000);
	fprintf(fptr, "nrPnts = %d\n", samples_per_echo);
	fprintf(fptr, "echoShift = %4.3f --imprecise\n",
			init_adc_delay_compensation);
	fprintf(fptr, "nrIterations = %d\n", number_of_iteration);
	fprintf(fptr, "dummyEchoes = 0\n");
	fprintf(fptr, "adcFreq = %4.3f\n", adc_ltc1746_freq);
	fprintf(fptr, "dwellTime = %4.3f\n", 1 / adc_ltc1746_freq);
	fclose (fptr);

// print matlab script to analyze datas
	sprintf(pathname, "measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr, "fid_iterate([data_folder,'%s']);\n", foldername);
	fclose(fptr);

// print matlab script to analyze datas
	sprintf(pathname, "current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr, "%s\n", foldername);
	fclose(fptr);

	int FILENAME_LENGTH = 100;
	char *name;
	name = (char*) malloc(FILENAME_LENGTH * sizeof(char));

// initialize sum data
	int Asum[samples_per_echo];
	for (i = 0; i < samples_per_echo; i++)
		Asum[i] = 0;

	int iterate = 1;
	for (iterate = 1; iterate <= number_of_iteration; iterate++)
	{
		// printf("\n*** RUN %d ***\n",iterate);
		print_progress(iterate, number_of_iteration);

		snprintf(name, FILENAME_LENGTH, "dat_%03d", iterate);

		noise(cpmg_freq, //cpmg_freq
				scan_spacing_us, //scan_spacing_us
				samples_per_echo, //samples_per_echo
				name, //filename for data
				enable_message);

#ifdef GET_RAW_DATA
		for (i = 0; i < samples_per_echo; i++)
		// Asum[i] += rddata_16[i];
		Asum[i] += rddata[i];
#endif
	}

// write raw data sum
	sprintf(pathname, "%s/%s", foldername, "asum"); // put the data into the data folder
	fptr = fopen(pathname, "w");
	for (i = 0; i < samples_per_echo; i++)
		fprintf(fptr, "%d\n", Asum[i]);
	fclose(fptr);

	free(name);

}

void tx_sampling_async(double tx_freq, double samp_freq,
		unsigned int tx_num_of_samples, char * filename)
{

	uint8_t ph_cycl_en = 0;

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

// KEEP THIS CODE AND ENABLE IT IF YOU USE C-ONLY, OPPOSED TO USING PYTHON
// activate signal_coup path (from the directional coupler) for the receiver
// write_i2c_int_cnt (DISABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
// write_i2c_int_cnt (ENABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

// set parameters for acquisition (using CPMG registers and CPMG sequence: not a good practice)
	alt_write_word((h2p_pulse1_addr), 0); // random safe number
	alt_write_word((h2p_delay1_addr), 0); // random safe number
	alt_write_word((h2p_pulse2_addr), 0); // random safe number
	alt_write_word((h2p_delay2_addr), tx_num_of_samples * 4 * 2); // *4 is because the system clock is 4*ADC clock. *2 factor is to increase the delay_window to about 2*acquisition window for safety.
	alt_write_word((h2p_init_adc_delay_addr),
			(unsigned int) (tx_num_of_samples / 2)); // put adc acquisition window exactly at the middle of the delay window
	alt_write_word((h2p_echo_per_scan_addr), 1);
	alt_write_word((h2p_adc_samples_per_echo_addr), tx_num_of_samples);

// set pll for the tx sampling
	Set_PLL(h2p_analyzer_pll_addr, 0, tx_freq, 0.5, DISABLE_MESSAGE);
	Set_PLL(h2p_analyzer_pll_addr, 1, tx_freq, 0.5, DISABLE_MESSAGE);
	Set_PLL(h2p_analyzer_pll_addr, 2, tx_freq, 0.5, DISABLE_MESSAGE);
	Set_PLL(h2p_analyzer_pll_addr, 3, tx_freq, 0.5, DISABLE_MESSAGE);
	Reset_PLL(h2p_ctrl_out_addr, PLL_ANALYZER_RST_ofst, ctrl_out);
	Wait_PLL_To_Lock(h2p_ctrl_in_addr, PLL_ANALYZER_lock_ofst);
	Set_DPS(h2p_analyzer_pll_addr, 0, 0, DISABLE_MESSAGE);
	Set_DPS(h2p_analyzer_pll_addr, 1, 90, DISABLE_MESSAGE);
	Set_DPS(h2p_analyzer_pll_addr, 2, 180, DISABLE_MESSAGE);
	Set_DPS(h2p_analyzer_pll_addr, 3, 270, DISABLE_MESSAGE);
	Wait_PLL_To_Lock(h2p_ctrl_in_addr, PLL_ANALYZER_lock_ofst);

// enable PLL_analyzer path, disable RF gate path
	ctrl_out &= ~(NMR_CLK_GATE_AVLN);
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);

// enable transmit on acquisition
	ctrl_out |= PULSE_ON_RX;
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

	runFSM(samp_freq * 4, ph_cycl_en, tx_num_of_samples, filename,
			SAV_INDV_SCAN, RD_DATA_VIA_SDRAM_OR_FIFO, RD_FIFO);

// disable PLL_analyzer path and enable the default RF gate path
	ctrl_out |= NMR_CLK_GATE_AVLN;
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);

// disable transmit on acquisition
	ctrl_out &= ~(PULSE_ON_RX);
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

// KEEP THIS CODE AND ENABLE IT IF YOU USE C-ONLY, OPPOSED TO USING PYTHON
// activate normal signal path for the receiver
// write_i2c_int_cnt (ENABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
// write_i2c_int_cnt (DISABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

}

void tx_sampling_sync(double tx_freq, unsigned int tx_num_of_samples,
		char * filename)
{

	uint8_t ph_cycl_en = 0;

// read the current ctrl_out
	ctrl_out = alt_read_word(h2p_ctrl_out_addr);

// KEEP THIS CODE AND ENABLE IT IF YOU USE C-ONLY, OPPOSED TO USING PYTHON
// activate signal_coup path (from the directional coupler) for the receiver
// write_i2c_int_cnt (DISABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
// write_i2c_int_cnt (ENABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

// set parameters for acquisition (using CPMG registers and CPMG sequence: not a good practice)
	alt_write_word((h2p_pulse1_addr), 0); // random safe number
	alt_write_word((h2p_delay1_addr), 0); // random safe number
	alt_write_word((h2p_pulse2_addr), 0); // random safe number
	alt_write_word((h2p_delay2_addr), tx_num_of_samples * 4 * 2); // *4 is because the system clock is 4*ADC clock. *2 factor is to increase the delay_window to about 2*acquisition window for safety.
	alt_write_word((h2p_init_adc_delay_addr),
			(unsigned int) (tx_num_of_samples / 2)); // put adc acquisition window exactly at the middle of the delay window
	alt_write_word((h2p_echo_per_scan_addr), 1);
	alt_write_word((h2p_adc_samples_per_echo_addr), tx_num_of_samples);

// set pll for the tx sampling
	//Set_PLL(h2p_analyzer_pll_addr, 0, tx_freq, 0.5, DISABLE_MESSAGE);
	//Set_PLL(h2p_analyzer_pll_addr, 1, tx_freq, 0.5, DISABLE_MESSAGE);
	//Set_PLL(h2p_analyzer_pll_addr, 2, tx_freq, 0.5, DISABLE_MESSAGE);
	//Set_PLL(h2p_analyzer_pll_addr, 3, tx_freq, 0.5, DISABLE_MESSAGE);
	//Reset_PLL(h2p_ctrl_out_addr, PLL_ANALYZER_RST_ofst, ctrl_out);
	//Wait_PLL_To_Lock(h2p_ctrl_in_addr, PLL_ANALYZER_lock_ofst);
	//Set_DPS(h2p_analyzer_pll_addr, 0, 0, DISABLE_MESSAGE);
	//Set_DPS(h2p_analyzer_pll_addr, 1, 90, DISABLE_MESSAGE);
	//Set_DPS(h2p_analyzer_pll_addr, 2, 180, DISABLE_MESSAGE);
	//Set_DPS(h2p_analyzer_pll_addr, 3, 270, DISABLE_MESSAGE);
	//Wait_PLL_To_Lock(h2p_ctrl_in_addr, PLL_ANALYZER_lock_ofst);

// enable PLL_analyzer path, disable RF gate path
	// ctrl_out &= ~(NMR_CLK_GATE_AVLN);
	// alt_write_word((h2p_ctrl_out_addr), ctrl_out);

	// disable PLL_analyzer path and enable the default RF gate path
	ctrl_out |= NMR_CLK_GATE_AVLN;
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);

// enable transmit on acquisition
	ctrl_out |= PULSE_ON_RX;
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

	runFSM(tx_freq * 16, ph_cycl_en, tx_num_of_samples, filename, SAV_INDV_SCAN,
			RD_DATA_VIA_SDRAM_OR_FIFO, RD_FIFO);

// disable PLL_analyzer path and enable the default RF gate path
	// ctrl_out |= NMR_CLK_GATE_AVLN;
	// alt_write_word((h2p_ctrl_out_addr), ctrl_out);

// disable transmit on acquisition
	ctrl_out &= ~(PULSE_ON_RX);
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(1);

// KEEP THIS CODE AND ENABLE IT IF YOU USE C-ONLY, OPPOSED TO USING PYTHON
// activate normal signal path for the receiver
// write_i2c_int_cnt (ENABLE, RX_IN_SEL_1_msk, DISABLE_MESSAGE);
// write_i2c_int_cnt (DISABLE, RX_IN_SEL_2_msk, DISABLE_MESSAGE);

}

void tune_board(double freq)
{
//double c_idx;
//c_idx = (freq-mtch_ntwrk_freq_sta)/mtch_ntwrk_freq_spa;	// find index for C
// round c_idx (integer conversion always floors the number (even for 0.99999), which causes trouble, so compensate for it)
//if ( c_idx - (double)((uint16_t)c_idx) > 0.5 ) {
//	c_idx += 0.5;
//}
// write_relay_cnt(cpar_tbl[(uint16_t)c_idx],cser_tbl[(uint16_t)c_idx]);	// find c values from table

// function uses obsolete ad5722r from old board
// double vvarac_idx;
// vvarac_idx = (freq-vvarac_freq_sta)/vvarac_freq_spa;	// find index for the vvarac
// round vvarac (integer conversion always floors the number (even for 0.99999), which causes trouble, so compensate for it)
// if ( vvarac_idx - (double)((uint16_t)vvarac_idx) > 0.5 ) {
//	vvarac_idx += 0.5;
// }
// write_vvarac(vvarac_tbl[(uint16_t)vvarac_idx]);	// function uses obsolete ad5722r from old board
// write_vbias(-1.25);								// function uses obsolete ad5722r from old board

// usleep(1000000);									// wait for the v_varac & v_bias to settle down
	usleep(10000); // wait for the v_varac & v_bias to settle down
}

void tx_acq_async(double startfreq, double stopfreq, double spacfreq,
		double sampfreq, unsigned int nsamples)
{

	// buffer in the fpga needs to be an even number, therefore the number of samples should be even as well
	if (nsamples % 2)
	{
		nsamples++;
	}

	create_measurement_folder("tx_acq");

	// print matlab script to analyze datas
	sprintf(pathname, "current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr, "%s\n", foldername);
	fclose (fptr);

	// print matlab script to analyze datas
	sprintf(pathname, "measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr, "tx_acq([data_folder,'%s']);\n", foldername);
	fclose(fptr);

	// print the NMR acquired settings
	//sprintf(pathname,"%s/matlab_settings.txt",foldername);	// put the data into the data folder
	//fptr = fopen(pathname, "a");
	//fprintf(fptr,"%f\n", startfreq);
	//fprintf(fptr,"%f\n", stopfreq);
	//fprintf(fptr,"%f\n", spacfreq);
	//fprintf(fptr,"%d\n", nsamples);
	//fclose(fptr);

	//sprintf(pathname,"%s/readable_settings.txt",foldername);	// put the data into the data folder
	//fptr = fopen(pathname, "a");
	//fprintf(fptr,"Start frequency: %f\n", startfreq);
	//fprintf(fptr,"Stop frequency: %f\n", stopfreq);
	//fprintf(fptr,"Spacing frequency: %f\n", spacfreq);
	//fprintf(fptr,"Number of samples: %d\n", nsamples);
	//fclose(fptr);
	//

	// print general measurement settings
	sprintf(pathname, "%s/acqu.par", foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr, "freqSta = %4.3f\n", startfreq);
	fprintf(fptr, "freqSto = %4.3f\n", stopfreq);
	fprintf(fptr, "freqSpa = %4.3f\n", spacfreq);
	fprintf(fptr, "nSamples = %d\n", nsamples);
	fprintf(fptr, "freqSamp = %4.3f\n", sampfreq);
	fclose(fptr);

	char * filename;
	double ifreq = 0;
	filename = (char*) malloc(100 * sizeof(char));
	stopfreq += (spacfreq / 2); // the (spacfreq/2) factor is to compensate double comparison error. double cannot be compared with '==' operator !
	for (ifreq = startfreq; ifreq < stopfreq; ifreq += spacfreq)
	{
		snprintf(filename, 100, "tx_acq_%4.3f", ifreq);
		// printf("freq: %4.3f\n", ifreq);
		tx_sampling_async(ifreq, sampfreq, nsamples, filename);
		usleep(1); // this delay is necessary. If it's not here, the system will not crash but the i2c will stop working (?), and the reading length is incorrect
	}

}

void tx_acq_sync(double startfreq, double stopfreq, double spacfreq,
		unsigned int nsamples)
{

	// buffer in the fpga needs to be an even number, therefore the number of samples should be even as well
	if (nsamples % 2)
	{
		nsamples++;
	}

	create_measurement_folder("tx_acq");

	// print matlab script to analyze datas
	sprintf(pathname, "current_folder.txt");
	fptr = fopen(pathname, "w");
	fprintf(fptr, "%s\n", foldername);
	fclose (fptr);

	// print matlab script to analyze datas
	sprintf(pathname, "measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr, "tx_acq([data_folder,'%s']);\n", foldername);
	fclose(fptr);

	// print the NMR acquired settings
	//sprintf(pathname,"%s/matlab_settings.txt",foldername);	// put the data into the data folder
	//fptr = fopen(pathname, "a");
	//fprintf(fptr,"%f\n", startfreq);
	//fprintf(fptr,"%f\n", stopfreq);
	//fprintf(fptr,"%f\n", spacfreq);
	//fprintf(fptr,"%d\n", nsamples);
	//fclose(fptr);

	//sprintf(pathname,"%s/readable_settings.txt",foldername);	// put the data into the data folder
	//fptr = fopen(pathname, "a");
	//fprintf(fptr,"Start frequency: %f\n", startfreq);
	//fprintf(fptr,"Stop frequency: %f\n", stopfreq);
	//fprintf(fptr,"Spacing frequency: %f\n", spacfreq);
	//fprintf(fptr,"Number of samples: %d\n", nsamples);
	//fclose(fptr);
	//

	// print general measurement settings
	sprintf(pathname, "%s/acqu.par", foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr, "freqSta = %4.3f\n", startfreq);
	fprintf(fptr, "freqSto = %4.3f\n", stopfreq);
	fprintf(fptr, "freqSpa = %4.3f\n", spacfreq);
	fprintf(fptr, "nSamples = %d\n", nsamples);
	fclose(fptr);

	char * filename;
	double ifreq = 0;
	filename = (char*) malloc(100 * sizeof(char));
	stopfreq += (spacfreq / 2); // the (spacfreq/2) factor is to compensate double comparison error. double cannot be compared with '==' operator !
	for (ifreq = startfreq; ifreq < stopfreq; ifreq += spacfreq)
	{
		snprintf(filename, 100, "tx_acq_%4.3f", ifreq);
		// printf("freq: %4.3f\n", ifreq);
		tx_sampling_sync(ifreq, nsamples, filename);
		usleep(1); // this delay is necessary. If it's not here, the system will not crash but the i2c will stop working (?), and the reading length is incorrect
	}

}

void init_default_system_param()
{

// initialize control lines to default value
	ctrl_out = CNT_OUT_default;
	alt_write_word((h2p_ctrl_out_addr), ctrl_out);
	usleep(100);

	alt_write_word(h2p_echo_skip_hw_addr, 1); // default skip factor number is 1. Which means it doesn't skip any echoes

// initialize i2c default
// ctrl_i2c = CNT_I2C_default;

// set reconfig configuration for pll's
	Reconfig_Mode(h2p_nmr_sys_pll_addr, 1); // polling mode for main pll
	Reconfig_Mode(h2p_analyzer_pll_addr, 1); // polling mode for main pll

//write_i2c_int_cnt (ENABLE, AMP_HP_LT1210_EN_msk, DISABLE_MESSAGE); // enable high-power transmitter
//write_i2c_int_cnt (ENABLE, PSU_5V_ADC_EN_msk|PSU_5V_ANA_P_EN_msk|PSU_5V_ANA_N_EN_msk|PSU_5V_TX_N_EN_msk|PSU_15V_TX_P_EN_msk|PSU_15V_TX_N_EN_msk, DISABLE_MESSAGE);
//write_i2c_int_cnt (ENABLE, PAMP_IN_SEL_RX_msk, DISABLE_MESSAGE);

	ctrl_out |= NMR_CLK_GATE_AVLN; // enable RF gate path, disable PLL_analyzer path
// ctrl_out &= ~NMR_CLK_GATE_AVLN;					// disable RF gate path, enable PLL analyzer path
// alt_write_word(h2p_ctrl_out_addr, ctrl_out);		// write down the control
// usleep(100);

	ctrl_out |= (DCONV_FIR_RST_RESET_N | DCONV_FIR_Q_RST_RESET_N); // enable the FIR filter

	/* manual selection of cshunt and cseries (put breakpoint before running)
	 int cshuntval = 120;
	 int cseriesval = 215;
	 while (1) {
	 write_relay_cnt(cshuntval,cseriesval, DISABLE_MESSAGE);
	 printf("put a breakpoint here!!!!");
	 }
	 */
// write_relay_cnt(19, 66, DISABLE_MESSAGE);
// init_dac_ad5722r();		// power up the dac and init its operation
	/* manual selection of vbias and vvarac (put breakpoint before running)
	 double vbias_test, vvarac_test;
	 vbias_test = -3.174;
	 vvarac_test = -1.77;
	 while (1) {
	 write_vbias(vbias_test);
	 write_vvarac(vvarac_test);
	 printf("put a breakpoint here!!!!");
	 }
	 */
// write_vbias(-3.35);		// default number for vbias is -3.35V (reflection at -20dB at 4.3MHz)
// write_vvarac(-1.2);	// the default number for v_varactor is -1.2V (gain of 23dB at 4.3 MHz)
// write_i2c_rx_gain (0x00 & 0x0F);	//// OBSOLETE. Gain is not controlled using this function anymores
// tune_board(4.3); 				// tune board frequency: input is frequency in MHz
// reset controller (CAUTION: this will fix the problem of crashed controller temporarily but won't really eliminate the problem: fix the state machine instead)
// the issue is the logic in ADC_WINGEN, where TOKEN is implemented to prevent retriggering. But also at the same time, if ADC_CLOCK is generated after ACQ_WND rises,
// the TOKEN is not resetted to 0, which will prevent the state machine from running. It is fixed by having reset button implemented to reset the TOKEN to 0 just
// before any acquisition.
	ctrl_out |= NMR_CNT_RESET;
	alt_write_word(h2p_ctrl_out_addr, ctrl_out); // write down the control
	usleep(10);
	ctrl_out &= ~(NMR_CNT_RESET);
	alt_write_word(h2p_ctrl_out_addr, ctrl_out); // write down the control

// usleep(500000); // this delay is extremely necessary! or data will be bad in first cpmg scan. also used to wait for vvarac and vbias to settle down

}

void close_system()
{
	write_relay_cnt(0, 0, DISABLE_MESSAGE); //  disable all relays

// 	write_i2c_rx_gain (0x0F); // OBSOLETE. Gain is not controlled using this function anymoren
}

// MAIN SYSTEM (ENABLE ONE AT A TIME)

/* Init default system param (rename the output to "init")
 int main()
 {
 // printf("Init system\n");

 open_physical_memory_device();
 mmap_peripherals();
 init_default_system_param();

 alt_write_word(h2p_rx_delay_addr, 20);

 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */

/* SPI for vbias and vvarac (rename the output to "preamp_tuning")
 int main(int argc, char * argv[]) {
 // printf("Preamp tuning with SPI\n");

 // input parameters
 double vbias = atof(argv[1]);
 double vvarac = atof(argv[2]);

 open_physical_memory_device();
 mmap_peripherals();

 init_dac_ad5724r();			// power up the dac and init its operation
 // wr_dac_ad5724 IS A NEW FUNCTION AND IS NOT VERIFIED!!!!!
 wr_dac_ad5724r (h2p_dac_preamp_addr, DAC_A, vbias, DISABLE_MESSAGE); // vbias cannot exceed 1V, due to J310 transistor gate voltage
 wr_dac_ad5724r (h2p_dac_preamp_addr, DAC_B, vvarac, DISABLE_MESSAGE);

 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */

/* I2C matching network control (rename the output to "i2c_mtch_ntwrk")
 int main(int argc, char * argv[]) {
 // printf("Matching network control with I2C\n");

 // input parameters
 unsigned int cshunt = atoi(argv[1]);
 unsigned int cseries = atoi(argv[2]);

 open_physical_memory_device();
 mmap_peripherals();

 write_relay_cnt(cshunt, cseries, DISABLE_MESSAGE);

 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */

/* I2C pamp input control (rename the output to "spi_pamp_input")
 int main(int argc, char * argv[]) {
 // printf("pamp input control control with SPI\n");

 // input parameters
 unsigned int cnt_in = atoi(argv[1]);

 open_physical_memory_device();
 mmap_peripherals();

 write_pamprelay_cnt(cnt_in, DISABLE_MESSAGE);

 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */

/* I2C general control (rename the output to "i2c_gnrl")
 int main(int argc, char * argv[]) { // argv cannot contain more than so many characters
 // printf("General control with I2C\n");

 // input parameters
 unsigned int gnrl_cnt = atoi(argv[1]);
 unsigned int gnrl_cnt1 = atoi(argv[2]);

 open_physical_memory_device();
 mmap_peripherals();

 write_i2c_int_cnt (ENABLE, (gnrl_cnt & 0xFFFF), (gnrl_cnt1 & 0xFFFF), DISABLE_MESSAGE); // enable the toggled index

 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */

/* Wobble Sync (rename the output to "wobble_sync")
 int main(int argc, char * argv[])
 {

 // the wobble function startfreq is minimum 1 MHz, otherwise the TX PLL (h2p_analyzer_pll_addr) won't lock.

 // input parameters
 double startfreq = atof(argv[1]);
 double stopfreq = atof(argv[2]);
 double spacfreq = atof(argv[3]);

 open_physical_memory_device();
 mmap_peripherals();
 // init_default_system_param();

 ctrl_out = alt_read_word(h2p_ctrl_out_addr);
 alt_write_word((h2p_ctrl_out_addr), (ctrl_out & (~TX_OPA_SD_MSK))); // mask out the TX_OPA_SD signal in the fpga (disable TX Opamp shutdown during reception)

 //WOBBLE
 // unsigned int wobb_samples = (unsigned int) (lround(sampfreq / spacfreq)); // the number of ADC samples taken
 unsigned int wobb_samples = (unsigned int) (lround(stopfreq / spacfreq)); // the number of ADC samples taken

 // memory allocation
 // rddata_16 = (unsigned int*) malloc(wobb_samples * sizeof(unsigned int));
 rddata = (int *) malloc(wobb_samples * sizeof(int));

 tx_acq_sync(startfreq, stopfreq, spacfreq, wobb_samples);

 alt_write_word((h2p_ctrl_out_addr), (ctrl_out | TX_OPA_SD_MSK)); // re-enable the TX_OPA_SD signal in the fpga

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 // free memory
 // free (rddata_16);
 free (rddata);

 return 0;
 }
 */

/* Wobble Async (rename the output to "wobble_async")
 int main(int argc, char * argv[])
 {

 // the wobble function startfreq is minimum 1 MHz, otherwise the TX PLL (h2p_analyzer_pll_addr) won't lock.

 // input parameters
 double startfreq = atof(argv[1]);
 double stopfreq = atof(argv[2]);
 double spacfreq = atof(argv[3]);
 double sampfreq = atof(argv[4]);

 open_physical_memory_device();
 mmap_peripherals();
 // init_default_system_param();

 ctrl_out = alt_read_word(h2p_ctrl_out_addr);
 alt_write_word((h2p_ctrl_out_addr), (ctrl_out & (~TX_OPA_SD_MSK))); // mask out the TX_OPA_SD signal in the fpga (disable TX Opamp shutdown during reception)

 //WOBBLE
 // unsigned int wobb_samples = (unsigned int) (lround(sampfreq / spacfreq)); // the number of ADC samples taken
 unsigned int wobb_samples = (unsigned int) (lround(stopfreq / spacfreq)); // the number of ADC samples taken

 // memory allocation
 // rddata_16 = (unsigned int*) malloc(wobb_samples * sizeof(unsigned int));
 rddata = (int *) malloc(wobb_samples * sizeof(int));

 tx_acq_async(startfreq, stopfreq, spacfreq, sampfreq, wobb_samples);

 alt_write_word((h2p_ctrl_out_addr), (ctrl_out | TX_OPA_SD_MSK)); // re-enable the TX_OPA_SD signal in the fpga

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 // free memory
 // free (rddata_16);
 free (rddata);

 return 0;
 }
 */

/* Preamp gain characterization (rename the output to "pamp_char_async")
 int main(int argc, char * argv[])
 {
 printf("Pamp characterization measurement starts\n");

 // input parameters
 double startfreq = atof(argv[1]);
 double stopfreq = atof(argv[2]);
 double spacfreq = atof(argv[3]);
 double sampfreq = atof(argv[4]);

 open_physical_memory_device();
 mmap_peripherals();
 // init_default_system_param();

 ctrl_out = alt_read_word(h2p_ctrl_out_addr);
 alt_write_word((h2p_ctrl_out_addr), (ctrl_out & (~TX_OPA_EN))); // disable the TX opamp

 unsigned int samples = (unsigned int) (lround(sampfreq / spacfreq)); // the number of ADC samples taken

 // memory allocation
 // rddata_16 = (unsigned int*) malloc(samples * sizeof(unsigned int));
 rddata = (int *) malloc(samples * sizeof(unsigned int));

 tx_acq_async(startfreq, stopfreq, spacfreq, sampfreq, samples);

 alt_write_word((h2p_ctrl_out_addr), (ctrl_out | TX_OPA_EN)); // re-enable the TX opamp (default)

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 // free memory
 // free (rddata_16);
 free (rddata);

 return 0;
 }
 */

// Preamp gain characterization (rename the output to "pamp_char_sync")
int main(int argc, char * argv[])
{
	printf("Pamp characterization measurement starts\n");

	// input parameters
	double startfreq = atof(argv[1]);
	double stopfreq = atof(argv[2]);
	double spacfreq = atof(argv[3]);

	open_physical_memory_device();
	mmap_peripherals();
	// init_default_system_param();

	ctrl_out = alt_read_word(h2p_ctrl_out_addr);
	alt_write_word((h2p_ctrl_out_addr), (ctrl_out & (~TX_OPA_EN))); // disable the TX opamp

	// unsigned int samples = (unsigned int) (lround(sampfreq / spacfreq)); // the number of ADC samples taken
	unsigned int samples = (unsigned int) (lround(stopfreq / spacfreq)); // the number of ADC samples taken

	// memory allocation
	// rddata_16 = (unsigned int*) malloc(samples * sizeof(unsigned int));
	rddata = (int *) malloc(samples * sizeof(unsigned int));

	tx_acq_sync(startfreq, stopfreq, spacfreq, samples);

	alt_write_word((h2p_ctrl_out_addr), (ctrl_out | TX_OPA_EN)); // re-enable the TX opamp (default)

	// close_system();
	munmap_peripherals();
	close_physical_memory_device();

	// free memory
	// free (rddata_16);
	free (rddata);

	return 0;
}
//

/* CPMG Iterate
 // rename the output to "cpmg_iterate_raw" and define GET_RAW_DATA to get raw data.
 // rename the output to "cpmg_iterate_dconv" and define GET_RAW_DCONV to get downconverted data.
 // if CPMG Sequence is used without writing to text file, rename the output to "cpmg_iterate_direct". Use STORE_TO_SDRAM_NOREAD in the cpmg_Sequence
 int main(int argc, char * argv[])
 {
 // this program can only be run with the power supply 'on' that enables ADC circuitry and clock.
 // To turn on the power supply, you can use the Python code and add breakpoint before CPMG_sequence().
 // printf("NMR system start\n");

 // input parameters
 double cpmg_freq = atof(argv[1]);
 double pulse1_us = atof(argv[2]);
 double pulse2_us = atof(argv[3]);
 double pulse1_dtcl = atof(argv[4]);
 double pulse2_dtcl = atof(argv[5]);
 double echo_spacing_us = atof(argv[6]);
 long unsigned scan_spacing_us = atoi(argv[7]);
 unsigned int samples_per_echo = atoi(argv[8]);
 unsigned int echoes_per_scan = atoi(argv[9]);
 double init_adc_delay_compensation = atof(argv[10]);
 unsigned int number_of_iteration = atoi(argv[11]);
 uint32_t ph_cycl_en = atoi(argv[12]);
 unsigned int pulse180_t1_int = atoi(argv[13]);
 unsigned int delay180_t1_int = atoi(argv[14]);
 unsigned int tx_opa_sd = atoi(argv[15]);	// shutdown tx during reception
 dconv_fact = atoi(argv[16]);	// down conversion factor
 echo_skip_hw = atoi(argv[17]);// echo skipping factor in hardware (echoes captured by the ADC are reduced by this factor)

 if (samples_per_echo % dconv_fact)
 {
 printf("\tERROR: samples_per_echo is not dconv_fact multiplication.\n");
 return 0;
 }
 if (dconv_fact < 4)
 {
 printf("\tERROR: dconv_fact is less than 4.\n");
 return 0;
 }

 // memory allocation
 #ifdef GET_RAW_DATA
 dsize = samples_per_echo*echoes_per_scan/echo_skip_hw; // container size, limited by max memory 1024x1024 usually
 if (dsize > 1024*1024)
 {
 printf("\tERROR: (samples_per_echo*echoes_scan/echo_skip_hw) is larger than 1024*1024.\n");
 return 0;
 }

 // 	rddata_16 = (unsigned int*)malloc(dsize*sizeof(unsigned int));
 rddata = (int *)malloc(dsize*sizeof(int));// divide by 2 because 1 beat contains 2 symbols
 #endif
 #ifdef GET_DCONV_DATA
 dconv_size = samples_per_echo * echoes_per_scan / dconv_fact / echo_skip_hw * 2; // multiply 2 because of IQ data
 if (dconv_size > 1024*1024)
 {
 printf("\tERROR: (samples_per_echo*echoes_scan/dconv_fact/echo_skip_hw*2) is larger than 1024*1024.\n");
 return 0;
 }

 dconv = (int *) malloc(dconv_size* sizeof(int));
 #endif

 open_physical_memory_device();
 mmap_peripherals();
 // init_default_system_param();

 // write t1-IR measurement parameters (put both to 0 if IR is not desired)
 alt_write_word(h2p_t1_pulse, pulse180_t1_int);
 alt_write_word(h2p_t1_delay, delay180_t1_int);

 // write downconversion factor
 alt_write_word(h2p_dec_fact_addr, dconv_fact);

 // write echo skip factor
 alt_write_word(h2p_echo_skip_hw_addr, echo_skip_hw);

 // read the current ctrl_out
 ctrl_out = alt_read_word(h2p_ctrl_out_addr);

 // enable the TX opamp during reception, will be controlled using the tx_opa_sd instead
 ctrl_out = ctrl_out | TX_OPA_EN;
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);

 if (tx_opa_sd)
 { // shutdown tx opamp during reception
 ctrl_out = ctrl_out | TX_OPA_SD_MSK;
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);
 }
 else
 { // power up tx opamp all the way during reception
 ctrl_out = ctrl_out & (~TX_OPA_SD_MSK);
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);
 }

 // read and write fir coefficients
 // fir registers cannot be read like a standard avalon-mm registers, it has sequence if the setting is set to read/write mode
 // look at the fir user guide to see this sequence
 // however, it can be set just to read mode or write mode and it supposed to work with avalon-mm
 ctrl_out &= ~(DCONV_FIR_RST_RESET_N | DCONV_FIR_Q_RST_RESET_N);	// reset the FIR filter
 alt_write_word(h2p_ctrl_out_addr, ctrl_out);	// write down the control
 usleep(1);
 ctrl_out |= (DCONV_FIR_RST_RESET_N | DCONV_FIR_Q_RST_RESET_N);// enable the FIR filter
 alt_write_word(h2p_ctrl_out_addr, ctrl_out);	// write down the control
 usleep(1);
 // alt_write_word(h2p_dconv_firQ_addr, 20);
 //

 alt_write_word(h2p_adc_val_sub, 9275);// do noise measurement and all the data to get this ADC DC bias integer value

 // printf("cpmg_freq = %0.3f\n",cpmg_freq);
 CPMG_iterate(cpmg_freq, pulse1_us, pulse2_us, pulse1_dtcl, pulse2_dtcl,
 echo_spacing_us, scan_spacing_us, samples_per_echo, echoes_per_scan,
 init_adc_delay_compensation, number_of_iteration, ph_cycl_en);

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 // free memory
 #ifdef GET_RAW_DATA
 // 	free(rddata_16);	//freeing up allocated memory required for multiple calls from host
 free(rddata);//petrillo 2Feb2019
 #endif
 #ifdef GET_DCONV_DATA
 free (dconv);
 #endif

 return 0;
 }
 */

/* FID Iterate (rename the output to "fid")
 int main(int argc, char * argv[])
 {

 // input parameters
 double cpmg_freq = atof(argv[1]);
 double pulse2_us = atof(argv[2]);
 double pulse2_dtcl = atof(argv[3]);
 long unsigned scan_spacing_us = atoi(argv[4]);
 unsigned int samples_per_echo = atoi(argv[5]);
 unsigned int number_of_iteration = atoi(argv[6]);
 unsigned int tx_opa_sd = atoi(argv[7]);

 // memory allocation
 #ifdef GET_RAW_DATA
 // rddata_16 = (unsigned int*)malloc(samples_per_echo*sizeof(unsigned int));
 rddata = (int *)malloc(samples_per_echo*sizeof(unsigned int));
 #endif

 open_physical_memory_device();
 mmap_peripherals();
 //init_default_system_param();

 // enable the TX opamp during reception (default), will be controlled using the tx_opa_sd instead
 ctrl_out = ctrl_out | TX_OPA_EN;
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);

 if (tx_opa_sd)
 { // shutdown tx opamp during reception
 ctrl_out = ctrl_out | TX_OPA_SD_MSK;
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);
 }
 else
 { // power up tx opamp all the way during reception
 ctrl_out = ctrl_out & (~TX_OPA_SD_MSK);
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);
 }

 // alt_write_word(h2p_adc_val_sub, 9732); // do noise measurement and all the data to get this ADC DC bias integer value

 FID_iterate(cpmg_freq, pulse2_us, pulse2_dtcl, scan_spacing_us,
 samples_per_echo, number_of_iteration, ENABLE_MESSAGE);

 // shutdown tx opamp during receptin (default)
 ctrl_out = ctrl_out | TX_OPA_SD_MSK;
 alt_write_word((h2p_ctrl_out_addr), ctrl_out);

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 // free memory
 #ifdef GET_RAW_DATA
 // free(rddata_16);
 free(rddata);
 #endif

 return 0;
 }
 */

/* noise Iterate (rename the output to "noise")
 int main(int argc, char * argv[])
 {

 // input parameters
 double samp_freq = atof(argv[1]);
 long unsigned scan_spacing_us = atoi(argv[2]);
 unsigned int samples_per_echo = atoi(argv[3]);
 unsigned int number_of_iteration = atoi(argv[4]);

 // memory allocation
 #ifdef GET_RAW_DATA
 // rddata_16 = (unsigned int*)malloc(samples_per_echo*sizeof(unsigned int)); // added malloc to this routine only - other routines will need to be updated when required
 rddata = (int *)malloc(samples_per_echo*sizeof(unsigned int));// petrillo 2Feb2019
 #endif

 open_physical_memory_device();
 mmap_peripherals();
 init_default_system_param();

 double cpmg_freq = samp_freq / 4; // the building block that's used is still nmr cpmg, so the sampling frequency is fixed to 4*cpmg_frequency
 noise_iterate(cpmg_freq, scan_spacing_us, samples_per_echo,
 number_of_iteration, DISABLE_MESSAGE);

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 // free memory
 #ifdef GET_RAW_DATA
 // free(rddata_16);	//freeing up allocated memory required for multiple calls from host
 free(rddata);//petrillo 2Feb2019
 #endif

 return 0;
 }
 */

/* parameter calculator (calculate the real delay and timing based on the verilog
 int main(int argc, char * argv[]) {

 double b1Freq = atof(argv[1]);			// nmr RF cpmg frequency (in MHz)
 double echoShift = atof(argv[2]);		// shift the 180 deg data capture relative to the middle of the 180 delay span. This is to compensate shifting because of signal path delay / other factors. This parameter could be negative as well
 double p90LengthGiven = atof(argv[3]);	// the length of cpmg 90 deg pulse
 double p180LengthGiven = atof(argv[4]);	// the length of cpmg 180 deg pulse
 double echoTimeGiven = atof(argv[5]);	// the length between one echo to the other (equal to pulse2_us + delay2_us)
 unsigned int nrPnts = atoi(argv[6]);	// the total adc samples captured in one echo

 unsigned int cpmg_param [5];
 double adc_ltc1746_freq = b1Freq*4;
 double nmr_fsm_clkfreq = b1Freq*16;

 cpmg_param_calculator_ltc1746(
 cpmg_param,
 nmr_fsm_clkfreq,
 b1Freq,
 adc_ltc1746_freq,
 echoShift,
 p90LengthGiven,
 p180LengthGiven,
 echoTimeGiven,
 nrPnts
 );

 unsigned int pulse1_int = *(cpmg_param+PULSE1_OFFST);
 unsigned int delay1_int = *(cpmg_param+DELAY1_OFFST);
 unsigned int pulse2_int = *(cpmg_param+PULSE2_OFFST);
 unsigned int delay2_int = *(cpmg_param+DELAY2_OFFST);
 unsigned int init_adc_delay_int = *(cpmg_param+INIT_DELAY_ADC_OFFST);

 double p90_run = (double)pulse1_int/nmr_fsm_clkfreq;
 double d90_run = (double)delay1_int/nmr_fsm_clkfreq;
 double p180_run = (double)pulse2_int/nmr_fsm_clkfreq;
 double d180_run = (double)delay2_int/nmr_fsm_clkfreq;
 double adc_delay_run; // the delay added in the state machine is minimum 2.25 ADC clock cycles for init_delay of 2 or less, and inherent 0.25 clock cycles for anything more than 2
 if (init_adc_delay_int<=2) {
 adc_delay_run = 2.25/adc_ltc1746_freq;
 }
 else {
 adc_delay_run = (((double)init_adc_delay_int)+0.25)/adc_ltc1746_freq;
 }

 double acq_wdw_run = (double)nrPnts/adc_ltc1746_freq;
 double acq_wdw_tail = d180_run - adc_delay_run - acq_wdw_run;

 printf("p90 Pulse Run = %f\n",p90_run);
 printf("d90 Delay Run = %f\n",d90_run);
 printf("p180 Pulse Run = %f\n",p180_run);
 printf("d180 Delay Run = %f\n",d180_run);
 printf("\tADC Delay at d180 = %f\n",adc_delay_run);
 printf("\tAcquisition window = %f\n",acq_wdw_run);
 printf("\tDelay after acq. window = %f\n",acq_wdw_tail);

 return 0;

 }
 */

/* Magnet controller
 // rename the output to "mgnt_ctrl"
 // this program controls the magnet controller fsm code
 int main(int argc, char * argv[])
 {
 // input parameters
 double chg_plen_us = atof(argv[1]); // charging pulse length
 double chg_dlen_us = atof(argv[2]); // charging delay length
 double dchg_plen_us = atof(argv[3]); // discharging pulse length
 double dchg_dlen_us = atof(argv[4]); // discharging delay length
 unsigned int n = atoi(argv[5]); // number of repetition
 unsigned int d = atoi(argv[6]); // delay after sequence
 double clk_freq = atof(argv[7]); // clock frequency used by the finite state machine

 open_physical_memory_device();
 mmap_peripherals();
 init_default_system_param();

 // read out the control output before writing it
 ctrl_out = alt_read_word(h2p_ctrl_out_addr);

 // set pll for CPMG
 Set_PLL(h2p_nmr_sys_pll_addr, 0, clk_freq, 0.5, DISABLE_MESSAGE);
 Reset_PLL(h2p_ctrl_out_addr, PLL_NMR_SYS_RST_ofst, ctrl_out);
 Set_DPS(h2p_nmr_sys_pll_addr, 0, 0, DISABLE_MESSAGE);
 Wait_PLL_To_Lock(h2p_ctrl_in_addr, PLL_NMR_SYS_lock_ofst);

 // write t1-IR measurement parameters (put both to 0 if IR is not desired)
 alt_write_word(h2p_mgnt_chg_plen_addr,
 (unsigned int) (chg_plen_us * clk_freq));
 alt_write_word(h2p_mgnt_chg_dlen_addr,
 (unsigned int) (chg_dlen_us * clk_freq));
 alt_write_word(h2p_mgnt_dchg_plen_addr,
 (unsigned int) (dchg_plen_us * clk_freq));
 alt_write_word(h2p_mgnt_dchg_dlen_addr,
 (unsigned int) (dchg_dlen_us * clk_freq));
 alt_write_word(h2p_mgnt_n_addr, n);
 alt_write_word(h2p_mgnt_d_addr, d);

 // reset the magnet controller
 ctrl_out |= MGNT_RST;
 alt_write_word(h2p_ctrl_out_addr, ctrl_out);
 ctrl_out &= ~(MGNT_RST);
 alt_write_word(h2p_ctrl_out_addr, ctrl_out);

 // start the FSM
 ctrl_out |= MGNT_START;
 alt_write_word(h2p_ctrl_out_addr, ctrl_out);
 ctrl_out &= ~(MGNT_START);
 alt_write_word(h2p_ctrl_out_addr, ctrl_out);

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();

 return 0;
 }
 */

/* standalone function
 // this main function doesn't rely on any other main functions (e.g. to initialize the system) to work on, unlike the main functions above
 int main(int argc, char * argv[])
 {
 // init
 open_physical_memory_device();
 mmap_peripherals();
 init_default_system_param();

 unsigned int x[10], y[10], z[10];

 hs_init(h2p_mgnt_hs_addr, ENABLE_MESSAGE);

 int ex = 0;
 while (1)
 {
 hs_rd_xyz(h2p_mgnt_hs_addr, x, y, z, 10);
 printf("x = %fmT, y = %fmT, z = %fmT\n", conv_to_Tesla(x[0]) * 1e3,
 conv_to_Tesla(y[0]) * 1e3, conv_to_Tesla(z[0]) * 1e3);
 // printf("x = %d, y = %d, z = %d\n\n", x[1], y[1], z[1]);
 usleep(500000);
 if (ex)
 break;
 }

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */

/* standalone function // this main function doesn't rely on any other main functions (e.g. to initialize the system) to work on, unlike the main functions above
 int main(int argc, char * argv[])
 {
 // init
 open_physical_memory_device();
 mmap_peripherals();
 init_default_system_param();

 // close_system();
 munmap_peripherals();
 close_physical_memory_device();
 return 0;
 }
 */
