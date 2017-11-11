#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#include "alt_generalpurpose_io.h"
#include "hps_linux.h"
#include "hwlib.h"
#include "socal/alt_gpio.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include "hps_soc_system.h"

#include "functions/general.h"
#include "functions/avalon_i2c.h"
#include "functions/tca9555_driver.h"
#include "functions/avalon_spi.h"
#include "functions/dac_ad5722r_driver.h"
#include "functions/general.h"
#include "functions/reconfig_functions.h"
#include "functions/pll_param_generator.h"
#include "functions/adc_functions.h"
#include "functions/cpmg_functions.h"
#include "functions/AlteraIP/altera_avalon_fifo_regs.h"

void open_physical_memory_device() {
    // We need to access the system's physical memory so we can map it to user
    // space. We will use the /dev/mem file to do this. /dev/mem is a character
    // device file that is an image of the main memory of the computer. Byte
    // addresses in /dev/mem are interpreted as physical memory addresses.
    // Remember that you need to execute this program as ROOT in order to have
    // access to /dev/mem.

    fd_dev_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd_dev_mem  == -1) {
        printf("ERROR: could not open \"/dev/mem\".\n");
        printf("    errno = %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

void close_physical_memory_device() {
    close(fd_dev_mem);
}

void mmap_hps_peripherals() {
    hps_gpio = mmap(NULL, hps_gpio_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, hps_gpio_ofst);
    if (hps_gpio == MAP_FAILED) {
        printf("Error: hps_gpio mmap() failed.\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }
}

void munmap_hps_peripherals() {
    if (munmap(hps_gpio, hps_gpio_span) != 0) {
        printf("Error: hps_gpio munmap() failed\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }

    hps_gpio = NULL;
}

void mmap_fpga_peripherals() {
    // Use mmap() to map the address space related to the fpga leds into user
    // space so we can interact with them.

    // The fpga leds are connected to the h2f_lw_axi_master, so its base
    // address is calculated from that of the h2f_lw_axi_master.

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

    h2f_lw_axi_master = mmap(NULL, h2f_lw_axi_master_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, h2f_lw_axi_master_ofst);
    if (h2f_lw_axi_master == MAP_FAILED) {
        printf("Error: h2f_lw_axi_master mmap() failed.\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }

    fpga_leds 						= h2f_lw_axi_master + LED_BASE;
    fpga_switches					= h2f_lw_axi_master + SW_BASE;

	h2p_ctrl_out_addr				= h2f_lw_axi_master + CTRL_OUT_BASE;
	h2p_ctrl_in_addr				= h2f_lw_axi_master + CTRL_IN_BASE;
	h2p_led_addr					= h2f_lw_axi_master + LED_BASE;
	h2p_pulse1_addr					= h2f_lw_axi_master + NMR_PARAMETERS_PULSE_90DEG_BASE;
	h2p_pulse2_addr					= h2f_lw_axi_master + NMR_PARAMETERS_PULSE_180DEG_BASE;
	h2p_delay1_addr					= h2f_lw_axi_master + NMR_PARAMETERS_DELAY_NOSIG_BASE;
	h2p_delay2_addr					= h2f_lw_axi_master + NMR_PARAMETERS_DELAY_SIG_BASE;
	h2p_nmr_pll_addr				= h2f_lw_axi_master + PLL_NMR_RECONFIG_BASE;
	h2p_echo_per_scan_addr			= h2f_lw_axi_master + NMR_PARAMETERS_ECHOES_PER_SCAN_BASE;
	h2p_i2ccommon_addr				= h2f_lw_axi_master + I2C_BASE;
	h2p_adc_fifo_addr				= h2f_lw_axi_master + ADC_FIFO_MEM_OUT_BASE;
	h2p_adc_fifo_status_addr		= h2f_lw_axi_master + ADC_FIFO_MEM_OUT_CSR_BASE;
	h2p_adc_samples_per_echo_addr	= h2f_lw_axi_master + NMR_PARAMETERS_SAMPLES_PER_ECHO_BASE;
	h2p_init_adc_delay_addr			= h2f_lw_axi_master + NMR_PARAMETERS_INIT_DELAY_BASE;
	h2p_nmr_pll_rst_dly_addr		= h2f_lw_axi_master + NMR_PARAMETERS_NMR_PLL_RST_DLY_BASE;
	h2p_dac_addr					= h2f_lw_axi_master + DAC_BASE;



	// initialize control lines to default value
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );

	// set reconfig configuration for both of pll's
	Reconfig_Mode(h2p_nmr_pll_addr,1); // polling mode for main pll



}

void munmap_fpga_peripherals() {
	write_i2c_relay_cnt(0,0);

	if (munmap(h2f_lw_axi_master, h2f_lw_axi_master_span) != 0) {
        printf("Error: h2f_lw_axi_master munmap() failed\n");
        printf("    errno = %s\n", strerror(errno));
        close(fd_dev_mem);
        exit(EXIT_FAILURE);
    }

    // turn off the relays to save the relay


    h2f_lw_axi_master	= NULL;
    fpga_leds			= NULL;
    fpga_switches		= NULL;


}

void mmap_peripherals() {
    mmap_hps_peripherals();
    mmap_fpga_peripherals();
}

void munmap_peripherals() {
    munmap_hps_peripherals();
    munmap_fpga_peripherals();
}

void setup_hps_gpio() {
    // Initialize the HPS PIO controller:
    //     Set the direction of the HPS_LED GPIO bit to "output"
    //     Set the direction of the HPS_KEY_N GPIO bit to "input"
    void *hps_gpio_direction = ALT_GPIO_SWPORTA_DDR_ADDR(hps_gpio);
    alt_setbits_word(hps_gpio_direction, ALT_GPIO_PIN_OUTPUT << HPS_LED_PORT_BIT);
    alt_setbits_word(hps_gpio_direction, ALT_GPIO_PIN_INPUT << HPS_KEY_N_PORT_BIT);
}

void setup_fpga_leds() {
    // Switch on first LED only
    alt_write_word(h2p_led_addr, 0xF0);
}

void handle_hps_led() {
    void *hps_gpio_data = ALT_GPIO_SWPORTA_DR_ADDR(hps_gpio);
    void *hps_gpio_port = ALT_GPIO_EXT_PORTA_ADDR(hps_gpio);

    uint32_t hps_gpio_input = alt_read_word(hps_gpio_port) & HPS_KEY_N_MASK;

    // HPS_KEY_N is active-low
    bool toggle_hps_led = (~hps_gpio_input & HPS_KEY_N_MASK);

    if (toggle_hps_led) {
        uint32_t hps_led_value = alt_read_word(hps_gpio_data);
        hps_led_value >>= HPS_LED_PORT_BIT;
        hps_led_value = !hps_led_value;
        hps_led_value <<= HPS_LED_PORT_BIT;
        alt_replbits_word(hps_gpio_data, HPS_LED_MASK, hps_led_value);
    }
}

void handle_fpga_leds() {
    uint32_t leds_mask = alt_read_word(h2p_led_addr);

    if (leds_mask != (0x01 << (LED_DATA_WIDTH - 1))) {
        // rotate leds
        leds_mask <<= 1;
    } else {
        // reset leds
        leds_mask = 0x1;
    }

    alt_write_word(h2p_led_addr, leds_mask);

}

void create_measurement_folder(char * foldertype) {
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	char command[60];
	sprintf(foldername,"%s_%04d_%02d_%02d_%02d_%02d_%02d",foldertype,tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	sprintf(command,"mkdir %s",foldername);
	system(command);

	// copy the executable file to the folder
	sprintf(command,"cp ./thesis_nmr %s/thesis_nmr",foldername);
	system(command);
}

void write_i2c_relay_cnt (uint8_t c_shunt, uint8_t c_series) {
	uint8_t i2c_addr_relay = 0x40;	// i2c address for TCA9555PWR used by the relay
	i2c_addr_relay >>= 1;			// shift by one because the LSB address is not used as an address (controlled by the Altera I2C IP)

	// reorder port1 data for CR1 because the schematic switched the order (LSB was switched to MSB, etc)
	uint8_t c_series_reorder = 0;
	c_series_reorder =
		((c_series & 0b00000001)<<7) |
		((c_series & 0b00000010)<<5) |
		((c_series & 0b00000100)<<3) |
		((c_series & 0b00001000)<<1) |
		((c_series & 0b00010000)>>1) |
		((c_series & 0b00100000)>>3) |
		((c_series & 0b01000000)>>5) |
		((c_series & 0b10000000)>>7);



	//alt_write_word( (h2p_i2ccommon_addr+CTRL_OFST), 0<<CORE_EN_SHFT); // disable i2c core
	//alt_write_word( (h2p_i2ccommon_addr+ISER_OFST), 0x00); // set the ISER_OFST
	//alt_write_word( (h2p_i2ccommon_addr+SCL_LOW_OFST), 250); // set the SCL_LOW_OFST
	//alt_write_word( (h2p_i2ccommon_addr+SCL_HIGH_OFST), 250); // set the SCL_HIGH_OFST
	//alt_write_word( (h2p_i2ccommon_addr+SDA_HOLD_OFST), 125); // set the SDA_HOLD_OFST

	alt_write_word( (h2p_i2ccommon_addr+CTRL_OFST), 1<<CORE_EN_SHFT); // enable i2c core
    
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT0 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );				// set port 0 as output
                                                      
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT1 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );					// set port 1 as output
                                                      
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT0 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((c_shunt) & I2C_DATA_MSK) );				// set output on port 0
                                                      
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_addr_relay<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT1 & I2C_DATA_MSK) );
	alt_write_word( (h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((c_series_reorder) & I2C_DATA_MSK) );	// set output on port 1
}

void sweep_matching_network() {
	uint8_t c_match_ntwrk = 0x0;
	while (1) {
		for (c_match_ntwrk = 0x00; c_match_ntwrk < 255; c_match_ntwrk += 5) {
			if (c_match_ntwrk == 0x00) {
				c_match_ntwrk = 100;
			}
			write_i2c_relay_cnt(c_match_ntwrk,c_match_ntwrk); //(c_shunt, c_series)
			printf("c_match_ntwrk = %d\n",c_match_ntwrk);
			usleep(1000000);
		}
	}
};

void matlab_mtching_ntwrk_cnt () {
	uint8_t cser = 0x0;
	uint8_t cpar = 0x0;


	char * command = malloc(sizeof(char) * 20);

	scanf("%s",command); // command is : s0010p1040e (4 number after s is the series capacitance, 4 number after p is parallel capacitance. e is the end of the line)
	while (command[0] == 's') {
		cser = 1000 *	(uint8_t)(command[1]-48)
				+ 100 *	(uint8_t)(command[2]-48)
				+ 10*	(uint8_t)(command[3]-48)
				+ 		(uint8_t)(command[4]-48);
		cpar = 1000 *	(uint8_t)(command[6]-48)
				+ 100 *	(uint8_t)(command[7]-48)
				+ 10*	(uint8_t)(command[8]-48)
				+ 		(uint8_t)(command[9]-48);

		write_i2c_relay_cnt( cpar, cser );
		printf("[%s]",command); // print the command
		printf("(%d)(%d)\n",cser, cpar); // print the integer dac_varac value

		scanf("%s",command);

	}
}

void test_dac_ad5722r() {
	// this is the very first working code for ad5722r.
	// it's not gonna be the main driver for ad5722r but is saved as a backup just in case the main driver is bad.

	// setup the control lines for dac clear and dac ldac
	ctrl_out = (ctrl_out & (~DAC_LDAC_en)) | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|PWR_CNT_REG|DAC_A_PU|DAC_B_PU|REF_PU );	// power up reference voltage, dac A, and dac B
	while (!(alt_read_word (h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit)  ) );				// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|OUT_RANGE_SEL_REG|DAC_AB|PN50 );			// set range voltage to +/- 5.0V
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );				// wait for the spi command to finish

	int16_t dac_v_var = 328 & 0xFFF;		// variable for V varactor --> DAC_A --> should be close to 5V going down to 3V-4V (start with max number = 2047, going down to 1000)
	int16_t dac_v_bias = -1146 & 0xFFF;	// variable for V bias --> DAC_B --> should be close to 0V going down to minus (start with 0 going down to -2048)
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_A|(dac_v_var<<4) );			// set the dac A voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );				// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_B|(dac_v_bias<<4) );			// set the dac B voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );				// wait for the spi command to finish

	/* read written data
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|DAC_REG|DAC_A );
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );
	dataread = alt_read_word(h2p_dac_addr + SPI_RXDATA_offst);
	*/
	/* read status data
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|PWR_CNT_REG );
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ) );
	statusread =  alt_read_word( (h2p_dac_addr + SPI_RXDATA_offst) );
	*/
}

void init_dac_ad5722r () {
	// setup the control lines for dac clear and dac ldac
	ctrl_out = ctrl_out | DAC_LDAC_en | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(10);

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|PWR_CNT_REG|DAC_A_PU|DAC_B_PU|REF_PU );	// power up reference voltage, dac A, and dac B
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));					// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|OUT_RANGE_SEL_REG|DAC_AB|PN50 );			// set range voltage to +/- 5.0V
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));					// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|Other_opt|Clamp_en);				// enable the current limit clamp
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));					// wait for the spi command to finish

	// clear the DAC output
	ctrl_out = ctrl_out & (~DAC_CLR) ;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(1);

	// release the clear pin
	ctrl_out = ctrl_out | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(1);

}

void print_warning_ad5722r() {
	int dataread;

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|PWR_CNT_REG );					// read the power control register
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_RRDY_bit) ));			// wait for the data to be ready
	dataread = alt_read_word( h2p_dac_addr + SPI_RXDATA_offst );								// read the data
	if (dataread & (1<<5) ) {
		printf("\nDevice is in thermal shutdown (TSD) mode!\n");
	}
	if (dataread & (1<<7)) {
		printf("DAC A (vvarac) overcurrent alert (OCa)!\n");
		usleep(50);
	}
	if (dataread & (1<<9)) {
		printf("DAC B (vbias) overcurrent alert (OCb)!\n");
		usleep(50);
	}

}

void write_vvarac (double vvarac) {
	int16_t vvarac_int;

	vvarac_int = (int16_t)((vvarac/5)*2048);
	if (vvarac_int > 2047) {
		vvarac_int = 2047;
	}

	write_vvarac_int(vvarac_int);
}

void write_vbias (double vbias) {
	int16_t vbias_int;

	vbias_int = (int16_t)((vbias/5)*2048);
	if (vbias_int > 2047) {
		vbias_int = 2047;
	}

	write_vbias_int(vbias_int);
}



void write_vbias_int(int16_t dac_v_bias) { //  easy method to write number to dac
	/* OLD DRIVER: DON'T DELETE
	// setup the control lines for dac clear and dac ldac
	ctrl_out = (ctrl_out & (~DAC_LDAC_en)) | DAC_CLR;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|PWR_CNT_REG|DAC_A_PU|DAC_B_PU|REF_PU );	// power up reference voltage, dac A, and dac B
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));				// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|OUT_RANGE_SEL_REG|DAC_AB|PN50 );			// set range voltage to +/- 5.0V
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));				// wait for the spi command to finish

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_B|((dac_v_bias & 0x0FFF)<<4) );			// set the dac B voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));				// wait for the spi command to finish
	*/

	/*
	// write data register to DAC output
	ctrl_out = ctrl_out & ~DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);
	*/

	// NEW CODE
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_B|((dac_v_bias & 0x0FFF)<<4) );			// set the dac B voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));

	// CODE BELOW IS WRITTEN BECAUSE SOMETIMES THE DAC DOESN'T WORK REALLY WELL
	// DATA WRITTEN IS NOT THE SAME AS DATA READ FROM THE ADC
	// THEREFORE THE DATA IS READ AND VERIFIED BEFORE IT IS USED AS AN OUTPUT

	// read the data just written to the dac
	int dataread;
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|DAC_REG|DAC_B|0x00 );			// read DAC B value
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_RRDY_bit) ));			// wait for read data to be ready
	dataread = alt_read_word( h2p_dac_addr + SPI_RXDATA_offst );								// read the data
	printf("vbias: %4.3f V ",(double)dac_v_bias/2048*5); 										// print the voltage desired
	printf("(w:0x%04x)", (dac_v_bias & 0x0FFF) ); 												// print the integer dac_varac value, truncate to 12-bit signed integer value
	printf("(r:0x%04x)\n",dataread>>4);															// print the read value

	// find out if warning has been detected
	usleep(1);
	print_warning_ad5722r();

	// if the data written to the dac is different than data read back, rewrite the dac
	// recursively until the data is right
	if ( (dac_v_bias&0x0FFF) != (dataread>>4)) {
		write_vvarac_int (dac_v_bias);
	}

	// write data register to DAC output
	ctrl_out = ctrl_out & ~DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);

	// disable LDAC one more time
	ctrl_out = ctrl_out | DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);

}

void write_vvarac_int(int16_t dac_v_varac) { //  easy method to write number to dac

	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|DAC_REG|DAC_A|((dac_v_varac & 0x0FFF)<<4) );			// set the dac A voltage
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));



	// CODE BELOW IS WRITTEN BECAUSE SOMETIMES THE DAC DOESN'T WORK REALLY WELL
	// DATA WRITTEN IS NOT THE SAME AS DATA READ FROM THE ADC
	// THEREFORE THE DATA IS READ AND VERIFIED BEFORE IT IS USED AS AN OUTPUT

	// read the data just written to the dac
	int dataread;
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , RD_DAC|DAC_REG|DAC_A|0x00 );			// read DAC A value
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	alt_write_word( (h2p_dac_addr + SPI_TXDATA_offst) , WR_DAC|CNT_REG|NOP );					// no operation (NOP)
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_TMT_bit) ));			// wait for the spi command to finish
	while (!(alt_read_word(h2p_dac_addr + SPI_STATUS_offst) & (1<<status_RRDY_bit) ));			// wait for read data to be ready
	dataread = alt_read_word( h2p_dac_addr + SPI_RXDATA_offst );								// read the data
	printf("vvarac: %4.3f V ",(double)dac_v_varac/2048*5); 										// print the voltage desired
	printf("(w:0x%04x)", (dac_v_varac & 0x0FFF) ); 												// print the integer dac_varac value, truncate to 12-bit signed integer value
	printf("(r:0x%04x)\n",dataread>>4);															// print the read value

	// find out if warning has been detected
	usleep(1);
	print_warning_ad5722r();

	// if the data written to the dac is different than data read back, rewrite the dac
	// recursively until the data is right
	if ( (dac_v_varac & 0x0FFF) != (dataread>>4)) {
		write_vvarac_int (dac_v_varac);
	}

	// write data register to DAC output
	ctrl_out = ctrl_out & ~DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);

	// disable LDAC one more time
	ctrl_out = ctrl_out | DAC_LDAC_en;
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;
	usleep(50);


}

void sweep_vbias (){
	int16_t dac_v_bias = -1300;
	int16_t init_bias_val = -2000;
	int16_t final_bias_val = 0;
	dac_v_bias = init_bias_val;
	while (1) {
		write_vbias_int(dac_v_bias);
		dac_v_bias += 100;
		if (dac_v_bias > final_bias_val) {
			dac_v_bias = init_bias_val;
		}
		usleep(4000000);
	}
}

void matlab_vbias_cnt () {
	double dac_v_bias = 0;
	int i = 0;
	double dot_pos = 10; // put the initial dot_pos to the maximum one, in case the dot is not found, it means we don't have to do the division


	// the command starts with 'c' to continue (followed by +/-, float numbers in ascii representation, and v as voltage), or 's' to stop
	// for example: c+1.23v
	char * command = malloc(sizeof(char) * 20);

	scanf("%s",command);
	while (command[0] == 'c') {
		dac_v_bias = 0;
		dot_pos = 10;

		for (i = 2; command[i]!='v'; i++ ) {
			if (command[i] == '.') {
				dot_pos = i;
			}
			else {
				dac_v_bias *= 10;
				dac_v_bias += (double)(command[i]-48); //convert the ascii to number
			}
		}

		// divide by (10^total_numbers_after_dot)
		for (; i>(dot_pos+1); i--) {
			dac_v_bias /= 10;
		}

		if (command[1] == '-') {
			dac_v_bias = -dac_v_bias;
		}

		// convert to -2048, 2047 scale
		dac_v_bias = (dac_v_bias*2048)/5;
		if (dac_v_bias >= 2048) {
			dac_v_bias = 2047;
		}
		if (dac_v_bias < -2048 ) {
			dac_v_bias = -2048;
		}




		write_vbias_int( (int16_t)dac_v_bias );
		printf("[%s]",command); // print the voltage
		printf("data: %f",(float)dac_v_bias/2048*5); // print the voltage
		printf("(%d)\n",(int)dac_v_bias); // print the integer dac_varac value

		scanf("%s",command);

	}
}

void sweep_vvarac () {
	int16_t dac_v_varac = -450;
	int16_t init_varac_val = 2047;
	int16_t final_varac_val = -2048;
	dac_v_varac = init_varac_val;

	while (1) {
		// clear the DAC output
		// ctrl_out = ctrl_out & (~DAC_CLR) ;
		// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;

		// release the clear pin
		// ctrl_out = ctrl_out | DAC_CLR;
		// alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ) ;

		// write vvarac
		write_vvarac_int(dac_v_varac);

		dac_v_varac -= 100;
		if (dac_v_varac < final_varac_val) {
			dac_v_varac = init_varac_val;
		}
		usleep(4000000);
	}
}

void matlab_vvarac_cnt () {
	double dac_v_varac = 0;
	int i = 0;
	double dot_pos = 10; // put the initial dot_pos to the maximum one, in case the dot is not found, it means we don't have to do the division


	// the command starts with 'c' to continue (followed by +/-, float numbers in ascii representation, and v as voltage), or 's' to stop
	// for example: c+1.23v
	char * command = malloc(sizeof(char) * 20);

	scanf("%s",command);
	while (command[0] == 'c') {
		dac_v_varac = 0;
		dot_pos = 10;

		for (i = 2; command[i]!='v'; i++ ) {
			if (command[i] == '.') {
				dot_pos = i;
			}
			else {
				dac_v_varac *= 10;
				dac_v_varac += (double)(command[i]-48); //convert the ascii to number
			}
		}

		// divide by (10^total_numbers_after_dot)
		for (; i>(dot_pos+1); i--) {
			dac_v_varac /= 10;
		}

		if (command[1] == '-') {
			dac_v_varac = -dac_v_varac;
		}

		// convert to -2048, 2047 scale
		dac_v_varac = (dac_v_varac*2048)/5;
		if (dac_v_varac >= 2048) {
			dac_v_varac = 2047;
		}
		if (dac_v_varac < -2048 ) {
			dac_v_varac = -2048;
		}




		write_vvarac_int( (int16_t)dac_v_varac );
		printf("[%s]",command); // print the voltage
		printf("data: %f",(float)dac_v_varac/2048*5); // print the voltage
		printf("(%d)\n",(int)dac_v_varac); // print the integer dac_varac value

		scanf("%s",command);

	}
}


void write_i2c_rx_gain (uint8_t rx_gain) { // 0 is the least gain, and
	uint8_t i2c_tx_gain_ctl_addr = 0x42;	// i2c address for TCA9555PWR used by the relay
	i2c_tx_gain_ctl_addr >>= 1;				// shift by one because the LSB address is not used as an address (controlled by the Altera I2C IP)

	// reorder the gain bits so that rx_gain is max at 1111 and min at 0000
	// 1111 also means infinite resistance / infinite gain / open circuit. 0000 means all 4 resistors are connected
	uint8_t rx_gain_reorder =
		((rx_gain & 0b00000001)<<4) |
		((rx_gain & 0b00000010)<<4) |
		((rx_gain & 0b00000100)<<5) |
		((rx_gain & 0b00001000)<<3);
	rx_gain_reorder = (~rx_gain_reorder) & 0xF0; // invert the bits so that 0x0F for the input means all the resistance are disconnected (max gain) and 0x00 means all the resistance are connected (min gain)

	alt_write_word( (h2p_i2ccommon_addr+CTRL_OFST), 1<<CORE_EN_SHFT ); // enable i2c core
    
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_tx_gain_ctl_addr<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT0 & I2C_DATA_MSK) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x00) & I2C_DATA_MSK) );				// set port 0 as output
                                                     
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_tx_gain_ctl_addr<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT0 & I2C_DATA_MSK) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | (rx_gain_reorder & I2C_DATA_MSK) );				// set output on port 0

/* PORT 1 is not connected to anything but the P1_0
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (1<<STA_SHFT) | (0<<STO_SHFT) | (i2c_tx_gain_ctl_addr<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_CONF_PORT1 & I2C_DATA_MSK) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((0x0) & I2C_DATA_MSK) );					// set port 1 as output
                                                     
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (i2c_tx_gain_ctl_addr<<AD_SHFT) | (WR_I2C<<RW_D_SHFT) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (0<<STO_SHFT) | (CNT_REG_OUT_PORT1 & I2C_DATA_MSK) );
	alt_write_word((h2p_i2ccommon_addr+TFR_CMD_OFST) , (0<<STA_SHFT) | (1<<STO_SHFT) | ((c_series_reorder) & I2C_DATA_MSK) );	// set output on port 1
*/
}

void sweep_rx_gain () {
	int i_rx_gain = 0;
	while (1) {
		for (i_rx_gain = 0; i_rx_gain<0x10; i_rx_gain++) {
			write_i2c_rx_gain (i_rx_gain);
			printf("current rx_gain_data : %d\n",i_rx_gain);
			usleep(3000000);
		}
	}

}



void CPMG_Sequence (double cpmg_freq, double pulse1_us, double pulse2_us, double pulse1_dtcl, double pulse2_dtcl, double echo_spacing_us, long unsigned scan_spacing_us, unsigned int samples_per_echo, unsigned int echoes_per_scan, double init_adc_delay_compensation, uint32_t ph_cycl_en, char * filename, char * avgname, uint32_t enable_message) {
	unsigned int cpmg_param [5];
	double adc_ltc1746_freq = 25;
	double nmr_fsm_clkfreq = 50;

	// local variables
	uint32_t fifo_mem_level; // the fill level of fifo memory

	// print starting fill memory number (to check the integrity of fifo)
	// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	// printf("fifomem_lvl start: %6d\n",fifo_mem_level);


	// selecting the ADC must be done before resetting the buffer and the adc
	ctrl_out &= (~LTC1746_en); // activate LTC1746
	ctrl_out |= (RX_IN_sel); // activate normal signal path for the receiver
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(100);

	cpmg_param_calculator_ltc1746(cpmg_param, nmr_fsm_clkfreq, (double)adc_ltc1746_freq, (double)init_adc_delay_compensation, pulse1_us, pulse2_us, echo_spacing_us, samples_per_echo);


	alt_write_word( (h2p_pulse1_addr) , cpmg_param[PULSE1_OFFST] );
	alt_write_word( (h2p_delay1_addr) , cpmg_param[DELAY1_OFFST] );
	alt_write_word( (h2p_pulse2_addr) , cpmg_param[PULSE2_OFFST] );
	alt_write_word( (h2p_delay2_addr) , cpmg_param[DELAY2_OFFST] );
	alt_write_word( (h2p_init_adc_delay_addr) , cpmg_param[INIT_DELAY_ADC_OFFST] );
	alt_write_word( (h2p_echo_per_scan_addr) , echoes_per_scan );
	alt_write_word( (h2p_adc_samples_per_echo_addr) , samples_per_echo );

	if (enable_message) {
		printf("CPMG Sequence Actual Parameter:\n");
		printf("\tPulse 1\t\t\t: %7.3f us\n", (double)cpmg_param[PULSE1_OFFST]/nmr_fsm_clkfreq);
		printf("\tDelay 1\t\t\t: %7.3f us\n", (double)cpmg_param[DELAY1_OFFST]/nmr_fsm_clkfreq);
		printf("\tPulse 2\t\t\t: %7.3f us\n", (double)cpmg_param[PULSE2_OFFST]/nmr_fsm_clkfreq);
		printf("\tDelay 2\t\t\t: %7.3f us\n", (double)cpmg_param[DELAY2_OFFST]/nmr_fsm_clkfreq);
		printf("\tADC init delay\t: %7.3f us\n", cpmg_param[INIT_DELAY_ADC_OFFST]/adc_ltc1746_freq);
		if (cpmg_param[INIT_DELAY_ADC_OFFST]/adc_ltc1746_freq == 0) {
			printf("\tWARNING: ADC init delay is 0!");
		}
	}

	// set pll for CPMG
	Set_PLL (h2p_nmr_pll_addr, 0, cpmg_freq, pulse2_dtcl, DISABLE_MESSAGE);
	Set_PLL (h2p_nmr_pll_addr, 1, cpmg_freq, pulse1_dtcl, ENABLE_MESSAGE);
	Set_PLL (h2p_nmr_pll_addr, 2, cpmg_freq, pulse2_dtcl, DISABLE_MESSAGE);
	Set_PLL (h2p_nmr_pll_addr, 3, cpmg_freq, pulse1_dtcl, DISABLE_MESSAGE);
	Reset_PLL (h2p_ctrl_out_addr, PLL_TX_rst_ofst, ctrl_out);
	Set_DPS (h2p_nmr_pll_addr, 0, 0, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 1, 90, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 2, 180, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 3, 270, DISABLE_MESSAGE);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_TX_lock_ofst);

	// print the NMR acquired settings
	sprintf(pathname,"%s/matlab_settings.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n", (double)cpmg_param[PULSE1_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"%f\n", (double)cpmg_param[DELAY1_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"%f\n", (double)cpmg_param[PULSE2_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"%f\n", (double)cpmg_param[DELAY2_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"%f\n", cpmg_param[INIT_DELAY_ADC_OFFST]/adc_ltc1746_freq);
	fprintf(fptr,"%f\n",cpmg_freq);
	fprintf(fptr,"%f\n",pulse1_dtcl);
	fprintf(fptr,"%f\n",pulse2_dtcl);
	fprintf(fptr,"%f\n",echo_spacing_us);
	fprintf(fptr,"%lu\n",scan_spacing_us);
	fprintf(fptr,"%d\n",samples_per_echo);
	fprintf(fptr,"%d\n",echoes_per_scan);
	fclose(fptr);

	sprintf(pathname,"%s/matlab_settings_hr.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"Actual Pulse 1 Length: %f us\n", (double)cpmg_param[PULSE1_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"Actual Delay 1 Length: %f us\n", (double)cpmg_param[DELAY1_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"Actual Pulse 2 Length: %f us\n", (double)cpmg_param[PULSE2_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"Actual Delay 2 Length: %f us\n", (double)cpmg_param[DELAY2_OFFST]/nmr_fsm_clkfreq);
	fprintf(fptr,"Actual Init Delay Length: %f us\n", cpmg_param[INIT_DELAY_ADC_OFFST]/adc_ltc1746_freq);
	fprintf(fptr,"Intended frequency : %f\n",cpmg_freq);
	fprintf(fptr,"Intended pulse 1 duty cycle : %f\n",pulse1_dtcl);
	fprintf(fptr,"Intended pulse 2 duty cycle : %f\n",pulse2_dtcl);
	fprintf(fptr,"Intended echo spacing : %f us\n",echo_spacing_us);
	fprintf(fptr,"Intended scan spacing : %lu us\n",scan_spacing_us);
	fprintf(fptr,"samples_per_echo : %d\n",samples_per_echo);
	fprintf(fptr,"echoes_per_scan : %d\n",echoes_per_scan);
	fclose(fptr);

	// cycle phase for CPMG measurement
	if (ph_cycl_en == ENABLE) {
		if (ctrl_out & (0x01<<PHASE_CYCLING_ofst)) {
			ctrl_out &= ~(0x01<<PHASE_CYCLING_ofst);
		}
		else {
			ctrl_out |= (0x01<<PHASE_CYCLING_ofst);
		}
		alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
		usleep(10);
	}

	// reset the selected ADC
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<ADC_LTC1746_RST_ofst) );
	usleep(10);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<ADC_LTC1746_RST_ofst) );
	usleep(10);


	// reset buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	// start fsm
	// it will reset the pll as well, so it's important to set the phase
	// the pll_rst_dly should be longer than the delay coming from changing the phase
	// otherwise, the fsm will start with wrong relationship between 4 pll output clocks (1/2 pi difference between clock)
	alt_write_word( (h2p_nmr_pll_rst_dly_addr) , 100000 );	// set the amount of delay for pll reset (with 50MHz system clock, every tick means 20ns)
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<FSM_START_ofst) );
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<FSM_START_ofst) );
	// shift the pll phase accordingly
	Set_DPS (h2p_nmr_pll_addr, 0, 0, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 1, 90, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 2, 180, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 3, 270, DISABLE_MESSAGE);
	usleep(scan_spacing_us);

	// wait until fsm stops
	while ( alt_read_word(h2p_ctrl_in_addr) & (0x01<<NMR_SEQ_run_ofst));
	usleep(300);

	fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	for (i=0; fifo_mem_level>0; i++) {			// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically. So, fetch only amount_of_data shifted by 2 to get amount_of_data/2.
		rddata[i] = alt_read_word(h2p_adc_fifo_addr);
		fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
		//usleep(1);
	}
	usleep(100);

	// read the fifo level at the end of acquisition (debugging purpose)
	//fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
	//printf("fifomem_lvl stop: %6d  requested data: %6d  iteration: %ld(*2)\n",fifo_mem_level,samples_per_echo*echoes_per_scan,i);




	if (i*2 == samples_per_echo*echoes_per_scan) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
		// printf("number of captured data vs requested data : MATCHED\n");

		j=0;
		for(i=0; i < ( ((long)samples_per_echo*(long)echoes_per_scan)>>1 ); i++) {
			rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
			rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
		}

		// write the raw data from adc to a file
		sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
		fptr = fopen(pathname, "w");
		if (fptr == NULL) {
			printf("File does not exists \n");
		}
		for(i=0; i < ( ((long)samples_per_echo*(long)echoes_per_scan) ); i++) {
			fprintf(fptr, "%d\n", rddata_16[i]);
		}
		fclose(fptr);


		// write the averaged data to a file
		unsigned int avr_data[samples_per_echo];
		// initialize array
		for (i=0; i<samples_per_echo; i++) {
			avr_data[i] = 0;
		};
		for (i=0; i<samples_per_echo; i++) {
			for (j=i; j<( ((long)samples_per_echo*(long)echoes_per_scan) ); j+=samples_per_echo) {
				avr_data[i] += rddata_16[j];
			}
		}
		sprintf(pathname,"%s/%s",foldername,avgname);	// put the data into the data folder
		fptr = fopen(pathname, "w");
		if (fptr == NULL) {
			printf("File does not exists \n");
		}
		for (i=0; i<samples_per_echo; i++) {
			fprintf(fptr, "%d\n", avr_data[i]);
		}
		fclose(fptr);



	// DON'T DELETE
	//	fprintf(fptr, "CPMG Frequency: %3.3f MHz\n", cpmg_freq);
	//	fprintf(fptr, "Pulse 1 Length: %2.2f us\n", pulse1_us);
	//	fprintf(fptr, "Pulse 2 Length: %2.2f us\n", pulse2_us);
	//	fprintf(fptr, "Pulse 1 Duty Cycle: %1.3f %%\n", pulse1_dtcl);
	//	fprintf(fptr, "Pulse 2 Duty Cycle: %1.3f %%\n", pulse2_dtcl);
	//	fprintf(fptr, "Echotime: %3.2f us\n", echo_spacing_us);
	//	fprintf(fptr, "Total Sample: %d\n", samples_per_echo);
	//	fprintf(fptr, "Capture Amount: %d\n\n", echoes_per_scan);

	}
	else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition




		printf("number of data captured (%ld) and data ordered (%d): NOT MATCHED\nReconfigure the FPGA immediately\n", i*2, samples_per_echo*echoes_per_scan);
	}

}

void tx_sampling(double freq, double tx_num_of_samples, char * filename) {
	alt_write_word( (h2p_init_adc_delay_addr) , 0 );	// don't need adc_delay for sampling the data
	alt_write_word( (h2p_adc_samples_per_echo_addr) , tx_num_of_samples ); // the number of samples taken for tx sampling

	// the bigger is the gain at this stage, the bigger is the impedance. The impedance should be ideally 50ohms which is achieved by using rx_gain between 0x00 and 0x07
	write_i2c_rx_gain (0x00 & 0x0F);	// set the gain of the last stage opamp --> 0x0F is to mask the unused 4 MSBs


	ctrl_out &= (~LTC1746_en); // activate LTC1746
	ctrl_out &= (~RX_IN_sel); // activate signal_coup path for the receiver
	//ctrl_out |= (RX_IN_sel); // activate normal signal path for the receiver
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out ); // write the control signal
	usleep(100);

	// set pll for the tx sampling
	Set_PLL (h2p_nmr_pll_addr, 0, freq, 0.5, DISABLE_MESSAGE);
	Set_PLL (h2p_nmr_pll_addr, 1, freq, 0.5, DISABLE_MESSAGE);
	Set_PLL (h2p_nmr_pll_addr, 2, freq, 0.5, DISABLE_MESSAGE);
	Set_PLL (h2p_nmr_pll_addr, 3, freq, 0.5, DISABLE_MESSAGE);
	Reset_PLL (h2p_ctrl_out_addr, PLL_TX_rst_ofst, ctrl_out);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_TX_lock_ofst);
	Set_DPS (h2p_nmr_pll_addr, 0, 0, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 1, 90, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 2, 180, DISABLE_MESSAGE);
	Set_DPS (h2p_nmr_pll_addr, 3, 270, DISABLE_MESSAGE);
	Wait_PLL_To_Lock (h2p_ctrl_in_addr, PLL_TX_lock_ofst);

	// reset the selected ADC
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out | (0x01<<ADC_LTC1746_RST_ofst) );
	usleep(10);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out & ~(0x01<<ADC_LTC1746_RST_ofst) );
	usleep(10);

	// reset buffer
	ctrl_out |= (0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);
	ctrl_out &= ~(0x01<<ADC_FIFO_RST_ofst);
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10);

	ctrl_out |= NMR_CLK_GATE_AVLN; // activate the clock gate
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(1000);

	// send ADC start pulse signal
	ctrl_out |= ACTIVATE_ADC_AVLN; // this signal is connected to pulser, so it needs to be turned of as quickly as possible after it is turned on
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	ctrl_out &= ~ACTIVATE_ADC_AVLN; // turning off the ADC start signal
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );
	usleep(10000); // delay for data acquisition

	ctrl_out &= ~NMR_CLK_GATE_AVLN; // disactivate the clock gate
	alt_write_word( (h2p_ctrl_out_addr) , ctrl_out );

	uint32_t fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	for (i=0; fifo_mem_level>0; i++) {
		rddata[i] = alt_read_word(h2p_adc_fifo_addr);

		fifo_mem_level--;
		if (fifo_mem_level == 0) {
			fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);
		}
	}
	// usleep(100);
	// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG);

	if (i*2 == tx_num_of_samples) { // if the amount of data captured matched with the amount of data being ordered, then continue the process. if not, then don't process the datas (requesting empty data from the fifo will cause the FPGA to crash, so this one is to avoid that)
		// printf("number of captured data vs requested data : MATCHED\n");

		j=0;
		// FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically.
		for(i=0; i <  ( (long)tx_num_of_samples>>1 ); i++) {
			rddata_16[j++] = (rddata[i] & 0x3FFF);		// 14 significant bit
			rddata_16[j++] = ((rddata[i]>>16)&0x3FFF);	// 14 significant bit
		}

		// write the raw data from adc to a file
		sprintf(pathname,"%s/%s",foldername,filename);	// put the data into the data folder
		fptr = fopen(pathname, "w");
		if (fptr == NULL) {
			printf("File does not exists \n");
		}
		for(i=0; i < ( (long)tx_num_of_samples ); i++) {
			fprintf(fptr, "%d\n", rddata_16[i]);
		}
		fclose(fptr);

	}
	else { // if the amount of data captured didn't match the amount of data being ordered, then something's going on with the acquisition
		printf("number of data captured and data order : NOT MATCHED\nReconfigure the FPGA immediately\n");
	}
}

void CPMG_iterate (
	double cpmg_freq,
	double pulse1_us,
	double pulse2_us,
	double pulse1_dtcl,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	unsigned int number_of_iteration,
	uint32_t ph_cycl_en
){

	create_measurement_folder("nmr");

	// print general measurement settings
	sprintf(pathname,"%s/CPMG_iterate_settings.txt",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"cpmg_freq:\t%f\n", cpmg_freq);
	fprintf(fptr,"pulse1_us:\t%f\n", pulse1_us);
	fprintf(fptr,"pulse2_us:\t%f\n", pulse2_us);
	fprintf(fptr,"pulse1_dtcl:\t%f\n", pulse1_dtcl);
	fprintf(fptr,"pulse2_dtcl:\t%f\n", pulse2_dtcl);
	fprintf(fptr,"echo_spacing_us:\t%f\n", echo_spacing_us);
	fprintf(fptr,"scan_spacing_us:\t%lu\n", scan_spacing_us);
	fprintf(fptr,"samples_per_echo:\t%d\n", samples_per_echo);
	fprintf(fptr,"echoes_per_scan:\t%d\n", echoes_per_scan);
	fprintf(fptr,"init_adc_delay_compensation:\t%f\n", init_adc_delay_compensation);
	fprintf(fptr,"number_of_iteration:\t%d\n", number_of_iteration);
	if (ph_cycl_en == ENABLE) {
		fprintf(fptr,"phase_cycling:\tyes\n");
	}
	else {
		fprintf(fptr,"phase_cycling:\tno\n");
	}
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_iterate([data_folder,'%s'],%d);\n",foldername,number_of_iteration);
	fclose(fptr);

	int iterate = 0;
	char *name;
	char *nameavg;
	for (iterate=0; iterate<=number_of_iteration; iterate++) {
		printf("\n*** RUN %d ***\n",iterate);
		int FILENAME_LENGTH = 100;
		name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		snprintf(name, FILENAME_LENGTH,"dat_%d.o",iterate);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%d.o",iterate);

		CPMG_Sequence (
			(double)cpmg_freq,				//cpmg_freq
			(double)pulse1_us,				//pulse1_us
			(double)pulse2_us,				//pulse2_us
			(double)pulse1_dtcl,			//pulse1_dtcl
			(double)pulse2_dtcl,			//pulse2_dtcl
			(double)echo_spacing_us,		//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			ENABLE_MESSAGE
		);

		free(name);
		free(nameavg);
	}
}

void CPMG_freq_sweep (
	double cpmg_freq_start,
	double cpmg_freq_stop,
	double cpmg_freq_spacing,
	double pulse1_us,
	double pulse2_us,
	double pulse1_dtcl,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	uint32_t ph_cycl_en
){

	create_measurement_folder("nmr_freqsweep");

	// print general measurement settings
	sprintf(pathname,"%s/CPMG_freq_sweep_settings.txt",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"cpmg_freq_start:\t%f\n", cpmg_freq_start);
	fprintf(fptr,"cpmg_freq_stop:\t%f\n", cpmg_freq_stop);
	fprintf(fptr,"cpmg_freq_spacing:\t%f\n", cpmg_freq_spacing);
	fprintf(fptr,"pulse1_us:\t%f\n", pulse1_us);
	fprintf(fptr,"pulse2_us:\t%f\n", pulse2_us);
	fprintf(fptr,"pulse1_dtcl:\t%f\n", pulse1_dtcl);
	fprintf(fptr,"pulse2_dtcl:\t%f\n", pulse2_dtcl);
	fprintf(fptr,"echo_spacing_us:\t%f\n", echo_spacing_us);
	fprintf(fptr,"scan_spacing_us:\t%lu\n", scan_spacing_us);
	fprintf(fptr,"samples_per_echo:\t%d\n", samples_per_echo);
	fprintf(fptr,"echoes_per_scan:\t%d\n", echoes_per_scan);
	fprintf(fptr,"init_adc_delay_compensation:\t%f\n", init_adc_delay_compensation);
	if (ph_cycl_en == ENABLE) {
		fprintf(fptr,"phase_cycling:\tyes\n");
	}
	else {
		fprintf(fptr,"phase_cycling:\tno\n");
	}
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_freqsw([data_folder,'%s'],0);\n",foldername);
	fclose(fptr);

	// print the NMR frequency sweep settings
	sprintf(pathname,"%s/freq_sweep.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n",cpmg_freq_start);
	fprintf(fptr,"%f\n",cpmg_freq_stop);
	fprintf(fptr,"%f\n",cpmg_freq_spacing);
	fclose(fptr);

	double cpmg_freq = 0;
	char *name;
	char *nameavg;
	for (cpmg_freq=cpmg_freq_start; cpmg_freq<=cpmg_freq_stop; cpmg_freq+=cpmg_freq_spacing) {
		printf("\n*** current cpmg_freq %f ***\n",cpmg_freq);
		int FILENAME_LENGTH = 100;
		name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		snprintf(name, FILENAME_LENGTH,"dat_%06.3f.o",cpmg_freq);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%06.3f.o",cpmg_freq);

		CPMG_Sequence (
			(double)cpmg_freq,				//cpmg_freq
			(double)pulse1_us,				//pulse1_us
			(double)pulse2_us,				//pulse2_us
			(double)pulse1_dtcl,			//pulse1_dtcl
			(double)pulse2_dtcl,			//pulse2_dtcl
			(double)echo_spacing_us,		//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			ENABLE_MESSAGE
		);

		free(name);
		free(nameavg);
	}
}

void CPMG_amp_dt1_sweep (
	double pulse1_dtcl_start,
	double pulse1_dtcl_stop,
	double pulse1_dtcl_spacing,
	double cpmg_freq,
	double pulse1_us,
	double pulse2_us,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	uint32_t ph_cycl_en
){

	create_measurement_folder("nmr_amp_dt1_sweep");

	// print general measurement settings
	sprintf(pathname,"%s/CPMG_amp_dt1_sweep_settings.txt",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"cpmg_freq:\t%f\n", cpmg_freq);
	fprintf(fptr,"pulse1_us:\t%f\n", pulse1_us);
	fprintf(fptr,"pulse2_us:\t%f\n", pulse2_us);
	fprintf(fptr,"pulse1_dtcl_start:\t%f\n", pulse1_dtcl_start);
	fprintf(fptr,"pulse1_dtcl_stop:\t%f\n", pulse1_dtcl_stop);
	fprintf(fptr,"pulse1_dtcl_spacing:\t%f\n", pulse1_dtcl_spacing);
	fprintf(fptr,"pulse2_dtcl:\t%f\n", pulse2_dtcl);
	fprintf(fptr,"echo_spacing_us:\t%f\n", echo_spacing_us);
	fprintf(fptr,"scan_spacing_us:\t%lu\n", scan_spacing_us);
	fprintf(fptr,"samples_per_echo:\t%d\n", samples_per_echo);
	fprintf(fptr,"echoes_per_scan:\t%d\n", echoes_per_scan);
	fprintf(fptr,"init_adc_delay_compensation:\t%f\n", init_adc_delay_compensation);
	if (ph_cycl_en == ENABLE) {
		fprintf(fptr,"phase_cycling:\tyes\n");
	}
	else {
		fprintf(fptr,"phase_cycling:\tno\n");
	}
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_dt1sw([data_folder,'%s'],0);\n",foldername);
	fclose(fptr);

	// print the NMR frequency sweep settings
	sprintf(pathname,"%s/dt1_sweep.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n",pulse1_dtcl_start);
	fprintf(fptr,"%f\n",pulse1_dtcl_stop);
	fprintf(fptr,"%f\n",pulse1_dtcl_spacing);
	fclose(fptr);

	double pulse1_dtcl = 0;
	char *name;
	char *nameavg;
	for (pulse1_dtcl=pulse1_dtcl_start; pulse1_dtcl<=pulse1_dtcl_stop; pulse1_dtcl+=pulse1_dtcl_spacing) {
		printf("\n*** current pulse1_dtcl %f ***\n",pulse1_dtcl);
		int FILENAME_LENGTH = 100;
		name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		snprintf(name, FILENAME_LENGTH,"dat_%03.3f.o",pulse1_dtcl);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%03.3f.o",pulse1_dtcl);

		CPMG_Sequence (
			(double)cpmg_freq,				//cpmg_freq
			(double)pulse1_us,				//pulse1_us
			(double)pulse2_us,				//pulse2_us
			(double)pulse1_dtcl,			//pulse1_dtcl
			(double)pulse2_dtcl,			//pulse2_dtcl
			(double)echo_spacing_us,		//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			ENABLE_MESSAGE
		);

		free(name);
		free(nameavg);
	}
}

void CPMG_amp_pulse1_length_sweep (
	double pulse1_us_start,
	double pulse1_us_stop,
	double pulse1_us_spacing,
	double cpmg_freq,
	double pulse1_dtcl,
	double pulse2_us,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	uint32_t ph_cycl_en
){

	create_measurement_folder("nmr_amp_pulse1_length_sweep");

	// print general measurement settings
	sprintf(pathname,"%s/CPMG_amp_pulse1_length_sweep_settings.txt",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"cpmg_freq:\t%f\n", cpmg_freq);
	fprintf(fptr,"pulse1_us_start:\t%f\n", pulse1_us_start);
	fprintf(fptr,"pulse1_us_stop:\t%f\n", pulse1_us_stop);
	fprintf(fptr,"pulse1_us_spacing:\t%f\n", pulse1_us_spacing);
	fprintf(fptr,"pulse2_us:\t%f\n", pulse2_us);
	fprintf(fptr,"pulse1_dtcl:\t%f\n", pulse1_dtcl);
	fprintf(fptr,"pulse2_dtcl:\t%f\n", pulse2_dtcl);
	fprintf(fptr,"echo_spacing_us:\t%f\n", echo_spacing_us);
	fprintf(fptr,"scan_spacing_us:\t%lu\n", scan_spacing_us);
	fprintf(fptr,"samples_per_echo:\t%d\n", samples_per_echo);
	fprintf(fptr,"echoes_per_scan:\t%d\n", echoes_per_scan);
	fprintf(fptr,"init_adc_delay_compensation:\t%f\n", init_adc_delay_compensation);
	if (ph_cycl_en == ENABLE) {
		fprintf(fptr,"phase_cycling:\tyes\n");
	}
	else {
		fprintf(fptr,"phase_cycling:\tno\n");
	}
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_p1lsw([data_folder,'%s'],0);\n",foldername);
	fclose(fptr);

	// print the NMR frequency sweep settings
	sprintf(pathname,"%s/p1_length_sweep.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n",pulse1_us_start);
	fprintf(fptr,"%f\n",pulse1_us_stop);
	fprintf(fptr,"%f\n",pulse1_us_spacing);
	fclose(fptr);

	double pulse1_us = 0;
	char *name;
	char *nameavg;
	for (pulse1_us=pulse1_us_start; pulse1_us<=pulse1_us_stop; pulse1_us+=pulse1_us_spacing) {
		printf("\n*** current pulse1_us %f ***\n",pulse1_us);
		int FILENAME_LENGTH = 100;
		name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		snprintf(name, FILENAME_LENGTH,"dat_%05.3f.o",pulse1_us);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%05.3f.o",pulse1_us);

		CPMG_Sequence (
			(double)cpmg_freq,				//cpmg_freq
			(double)pulse1_us,				//pulse1_us
			(double)pulse2_us,				//pulse2_us
			(double)pulse1_dtcl,			//pulse1_dtcl
			(double)pulse2_dtcl,			//pulse2_dtcl
			(double)echo_spacing_us,		//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			ENABLE_MESSAGE
		);

		free(name);
		free(nameavg);
	}
}

void CPMG_amp_pulse2_length_sweep (
	double pulse2_us_start,
	double pulse2_us_stop,
	double pulse2_us_spacing,
	double cpmg_freq,
	double pulse1_us,
	double pulse1_dtcl,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	uint32_t ph_cycl_en
){

	create_measurement_folder("nmr_amp_pulse2_length_sweep");

	// print general measurement settings
	sprintf(pathname,"%s/CPMG_amp_pulse2_length_sweep_settings.txt",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"cpmg_freq:\t%f\n", cpmg_freq);
	fprintf(fptr,"pulse1_us:\t%f\n", pulse1_us);
	fprintf(fptr,"pulse2_us_start:\t%f\n", pulse2_us_start);
	fprintf(fptr,"pulse2_us_stop:\t%f\n", pulse2_us_stop);
	fprintf(fptr,"pulse2_us_spacing:\t%f\n", pulse2_us_spacing);
	fprintf(fptr,"pulse1_dtcl:\t%f\n", pulse1_dtcl);
	fprintf(fptr,"pulse2_dtcl:\t%f\n", pulse2_dtcl);
	fprintf(fptr,"echo_spacing_us:\t%f\n", echo_spacing_us);
	fprintf(fptr,"scan_spacing_us:\t%lu\n", scan_spacing_us);
	fprintf(fptr,"samples_per_echo:\t%d\n", samples_per_echo);
	fprintf(fptr,"echoes_per_scan:\t%d\n", echoes_per_scan);
	fprintf(fptr,"init_adc_delay_compensation:\t%f\n", init_adc_delay_compensation);
	if (ph_cycl_en == ENABLE) {
		fprintf(fptr,"phase_cycling:\tyes\n");
	}
	else {
		fprintf(fptr,"phase_cycling:\tno\n");
	}
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_p2lsw([data_folder,'%s'],0);\n",foldername);
	fclose(fptr);

	// print the NMR frequency sweep settings
	sprintf(pathname,"%s/p2_length_sweep.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n",pulse2_us_start);
	fprintf(fptr,"%f\n",pulse2_us_stop);
	fprintf(fptr,"%f\n",pulse2_us_spacing);
	fclose(fptr);

	double pulse2_us = 0;
	char *name;
	char *nameavg;
	for (pulse2_us=pulse2_us_start; pulse2_us<=pulse2_us_stop; pulse2_us+=pulse2_us_spacing) {
		printf("\n*** current pulse2_us %f ***\n",pulse2_us);
		int FILENAME_LENGTH = 100;
		name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		snprintf(name, FILENAME_LENGTH,"dat_%05.3f.o",pulse2_us);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%05.3f.o",pulse2_us);

		CPMG_Sequence (
			(double)cpmg_freq,				//cpmg_freq
			(double)pulse1_us,				//pulse1_us
			(double)pulse2_us,				//pulse2_us
			(double)pulse1_dtcl,			//pulse1_dtcl
			(double)pulse2_dtcl,			//pulse2_dtcl
			(double)echo_spacing_us,		//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			ENABLE_MESSAGE
		);

		free(name);
		free(nameavg);
	}
}

void CPMG_amp_length_sweep (
	double pulse_us_start,
	double pulse_us_stop,
	double pulse_us_spacing,
	double cpmg_freq,
	double pulse1_dtcl,
	double pulse2_dtcl,
	double echo_spacing_us,
	long unsigned scan_spacing_us,
	unsigned int samples_per_echo,
	unsigned int echoes_per_scan,
	double init_adc_delay_compensation,
	uint32_t ph_cycl_en
){

	create_measurement_folder("nmr_amp_length_sweep");

	// print general measurement settings
	sprintf(pathname,"%s/CPMG_amp_length_sweep_settings.txt",foldername);
	fptr = fopen(pathname, "a");
	fprintf(fptr,"cpmg_freq:\t%f\n", cpmg_freq);
	fprintf(fptr,"pulse_us_start:\t%f\n", pulse_us_start);
	fprintf(fptr,"pulse_us_stop:\t%f\n", pulse_us_stop);
	fprintf(fptr,"pulse_us_spacing:\t%f\n", pulse_us_spacing);
	fprintf(fptr,"pulse1_dtcl:\t%f\n", pulse1_dtcl);
	fprintf(fptr,"pulse2_dtcl:\t%f\n", pulse2_dtcl);
	fprintf(fptr,"echo_spacing_us:\t%f\n", echo_spacing_us);
	fprintf(fptr,"scan_spacing_us:\t%lu\n", scan_spacing_us);
	fprintf(fptr,"samples_per_echo:\t%d\n", samples_per_echo);
	fprintf(fptr,"echoes_per_scan:\t%d\n", echoes_per_scan);
	fprintf(fptr,"init_adc_delay_compensation:\t%f\n", init_adc_delay_compensation);
	if (ph_cycl_en == ENABLE) {
		fprintf(fptr,"phase_cycling:\tyes\n");
	}
	else {
		fprintf(fptr,"phase_cycling:\tno\n");
	}
	fclose(fptr);

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"compute_plsw([data_folder,'%s'],0);\n",foldername);
	fclose(fptr);

	// print the NMR frequency sweep settings
	sprintf(pathname,"%s/p_length_sweep.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n",pulse_us_start);
	fprintf(fptr,"%f\n",pulse_us_stop);
	fprintf(fptr,"%f\n",pulse_us_spacing);
	fclose(fptr);

	double pulse_us = 0;
	char *name;
	char *nameavg;
	for (pulse_us=pulse_us_start; pulse_us<=pulse_us_stop; pulse_us+=pulse_us_spacing) {
		printf("\n*** current pulse2_us %f ***\n",pulse_us);
		int FILENAME_LENGTH = 100;
		name = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		nameavg = (char*) malloc (FILENAME_LENGTH*sizeof(char));
		snprintf(name, FILENAME_LENGTH,"dat_%05.3f.o",pulse_us);
		snprintf(nameavg, FILENAME_LENGTH,"avg_%05.3f.o",pulse_us);

		CPMG_Sequence (
			(double)cpmg_freq,				//cpmg_freq
			(double)pulse_us,				//pulse1_us
			(double)pulse_us*1.6,			//pulse2_us
			(double)pulse1_dtcl,			//pulse1_dtcl
			(double)pulse2_dtcl,			//pulse2_dtcl
			(double)echo_spacing_us,		//echo_spacing_us
			scan_spacing_us,				//scan_spacing_us
			samples_per_echo,				//samples_per_echo
			echoes_per_scan,				//echoes_per_scan
			init_adc_delay_compensation,	//compensation delay number (counted by the adc base clock)
			ph_cycl_en,						//phase cycle enable/disable
			name,							//filename for data
			nameavg,						//filename for average data
			ENABLE_MESSAGE
		);

		free(name);
		free(nameavg);
	}
}

void wobble_function (double startfreq, double stopfreq, double spacfreq, unsigned int wobb_samples) {

	create_measurement_folder("nmr_wobb");

	// print matlab script to analyze datas
	sprintf(pathname,"measurement_history_matlab_script.txt");
	fptr = fopen(pathname, "a");
	fprintf(fptr,"wobble_plot([data_folder,'%s']);\n",foldername);
	fclose(fptr);

	// print the NMR acquired settings
	sprintf(pathname,"%s/matlab_settings.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"%f\n", startfreq);
	fprintf(fptr,"%f\n", stopfreq);
	fprintf(fptr,"%f\n", spacfreq);
	fprintf(fptr,"%d\n", wobb_samples);
	fclose(fptr);

	sprintf(pathname,"%s/readable_settings.txt",foldername);	// put the data into the data folder
	fptr = fopen(pathname, "a");
	fprintf(fptr,"Start frequency: %f\n", startfreq);
	fprintf(fptr,"Stop frequency: %f\n", stopfreq);
	fprintf(fptr,"Spacing frequency: %f\n", spacfreq);
	fprintf(fptr,"Number of samples: %d\n", wobb_samples);
	fclose(fptr);

	char * wobbname;
	double wobbfreq = 0;
	wobbname = (char*) malloc (100*sizeof(char));
	for (wobbfreq = startfreq; wobbfreq <= stopfreq; wobbfreq += spacfreq) {
		snprintf(wobbname, 100,"wobbdata_%2.2f.o",wobbfreq);
		tx_sampling(wobbfreq, wobb_samples, wobbname);
		// usleep(100);		// this delay is necessary. If it's not here, the system will not crash but the i2c will stop working (?), and the reading length is incorrect
	}
}

void test_leds_and_switches () {
    // initialize gpio for the hps
    setup_hps_gpio();

    // switch on first led
    setup_fpga_leds();


    while (true) {
        handle_hps_led();
        handle_fpga_leds();
        printf("%d\n",alt_read_word(fpga_switches));
        usleep(ALT_MICROSECS_IN_A_SEC / 10);
    }
}

void init_default_system_param() {
	ctrl_out &= ~(_15V_LP_en);							// enable the low power 15V power
	alt_write_word(h2p_ctrl_out_addr, ctrl_out);		// write down the control

	// write_i2c_relay_cnt(115,150);						// the default number for oldest coil sensor
	write_i2c_relay_cnt(24,50);							// the default number for coil sensor orange

	init_dac_ad5722r();									// power up the dac and init its operation
	write_vbias(-3.174);								// default number for vbias is -3.174V (reflection at -11dB)
	write_vvarac(-1.24);								// the default number for v_varactor is between -1.1V and -1.35V
	usleep(500000);										// wait for the v_varac & v_bias to settle down

	write_i2c_rx_gain (0x00 & 0x0F);					// 0x0F is to mask the unused 4 MSBs

}

int main() {
    printf("NMR system start\n");

    // open device file of the memory
    open_physical_memory_device();

    // map hps and fpga memory
    mmap_peripherals();

    // initialize parameters needed by the fpga and peripherals
    init_default_system_param();

    // TEST COMMAND
	// sweep_matching_network();
	// test_dac_ad5722r();
    // sweep_vbias(); // vbias cannot be more than about 0.8v due to transistor breakdown, I guess
	// sweep_vvarac();
	// sweep_rx_gain();
    // test_leds_and_switches();

    // HARDWARE CHARACTERIZATION COMMAND (activate only 1 at a time and run the corresponding matlab code)
    // matlab_vvarac_cnt(); // matlab code: agilent_4395a_preamp_characterization.m
    // matlab_vbias_cnt(); // matlab code: agilent_4395a_preamp_characterization.m or agilent_4395a_impedance_characterization.m
    // matlab_mtching_ntwrk_cnt(); // matlab code: agilent_4395a_mtching_reflection_characterization.m



	/* CPMG WOBBLE
	// write_i2c_relay_cnt(24,50);			// the default number for matching network capacitance
    double startfreq 			= 3.0;		// wobble start frequency in MHz
	double stopfreq 			= 5.0;		// wobble stop frequency in MHz
	double spacfreq 			= 0.02;		// wobble frequency spacing in MHz
	unsigned int wobb_samples 	= 1024*128;	// the number of ADC samples taken
    wobble_function (
    		startfreq,
			stopfreq,
			spacfreq,
			wobb_samples
	);



    /* CPMG ITERATE
	double cpmg_freq = 4.3;
	double pulse1_us = 5;
	double pulse2_us = 8;
	double pulse1_dtcl = 0.3;
	double pulse2_dtcl = 0.3;
	double echo_spacing_us = 80;
	long unsigned scan_spacing_us = 500000; // 4000000
	unsigned int samples_per_echo = 512; //2048
	unsigned int echoes_per_scan = 256; //64
	double init_adc_delay_compensation = 7;	// the delay to compensate signal shifting due to path delay in the AFE, in microseconds
	unsigned int number_of_iteration = 10;
	uint32_t ph_cycl_en = ENABLE;
	CPMG_iterate (
		cpmg_freq,
		pulse1_us,
		pulse2_us,
		pulse1_dtcl,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		number_of_iteration,
		ph_cycl_en
	);
	*/


    /* FREQUENCY SWEEP
	double cpmg_freq_start = 3.5;
	double cpmg_freq_stop = 5;
	double cpmg_freq_spacing = 0.005; // the minimum is 20kHz spacing or 0.02. Don't know why it can't go less than that. The signal seems to be gone if it's less than that. The transmitter signal looks fine, but the signal received looks nothing.
	double pulse1_us = 5;
	double pulse2_us = 8;
	double pulse1_dtcl = 0.3;
	double pulse2_dtcl = 0.3;
	double echo_spacing_us = 80;
	long unsigned scan_spacing_us = 250000;
	unsigned int samples_per_echo = 512;
	unsigned int echoes_per_scan = 256;
	double init_adc_delay_compensation = 6;	// the delay to compensate signal shifting due to path delay in the AFE, in microseconds
	uint32_t ph_cycl_en = DISABLE;
	CPMG_freq_sweep (
		cpmg_freq_start,
		cpmg_freq_stop,
		cpmg_freq_spacing,
		pulse1_us,
		pulse2_us,
		pulse1_dtcl,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		ph_cycl_en
	);
	*/



    /* PULSE1 DTCL SWEEP
	double pulse1_dtcl_start = 0.01;
	double pulse1_dtcl_stop = 0.40;
	double pulse1_dtcl_spacing = 0.001;
	double cpmg_freq = 4.3;
	double pulse1_us = 5;
	double pulse2_us = 8;
	double pulse2_dtcl = 0.3;
	double echo_spacing_us = 200;
	long unsigned scan_spacing_us = 4000000;
	unsigned int samples_per_echo = 512;
	unsigned int echoes_per_scan = 256;
	double init_adc_delay_compensation = 6;	// the delay to compensate signal shifting due to path delay in the AFE, in microseconds
	uint32_t ph_cycl_en = DISABLE;
	CPMG_amp_dt1_sweep (
		pulse1_dtcl_start,
		pulse1_dtcl_stop,
		pulse1_dtcl_spacing,
		cpmg_freq,
		pulse1_us,
		pulse2_us,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		ph_cycl_en
	);
	*/


    /* PULSE1 LENGTH SWEEP
	double pulse1_us_start = 1;
	double pulse1_us_stop = 10;
	double pulse1_us_spacing = 0.1;
	double cpmg_freq = 4.3;
	double pulse1_dtcl = 0.3;
	double pulse2_us = 8;
	double pulse2_dtcl = 0.3;
	double echo_spacing_us = 80;
	long unsigned scan_spacing_us = 500000;
	unsigned int samples_per_echo = 512;
	unsigned int echoes_per_scan = 256;
	double init_adc_delay_compensation = 6;	// the delay to compensate signal shifting due to path delay in the AFE, in microseconds
	uint32_t ph_cycl_en = DISABLE;
	CPMG_amp_pulse1_length_sweep (
		pulse1_us_start,
		pulse1_us_stop,
		pulse1_us_spacing,
		cpmg_freq,
		pulse1_dtcl,
		pulse2_us,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		ph_cycl_en
	);
	*/



    // PULSE2 LENGTH SWEEP
	double pulse2_us_start = 5;
	double pulse2_us_stop = 15;
	double pulse2_us_spacing = 0.1;
	double cpmg_freq = 4.3;
	double pulse1_dtcl = 0.3;
	double pulse1_us = 5;
	double pulse2_dtcl = 0.28;
	double echo_spacing_us = 80;
	long unsigned scan_spacing_us = 500000;
	unsigned int samples_per_echo = 512;
	unsigned int echoes_per_scan = 256;
	double init_adc_delay_compensation = 6;	// the delay to compensate signal shifting due to path delay in the AFE, in microseconds
	uint32_t ph_cycl_en = DISABLE;
	CPMG_amp_pulse2_length_sweep (
		pulse2_us_start,
		pulse2_us_stop,
		pulse2_us_spacing,
		cpmg_freq,
		pulse1_us,
		pulse1_dtcl,
		pulse2_dtcl,
		echo_spacing_us,
		scan_spacing_us,
		samples_per_echo,
		echoes_per_scan,
		init_adc_delay_compensation,
		ph_cycl_en
	);



    /* CPMG PULSE LENGTH SWEEP (1.6 fixed ratio)
   	double pulse_us_start = 1;
   	double pulse_us_stop = 20;
   	double pulse_us_spacing = 0.2;
   	double cpmg_freq = 4.3;
   	double pulse1_dtcl = 0.3;
   	double pulse2_dtcl = 0.3;
   	double echo_spacing_us = 100;
   	long unsigned scan_spacing_us = 500000;
   	unsigned int samples_per_echo = 512;
   	unsigned int echoes_per_scan = 256;
   	double init_adc_delay_compensation = 6;	// the delay to compensate signal shifting due to path delay in the AFE, in microseconds
   	uint32_t ph_cycl_en = DISABLE;
   	CPMG_amp_length_sweep (
   		pulse_us_start,
   		pulse_us_stop,
   		pulse_us_spacing,
   		cpmg_freq,
   		pulse1_dtcl,
   		pulse2_dtcl,
   		echo_spacing_us,
   		scan_spacing_us,
   		samples_per_echo,
   		echoes_per_scan,
   		init_adc_delay_compensation,
   		ph_cycl_en
   	);
	*/



    // unmap hps and fpga memory
    munmap_peripherals();

    // close device file of the memory
    close_physical_memory_device();

    return 0;
}
