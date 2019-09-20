#ifndef HPS_LINUX_H_
#define HPS_LINUX_H_

#include <stdbool.h>
#include <stdint.h>

#include <socal/hps.h>
#include "functions/general.h"
#include "hps_soc_system.h"

#define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) // Bridge span
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )

// |=============|==========|==============|==========|
// | Signal Name | HPS GPIO | Register/bit | Function |
// |=============|==========|==============|==========|
// |   HPS_LED   |  GPIO53  |   GPIO1[24]  |    I/O   |
// |=============|==========|==============|==========|
#define HPS_LED_IDX      (ALT_GPIO_1BIT_53)                      // GPIO53
#define HPS_LED_PORT     (alt_gpio_bit_to_pid(HPS_LED_IDX))      // ALT_GPIO_PORTB
#define HPS_LED_PORT_BIT (alt_gpio_bit_to_port_pin(HPS_LED_IDX)) // 24 (from GPIO1[24])
#define HPS_LED_MASK     (1 << HPS_LED_PORT_BIT)

// |=============|==========|==============|==========|
// | Signal Name | HPS GPIO | Register/bit | Function |
// |=============|==========|==============|==========|
// |  HPS_KEY_N  |  GPIO54  |   GPIO1[25]  |    I/O   |
// |=============|==========|==============|==========|
#define HPS_KEY_N_IDX      (ALT_GPIO_1BIT_54)                        // GPIO54
#define HPS_KEY_N_PORT     (alt_gpio_bit_to_pid(HPS_KEY_N_IDX))      // ALT_GPIO_PORTB
#define HPS_KEY_N_PORT_BIT (alt_gpio_bit_to_port_pin(HPS_KEY_N_IDX)) // 25 (from GPIO1[25])
#define HPS_KEY_N_MASK     (1 << HPS_KEY_N_PORT_BIT)

// physical memory file descriptor
int fd_dev_mem = 0;

// memory-mapped peripherals
void   *hps_gpio     = NULL;
size_t hps_gpio_span = ALT_GPIO1_UB_ADDR - ALT_GPIO1_LB_ADDR + 1;
size_t hps_gpio_ofst = ALT_GPIO1_OFST;

void   *h2f_lw_axi_master     = NULL;
size_t h2f_lw_axi_master_span = ALT_LWFPGASLVS_UB_ADDR - ALT_LWFPGASLVS_LB_ADDR + 1;
size_t h2f_lw_axi_master_ofst = ALT_LWFPGASLVS_OFST;


void *h2f_axi_master = NULL;
size_t h2f_axi_master_span = HW_FPGA_AXI_SPAN;
size_t h2f_axi_master_ofst = ALT_AXI_FPGASLVS_OFST;

void *fpga_leds = NULL;
void *fpga_switches = NULL;

// general input / output fsm control addresses
void *h2p_ctrl_out_addr					= NULL; // control output signal for NMR FSM
void *h2p_ctrl_in_addr					= NULL; // control input signal for NMR FSM

// general i/o addresses
// volatile unsigned int is used when computing the offset address is needed
// for example, if the address is pointing to 0, *(volatile unsigned int + 1) will result in address of 4 (because one integer uses 32 bits or 4 bytes)
// while *(void+1) will result in address 1, which is incorrect
// this is due to the system in qsys usually uses byte addresses instead of word addresses. With one word is usually 4 bytes or 32 bits
void *h2p_adcdata_addr							= NULL; // gpio for adc high speed
void *h2p_led_addr								= NULL; // gpio for LEDs
volatile unsigned long *h2p_i2c_ext_addr		= NULL; // gpio for i2c (used for relay control through io expander chip TCA9555PWR, and also rx gain selector)
volatile unsigned long *h2p_i2c_int_addr		= NULL; // gpio for i2c (used for relay control through io expander chip TCA9555PWR, and also rx gain selector)
volatile unsigned int *h2p_dac_addr				= NULL; // gpio for dac (spi)
volatile unsigned int *h2p_spi_mtch_ntwrk_addr	= NULL; // spi for matching network control (PCB v4 onwards)
volatile unsigned int *h2p_spi_afe_relays_addr	= NULL; // spi for afe relays or preamp
// void *h2p_i2c_ext_addr		= NULL; // gpio for i2c (used for relay control through io expander chip TCA9555PWR, and also rx gain selector)
// void *h2p_dac_addr			= NULL; // gpio for dac (spi)


// pll reconfig address for NMR transmitter
// void *h2p_nmr_pll_addr 							= NULL; // PLL reconfiguration control for the NMR transmitter

// pll reconfig address for the nmr system
void *h2p_nmr_sys_pll_addr					= NULL; // nmr system pll reconfiguration

// pll reconfig address for the analyzer / hardware characterizer
void *h2p_analyzer_pll_addr					= NULL;

// NMR sequence fsm parameter addresses
void *h2p_pulse1_addr						= NULL; // 90-deg length
void *h2p_pulse2_addr						= NULL; // 180-deg length
void *h2p_delay1_addr						= NULL; // delay length after 90-deg signal
void *h2p_delay2_addr						= NULL; // delay length after 180-deg signal
void *h2p_pulse_t1_addr						= NULL; // pulse t1 length
void *h2p_delay_t1_addr						= NULL; // delay t1 length
void *h2p_echo_per_scan_addr 				= NULL; // the amount of echoes on 1 NMR scan
void *h2p_t1_pulse							= NULL; // the pulse length before CPMG (T1 measurement)
void *h2p_t1_delay							= NULL; // the delay length before CPMG (T1 measurement)
void *h2p_adc_val_sub						= NULL; // the zero bias ADC voltage for downconversion DC bias removal

volatile unsigned int *h2p_dmadummy_addr				= NULL;
volatile unsigned int *h2p_fifoin_dummy_addr			= NULL;
volatile unsigned int *h2p_fifoout_dummy_addr			= NULL;
volatile unsigned int *h2p_fifoincsr_dummy_addr			= NULL;
volatile unsigned int *h2p_fifoin64dummy_addr			= NULL;
volatile unsigned int *h2p_fifoout64dummy_addr			= NULL;
volatile unsigned int *h2p_fifoout64csrdummy_addr		= NULL;


// adc addresses
void *h2p_adc_fifo_addr						= NULL; // ADC FIFO output data address
// void *h2p_adc_fifo_status_addr			= NULL; // ADC FIFO status address
// void *h2p_adc_str_fifo_status_addr		= NULL; // ADC streaming FIFO status address
volatile unsigned int *h2p_adc_fifo_status_addr		= NULL; // ADC FIFO status address
volatile unsigned int *h2p_adc_str_fifo_status_addr	= NULL; // ADC streaming FIFO status address
void *h2p_adc_samples_per_echo_addr						= NULL; // The number of ADC capture per echo
void *h2p_init_adc_delay_addr							= NULL; // The cycle number for delay in an echo after pulse 180 is done. The idea is to put adc capture in the middle of echo window and giving some freedom to move the ADC capture window within the echo window
void *h2p_rx_delay_addr							= NULL; // generated delay for rx enable in Jarred's broadband board
void *h2p_switches_addr						= NULL;

void 					*h2p_dconvi_addr 		= NULL; // downconverted data i fifo
void 					*h2p_dconvq_addr 		= NULL; // downconverted data q fifo
volatile unsigned int	*h2p_dconvi_csr_addr	= NULL; // downconverted data i fifo status reg
volatile unsigned int	*h2p_dconvq_csr_addr	= NULL; // downconverted data q fifo status reg

// DMA & SDRAM
volatile unsigned int *h2p_dma_addr = NULL;
volatile unsigned int *h2p_dconvi_dma_addr = NULL;
volatile unsigned int *h2p_dconvq_dma_addr = NULL;
volatile unsigned int *h2p_sdram_addr = NULL;


void open_physical_memory_device();
void close_physical_memory_device();
void mmap_hps_peripherals();
void munmap_hps_peripherals();
void mmap_fpga_peripherals();
void munmap_fpga_peripherals();
void mmap_peripherals();
void munmap_peripherals();
void setup_hps_gpio();
void setup_fpga_leds();
void handle_hps_led();
void handle_fpga_leds();

// FUNCTIONS
void create_measurement_folder();								// create a folder in the system for the measurement data
int exit_program();												// terminate the program
void init_dac_ad5722r ();
void print_warning_ad5722r();
void init_default_system_param();								// initialize the system with tuned default parameter
void write_i2c_relay_cnt (uint16_t c_shunt, uint16_t c_series, uint8_t en_mesg);	// program the capacitance with the relay
void write_i2c_cnt (uint32_t en, uint32_t addr_msk, uint8_t en_mesg);
void write_i2c_rx_gain (uint8_t rx_gain);						// program the final receiver gain. 0x00 means minimum gain (not really zero gain) and 0x0F means max gain (infinite/open circuit). So effective value is 0x00 to 0x0E
void sweep_matching_network();									// sweep the capacitance in matching network by sweeping the relay (FOREVER LOOP)
void write_vbias (double vbias);								// write vbias as voltage format
void write_vvarac (double vvarac);								// write vvarac as voltage format
void write_vbias_int(int16_t dac_v_bias);						// write vbias with a value
void sweep_vbias();												// sweep the vbias
void write_vvarac_int(int16_t dac_v_varac);						// write v_varactor with a value
void sweep_vvarac ();											// sweep the v_varactor
void sweep_rx_gain ();										// sweep the rx gain (FOREVER LOOP)
void fifo_to_sdram_dma_trf (volatile unsigned int * dma_addr, uint32_t rd_addr, uint32_t wr_addr, uint32_t transfer_length);
void datawrite_with_dma (uint32_t transfer_length, uint8_t en_mesg);
void close_system ();
void CPMG_Sequence (
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
	uint32_t ph_cycl_en,
	char * filename,
	char * avgname,
	uint32_t enable_message
);
void tx_sampling(double tx_freq, double sampfreq, unsigned int samples_per_echo, char * filename);


// global variables
FILE	*fptr;
long i;
long j;


unsigned int *rddata;
unsigned int *rddata_16;
int *dconvi;
int *dconvq;

// unsigned int rddata [128000];
// unsigned int rddata_16[256000];
//unsigned int dconvi[64000]; // downconverted i data
//unsigned int dconvq[64000]; // downconverted q data

char foldername[50]; // variable to store folder name of the measurement data
char pathname[60];

int dconv_fact = 1; // downconversion factor, see Qsys FIR filter

// FPGA control signal address
uint32_t ctrl_out = CNT_OUT_default;					// default variable to store the current control state
uint32_t ctrl_i2c0 = CNT_I2C_default;
uint32_t ctrl_i2c1 = CNT_I2C_default;

#endif
