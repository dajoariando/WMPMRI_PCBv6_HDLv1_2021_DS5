// Output control signal to FPGA
#define ACTIVATE_ADC_AVLN	(1<<14)
#define NMR_CLK_GATE_AVLN	(1<<13)
#define ADC_LTC1746_RST		(1<<12)
#define ADC_LTC1746_START	(1<<11)
#define ADC_FIFO_RST		(1<<10)
#define FSM_START			(1<<9)
#define PHASE_CYCLING		(1<<8)
#define PLL_TX_rst			(1<<7)
#define DAC_LDAC_en			(1<<6)
#define DAC_CLR				(1<<5)
#define LTC1746_en			(1<<4)
#define LT1210_tx_en		(1<<3)
#define _15V_HP_en			(1<<2)
#define _15V_LP_en			(1<<1)
#define RX_IN_sel			(1<<0)
#define CNT_OUT_default		0x77
// offsets for output control signal
#define ACTIVATE_ADC_AVLN_ofst		(14)
#define NMR_CLK_GATE_AVLN_ofst		(13)
#define ADC_LTC1746_RST_ofst		(12)
#define ADC_LTC1746_START_ofst		(11)
#define ADC_FIFO_RST_ofst			(10)
#define FSM_START_ofst				(9)
#define PHASE_CYCLING_ofst			(8)
#define PLL_TX_rst_ofst				(7)
#define DAC_LDAC_en_ofst			(6)
#define DAC_CLR_ofst				(5)
#define LTC1746_en_ofst				(4)
#define LT1210_tx_en_ofst			(3)
#define _15V_HP_en_ofst				(2)
#define _15V_LP_en_ofst				(1)
#define RX_IN_sel_ofst				(0)



// Input status signal from FPGA
#define NMR_SEQ_run				(1<<1)
#define PLL_TX_lock				(1<<0)
// Offsets for input status signal
#define NMR_SEQ_run_ofst			(1)
#define PLL_TX_lock_ofst			(0)



// general variable
#define ENABLE_MESSAGE	1
#define DISABLE_MESSAGE 0
#define ENABLE 1
#define DISABLE 0

#define SEL_ADC1746		1
