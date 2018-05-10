// offsets for output control signal
#define NMR_CNT_RESET_ofst			(14)
#define PLL_ANALYZER_RST_ofst		(13)
#define PLL_NMR_SYS_RST_ofst		(12)
#define ACTIVATE_ADC_AVLN_ofst		(11)
#define NMR_CLK_GATE_AVLN_ofst		(10)
#define ADC_FIFO_RST_ofst			(9)
#define FSM_START_ofst				(8)
#define PHASE_CYCLING_ofst			(7)
#define DAC_LDAC_en_ofst			(6)
#define DAC_CLR_ofst				(5)
#define LTC1746_en_ofst				(4)
#define LT1210_tx_en_ofst			(3)
#define _15V_HP_en_ofst				(2)
#define _15V_LP_en_ofst				(1)
#define RX_IN_sel_ofst				(0)

// Output control signal to FPGA
#define NMR_CNT_RESET		(1<<NMR_CNT_RESET_ofst)
#define PLL_ANALYZER_RST	(1<<PLL_ANALYZER_RST_ofst)
#define PLL_NMR_SYS_RST		(1<<PLL_NMR_SYS_RST_ofst)
#define ACTIVATE_ADC_AVLN	(1<<ACTIVATE_ADC_AVLN_ofst)
#define NMR_CLK_GATE_AVLN	(1<<NMR_CLK_GATE_AVLN_ofst)
#define ADC_FIFO_RST		(1<<ADC_FIFO_RST_ofst)
#define FSM_START			(1<<FSM_START_ofst)
#define PHASE_CYCLING		(1<<PHASE_CYCLING_ofst)
#define DAC_LDAC_en			(1<<DAC_LDAC_en_ofst)
#define DAC_CLR				(1<<DAC_CLR_ofst)
#define LTC1746_en			(1<<LTC1746_en_ofst)
#define LT1210_tx_en		(1<<LT1210_tx_en_ofst)
#define _15V_HP_en			(1<<_15V_HP_en_ofst)
#define _15V_LP_en			(1<<_15V_LP_en_ofst)
#define RX_IN_sel			(1<<RX_IN_sel_ofst)
#define CNT_OUT_default		0b01010011



// Offsets for input status signal
#define PLL_ANALYZER_lock_ofst		(2)
#define NMR_SEQ_run_ofst			(1)
#define PLL_NMR_SYS_lock_ofst		(0)
// Input status signal from FPGA
#define PLL_ANALYZER_lock			(1<<PLL_ANALYZER_lock_ofst)
#define NMR_SEQ_run					(1<<NMR_SEQ_run_ofst)
#define PLL_NMR_SYS_lock			(1<<PLL_NMR_SYS_lock_ofst)




// general variable
#define ENABLE_MESSAGE	1
#define DISABLE_MESSAGE 0
#define ENABLE 1
#define DISABLE 0

// RX signal path
#define SIG_NORM_PATH	0
#define SIG_S11_PATH	1

#define SEL_ADC1746		1
