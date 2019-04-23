// offsets for output control signal
#define NMR_CNT_RESET_ofst			(9)
#define PLL_ANALYZER_RST_ofst		(8)
#define PLL_NMR_SYS_RST_ofst		(7)
#define ACTIVATE_ADC_AVLN_ofst		(6)
#define NMR_CLK_GATE_AVLN_ofst		(5)
#define ADC_FIFO_RST_ofst			(4)
#define FSM_START_ofst				(3)
#define PHASE_CYCLING_ofst			(2)
#define DAC_LDAC_en_ofst			(1)
#define DAC_CLR_ofst				(0)

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
#define CNT_OUT_default		(DAC_CLR | NMR_CLK_GATE_AVLN)

// Output control signal to FPGA via I2C
#define PSU_15V_TX_P_EN_ofst	(0)
#define PSU_15V_TX_N_EN_ofst	(1)
#define AMP_HP_LT1210_EN_ofst	(2)
#define PSU_5V_TX_N_EN_ofst		(3)
#define PAMP_IN_SEL_TEST_ofst	(4)
#define PAMP_IN_SEL_RX_ofst		(5)
#define GPIO_GEN_PURP_1_ofst	(6)
#define PSU_5V_ADC_EN_ofst		(7)
#define RX_AMP_GAIN_2_ofst		(8)
#define RX_AMP_GAIN_1_ofst		(9)
#define RX_AMP_GAIN_4_ofst		(10)
#define RX_AMP_GAIN_3_ofst		(11)
#define RX_IN_SEL_1_ofst		(12)
#define RX_IN_SEL_2_ofst		(13)
#define PSU_5V_ANA_P_EN_ofst	(14)
#define PSU_5V_ANA_N_EN_ofst	(15)

#define PSU_15V_TX_P_EN_msk		(1<<PSU_15V_TX_P_EN_ofst)
#define PSU_15V_TX_N_EN_msk		(1<<PSU_15V_TX_N_EN_ofst)
#define AMP_HP_LT1210_EN_msk	(1<<AMP_HP_LT1210_EN_ofst)
#define PSU_5V_TX_N_EN_msk		(1<<PSU_5V_TX_N_EN_ofst)
#define PAMP_IN_SEL_TEST_msk	(1<<PAMP_IN_SEL_TEST_ofst)
#define PAMP_IN_SEL_RX_msk		(1<<PAMP_IN_SEL_RX_ofst)
#define GPIO_GEN_PURP_1_msk		(1<<GPIO_GEN_PURP_1_ofst)
#define PSU_5V_ADC_EN_msk		(1<<PSU_5V_ADC_EN_ofst)
#define RX_AMP_GAIN_2_msk		(1<<RX_AMP_GAIN_2_ofst)
#define RX_AMP_GAIN_1_msk		(1<<RX_AMP_GAIN_1_ofst)
#define RX_AMP_GAIN_4_msk		(1<<RX_AMP_GAIN_4_ofst)
#define RX_AMP_GAIN_3_msk		(1<<RX_AMP_GAIN_3_ofst)
#define RX_IN_SEL_1_msk			(1<<RX_IN_SEL_1_ofst)
#define RX_IN_SEL_2_msk			(1<<RX_IN_SEL_2_ofst)
#define PSU_5V_ANA_P_EN_msk		(1<<PSU_5V_ANA_P_EN_ofst)
#define PSU_5V_ANA_N_EN_msk		(1<<PSU_5V_ANA_N_EN_ofst)

#define CNT_I2C_default		0b0000000000000000



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
