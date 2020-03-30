// offsets for output control signal
#define DCONV_FIR_RST_RESET_N_ofst		(11)
#define DCONV_FIR_Q_RST_RESET_N_ofst	(10)
#define NMR_CNT_RESET_ofst				(9)
#define PLL_ANALYZER_RST_ofst			(8)
#define PLL_NMR_SYS_RST_ofst			(7)
#define ACTIVATE_ADC_AVLN_ofst			(6)
#define NMR_CLK_GATE_AVLN_ofst			(5)
#define ADC_FIFO_RST_ofst				(4)
#define FSM_START_ofst					(3)
#define PHASE_CYCLING_ofst				(2)
#define DAC_LDAC_en_ofst				(1)
#define DAC_CLR_ofst					(0)

// Output control signal to FPGA
#define DCONV_FIR_RST_RESET_N		(1<<DCONV_FIR_RST_RESET_N_ofst)
#define DCONV_FIR_Q_RST_RESET_N		(1<<DCONV_FIR_Q_RST_RESET_N_ofst)
#define NMR_CNT_RESET				(1<<NMR_CNT_RESET_ofst)
#define PLL_ANALYZER_RST			(1<<PLL_ANALYZER_RST_ofst)
#define PLL_NMR_SYS_RST				(1<<PLL_NMR_SYS_RST_ofst)
#define ACTIVATE_ADC_AVLN			(1<<ACTIVATE_ADC_AVLN_ofst)
#define NMR_CLK_GATE_AVLN			(1<<NMR_CLK_GATE_AVLN_ofst)
#define ADC_FIFO_RST				(1<<ADC_FIFO_RST_ofst)
#define FSM_START					(1<<FSM_START_ofst)
#define PHASE_CYCLING				(1<<PHASE_CYCLING_ofst)
#define DAC_LDAC_en					(1<<DAC_LDAC_en_ofst)
#define DAC_CLR						(1<<DAC_CLR_ofst) // not used for PCB v5.0

#define CNT_OUT_default		(DAC_CLR | NMR_CLK_GATE_AVLN)

#if defined(PCBv2_FEB2018) || defined(PCBv4_APR2019)
	// Output control signal to FPGA via I2C
	#define PSU_15V_TX_P_EN_ofst	(0)
	#define PSU_15V_TX_N_EN_ofst	(1)
	#define AMP_HP_LT1210_EN_ofst	(2)
	#define PSU_5V_TX_N_EN_ofst		(3)
	#define PAMP_IN_SEL_TEST_ofst	(4)
	#define PAMP_IN_SEL_RX_ofst		(5)
	#define GPIO_GEN_PURP_1_ofst	(6)
	#define MTCH_NTWRK_RST_ofst		(6) //  the same pin as GPIO_GEN_PURP_1_ofst
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
	#define MTCH_NTWRK_RST_msk		(1<<MTCH_NTWRK_RST_ofst) //  the same pin as GPIO_GEN_PURP_1_ofst, active-low
	#define PSU_5V_ADC_EN_msk		(1<<PSU_5V_ADC_EN_ofst)
	#define RX_AMP_GAIN_2_msk		(1<<RX_AMP_GAIN_2_ofst)
	#define RX_AMP_GAIN_1_msk		(1<<RX_AMP_GAIN_1_ofst)
	#define RX_AMP_GAIN_4_msk		(1<<RX_AMP_GAIN_4_ofst)
	#define RX_AMP_GAIN_3_msk		(1<<RX_AMP_GAIN_3_ofst)
	#define RX_IN_SEL_1_msk			(1<<RX_IN_SEL_1_ofst)
	#define RX_IN_SEL_2_msk			(1<<RX_IN_SEL_2_ofst)
	#define PSU_5V_ANA_P_EN_msk		(1<<PSU_5V_ANA_P_EN_ofst)
	#define PSU_5V_ANA_N_EN_msk		(1<<PSU_5V_ANA_N_EN_ofst)
#elif defined(PCBv5_JUN2019)
	// Output control signal to FPGA via I2C addr:0x40
	#define RX_FL_ofst		(0)
	#define RX_FH_ofst		(1)
	#define RX_SEL2_ofst	(2)
	#define RX_SEL1_ofst	(3)
	#define RX3_L_ofst		(4)
	#define RX3_H_ofst		(5)
	#define RX2_L_ofst		(6)
	#define RX2_H_ofst		(7)
	//#define ___			(8)
	//#define ___			(9)
	//#define ___			(10)
	#define PAMP_RDY_ofst	(11)
	#define RX1_1H_ofst		(12)
	#define RX1_1L_ofst		(13)
	#define RX1_2H_ofst		(14)
	#define RX1_2L_ofst		(15)
	// -----------------------------------------
	#define RX_FL_msk		(1<<RX_FL_ofst)
	#define RX_FH_msk		(1<<RX_FH_ofst)
	#define RX_SEL2_msk		(1<<RX_SEL2_ofst)
	#define RX_SEL1_msk		(1<<RX_SEL1_ofst)
	#define RX3_L_msk		(1<<RX3_L_ofst)
	#define RX3_H_msk		(1<<RX3_H_ofst)
	#define RX1_2L_msk		(1<<RX1_2L_ofst)
	#define RX1_2H_msk		(1<<RX1_2H_ofst)
	//#define ___			(8)
	//#define ___			(9)
	//#define ___			(10)
	#define PAMP_RDY_msk	(1<<PAMP_RDY_ofst)
	#define RX1_1H_msk		(1<<RX1_1H_ofst)
	#define RX1_1L_msk		(1<<RX1_1L_ofst)
	#define RX2_H_msk		(1<<RX2_H_ofst)
	#define RX2_L_msk		(1<<RX2_L_ofst)

	// Output control signal to FPGA via I2C addr:0x41
	//#define ___					(0+16)
	//#define ___					(1+16)
	//#define ___					(2+16)
	//#define ___					(3+16)
	//#define ___					(4+16)
	//#define ___					(5+16)
	#define DUP_STAT_ofst			(6+16)
	#define QSW_STAT_ofst			(7+16)
	#define PSU_5V_ADC_EN_ofst		(8+16)
	#define PSU_5V_ANA_N_EN_ofst	(9+16)
	#define PSU_5V_ANA_P_EN_ofst	(10+16)
	#define MTCH_NTWRK_RST_ofst		(11+16)
	#define PSU_15V_TX_P_EN_ofst	(12+16)
	#define PSU_15V_TX_N_EN_ofst	(13+16)
	#define PSU_5V_TX_N_EN_ofst		(14+16)
	//#define ___					(15+16)
	//--------------------------------------------
	//#define ___					(0+16)
	//#define ___					(1+16)
	//#define ___					(2+16)
	//#define ___					(3+16)
	//#define ___					(4+16)
	//#define ___					(5+16)
	#define DUP_STAT_msk			(1<<DUP_STAT_ofst)
	#define QSW_STAT_msk			(1<<QSW_STAT_ofst)
	#define PSU_5V_ADC_EN_msk		(1<<PSU_5V_ADC_EN_ofst)
	#define PSU_5V_ANA_N_EN_msk		(1<<PSU_5V_ANA_N_EN_ofst)
	#define PSU_5V_ANA_P_EN_msk		(1<<PSU_5V_ANA_P_EN_ofst)
	#define MTCH_NTWRK_RST_msk		(1<<MTCH_NTWRK_RST_ofst)
	#define PSU_15V_TX_P_EN_msk		(1<<PSU_15V_TX_P_EN_ofst)
	#define PSU_15V_TX_N_EN_msk		(1<<PSU_15V_TX_N_EN_ofst)
	#define PSU_5V_TX_N_EN_msk		(1<<PSU_5V_TX_N_EN_ofst)
	//#define ___					(15+16)

#endif

#ifdef PCBv4_APR2019
	#define CNT_I2C_default		(MTCH_NTWRK_RST_msk)
#endif /* PCBv4_APR2019 */
#ifdef PCBv2_FEB2018
	#define CNT_I2C_default		(0)
#endif /* PCBv2_FEB2018 */
#ifdef PCBv5_JUN2019
	#define CNT_I2C_default		(0)
#endif /* PCBv5_JUN2019 */

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
