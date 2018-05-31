/*
	PLL Calculator calculates n counter, m, counter, c counter, and m_frac of the altera fractional pll reconfiguration.
	The principal of operation is:
	1. Set the limit of the VCO (in MHz range). 1000 means 1 GHz. Usually it's good to set it to hundreds of MHz
	2. Set maximum duty cycle resolution and minimum duty cycle resolution. Duty cycle of 100 means we can change duty cycle up to 1% increment, and duty cycle of 10 means 10% increment
	3. Set maximum Frequency input division, which is frequency divider before the PLL machine. It is practically good not to use this one because it will reduce the frequency at the input of the PLL. Use it only when it's really necessary
	4. Set minimum and maximum MFRAC
*/

#define VCO_LIMIT 1200			// max VCO frequency (MHz)
#define MAX_DUTY_CYCLE_RES 200	// max duty cycle resolution
#define MIN_DUTY_CYCLE_RES 1	// min duty cycle resolution
#define MAX_FIN_DIV 100			// max frequency input division
#define MIN_DELTA 0.05			// min MFRAC (advised by the datasheet of PLL)
#define MAX_DELTA 0.95			// max MFRAC (advised by the datasheet of PLL)

#define FRACTIONAL_CARRY_OUT 32 // set by pll configuration in quartus, can't be changed after the pll is compiled into FPGA



#define N_COUNTER_ADDR	0		// output base address of n counter
#define M_COUNTER_ADDR	1		// output base address of m counter
#define C_COUNTER_ADDR	2		// output base address of c counter
#define M_FRAC_ADDR		3		// output base address of mfrac
#define TOTAL_PLL_PARAM	4		// the total parameter produced by the calculator (count all output base address above)

// the principal of operation:
// the highest priority is highest VCO (means we can have higher duty cycle resolution)
// the second priority of is getting the right MFRAC value, between 0.05 and 0.95
// last priority, is to divide the input frequency of the PLL (this one isn't gonna be used unless there's no choice)
unsigned int pll_calculator (unsigned int * output, double fout, double fin);
