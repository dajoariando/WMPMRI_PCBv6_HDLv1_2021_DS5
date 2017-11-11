/*
	PLL Calculator calculates n counter, m, counter, c counter, and m_frac of the altera fractional pll reconfiguration.
	The principal of operation is:
	1. Set the limit of the VCO (in MHz range). 1000 means 1 GHz. Usually it's good to set it to hundreds of MHz
	2. Set maximum duty cycle resolution and minimum duty cycle resolution. Duty cycle of 100 means we can change duty cycle up to 1% increment, and duty cycle of 10 means 10% increment
	3. Set maximum Frequency input division, which is frequency divider before the PLL machine. It is practically good not to use this one because it will reduce the frequency at the input of the PLL. Use it only when it's really necessary
	4. Set minimum and maximum MFRAC
*/

#include <stdio.h>
#include "pll_calculator.h"

// the principal of operation:
// the highest priority is highest VCO (means we can have higher duty cycle resolution)
// the second priority of is getting the right MFRAC value, between 0.05 and 0.95
// last priority, is to divide the input frequency of the PLL (this one isn't gonna be used unless there's no choice)

unsigned int pll_calculator (unsigned int * output, double fout, double fin) {
	unsigned int n_counter,c_counter,m_counter;
	double div_fin, vco, delta;
	
	unsigned int fail = 1;	// fail (cannot find MFRAC value) is enabled at the beginning and will be disabled if MFRAC is found
	delta = 1;	// initial value for MFRAC, is set to initialize the program, so that it can make it into the first loop
	unsigned int delta_offset = 0;
	
	for (n_counter=1; n_counter<MAX_FIN_DIV; n_counter++) {		// last priority: change the n_counter
		div_fin = fin/n_counter;	// divide the input of pll (initially, there's no division because the denominator is 1)
	
		for (c_counter = MAX_DUTY_CYCLE_RES; c_counter>=MIN_DUTY_CYCLE_RES; c_counter--) {	// first priority: change the duty cycle
			vco = fout*c_counter;	// multiply the output to get the designated VCO value, and check if that value is within range and also there's a corresponding MFRAC for that
			if (vco <= VCO_LIMIT) {	// check if the vco is within limit
				for (m_counter=1; vco>=div_fin*m_counter; m_counter++) {	// second priority: find the right M and corresponding MFRAC value
					delta = (vco-div_fin*m_counter)/div_fin;	// calculate MFRAC (delta in this case)
					if (delta==0 || (delta>MIN_DELTA && delta<MAX_DELTA)) {	// test whether MFRAC is within limit
						delta_offset = (unsigned int) (delta*(1<<(FRACTIONAL_CARRY_OUT-2)));	// convert MFRAC double value to unsigned integer
						delta_offset <<= 2;								// need 2 stages because of technical conversion problem
						fail = 0;										// fail is disabled
						break;
					}
				}
			}
			if (delta==0 || (delta>MIN_DELTA && delta<MAX_DELTA)) {
				fail = 0;
				break;		// escape loop when the value is found
			}
		}
		if (delta==0 || (delta>MIN_DELTA && delta<MAX_DELTA)) {
			fail = 0;
			break;		// escape loop when the value is found
		}
	}
	if (fail) {
		printf("The right parameter for fout = %f MHz cannot be found\n",fout);
		return 0;
	}
	else {	// put the result at the output
		*(output+N_COUNTER_ADDR) = n_counter;
		*(output+M_COUNTER_ADDR) = m_counter;
		*(output+C_COUNTER_ADDR) = c_counter;
		*(output+M_FRAC_ADDR) = delta_offset;
		// printf("n_counter: %d, m_counter: %d, c_counter: %d, delta: %f, delta_offset: %u\n", n_counter, m_counter, c_counter, delta, delta_offset);
		return 1;
	}
}

/* test code : uncomment the printf function above and also this main function below, edit the makefile, and run
int main () {
	unsigned int dave[4];
	float i;
	for (i=1;i<=10 ;i=i+0.001) {
		printf("i: %f :: ",i);
		pll_calculator(dave,i,50);
	}
	
	return 0;
}
*/