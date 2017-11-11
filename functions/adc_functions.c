#include "adc_functions.h"
#include <stdio.h>

unsigned int adc_delay_calc (
	double adc_desired_freq,	// the desired sampling frequency
	double adc_pll_freq			// the chosen operating frequency of the PLL used by the LTC2314
){
	// CLK_DIV in ADC is set to 2 (fix) in this equation
	// the total time needed to take 16-bit sample is 2*16 = 32 clock cycles of pll_freq (only 14 bits are used on LTC2314)
	double output;
	output = (double)(((unsigned int)(adc_pll_freq / adc_desired_freq))+1) - 32; // +1 is for rounding up

	if ((output/adc_pll_freq) < 0.04) { // if the sampling time is less than 40ns which is the minimum required by the LTC2314 ADC, then show an error message
		printf("ERROR: Chosen sampling time is less than 40ns. Decrease the sampling frequency or increase the adc pll frequency.\nSampling time is at: %f",output);
	}
	if (adc_pll_freq > 175) {
		printf("ERROR: Maximum allowable PLL frequency for LTC2314 is 175 MHz. Decrease the PLL frequency.\n");
	}


	return ((unsigned int) output); 
}
