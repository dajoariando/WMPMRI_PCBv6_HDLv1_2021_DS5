// calculate the adc sample-and-hold delay from the desired sampling frequency and the adc pll frequency
// to get one sample (16-bit), it needs 32 pll clock cycles with minimum of 40ns sample-and-hold time
// 40ns sample-and-hold time is achieved with 7 pulses if 175 MHz PLL clock is used by ADC (which is also the max frequency available for LTC2314)

unsigned int adc_delay_calc (
	double adc_desired_freq,	// the desired sampling frequency
	double adc_pll_freq			// the chosen operating frequency of the PLL used by the LTC2314
);