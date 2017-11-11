#define PULSE1_OFFST			0
#define DELAY1_OFFST			1
#define PULSE2_OFFST			2
#define DELAY2_OFFST			3
#define INIT_DELAY_ADC_OFFST	4

void cpmg_param_calculator_ltc2314 (
	unsigned int * output,		// cpmg parameter output
	double cpmg_freq,			// cpmg operating frequency
	double adc_sampling_freq,	// adc sampling frequency (for LTC2134 it is 4.5 Msps)
	double adc_pll_freq,		// adc pll frequency for LTC2134 (max at 175 MHz)
	double shift_sampling_us,	// shift the 180 deg data capture relative to the middle of the 180 delay span. This is to compensate shifting because of signal path delay / other factors. This parameter could be negative as well
	double pulse1_us,			// the length of cpmg 90 deg pulse
	double pulse2_us,			// the length of cpmg 180 deg pulse
	double echotime_us,			// the length between one echo to the other (equal to pulse2_us + delay2_us)
	unsigned int total_sample	// the total adc samples captured in one echo
);

void cpmg_param_calculator_ltc1746 (
	unsigned int * output,		// cpmg parameter output
	double cpmg_freq,			// cpmg operating frequency
	double adc_sampling_freq,	// adc sampling frequency (for LTC2134 it is 4.5 Msps)
	double shift_sampling_us,	// shift the 180 deg data capture relative to the middle of the 180 delay span. This is to compensate shifting because of signal path delay / other factors. This parameter could be negative as well
	double pulse1_us,			// the length of cpmg 90 deg pulse
	double pulse2_us,			// the length of cpmg 180 deg pulse
	double echotime_us,			// the length between one echo to the other (equal to pulse2_us + delay2_us)
	unsigned int total_sample	// the total adc samples captured in one echo
);