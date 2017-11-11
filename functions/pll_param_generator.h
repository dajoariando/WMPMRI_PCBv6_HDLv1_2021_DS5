// This algorithm is developed to bridge between low level reconfig_function.h and pll_calculator.h

#define INPUT_FREQ 50 // 50MHz

void Set_M (void *addr, uint32_t * pll_param, uint32_t enable_message);
void Set_N (void *addr, uint32_t * pll_param, uint32_t enable_message);
void Set_C (void *addr, uint32_t * pll_param, uint32_t counter_select, double duty_cycle, uint32_t enable_message);
void Set_DPS (void *addr, uint32_t counter_select, uint32_t phase, uint32_t enable_message); // phase is 0 to 360
void Set_MFrac (void *addr, uint32_t * pll_param, uint32_t enable_message);
void Set_PLL (void *addr, uint32_t counter_select, double out_freq, double duty_cycle, uint32_t enable_message);
