#define GNRL_CALL_CMD			0x00 // 1st byte for general call commands
	// 2nd byte command for general call
	#define GNRL_CALL_RST		0x06 // at the acknowledgement of this byte, device will do internal reset (like power-on-reset) and VOUT is available immediately
	#define GNRL_CALL_WKUP		0x09 // general call wake-up, device will reset the power-down bits (PD1,PD0 = 0,0)
	#define GNRL_CALL_SWR_UPDT	0x08 // software update, device updates all DAC analog outputs at the same time
	#define GNRL_CALL_RD_ADDR	0xC0 // the general call for reading bits in EEPROM and register. LDAC needs to transition from "high" to "low" after 2nd byte to work!

//// INCOMPLETE!!!!
