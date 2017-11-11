#define WR_DAC				(0x00<<23)	// write dac
#define RD_DAC				(0x01<<23)	// read dac

#define DAC_REG				(0x00<<19)
#define OUT_RANGE_SEL_REG	(0x01<<19)	// output range select register
	#define P50				(0x00<<0)	// output range +5.0
	#define P100			(0x01<<0)	// output range +10.0
	#define P108			(0x02<<0)	// output range +10.8
	#define PN50			(0x03<<0)	// output range +-5.0
	#define PN100			(0x04<<0)	// output range +-10.0
	#define PN108			(0x05<<0)	// output range +-10.8
#define PWR_CNT_REG			(0x02<<19)	// power control register
	#define DAC_A_PU		(0x01<<0)	// dac A power up
	#define DAC_B_PU		(0x01<<2)	// dac B power up
	#define REF_PU			(0x01<<4)	// reference power up
	//readonly
	#define TSD				(0x01<<5)	// thermal shutdown alert
	#define OC_A			(0x01<<7)	// dac A overcurrent alert
	#define OC_B			(0x01<<9)	// dac B overcurrent alert
#define CNT_REG				(0x03<<19)
	#define NOP				(0x00<<16)	// no operation instruction used in readback operations
	#define Other_opt		(0x01<<16)	// other options
		#define TSD_en		(0x01<<3)	// enable thermal shutdown feature
		#define Clamp_en	(0x01<<2)	// enable current limit clamp
		#define CLR_sel		(0x01<<1)	// set clear to midscale (unipolar), negative full scale (bipolar). Otherwise it'll be 0V
		#define SDO_disable	(0x01<<0)	// set to disable SDO output. clear to enable SDO output
	#define Clear			(0x04<<16)	// clear the DAC registers to its CLR_sel definition
	#define Load			(0x05<<16)	// load DAC registers & outputs
#define DAC_A				(0x00<<16)	// the address of DAC A register
#define DAC_B				(0x02<<16)	// the address of DAC B register
#define DAC_AB				(0x04<<16)	// the address of both DACs register
