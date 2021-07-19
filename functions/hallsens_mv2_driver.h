/*
 * hallsens_mv2_driver.h
 *
 *  Created on: Jul 16, 2021
 *      Author: dave
 */

#ifndef FUNCTIONS_HALLSENS_MV2_DRIVER_H_
#define FUNCTIONS_HALLSENS_MV2_DRIVER_H_

// R00 register
#define MV2_R00 0x00 // register 00, primary measurement specifications
#define MV2_R00_default 0x00 // default value of register 00 is 0

#define MV2_R00_MA_msk (0x03)
#define MV2_R00_MA_ofst (6) // Measurement Axis LSB. 00:3-axis. 01:X-axis. 10:Y-axis. 11:Z-axis.
#define MV2_R00_MA_3axis_msk (0x00 << MV2_R00_MA_ofst)
#define MV2_R00_MA_Xaxis_msk (0x01 << MV2_R00_MA_ofst)
#define MV2_R00_MA_Yaxis_msk (0x02 << MV2_R00_MA_ofst)
#define MV2_R00_MA_Zaxis_msk (0x03 << MV2_R00_MA_ofst)

#define MV2_R00_RE_msk (0x03) // Resolution MSB
#define MV2_R00_RE_ofst (4) // Resolution LSB. 00:14bits,3kHz refresh rate. 01:15bits,1.5kHz. 10:16bits,0.75kHz. 11:16bits,0.375kHz
#define MV2_R00_RE_14b_3kHz_msk (0x00 << MV2_R00_RE_ofst)
#define MV2_R00_RE_15b_1p5kHz_msk (0x01 << MV2_R00_RE_ofst)
#define MV2_R00_RE_16b_0p7kHz_msk (0x02 << MV2_R00_RE_ofst)
#define MV2_R00_RE_16b_0p3kHz_msk (0x03 << MV2_R00_RE_ofst)

#define MV2_R00_RA_msk (0x03) // Range MSB, amplification gain
#define MV2_R00_RA_ofst (2) // Range LSB, amplification gain. 00:+/-100mT. 01:+/-300mT. 10:+/-1T. 11:+/-3T
#define MV2_R00_RA_100mT_msk (0x00 << MV2_R00_RA_ofst)
#define MV2_R00_RA_300mT_msk (0x01 << MV2_R00_RA_ofst)
#define MV2_R00_RA_1T_msk (0x02 << MV2_R00_RA_ofst)
#define MV2_R00_RA_3T_msk (0x03 << MV2_R00_RA_ofst)

#define MV2_R00_OS_msk (0x03<<0) // 20bits. Output Selection. SPI ONLY. 00:Bx. 01:By. 10:Bz. 11:T
#define MV2_R00_OS_Xaxis 0x00
#define MV2_R00_OS_Yaxis 0x01
#define MV2_R00_OS_Zaxis 0x02
#define MV2_R00_OS_T 0x03

// R01 register
#define MV2_R01 0x01 // register 01, other measurement options
#define MV2_R01_default 0x00 // default value of register 01 is 0
#define MV2_R01_LMR_msk (1<<7) // large measurement range. increases RA selected range by 10x to cover 10T to 30T
#define MV2_R01_SC_msk (1<<6) // spinning current
#define MV2_R01_EMR_msk (1<<5) // extended measurement range by 30% when put to 1 to compensate voltage swing of output buffer when using 3.3V instead of 5V
#define MV2_R01_HC_msk (1<<4) // high clock. doubles the analog clock, which improves SNR by sqrt(2). Not recommended for 3-axis measurement (MA=00) due to worse cross sensitivity.
#define MV2_R01_INV_msk (1<<3) // invert the bias current, voltage is inverted with respect to REF pin (VCC/2). Can be used to emulate low-freq modulator.
#define MV2_R01_LP_msk (1<<2) // low power when the logic is 1, reduced power by ~6mA and SNR by sqrt(2)
#define MV2_R01_PO_msk (1<<1) // permanent output. Put to 1 so that MISO pin is always driven, instead of high-impedance (which allows for multiple chip to use the same MISO bus).
#define MV2_R01_SP_msk (1<<0) // status position. Generally put to 0. If put 1, the MISO pin can also be used as DR (data ready) signal to reduce wires.

// R10 register
#define MV2_R10 0x02 // register 10, temperature compensation and test
#define MV2_R10_default (0x01<<3) // DSB = TC = 0. TC = 0b0001
#define MV2_R10_DSB (1<<7) // disable separate bias, must be zero

#define MV2_R10_TC_msk (0x0F) // temperature compensation 1
#define MV2_R10_TC_ofst (3) // temperature compensation 0. Temperature drift compensation, needs to be found experimentally.

#define MV2_R10_TSC (1<<2) // test system clock, must be zero
// #define MV2_R10_ // bit 1 is unused
// #define MV2_R10_ // bit 0 is unused

// SPI data control parameters
#define MV2_DATA_ofst 0
#define MV2_DATA_msk 0xFF
#define MV2_RD_msk (1<<12)
#define MV2_WR_msk (1<<13)
#define MV2_REG_ofst 8
#define MV2_REG_msk 0b11
#define MV2_SPI_DEFAULT (0x3 << 10) // bit 10 and 11 should be always 1

// functions
void hs_init(volatile unsigned int * hs_addr, unsigned char en_mesg);
unsigned int hs_read_settings(volatile unsigned int * hs_addr,
		unsigned char en_mesg);
void hs_wr_settings(volatile unsigned int * hs_addr, unsigned int hs_settings);
unsigned int hs_wr_reg(volatile unsigned int * hs_addr, unsigned char reg_addr,
		unsigned char data);
unsigned char hs_rd_reg(volatile unsigned int * hs_addr,
		unsigned char reg_addr);
void hs_rd_xyz(volatile unsigned int * hs_addr, unsigned int * x,
		unsigned int * y, unsigned int *z, unsigned int no_samples);
float conv_to_Tesla(unsigned int data);

#endif /* FUNCTIONS_HALLSENS_MV2_DRIVER_H_ */
