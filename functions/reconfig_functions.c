#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "reconfig_functions.h"
#include "../hps_soc_system.h"

// counter C read address (write address is different from read address)
uint32_t COUNTER_READ_ADDR[18] =
{ 0x28,	// address C00
		0x2C,   // address C01
		0x30,   // address C02
		0x34,   // address C03
		0x38,   // address C04
		0x3C,   // address C05
		0x40,   // address C06
		0x44,   // address C07
		0x48,   // address C08
		0x4C,   // address C09
		0x50,   // address C10
		0x54,   // address C11
		0x58,   // address C12
		0x5C,   // address C13
		0x60,   // address C14
		0x64,   // address C15
		0x68,   // address C16
		0x6C    // address C17
		};

void Reconfig_Mode(void * addr, uint32_t val)
{
	//Write in Mode Register "0" for waitrequest mode, "1" for polling mode
	alt_write_word((addr + MODE), val);
	// usleep(100);
}

void Reconfig_N(void * addr, uint32_t low_count, uint32_t high_count,
		uint32_t bypass_enable, uint32_t odd_division)
{
	uint32_t val = (odd_division << 17) + (bypass_enable << 16)
			+ (high_count << 8) + (low_count);

	//change the register value to val
	alt_write_word((addr + N_COUNTER), val);

}

void Reconfig_M(void * addr, uint32_t low_count, uint32_t high_count,
		uint32_t bypass_enable, uint32_t odd_division)
{
	uint32_t val = (odd_division << 17) + (bypass_enable << 16)
			+ (high_count << 8) + (low_count);

	//change the register value to val
	alt_write_word((addr + M_COUNTER), val);

}

void Reconfig_C(void * addr, uint32_t counter_select, uint32_t low_count,
		uint32_t high_count, uint32_t bypass_enable, uint32_t odd_division // the counter number starts from 0 and ends with 17, instead of 1 to 18

		)
{
	// Every PLL has 18-counter. It needs to be selected before the value can be changed
	// The counter can be selected by changing the five bit value, from [22:18]
	// The write address val is 0 to 17, but the read operation has different address, which is down below

	uint32_t val = (counter_select << 18) + (odd_division << 17)
			+ (bypass_enable << 16) + (high_count << 8) + (low_count);

	//change the register value to val
	alt_write_word((addr + C_COUNTER), val);
	//printf("C_COUNTER: %x",val);
}

void Reconfig_DPS(void * addr, uint32_t DPS_select, uint32_t DPS,
		uint32_t DPS_direction// 1 for positive phase shift, 0 for negative phase shift
		)
{
	uint32_t val = (DPS_direction << 21) + (DPS_select << 16) + (DPS);
	//printf("val for dps : %x\n",val);

	alt_write_word((addr + DPS_REG), val);
}

void Reconfig_MFrac(void * addr, uint32_t MFrac)
{
	//MFrac=K[X:0]/(2^X), X=6,16,24 or 32
	//Only MFrac between 0.05 to 0.95 is allowed
	//MTotal=M+MFrac
	alt_write_word((addr + FRAC_REG), MFrac);
}

void Reconfig_BS(void * addr, uint32_t BS)
{
	alt_write_word((addr + BS_REG), BS);
}

void Reconfig_CPS(void * addr, uint32_t CPS)
{
	alt_write_word((addr + CPS_REG), CPS);
}

void Reconfig_VCO_DIV(void * addr, uint32_t VCO_DIV)
{
	alt_write_word((addr + VCO_DIV_REG), VCO_DIV);
}

void Start_Reconfig(void * addr, uint32_t enable_message)
{
	unsigned int status_reconfig;

	//Write anything to Start Register to Reconfiguration
	alt_write_word((addr + START), 0x01);

	//Polling Status Register
	do
	{
		status_reconfig = alt_read_word(addr + STATUS);
	} while ((!status_reconfig) & 0x01);

	if (enable_message)
	{
		Read_Reconfig_Registers(addr);
	}
}

void Read_Reconfig_Registers(void * addr)
{
	printf("\nMode: %d\n", alt_read_word(addr + MODE));
	printf("N_COUNTER: %x\n", alt_read_word(addr + N_COUNTER));
	printf("M_COUNTER: %x\n", alt_read_word(addr + M_COUNTER));
	unsigned int i = 0;
	printf("C_Counter Value:\n");
	for (i = 0; i < 18; i++)
	{
		printf("\tC%02d : %x\n", i, alt_read_word(addr + COUNTER_READ_ADDR[i]));
	}
	printf("Bandwidth Setting: %i\n", alt_read_word(addr + BS_REG));
	printf("Charge Pump Setting: %i\n", alt_read_word(addr + CPS_REG));
	printf("VCO DIV Setting: %i\n", alt_read_word(addr + VCO_DIV_REG));
}

uint32_t Read_C_Counter(void * addr, uint32_t counter_select)
{
	uint32_t reg_value = alt_read_word(
			addr + COUNTER_READ_ADDR[counter_select]);
	return ((reg_value & 0xFF) + ((reg_value & 0xFF00) >> 8));
}

// reset PLL needs control out register, which is connected to the reset input of the PLL
// reset offset is the offset of the reset input signal in the control register (the control register is general register, so it's not just for pll)
// ctrl_out_signal is the current value of the control register. we don't want to change everything, but only the corresponding control bit for pll reset
void Reset_PLL(void *ctl_out_reg, uint32_t rst_ofst, uint32_t ctrl_out_signal)
{
	alt_write_word((ctl_out_reg), (ctrl_out_signal | (0x01 << rst_ofst)));// reset pll
	// usleep(1);
	alt_write_word((ctl_out_reg), (ctrl_out_signal & ~(0x01 << rst_ofst)));	// deassert reset pll
}

// ctl_in_reg is the register for lock signal coming from pll
// lock_ofst is the corresponding bit for the lock signal on the ctl_in_reg
void Wait_PLL_To_Lock(void *ctl_in_reg, uint32_t lock_ofst)
{
	while (!(alt_read_word(ctl_in_reg) & (0x01 << lock_ofst)))
		;	// wait for pll to lock
}
