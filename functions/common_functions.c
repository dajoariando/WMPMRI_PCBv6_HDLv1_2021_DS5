#include <alt_generalpurpose_io.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <hwlib.h>
#include <math.h>
#include <socal/alt_gpio.h>
#include <socal/hps.h>
#include <socal/socal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "AlteraIP/altera_avalon_fifo_regs.h"

unsigned int rd_FIFO(volatile unsigned int *FIFO_status_addr,
		void *FIFO_data_addr, int * buf32)
{

	// local variables
	uint32_t fifo_mem_level; // the fill level of fifo memory
	unsigned int i;

	// PRINT # of DATAS in FIFO
	// fifo_mem_level = alt_read_word(h2p_adc_fifo_status_addr+ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	// printf("num of data in fifo: %d\n",fifo_mem_level);

	// READING DATA FROM FIFO
	fifo_mem_level = alt_read_word(
			FIFO_status_addr + ALTERA_AVALON_FIFO_LEVEL_REG); // the fill level of FIFO memory
	for (i = 0; fifo_mem_level > 0; i++)
	{ // this is old comment, the FIFO is now only one 32-bit data per beat:: FIFO is 32-bit, while 1-sample is only 16-bit. FIFO organize this automatically. So, fetch only amount_of_data shifted by 2 to get amount_of_data/2.
		buf32[i] = alt_read_word(FIFO_data_addr);

		fifo_mem_level--;
		if (fifo_mem_level == 0)
		{
			fifo_mem_level = alt_read_word(
					FIFO_status_addr + ALTERA_AVALON_FIFO_LEVEL_REG);
		}
		//usleep(1);
	}
	// usleep(100);

	return i;
}

void buf32_to_buf16(int * buf32, unsigned int * buf16, unsigned int length)
{
	unsigned int i, j;

	j = 0;
	for (i = 0; i < (length); i++)
	{
		buf16[j++] = ((unsigned int) buf32[i] & 0x3FFF); // 14 significant bit
		buf16[j++] = ((unsigned int) (buf32[i] >> 16) & 0x3FFF); // 14 significant bit
	}

}

void wr_File(char * pathname, unsigned int length, int * buf,
		char binary_OR_ascii)
{

	// binary_OR_ascii = 1 save binary output into the text file (1). Otherwise, it'll be ASCII output (0)

	FILE *fptr;

	fptr = fopen(pathname, "w");
	if (fptr == NULL)
	{
		printf("File does not exists \n");
	}

	if (binary_OR_ascii)
	{ // binary output
		fwrite(buf, sizeof(uint32_t), length, fptr);
	}

	else
	{ // ascii output
		long i;

		for (i = 0; i < ((length)); i++)
		{
			fprintf(fptr, "%d\n", buf[i]);
		}

	}

}

void print_progress(int iterate, int number_of_iteration)
{
	float static old_progress = 0;
	float new_progress = (float) iterate / (float) number_of_iteration * 100;

	if (new_progress >= (old_progress + 10))
	{
		printf("\t %0.1f%%\n", new_progress);
		old_progress = new_progress;
	}
	new_progress = (float) iterate / (float) number_of_iteration * 100;
}
