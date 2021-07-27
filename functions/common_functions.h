/*
 * common_functions.h
 *
 *  Created on: May 9, 2020
 *      Author: David Ariando
 */

#ifndef FUNCTIONS_COMMON_FUNCTIONS_H_
#define FUNCTIONS_COMMON_FUNCTIONS_H_

unsigned int rd_FIFO(volatile unsigned int *FIFO_status_addr,
		void *FIFO_data_addr, int * buf32);
void buf32_to_buf16(int * buf32, unsigned int * buf16, unsigned int length);
void wr_File(char * pathname, unsigned int length, int* buf,
		char binary_OR_ascii);
void wr_File_long(char * pathname, unsigned int length, long * buf,
		char binary_OR_ascii);
void wr_File_longlong(char * pathname, unsigned int length,
		unsigned long long * buf, char binary_OR_ascii);
void print_progress(int iterate, int number_of_iteration);

#endif /* FUNCTIONS_COMMON_FUNCTIONS_H_ */
