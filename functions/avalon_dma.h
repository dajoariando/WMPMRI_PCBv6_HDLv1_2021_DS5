#define DMA_STATUS_OFST			0x00	// status register
	#define DMA_STAT_DONE_MSK	(1<<0)	// is 1 when DMA transaction is complete, write 0 to status register to clear 'DONE' bit
	#define DMA_STAT_BUSY_MSK	(1<<1)	// is 1 when DMA transaction is in progress
	#define DMA_STAT_REOP_MSK	(1<<2)	// is 1 when DMA transaction is completed due to end-of-packet on read side
	#define DMA_STAT_WEOP_MSK	(1<<3)	// is 1 when DMA transaction is completed due to end-of-packet on write side
	#define DMA_STAT_LEN_MSK	(1<<4)	// is 1 when the length register decrements to 0

#define DMA_READADDR_OFST 		0x01	// read master start address
#define DMA_WRITEADDR_OFST 		0x02	// write master start address
#define DMA_LENGTH_OFST 		0x03	// DMA transaction length (in bytes)

#define DMA_CONTROL_OFST 		0x06	// control register
	#define DMA_CTRL_SWRST_MSK	(1<<12) // software reset the DMA by writing this bit to 1 twice
	#define DMA_CTRL_QWORD_MSK	(1<<11) // specifies quadword transfers
	#define DMA_CTRL_DWORD_MSK	(1<<10) // specifies doubleword transfers
	#define DMA_CTRL_WCON_MSK	(1<<9)	// write to constant address. If this bit is '0', the write address increments after every transfer
	#define DMA_CTRL_RCON_MSK	(1<<8)	// reads from constant address. If this bit is '0', the read address increments after every data transfer
	#define DMA_CTRL_LEEN_MSK	(1<<7)	// ends transaction when 'length' register reaches 0. If this bit is '0', 'length' reaching 0 does not cause transaction to end. In this case, the DMA transaction must be terminated by end-of-packet signal from either read or write master port
	#define DMA_CTRL_WEEN_MSK	(1<<6)	// ends transaction on write-side end-of-packet. Should be set to 0
	#define DMA_CTRL_REEN_MSK	(1<<5)	// ends transaction on read-side end-of-packet. if '1', then a slave port with flow control on read-side may end DMA transaction
	#define DMA_CTRL_I_EN_MSK	(1<<4)	// enables interrupt requests (IRQ). If '1', DMA generates IRQ when 'DONE' bit is set to '1'.
	#define DMA_CTRL_GO_MSK		(1<<3)	// enables DMA transaction. If '1', then DMA transfer occurs if 'length' register is non-zero. If it is de-asserted low before transaction is completed, 'DONE' will never go high. So modify this bit only during idle stage.
	#define DMA_CTRL_WORD_MSK	(1<<2)	// specifies word (32-bit) transfers
	#define DMA_CTRL_HW_MSK		(1<<1)	// specifies halfword (16-bit) transfers
	#define DMA_CTRL_BYTE_MSK	(1<<0)	// specifies byte (8-bit) transfers
