#define I2C_FIFO_DEPTH			256
#define I2C_DATA_MSK			0xFF

#define WR_I2C					0x00
#define RD_I2C					0x01

#define TFR_CMD_OFST			0x00		// transfer command fifo
	#define STA_SHFT				0x09	// start sequence
	#define STO_SHFT				0x08	// stop sequence
	#define AD_SHFT					0x01	// address
	#define RW_D_SHFT				0x00	// read/write
#define RX_DATA_OFST			0x01	// receive data fifo
	#define RX_DATA_MSK				0xFF
#define CTRL_OFST				0x02	// control register
	#define RX_DATA_FIFO_THD_SHFT	0x04
	#define RX_DATA_FIFO_THD_MSK	0x30
	#define TFR_CMD_FIFO_THD_SHFT	0x02
	#define TFR_CMD_FIFO_THD_MSK	0x0C
	#define BUS_SPEED_SHFT			0x01
	#define BUS_SPEED_MSK			0x02
	#define CORE_EN_SHFT			0x00
	#define CORE_EN_MSK				0x01
#define ISER_OFST				0x03	// interrupt status enable register
	#define RX_OVER_EN_SHFT			0x04
	#define RX_OVER_EN_MSK			0x10
	#define ARBLOST_DET_EN_SHFT		0x03
	#define ARBLOST_DET_EN_MSK		0x08
	#define NACK_DET_EN_SHFT		0x02
	#define NACK_DET_EN_MSK			0x04
	#define RX_READY_EN_SHFT		0x01
	#define RX_READY_EN_MSK			0x02
	#define TX_READY_EN_SHFT		0x00
	#define TX_READY_EN_MSK			0x01
#define ISR_OFST				0x04	// interrupt status register
	#define RX_OVER_SHFT			0x04
	#define RX_OVER_MSK				0x10
	#define ARBLOST_DET_SHFT		0x03
	#define ARBLOST_DET_MSK			0x08
	#define NACK_DET_SHFT			0x02
	#define NACK_DET_MSK			0x04
	#define RX_READY_SHFT			0x01
	#define RX_READY_MSK			0x02
	#define TX_READY_SHFT			0x00
	#define TX_READY_MSK			0x01
#define STATUS_OFST				0x05	// status register
	#define CORE_STATUS_MSK			0x01
#define TFR_CMD_FIFO_LVL_OFST	0x06	// TFR cmd fifo level, log 2 of the fifo depth level
	#define  TFR_CMD_FIFO_LVL_MSK	(((I2C_FIFO_DEPTH-1)<<1) | 0x01)
#define RX_DATA_FIFO_LVL_OFST	0x07
	#define  RX_DATA_FIFO_LVL_MSK	(((I2C_FIFO_DEPTH-1)<<1) | 0x01)
#define SCL_LOW_OFST			0x08	// low period of scl in term of number of clock cycles
	#define SCL_LOW_MSK				0xFFFF
#define SCL_HIGH_OFST			0x09	// high period of scl in term of number of clock cycles
	#define HIGH_LOW_MSK			0xFFFF
#define SDA_HOLD_OFST			0x0A	// hold period of sda in term of number of clock cycles
	#define SDA_HOLD_MSK			0xFFFF
