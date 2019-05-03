#ifndef ENS210_H_
#define ENS210_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"

// Slave address of device
#define ENS210_ADDRESS 0x43

// BEGIN REGISTER MAP
#define ENS210_PART_ID 		0x00	// 2B, R(active only), Identifies part as ens210
#define ENS210_UID		0x04	// 8B, R(active only), Unique ID
#define ENS210_SYS_CTRL		0x10	// 1B, R/W, System configuration
#define ENS210_SYS_STAT	 	0x11	// 1B, R, System Status
#define ENS210_SENS_RUN	 	0x21	// 1B, R/W, Run mode either single shot or continuous
#define ENS210_SENS_START	0x22	// 1B, W, Start measurement
#define ENS210_SENS_STOP	0x23	// 1B, W, Stop continuous measurement
#define ENS210_SENS_STAT	0x24	// 1B, R, Sensor status either idle or measuring
#define ENS210_T_VAL		0x30	// 3B, R, Temperature readout
#define ENS210_H_VAL		0x31	// 3B, R, Relative humidity readout

// Macros
#define GET_H_STAT(x) ((x & 0x02) >> 1)
#define GET_T_STAT(x) (x & 0x01)

// Masks
#define W_MASK 0x00
#define R_MASK 0x01

// CRC7 defines
#define CRC7WIDTH	7
#define CRC7POLY  	0x89
#define CRC7IVEC	0x7F
#define DATA7WIDTH	17
#define DATA7MASK	((1UL<<DATA7WIDTH)-1) // 0b 0 1111 1111 1111 1111
#define DATA7MSB	(1UL<<(DATA7WIDTH-1)) // Ob 1 0000 0000 0000 0000


/* i2c bus declaration */
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;


/* Forward declared device structure for use in function pointer */
typedef struct ens210_dev_s ens210_dev_t;

/* Function pointer for ens210 device read/write functions */
typedef uint8_t (*ens210_func_ptr)(ens210_dev_t *dev, uint16_t reg_addr,
		uint16_t addr_size, uint8_t *data, uint16_t len);

struct ens210_dev_s {
	/* Device address; default is 0x43 */
	uint8_t address;
	/* Device Part ID */
	uint16_t part_id;
	/* Read function pointer for the I2C implementation */
	ens210_func_ptr read;
	/* Write function pointer for the I2C implementation */
	ens210_func_ptr write;
};

typedef struct ens210_status_s {
	/* Low power mode setting, 0 is disabled */
	uint8_t low_power;
	/* System power state */
	uint8_t power_state;
	/* Sensor run modes, bit 0 is temp and bit 1 is humidity */
	uint8_t sensor_run_modes;
	/* Sensor start information, bit 0 is temp and bit 1 is humidity */
	uint8_t start;
} ens210_status_t;

typedef struct ens210_data_s {
	float T_VAL;
	float H_VAL;
} ens210_data_t;

uint8_t ens210_init(ens210_dev_t *dev);
uint8_t ens210_get_status(ens210_dev_t *dev, ens210_status_t *status);
uint8_t read_ens210_i2c(ens210_dev_t *dev, uint16_t reg_addr, uint16_t addr_size, uint8_t *data, uint16_t len);
uint8_t write_ens210_i2c(ens210_dev_t *dev, uint16_t reg_addr, uint16_t addr_size, uint8_t *data, uint16_t len);
uint32_t crc7(uint32_t val);
uint8_t ens210_get_data(ens210_dev_t *dev, ens210_data_t *ens210_data);


#endif /* ENS210_H_ */

	


