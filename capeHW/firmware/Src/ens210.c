#include "ens210.h"

// BEGIN PRIVATE FUNCTIONS

/*
 * @brief Initializes ens210 device by setting operating parameters.
 */
uint8_t ens210_init(ens210_dev_t *dev) {

	uint8_t control_mode, run_mode, start, id[2];
	uint8_t error = 0;

	control_mode = 0x00;
	run_mode = 0x03;
	start = 0x03;

	/* set run mode */
	error |= dev->write(dev, ENS210_SENS_RUN, 1, &run_mode, 1);
	/* set control mode */
	error |= dev->write(dev, ENS210_SYS_CTRL, 1, &control_mode, 1);
	/* start sampling sensor readings */
	error |= dev->write(dev, ENS210_SENS_START, 1, &start, 1);
	/* get part id */
	error |= dev->read(dev, ENS210_PART_ID, 1, id, 2);

	dev->part_id = ((uint16_t)id[0] << 8) | id[1];

	return error;
}

uint8_t ens210_get_status(ens210_dev_t *dev, ens210_status_t *status) {

	uint8_t error = 0;

	/* Read low power status */
	error |= dev->read(dev, ENS210_SYS_CTRL, 1, &status->low_power, 1);
	/* Read system power state */
	error |= dev->read(dev, ENS210_SYS_STAT, 1, &status->power_state, 1);
	/* Read sensor run modes */
	error |= dev->read(dev, ENS210_SENS_RUN, 1, &status->sensor_run_modes, 1);
	/* Read if sensors have started sampling */
	error |= dev->read(dev, ENS210_SENS_START, 1, &status->start, 1);

	return error;
}

uint32_t crc7(uint32_t val) {

	// set up polynomial
	uint32_t pol = CRC7POLY;
	// align poly with data
	pol = pol << (DATA7WIDTH - CRC7WIDTH - 1);

	// loop variable indicates which bit to test, starts with highest
	uint32_t bit = DATA7MSB;
	// Make room for crc value
	val = val << CRC7WIDTH;
	bit = bit << CRC7WIDTH;
	pol = pol << CRC7WIDTH;

	// Insert initial vector
	val |= CRC7IVEC;

	// Apply division untial all bits are done
	while ( bit & (DATA7MASK << CRC7WIDTH) ) {
		if (bit & val) 
			val ^= pol;
		bit >>= 1;
		pol >>= 1;
	}

	return val;
}

uint8_t ens210_get_data(ens210_dev_t *dev, ens210_data_t *ens210_data) {

	uint8_t error = 0;
	uint8_t rbuf[6];
	uint32_t t_val;
	uint32_t h_val;
	float t_data, h_data;

	error |= dev->read(dev, ENS210_T_VAL, 1, rbuf, 3);
	t_val = (rbuf[2] << 16) + (rbuf[1] << 8) + rbuf[0];
	error |= dev->read(dev, ENS210_H_VAL, 1, rbuf, 3);
	h_val = (rbuf[2] << 16) + (rbuf[1] << 8) + rbuf[0];

	t_data = (float)(t_val & 0xFFFF);
	

	float TinK = t_data / 64;
	float TinC = TinK - 273.15;
	float TinF = 1.8 * TinC + 32;

	ens210_data->T_VAL = TinF;

	// get humidity
	
	h_data = (float)(h_val & 0xFFFF);
	float H = h_data / 512;

	ens210_data->H_VAL = H;

	return error;
}


uint8_t read_ens210_i2c(ens210_dev_t *dev, uint16_t reg_addr, uint16_t addr_size, uint8_t *data, uint16_t len) {

	uint8_t result = 0;
	result = HAL_I2C_Mem_Read(&hi2c2, (dev->address | R_MASK), reg_addr, addr_size, data, len, 100);

	if (result != HAL_OK) {
		char okay_buff[30];
		sprintf(okay_buff, "ENS210 read result failed\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)okay_buff, sizeof(okay_buff), HAL_MAX_DELAY);
	}

	return hi2c2.ErrorCode;
}


uint8_t write_ens210_i2c(ens210_dev_t *dev, uint16_t reg_addr, uint16_t addr_size, uint8_t *data, uint16_t len) {

	uint8_t result = 0;
	result = HAL_I2C_Mem_Write(&hi2c2, (dev->address | W_MASK), reg_addr, addr_size, data, len, 100);

	if (result != HAL_OK) {
		char okay_buff[30];
		sprintf(okay_buff, "ENS210 write result failed\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)okay_buff, sizeof(okay_buff), HAL_MAX_DELAY);
	}

	return hi2c2.ErrorCode;
}
