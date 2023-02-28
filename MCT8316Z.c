/*
 * MCT8316Z.c
 *
 *  Created on: Dec 21, 2022
 *      Author: Fabian
 */

#include "MCT8316Z.h"
#include "main.h"

unsigned char parity_calc(unsigned char value) {
	// unsigned char because of 8 bit parity check
	// Initialize a variable to hold the parity value
	unsigned char parity = value;

	// XOR value with itself shifted right by 1 bit to get the parity of the first 2 bits
	parity = parity ^ (value >> 1);
	// XOR the result with itself shifted right by 2 bits to get the parity of the first 4 bits
	parity = parity ^ (parity >> 2);
	// XOR the result with itself shifted right by 4 bits to get the parity of all 8 bits
	parity = parity ^ (parity >> 4);

	// The parity is the least significant bit of the result
	return parity & 1;
}

uint8_t MCT8316_Init(SPI_HandleTypeDef *hspi, MCT8316 *dev) {

	uint8_t code = 0;

	dev->spiHandle = hspi;

	// read the status of the device
	// Read first register
	uint8_t rxData[2];
	uint8_t txData;
	uint8_t address = IC_STATUS_REGISTER_ADDRESS;
	if (MCT8316_Read(dev, &address, rxData) != HAL_OK) {
		code = 1;
	}
	dev->statReg.data = rxData[0];

	address = IC_STATUS_REGISTER_1_ADDRESS;
	if (MCT8316_Read(dev, &address, rxData)) {
		code = 2;
	}
	dev->statReg1.data = rxData[0];

	address = IC_STATUS_REGISTER_2_ADDRESS;
	if (MCT8316_Read(dev, &address, rxData) != HAL_OK) {
		code = 3;
	}
	dev->statReg2.data = rxData[0];

	if (statusRegisterFault(dev) != HAL_OK) {
		errorHandler();
	}

	// Disable the Buck Converter
	address = IC_CONTROL_REGISTER_6_ADDRESS;
	dev->ctrlReg6.fields.BUCK_DIS = CONTROL_REGISTER_6_BUCK_DISABLED;
	txData = dev->ctrlReg6.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}
	return code;
}

HAL_StatusTypeDef MCT8316_Write(MCT8316 *dev, uint8_t *address, uint8_t *data,
		uint8_t *oldData) {
	// Set the read/write bit to 0 to indicate a write operation
	uint8_t rw = WRITE;
	HAL_StatusTypeDef ret;
	// Calculate the parity of the data
	unsigned char parity = parity_calc(data);

	// Create the transmission data array
	uint8_t pTxData[2];
	pTxData[0] = (rw << 7) | (*address << 1) | (parity); // First byte: R/W, address, parity
	pTxData[1] = *data; // Second byte: data

	// Transmit and receive data over the SPI interface
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	//ret = HAL_SPI_TransmitReceive(dev->spiHandle, pTxData, oldData, 2, 100);
	ret = HAL_SPI_Transmit(dev->spiHandle, pTxData, 2, 100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	return ret;
}

HAL_StatusTypeDef MCT8316_Read(MCT8316 *dev, uint8_t *address, uint8_t *data) {
// Set the read/write bit to 1 to indicate a read operation
	uint8_t rw = READ;
	HAL_StatusTypeDef ret;

// Set the data to be transmitted to 0
	uint8_t transmitData = 0;

// Calculate the parity of the data
	unsigned char parity = parity_calc(&transmitData);

// Create the transmission data array
	uint8_t pTxData[2];
	pTxData[0] = (rw << 7) | (*address << 1) | (parity); // First byte: R/W, address, parity
	pTxData[1] = transmitData; // Second byte: data (all 0s)

// Transmit and receive data over the SPI interface
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive(dev->spiHandle, pTxData, data, 2, 100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	return ret;
}

uint8_t statusRegisterFault(MCT8316 *dev) {
	/*
	 * The return value is depending on where an Error accrued
	 * Statusregister Error is a 1
	 * Statusregister1 Error is a 10
	 * Statusregister2 Error is a 100
	 * No Error is a 0
	 *
	 * EXAMPLE a return value of 101 is an Error in the first and last register
	 */

	uint8_t status = 0;
	if (dev->statReg.data != 0) {
		status = status + 1;
	}
	if (dev->statReg1.data != 0) {
		status = status + 10;
	}

	if (dev->statReg2.data != 0) {
		status = status + 100;
	}

	return status;
}

void errorHandler() {

}
