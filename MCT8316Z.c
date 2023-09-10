/*
 * MCT8316Z.c
 *
 *  Created on: Dec 21, 2022
 *      Author: Fabian
 */

#include "MCT8316Z.h"
#include "main.h"

void self_delay() {
	volatile int i = 0;
	for (i = 0; i < 1000; ++i) {

	}
}

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

// Check if a register contains a specified value
// Return 99 no register found
uint8_t checkRegisterOnValue(MCT8316 *dev,uint8_t reg, uint8_t value) {
uint8_t devReg = 0;

	switch (reg) {
		case IC_STATUS_REGISTER_ADDRESS:
			devReg = dev->statReg.data;

			break;
		case IC_STATUS_REGISTER_1_ADDRESS:
			devReg = dev->statReg1.data;

			break;
		case IC_STATUS_REGISTER_2_ADDRESS:
			devReg = dev->statReg2.data;

			break;
		case IC_CONTROL_REGISTER_1_ADDRESS:
			devReg = dev->ctrlReg1.data;
			devReg = devReg & CONTROL_REGISTER_1_MASK;

			break;
		case IC_CONTROL_REGISTER_2_ADDRESS:
			devReg = dev->ctrlReg2.data;
			devReg = devReg & CONTROL_REGISTER_2_MASK;

			break;
		case IC_CONTROL_REGISTER_3_ADDRESS:
			devReg = dev->ctrlReg3.data;
			devReg = devReg & CONTROL_REGISTER_3_MASK;

			break;
		case IC_CONTROL_REGISTER_4_ADDRESS:
			devReg = dev->ctrlReg4.data;
			devReg = devReg & CONTROL_REGISTER_4_MASK;

			break;
		case IC_CONTROL_REGISTER_5_ADDRESS:
			devReg = dev->ctrlReg5.data;
			devReg = devReg & CONTROL_REGISTER_5_MASK;

			break;
		case IC_CONTROL_REGISTER_6_ADDRESS:
			devReg = dev->ctrlReg6.data;
			devReg = devReg & CONTROL_REGISTER_6_MASK;

			break;
		case IC_CONTROL_REGISTER_7_ADDRESS:
			devReg = dev->ctrlReg7.data;
			devReg = devReg & CONTROL_REGISTER_7_MASK;

			break;
		case IC_CONTROL_REGISTER_8_ADDRESS:
			devReg = dev->ctrlReg8.data;
			devReg = devReg & CONTROL_REGISTER_8_MASK;

			break;
		case IC_CONTROL_REGISTER_9_ADDRESS:
			devReg = dev->ctrlReg9.data;
			devReg = devReg & CONTROL_REGISTER_9_MASK;

			break;
		case IC_CONTROL_REGISTER_10_ADDRESS:
			devReg = dev->ctrlReg10.data;
			devReg = devReg & CONTROL_REGISTER_10_MASK;

			break;
		default:
			return 99;
			break;
	}

return devReg == value;



}
/*
 * In This Function the MCT8316 get initialized. This must be edited with new Sensors or with a new motor
 */

uint8_t MCT8316_Init(SPI_HandleTypeDef *hspi,GPIO_TypeDef *NSS_Port,uint16_t NSS_Pin, MCT8316 *dev) {
	uint8_t rxData[2];
	uint8_t txData;
	uint8_t address;
	uint8_t code = 0;
	bool fail = false;

	dev->spiHandle = hspi;
	dev->NSS_Port = NSS_Port;
	dev->NSS_Pin = NSS_Pin;

	// set the device Registers

	// Write enable, without this the registers are locked
	dev->ctrlReg1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_WRITE_011B_TO_UNLOCK_ALL_REGISTERS;
	address = IC_CONTROL_REGISTER_1_ADDRESS;
	txData = dev->ctrlReg1.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 1;
	}

	address = IC_CONTROL_REGISTER_1_ADDRESS;
	if (MCT8316_Read(dev, &address, rxData) != HAL_OK) {
		code = 1;
	}


	// Disable the Buck Converter
	dev->ctrlReg6.fields.BUCK_DIS = CONTROL_REGISTER_6_BUCK_DISABLED;
	address = IC_CONTROL_REGISTER_6_ADDRESS;
	txData = dev->ctrlReg6.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	// Clear Fault
	dev->ctrlReg2.fields.CLR_FLAG = CONTROL_REGISTER_2_CLR_FLAG_CLEAR_LATCHED_FAULT_BITS;
	address = IC_CONTROL_REGISTER_2_ADDRESS;
	txData = dev->ctrlReg2.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	// Read the Buck Converter data
	address = IC_CONTROL_REGISTER_6_ADDRESS;
	if (MCT8316_Read(dev, &address, rxData) != HAL_OK) {
		code = 1;
	}



//	// Enable the Brake Mode
	dev->ctrlReg7.fields.BRAKE = CONTROL_REGISTER_7_BRAKE_ENABLED;
	address = IC_CONTROL_REGISTER_7_ADDRESS;
	txData = dev->ctrlReg7.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	// HYS
	dev->ctrlReg7.fields.HALL_HYS = CONTROL_REGISTER_7_HALL_HYS_5MV;
	address = IC_CONTROL_REGISTER_7_ADDRESS;
	txData = dev->ctrlReg7.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	// SLEW RATE
	dev->ctrlReg2.fields.SLEW = CONTROL_REGISTER_2_SLEW_RATE_200_V_PER_US;
	// PWM Mode to digital Hall Async
	dev->ctrlReg2.fields.PWM_MODE = CONTROL_REGISTER_2_PWM_MODE_ASYNCHRONOUS_RECTIFICATION_WITH_DIGITAL_HALL;
	address = IC_CONTROL_REGISTER_2_ADDRESS;
	txData = dev->ctrlReg2.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	// Set Motor Lock to 5000MS and no report if blocked
	dev->ctrlReg8.fields.MTR_LOCK_TDET = CONTROL_REGISTER_8_MTR_LOCK_TDET_5000MS;
	// Motor Lock set to not reported no action
	dev->ctrlReg8.fields.MTR_LOCK_MODE = CONTROL_REGISTER_8_MTR_LOCK_MODE_NO_REPORT;
	address = IC_CONTROL_REGISTER_8_ADDRESS;
	txData = dev->ctrlReg8.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

//	// PHASE to 0 deg
	dev->ctrlReg9.fields.ADVANCED_LVL = CONTROL_REGISTER_9_ADVANCE_LVL_0DEG;
	address = IC_CONTROL_REGISTER_9_ADDRESS;
	txData = dev->ctrlReg9.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	rxData[0] = 0;
	rxData[1] = 0;

	address = IC_STATUS_REGISTER_ADDRESS;
	if (MCT8316_Read(dev, &address, rxData) != HAL_OK) {
		code = 1;
	}

	// Write disable
	dev->ctrlReg1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_WRITE_110B_TO_LOCK_SETTINGS_IGNORING_FURTHER_WRITES;
	address = IC_CONTROL_REGISTER_1_ADDRESS;
	txData = dev->ctrlReg1.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 1;
	}

	return code;

}

uint8_t MCT8316_Fault_Clear(MCT8316 *dev) {
	uint8_t rxData[2];
	uint8_t txData;
	uint8_t address;
	uint8_t code = 0;

	// Write enable, without this the registers are locked
	dev->ctrlReg1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_WRITE_011B_TO_UNLOCK_ALL_REGISTERS;
	address = IC_CONTROL_REGISTER_1_ADDRESS;
	txData = dev->ctrlReg1.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 1;
	}

	// Clear Fault
	dev->ctrlReg2.fields.CLR_FLAG = CONTROL_REGISTER_2_CLR_FLAG_CLEAR_LATCHED_FAULT_BITS;
	address = IC_CONTROL_REGISTER_2_ADDRESS;
	txData = dev->ctrlReg2.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 1;
	}

	// Write disable
	dev->ctrlReg1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_WRITE_110B_TO_LOCK_SETTINGS_IGNORING_FURTHER_WRITES;
	address = IC_CONTROL_REGISTER_1_ADDRESS;
	txData = dev->ctrlReg1.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 1;
	}

	return code;
}

uint8_t MCT8316_Write_Extern(MCT8316 *dev, uint8_t addressp, uint8_t *data) {
	uint8_t rxData[2];
	uint8_t txData;
	uint8_t address;
	uint8_t code = 0;

	// Write enable, without this the registers are locked
	dev->ctrlReg1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_WRITE_011B_TO_UNLOCK_ALL_REGISTERS;
	address = IC_CONTROL_REGISTER_1_ADDRESS;
	txData = dev->ctrlReg1.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 1;
	}

	dev->ctrlReg2.fields.CLR_FLAG = CONTROL_REGISTER_2_CLR_FLAG_CLEAR_LATCHED_FAULT_BITS;
	address = IC_CONTROL_REGISTER_2_ADDRESS;
	txData = dev->ctrlReg2.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 2;
	}

	address = addressp;
	txData = data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 3;
	}


	dev->ctrlReg2.fields.CLR_FLAG = CONTROL_REGISTER_2_CLR_FLAG_CLEAR_LATCHED_FAULT_BITS;
	address = IC_CONTROL_REGISTER_2_ADDRESS;
	txData = dev->ctrlReg2.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 4;
	}

	// Write disable
	dev->ctrlReg1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_WRITE_110B_TO_LOCK_SETTINGS_IGNORING_FURTHER_WRITES;
	address = IC_CONTROL_REGISTER_1_ADDRESS;
	txData = dev->ctrlReg1.data;
	if (MCT8316_Write(dev, &address, &txData, rxData) != HAL_OK) {
		code = 5;
	}
	return code;
}

HAL_StatusTypeDef MCT8316_Write(MCT8316 *dev, uint8_t *address, uint8_t *data, uint8_t *oldData) {
	// Set the read/write bit to 0 to indicate a write operation
	uint8_t rw = WRITE;
	HAL_StatusTypeDef ret;
	// Calculate the parity of the data
	unsigned char parity_1 = parity_calc((rw << 7) | (*address << 1));
	unsigned char parity_2 = parity_calc(*data);

	// the parity is for the whole
	unsigned char parity = parity_1 ^ parity_2;


	// Create the transmission data array
	uint8_t pTxData[2];

	// Transmit and receive data over the SPI interface
	HAL_GPIO_WritePin(dev->NSS_Port, dev->NSS_Pin, GPIO_PIN_RESET);
	self_delay();
	pTxData[0] = (rw << 7) | (*address << 1) | (parity); // First byte: R/W, address, parity
	pTxData[1] = *data; // Second byte: data
	ret = HAL_SPI_TransmitReceive(dev->spiHandle, pTxData, oldData, 2, 100);
	self_delay();
	HAL_GPIO_WritePin(dev->NSS_Port, dev->NSS_Pin, GPIO_PIN_SET);
	self_delay();
	return ret;
}

HAL_StatusTypeDef MCT8316_Read(MCT8316 *dev, uint8_t *address, uint8_t *data) {
// Set the read/write bit to 1 to indicate a read operation
	uint8_t rw = READ;
	HAL_StatusTypeDef ret;

// Set the data to be transmitted to 0
	uint8_t transmitData = 0;

// Calculate the parity of the address
	unsigned char parity_1 = parity_calc((rw << 7) | (*address << 1));
	unsigned char parity_2 = parity_calc(transmitData);

	// the parity is for the whole
	unsigned char parity = parity_1 ^ parity_2;

//	parity = !parity;

// Create the transmission data array
	uint8_t pTxData[2];

// Transmit and receive data over the SPI interface
	HAL_GPIO_WritePin(dev->NSS_Port, dev->NSS_Pin, GPIO_PIN_RESET);
	pTxData[0] = (rw << 7) | (*address << 1) | (parity); // First byte: R/W, address, parity
	pTxData[1] = transmitData; // Second byte: data (all 0s)
	ret = HAL_SPI_TransmitReceive(dev->spiHandle, pTxData, data, 2, 100);
	HAL_GPIO_WritePin(dev->NSS_Port, dev->NSS_Pin, GPIO_PIN_SET);
//	HAL_Delay(5);
	self_delay();
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
