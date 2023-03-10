/*
 * eeprom.h
 *
 *  Created on: Feb 27, 2023
 *      Author: artem
 */

#include "stm32f4xx_hal.h"
#include "main.h"


#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_


// ----------------  Instruction set -------------------
#define WREN 0b0110 		// Write enable
#define WRDI 0b0100 		// Write disable
#define RDSR 0b0101 		// Read Status register
#define WRSR 0b0001 		// Write Status register
#define READ 0b0011			// Read from memory array
#define WRITE 0b0010		// Write to Memory array
#define RDID 0b10000011		// Read Identification page
#define WRID 0b10000010		// Write Identification page
#define RDLS 0b10000011		// Reads the Identification page lock status
#define LID 0b10000010		// Locks the Identification page in read-only mode


void EEPROM_write(SPI_HandleTypeDef *hspi, uint16_t addres, uint8_t data);
uint8_t EEPROM_read(SPI_HandleTypeDef *hspi, uint16_t addres);
uint16_t EEPROM_readState(SPI_HandleTypeDef *hspi);

#endif /* INC_EEPROM_H_ */
