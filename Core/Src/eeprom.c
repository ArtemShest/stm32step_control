/*
 * eeprom.c
 *
 *  Created on: Feb 27, 2023
 *      Author: artem
 */


#include "eeprom.h"

void EEPROM_write(SPI_HandleTypeDef *hspi, uint16_t addres, uint8_t data)
{
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, RESET);
	HAL_SPI_Transmit(hspi, WREN, 1, 0xFFFFFFFF);
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, SET);

	uint8_t buf[] = {0x02, addres >> 8, addres, data};
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, RESET);
	HAL_SPI_Transmit(hspi, &buf, 4, 0xFFFFFFFF);
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, SET);

}

uint8_t EEPROM_read(SPI_HandleTypeDef *hspi, uint16_t addres)
{

	uint8_t buf[] = {0x03, addres >> 8, addres};
	uint8_t *rbyte;
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, RESET);
	HAL_SPI_Transmit(hspi, &buf, 3, 0xFFFFFFFF);
	HAL_SPI_Receive(hspi, &rbyte, 1, 1000);

	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, SET);
	return rbyte;
}

uint16_t EEPROM_readState(SPI_HandleTypeDef *hspi)
{
	uint8_t buf[2];
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, RESET);
	HAL_SPI_Transmit(hspi, RDSR, 1, 0xFFFFFFFF);
	HAL_SPI_Receive(hspi, &buf, 2, 1000);
	HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, SET);

	  printf ("%d ", buf[0]);
	  printf ("%d", buf[1]);
	  printf ("\r\n");


	return 1;
}
