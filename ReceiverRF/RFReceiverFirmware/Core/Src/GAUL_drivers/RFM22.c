/*
 * RFM22.c
 *
 *  Created on: Apr 15, 2025
 *      Author: victo
 */

#include "Gaul_drivers/RFM22.h"


uint8_t RFM22_init(RFM22 *dev, RFM22_configs *confs)
{
	//reset RFM
	HAL_GPIO_WritePin(dev->snd_port, dev->snd_pin, 1);
	HAL_GPIO_WritePin(dev->snd_port, dev->snd_pin, 0);

	//première lecture port SPI
	uint8_t *rx;
	RFM22_SPI_read(dev, RH_RF22_REG_00_DEVICE_TYPE, rx, 1);

	return 0;
}


void RFM22_SPI_write(RFM22 *dev, uint8_t addr, uint8_t *tx_buffer, uint8_t size)
{
	HAL_StatusTypeDef status;
	uint8_t write_addr = 0x80 | addr;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 0);
	HAL_SPI_Transmit(dev->SPIx, &write_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(dev->SPIx, tx_buffer, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 1);
	return status;
}


void RFM22_SPI_read(RFM22 *dev, uint8_t addr, uint8_t *rx_buffer, uint8_t size)
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 0);
	HAL_SPI_TransmitReceive(dev->SPIx, &addr, rx_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(dev->SPIx, rx_buffer, rx_buffer, size, HAL_MAX_DELAY); //vérifier qu'envoyer le rx buffer ok
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 1);
	return status;
}



