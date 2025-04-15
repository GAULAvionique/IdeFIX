/*
 * RFM22.c
 *
 *  Created on: Apr 15, 2025
 *      Author: victo
 */

#include "Gaul_drivers/RFM22.h"


uint8_t RFM22_init(RFM22 *dev, RFM22_configs *confs)
{
	return 0;
}


void RFM22_SPI_write(RFM22 *dev, uint8_t addr, uint8_t tx_value)
{
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 0);
	//HAL_SPI_Transmit(dev->SPIx, , , );
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 1);
}


void RFM22_SPI_read(RFM22 *dev, uint8_t addr, uint8_t rx_data[], uint8_t size)
{

}



