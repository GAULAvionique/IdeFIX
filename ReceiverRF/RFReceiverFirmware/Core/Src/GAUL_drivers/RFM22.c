/*
 * RFM22.c
 *
 *  Created on: Apr 15, 2025
 *      Author: victo
 */

#include "Gaul_drivers/RFM22.h"
#include <math.h>


uint8_t RFM22_init(RFM22 *dev, RFM22_configs *confs)
{
	//reset RFM
	HAL_GPIO_WritePin(dev->snd_port, dev->snd_pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(dev->snd_port, dev->snd_pin, 0);
	HAL_Delay(100);

	//première lecture port SPI
	uint8_t rx_buffer[] = {0, 0};
	RFM22_SPI_read(dev, RH_RF22_REG_00_DEVICE_TYPE, rx_buffer, 1);
	if (rx_buffer[0] != 8) return 0; //connection non établie
	//wait for chip ready
	/*
	do {
		RFM22_SPI_read(dev, RH_RF22_REG_04_INTERRUPT_STATUS2, rx_buffer, 1);
	}
	while (!((rx_buffer[0] & RH_RF22_ICHIPRDY) >> 1)); //check for ichiprdy
	*/

	uint8_t tx_buffer[8] = {0};
	// lire registres interrupt
	RFM22_SPI_read(dev, RH_RF22_REG_03_INTERRUPT_STATUS1, rx_buffer, 2);
	// désactiver GPIO
	RFM22_SPI_write(dev, RH_RF22_REG_0B_GPIO_CONFIGURATION0, tx_buffer, 4);
	// mettre en mode standby
	RFM22_SPI_write(dev, RH_RF22_REG_07_OPERATING_MODE1, tx_buffer, 2);
	// active toutes interruptions
	tx_buffer[0] = 0xFF;
	tx_buffer[1] = 0xFF;
	RFM22_SPI_write(dev, RH_RF22_REG_05_INTERRUPT_ENABLE1, tx_buffer, 2);



	// modulation 0x71
	tx_buffer[0] = 0x2C; // défault + txdtrtscale
	tx_buffer[1] = RH_RF22_MODTYP_GFSK | RH_RF22_DTMOD_FIFO;
	RFM22_SPI_write(dev, RH_RF22_REG_70_MODULATION_CONTROL1, tx_buffer, 2);

	//config frequence et channels
	// recommandé: f_nom=433M ch_step=10k f_dev=drate=1.25k
	uint8_t fb = (uint8_t)(confs->nominal_freq / 10e6) - 24;
	uint16_t fc = round((((float)confs->nominal_freq / (float)10e6) - (float)fb - 24.0f) * 64000);
	uint8_t fhs = confs->channel_step / 10e3;
	uint8_t fd = confs->freq_dev / 625;
	uint8_t hbsel = 0;

	tx_buffer[0] = fd;
	tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = ((hbsel << 6) | fb);
	tx_buffer[4] = ((fc & 0xFF00) >> 8);
	tx_buffer[5] = (fc & 0x00FF);
	tx_buffer[6] = 0; //channel 0
    tx_buffer[7] = fhs;
	RFM22_SPI_write(dev, RH_RF22_REG_72_FREQUENCY_DEVIATION, tx_buffer, 8);

	// config Puissance et datarate
	uint16_t txdr = confs->datarate*pow(2, 21) / 1e6;
	tx_buffer[0] = RH_RF22_LNA_SW | RH_RF22_TXPOW_20DBM;
	tx_buffer[1] = (txdr & 0xFF00) >> 8;
	tx_buffer[2] = (txdr & 0x00FF);
	RFM22_SPI_write(dev, RH_RF22_REG_6D_TX_POWER, tx_buffer, 3);

	// disable AFC, set filter
	tx_buffer[0] = 0x3F; //gros chat, documentation insuffisante sur le filtre
    tx_buffer[1] = 0;

	// packet handler
    tx_buffer[0] = RH_RF22_ENPACRX | RH_RF22_ENPACTX;
    RFM22_SPI_write(dev, RH_RF22_REG_30_DATA_ACCESS_CONTROL, tx_buffer, 1);
    tx_buffer[0] = RH_RF22_HDCH_NONE | RH_RF22_BCEN_NONE;
    tx_buffer[1] = (1 << 4) | (1 << 1); //header len, sync len - 1
    tx_buffer[2] = 20; //preamble len
    tx_buffer[3] = (8 << 3); //preamble detection threshold
    tx_buffer[4] = 0xCC; //sync word
    tx_buffer[5] = 0xCC;
    RFM22_SPI_write(dev, RH_RF22_REG_32_HEADER_CONTROL1, tx_buffer, 6);
    tx_buffer[0] = 0xFF; //header word
    tx_buffer[1] = 0xFF;
    tx_buffer[2] = 0xFF;
    tx_buffer[3] = 0xFF;
    RFM22_SPI_write(dev, RH_RF22_REG_32_HEADER_CONTROL1, tx_buffer, 4);

	return 1;
}

//transmet les bits dans tx_buffer et retourne en mode standby
uint8_t RFM22_transmit(RFM22 *dev, uint8_t *tx_buffer, uint8_t lenght)
{
	if (lenght > 64)
	{
		return 0; // pas assez de place dans FIFO
	}
	uint8_t mode_reg[] = {0};
	RFM22_SPI_read(dev, RH_RF22_REG_07_OPERATING_MODE1, mode_reg, 1);
	if (mode_reg[0] & RH_RF22_TXON)
	{
		return 0; // already transmitting
	}


	// clear FIFO
	uint8_t reg_value = RH_RF22_FFCLRTX;
	uint8_t *reg_set = &reg_value;
	RFM22_SPI_write(dev, RH_RF22_REG_08_OPERATING_MODE2, reg_set, 1);

	// send dans la FIFO et set longueur packet
	RFM22_SPI_write(dev, RH_RF22_REG_7F_FIFO_ACCESS, tx_buffer, lenght);
	*reg_set = lenght;
	RFM22_SPI_write(dev, RH_RF22_REG_3E_PACKET_LENGTH, reg_set, 1);

	// mode tx
	*reg_set = RH_RF22_TXON;
	RFM22_SPI_write(dev, RH_RF22_REG_07_OPERATING_MODE1, reg_set, 1);

	return 1;
}

// commence à écouter pour des packets. Retourne en mode standy une fois qu'un packet est reçu
uint8_t RFM22_rx_mode(RFM22 *dev)
{
	// enable ipvalid interrupt
	// mode rx
	uint8_t reg_value = RH_RF22_RXON;
	uint8_t *reg_set = &reg_value;
	RFM22_SPI_write(dev, RH_RF22_REG_07_OPERATING_MODE1, reg_set, 1);
}

uint8_t RFM22_standby(RFM22 *dev)
{

	uint8_t reg_value = 0;
	uint8_t *reg_set = &reg_value;
	RFM22_SPI_write(dev, RH_RF22_REG_07_OPERATING_MODE1, reg_set, 1);
	return 1;
}


// retourn nbr d'octets disponnibles
uint8_t RFM22_available(RFM22 *dev)
{
	uint8_t *lenght;
	RFM22_SPI_read(dev, RH_RF22_REG_4B_RECEIVED_PACKET_LENGTH, lenght, 1);
	return *lenght;
}


void RFM22_read_rx(RFM22 *dev, uint8_t *rx_data, uint8_t size)
{
	RFM22_SPI_read(dev, RH_RF22_REG_7F_FIFO_ACCESS, rx_data, size);
}


void RFM22_handle_interrupt(RFM22 *dev, uint8_t *rx_data, GPIO_TypeDef rx_led_port, uint8_t rx_led_pin, GPIO_TypeDef tx_led_port, uint8_t tx_led_pin)
{
	// read interrupts
	uint8_t interrupts[] = {0, 0};
    RFM22_SPI_read(dev, RH_RF22_REG_02_DEVICE_STATUS, interrupts, 2);

    // pksent
    if (interrupts[0] & RH_RF22_IPKSENT)
    {
    	HAL_GPIO_TogglePin(&tx_led_port, tx_led_pin);
    }

    // pkvalid
    if (interrupts[0] & RH_RF22_IPKVALID)
    {
    	uint8_t *lenght;
    	RFM22_SPI_read(dev, RH_RF22_REG_4B_RECEIVED_PACKET_LENGTH, lenght, 1);
    }

}


void RFM22_SPI_write(RFM22 *dev, uint8_t addr, uint8_t *tx_buffer, uint8_t size)
{
	HAL_StatusTypeDef status;
	uint8_t write_addr = 0x80 | addr;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 0);
	HAL_SPI_Transmit(dev->SPIx, &write_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(dev->SPIx, tx_buffer, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 1);
}


void RFM22_SPI_read(RFM22 *dev, uint8_t addr, uint8_t *rx_buffer, uint8_t size)
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 0);
	HAL_SPI_TransmitReceive(dev->SPIx, &addr, rx_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(dev->SPIx, rx_buffer, rx_buffer, size, HAL_MAX_DELAY); //vérifier qu'envoyer le rx buffer ok
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, 1);
}



