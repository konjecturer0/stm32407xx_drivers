/*
 * spi_tx_testing.c
 *
 *  Created on: Mar 26, 2020
 *      Author: vladi
 */

#include <string.h>
#include "stm32f407xx.h"


/*
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB13 -> SCLK
 * PB12 -> NSS
 * ATL -- AF5
 */

void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // actual value 8Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);

}

int main(void) {


	char some_data[] = "Hello World";

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 Peripheral parameters
	SPI2_Inits();

	// Set the SSI to high during SSM set 1
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_Send(SPI2, (uint8_t *)some_data, strlen(some_data));

	// Disable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1); // Keep the processor busy

	return 0;
}
