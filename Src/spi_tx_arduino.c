/*
 * spi_tx_arduino.c
 *
 *  Created on: Mar 27, 2020
 *      Author: vladi
 */


/*
 * spi_tx_testing.c
 *
 *  Created on: Mar 26, 2020
 *      Author: vladi
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


/*
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB13 -> SCLK
 * PB12 -> NSS
 * ATL -- AF5
 */

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;  // Initial state needs to be set!
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);

	// MISO
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	// GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // actual value 8Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management

	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit(void) {

	GPIO_Handle_t Button_01;

	Button_01.pGPIOx = GPIOA;
	Button_01.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Button_01.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button_01.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button_01.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&Button_01);

	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}

char static some_data[] = "Hello World";

int main(void) {

	GPIO_ButtonInit();

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 Peripheral parameters
	SPI2_Inits();

	// Set the configuration cell the peripheral should work on (Multimaster environment)
	SPI_SSOEConfig(SPI2, ENABLE);

	/*
	 * Makes sense when SSM is set to 1
	 */
	// Set the SSI to high during SSM set 1
	// SPI_SSIConfig(SPI2, ENABLE);

#if 0
	// Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// Disable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
#endif

	while(1) { // Keep the processor busy and at the same time keep the whole flow running

		while ( SPI_GetStatus(SPI2, 7) );

		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}

void EXTI0_IRQHandler(void) {
	GPIO_IRQHandling(0);
	SPI_PeripheralControl(SPI2, ENABLE);
	// Send length of data before commencing communication of real data
	uint8_t lengthData = strlen(some_data);
	SPI_Send(SPI2, &lengthData, 1);
	// Send actual data
	SPI_Send(SPI2, (uint8_t *)some_data, strlen(some_data));
}






