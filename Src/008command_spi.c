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


#define LED_ON		1U
#define LED_OFF		0U

//arduino analog pins
#define ANALOG_PIN0	0
#define ANALOG_PIN1	1
#define ANALOG_PIN2	2
#define ANALOG_PIN3	3
#define ANALOG_PIN4	4

// arduino leds
#define LED_PIN		9U

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

uint8_t dummybyte = 0xFF;
uint8_t dummyread;

uint8_t static ackbyte;


typedef struct {
	uint8_t code;
	char *args[2];

} command;

command _CMD_LED_CTR = {
		0x50,
		{"<pin no(1)>", "value(1)"}
};

command _CMD_SENSOR_READ = {
		0x51,
		{"<analog pin number(1)>"}
};

command _CMD_LED_READ = {
		0x52,
		{"<pin no(1)>"}
};

command _CMD_PRINT = {
		0x53,
		{"<len(2)>", "<message(len)>"}
};

command _CMD_ID_READ = {
		0x54
};

struct commands {
	command* CMD_LED_CTR;
	command* CMD_SENSOR_READ;
	command* CMD_LED_READ;
	command* CMD_PRINT;
	command* CMD_ID_READ;

} spiCommands = {
		&_CMD_LED_CTR,
		&_CMD_SENSOR_READ,
		&_CMD_LED_READ,
		&_CMD_PRINT,
		&_CMD_ID_READ
};

int main(void) {

	GPIO_ButtonInit();

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 Peripheral parameters
	SPI2_Inits();

	// Set the configuration cell the peripheral should work on (Multimaster environment)
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1) { // Keep the processor busy and at the same time keep the whole flow running

		while ( SPI_GetStatus(SPI2, 7) );

		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {

	if (ackbyte == (uint8_t) 0xF5) {
		// ack
		return 1;
	} else  {
		// nack
		return 0;
	}

}

void setupCommunication(uint8_t code) {
	/*** 8 Bit data communication !! ***/
	// 1. CMD_LED_CTRL
	SPI_SendData(SPI2, &code, 1);

	// dummy read to clear off the RXNE (Register)
	SPI_Receive(SPI2, &dummyread, 1);

	// Send dummy bits (1 byte) to fetch the response from slave
	SPI_SendData(SPI2, &dummybyte, 1);

	// read the ack byte
	SPI_Receive(SPI2, &ackbyte, 1);

}

uint8_t static state_handler = 0;

void SPI2_IRQHandler(void) {
	SPI_IRQHandling();
}

void EXTI0_IRQHandler(void) {
	GPIO_IRQHandling(0);
	SPI_PeripheralControl(SPI2, ENABLE);

	if (state_handler == 0) {
		setupCommunication(spiCommands.CMD_ID_READ->code);

		/*** 8 Bit data communication !! ***/

		if (SPI_VerifyResponse(ackbyte)) {

			spiCommands.CMD_LED_CTR->args[0] = (char *)LED_PIN;
			spiCommands.CMD_LED_CTR->args[1] = (char *)LED_ON;
			SPI_SendArgs(SPI2, (uint8_t **)spiCommands.CMD_LED_CTR->args, 2);
			state_handler += 1;

		}
	} else if (state_handler == 1) {
		setupCommunication(spiCommands.CMD_SENSOR_READ->code);

		/*** 8 Bit data communication !! ***/

		if (SPI_VerifyResponse(ackbyte)) {

			spiCommands.CMD_SENSOR_READ->args[0] = (char *)ANALOG_PIN0;
			SPI_SendArgs(SPI2, (uint8_t **)spiCommands.CMD_SENSOR_READ->args, 1);

			SPI_Receive(SPI2, &dummyread, 1);

			delay();

			SPI_SendData(SPI2, &dummybyte, 1);

			uint8_t analog_read;

			SPI_Receive(SPI2, &analog_read, 1);
			state_handler += 1;
		}

	} else if (state_handler == 2) {
		// todo
	} else if (state_handler == 3) {
		// todo
	} else if (state_handler == 4) {
		// todo
	}







}






