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
#include "core_cm4.h"


#define __ENABLE	1
#define __DISABLE	0

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

// Enable better control flow
#define ACK_DONE	0
#define ACK_NOT_YET	1

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

SPI_Handle_t static SPI2handle;
uint8_t static *placeholder_ARGS = 0;

void SPI2_Inits(void) {

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // actual value 8Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management
	// Init buffers

	SPI2handle.args = &placeholder_ARGS;
	SPI2handle.pRxBuffer = (uint8_t *) 0;
	SPI2handle.pTxBuffer = (uint8_t *) 0;

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
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, __ENABLE);
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 12);
	//NVIC_EnableIRQ(IRQ_NO_EXTI0);
	NVIC_SetPriority(IRQ_NO_EXTI0, 12);
}

uint8_t dummybyte = 0xFF;
uint8_t dummyread;

uint8_t static ackbyte;


#if 0
typedef struct {
	uint8_t code;
	char *args[2];

} command;
#endif

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
	SPI_SSOEConfig(SPI2, __ENABLE);

	uint8_t v = 1;


	// 11111010000001010 0000 000 00000 000
	//SCB->AIRCR = 0x05FA0000;
	//NVIC_SetPriorityGrouping((uint32_t)0x300);

	printf("Lets see: %d", v);
	fflush(stdout);

	// Interrupt config
	SPI_IRQInterruptConfig(IRQ_NO_SPI2, __ENABLE);


	//NVIC_EnableIRQ(IRQ_NO_SPI2);
	NVIC_SetPriority(IRQ_NO_SPI2, 2);		/* TODO: Replace by own function */

	//SPI_IRQPriorityConfig(IRQ_NO_SPI2, 2);


	while(1) { // Keep the processor busy and at the same time keep the whole flow running

		while ( SPI_GetStatus(SPI2, 7) );

		SPI_PeripheralControl(SPI2, __DISABLE);

	}

	return 0;
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {

	if (ackbyte == (uint8_t)0xF5) {
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

uint8_t static *context_code;

void SPI2_IRQHandler(void) {
	SPI_IRQHandling(&SPI2handle);
}

void EXTI0_IRQHandler(void) {

	GPIO_IRQHandling(0);

	// Init context code
	if (context_code == 0) { context_code = &spiCommands.CMD_LED_CTR->code; }


	SPI_PeripheralControl(SPI2, __ENABLE);

	// 1. Command
	if (context_code == &spiCommands.CMD_LED_CTR->code) {
		SPI_SendDataIT(&SPI2handle, context_code, 1);
	} else if (context_code == &spiCommands.CMD_SENSOR_READ->code) {
		// 2. Command
		SPI_SendDataIT(&SPI2handle, context_code, 1);
	}



}

void executeCommand(int8_t *ack) {

	if (context_code == &spiCommands.CMD_LED_CTR->code) {
		spiCommands.CMD_LED_CTR->args[0] = (char *)LED_PIN;
		spiCommands.CMD_LED_CTR->args[1] = (char *)LED_ON;
		SPI_SendArgsIT(&SPI2handle, spiCommands.CMD_LED_CTR, 2);
		*ack = ACK_DONE;
	} else if (context_code == &spiCommands.CMD_SENSOR_READ->code) {


		if (*ack == 1) {
			spiCommands.CMD_SENSOR_READ->args[0] = (char *)ANALOG_PIN0;
			SPI_SendArgsIT(&SPI2handle, spiCommands.CMD_SENSOR_READ, 1);
			*ack += 1;
		} else if (*ack == 2) {
			SPI_ReceiveIT(&SPI2handle, &dummyread, 1);
			delay();
			*ack += 1;
		} else if (*ack == 3) {
			SPI_SendDataIT(&SPI2handle, &dummybyte, 1);
			*ack +=1;
		} else if (*ack == 4) {
			uint8_t analog_read;

			SPI_ReceiveIT(&SPI2handle, &analog_read, 1);
			*ack = ACK_DONE;
		}

	}

}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event) {

	// Ack to enable control flow
	int8_t static ack = -3;

	if (event == SPI_EVENT_TX_CMPLT) {
		if (ack < 0) {
			// Setup for next command
			if (ack == -3) {
				SPI_ReceiveIT(&SPI2handle, &dummyread, 1); 		// cler the RX register
				ack += 1;
			}else if (ack == -1) {
				SPI_ReceiveIT(&SPI2handle, &ackbyte, 1);		// receive the remote ack
				ack += 1;
			}
		}
	} else if (event == SPI_EVENT_RX_CMPLT) {
		if (ack == -2) {
			SPI_SendDataIT(&SPI2handle, &dummybyte, 1); 	// fetch the data
			ack += 1;
		} else if (ack == 0) {
			// EXECUTE NECESSARY CONTROL FLOW COMMAND AT ACK=0
			if (SPI_VerifyResponse(ackbyte)) {
				ack = ACK_NOT_YET;
				executeCommand(&ack);
			}
		} else if (ack) {
			// upper ack control flow is handled by the command function block
		}
	} else if (event == SPI_EVENT_OVR_ERR) {
		SPI_ClearOVR(SPI2);
	}

}


#if 0
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
#endif




