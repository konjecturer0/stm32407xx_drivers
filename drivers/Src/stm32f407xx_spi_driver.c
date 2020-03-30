/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Mar 26, 2020
 *      Author: vladi
 */


#include "stm32f407xx_spi_driver.h"
#include <stdio.h>

void static spi_txe_handle(SPI_Handle_t *pSPIHandle);

void static spi_rxe_handle(SPI_Handle_t *pSPIHandle);

void static spi_ovr_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnDi) {

	if (EnDi == 1) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}

}

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// Configure the SPI_CR1 Register
	uint32_t tempreg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, 1);

	// 1. Configure the Device mode ( << 2)
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
	// pSPIHandle->pSPIx->CR1.MSTR = pSPIHandle->SPIConfig.SPI_DeviceMode [with object oriented approach]

	// 2. Configure the BusConfig
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {

		tempreg &= ~(1 << 15);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {

		tempreg |= (1 << 15);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {

		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);

	}

	// 3. Configure the SCLK Speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	// 4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	// 6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	// 7. Configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_Deinit(SPI_RegDef_t *pSPIx) {

	if (pSPIx == SPI1) {
		SPI1_RCC_DI();
	} else if (pSPIx == SPI2) {
		SPI2_RCC_DI();
	} else if (pSPIx == SPI3) {
		SPI3_RCC_DI();
	}

}
/*
 * Data send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

	while(len > 0) {

		while (SPI_GetStatus(pSPIx, 1) == 0);

		 // By the arguments you assume the length is in terms of **bytes**
		 // [In theory a byte represents just a number while 2 bytes represent 16 bits or 2 numbers]
		if ( ((pSPIx->CR1 >> 11) & 0x1) ) {
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		} else {
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}

}

void SPI_SendArgs(SPI_RegDef_t *pSPIx, uint8_t **args, uint32_t len) {
	while(len > 0) {

		while (SPI_GetStatus(pSPIx, 1) == 0);

		 // By the arguments you assume the length is in terms of **bytes**
		 // [In theory a byte represents just a number while 2 bytes represent 16 bits or 2 numbers]
		if ( ((pSPIx->CR1 >> 11) & 0x1) ) {
			pSPIx->DR = *((uint16_t*)args);
			len--;
			len--;
			(uint16_t*)args++; // incrementing the pointer that it points to
		} else {
			printf("[BUFFER]: %d", (*((uint8_t*)args)));
			fflush(stdout);
			pSPIx->DR = (*((uint8_t*)args));
			len--;
			args++; 	// incrementing the pointer that it points to
		}
	}
}

void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {

	while(len > 0) {

		while (SPI_GetStatus(pSPIx, 0) == 0);

		 // By the arguments you assume the length is in terms of **bytes**
		 // [In theory a byte represents just a number while 2 bytes represent 16 bits or 2 numbers]
		if ( ((pSPIx->CR1 >> 11) & 0x1) ) {
			// 16 DFF
			*((uint16_t*)pRxBuffer) = (uint16_t) pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer++;
		} else {
			// 8 DFF
			*pRxBuffer = (uint8_t) pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}

}

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == 1) {

		if (IRQNumber <= 31) {
			// ISER0 Register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// ISER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));

		} else if (IRQNumber < 64  && IRQNumber < 95) {
			// ISER2 Register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64));
		}

	} else {

		if (IRQNumber <= 31) {
			// ICER0 Register
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// ICER1 Register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));

		} else if (IRQNumber < 64  && IRQNumber < 95) {
			// ICER2 Register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64));

		}

	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other SPI Peripheral Control
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if (EnorDi == 1) {
		pSPIx->CR1 |= ( 1 << 6 );
	} else {
		pSPIx->CR1 &= ~( 1 << 6 );
	}

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == 1) {
		pSPIx->CR1 |= ( 1 << 8 );
	} else {
		pSPIx->CR1 &= ~( 1 << 8 );
	}


}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == 1) {
		pSPIx->CR2 |= ( 1 << 2 );
	} else {
		pSPIx->CR2 &= ~( 1 << 2 );
	}


}

uint8_t SPI_GetStatus(SPI_RegDef_t *pSPIx, uint8_t flag) {

	if ( (pSPIx->SR >> flag) & 0x1 ) {
		return 1;
	} else {
		return 0;
	}

}

uint8_t SPI_SendArgsIT(SPI_Handle_t *pSPIHandle, command *commando, uint32_t len) {

	// while loop constantly checking the following state (clever trick)
	uint8_t state = pSPIHandle->TxState;

	// check
	if (state != SPI_BUSY_IN_TX) {

		// 1. Save the TX Buffer address and Len info in some global variables
		pSPIHandle->args = (uint8_t **)commando->args;
		pSPIHandle->TxLen = len;

		// 2. Mark the SPI as busy in transmission so that no other code can take over the same SPI peripheral until the transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXIE control bit to get the interrupt whenever the TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << 7);

		// 4. Data Transmission will be handled by the ISR Code (implemented).

	}

	return pSPIHandle->TxState;

}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {

	// while loop constantly checking the following state (clever trick)
	uint8_t state = pSPIHandle->TxState;

	// check
	if (state != SPI_BUSY_IN_TX) {

		// 1. Save the TX Buffer address and Len info in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// 2. Mark the SPI as busy in transmission so that no other code can take over the same SPI peripheral until the transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXIE control bit to get the interrupt whenever the TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << 7);

		// 4. Data Transmission will be handled by the ISR Code (implemented).

	}

	return pSPIHandle->TxState;

}

uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {

	// while loop constantly checking the following state (clever trick)
	uint8_t state = pSPIHandle->RxState;

	// check
	if (state != SPI_BUSY_IN_RX) {

		// 1. Save the RX Buffer address and Len info in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// 2. Mark the SPI as busy in transmission so that no other code can take over the same SPI peripheral until the reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXE control bit to get the interrupt whenever the RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << 6);

		// 4. Data Transmission will be handled by the ISR Code (implemented).

	}

	return pSPIHandle->RxState;

}


// Handler functions

void static spi_txe_handle(SPI_Handle_t *pSPIHandle) {

	while (pSPIHandle->TxLen > 0) {
		if ( ((pSPIHandle->pSPIx->CR1 >> 11) & 0x1) ) {
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		} else {
			if ( (*(pSPIHandle->args)) != 0 ) {
				pSPIHandle->pSPIx->DR = *(pSPIHandle->args);
				pSPIHandle->args++;
			} else {
				pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
				pSPIHandle->pTxBuffer++;
			}
			pSPIHandle->TxLen--;
		}
	}


	if (!pSPIHandle->TxLen) {
		// close SPI transmission - prevents interrupts from TXE
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}

void static spi_rxe_handle(SPI_Handle_t *pSPIHandle) {

	while (pSPIHandle->RxLen > 0) {
		if ( ((pSPIHandle->pSPIx->CR1 >> 11) & 0x1) ) {
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			(uint16_t*)pSPIHandle->pRxBuffer++;
		} else {
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
	}

	if (!pSPIHandle->RxLen) {
		// close SPI transmission - prevents interrupts from TXE
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}

}

void static spi_ovr_handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;
	// 1. Clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	// 2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

	uint8_t temp1, temp2;

	// Check for TXE
	temp1 = (pSPIHandle->pSPIx->SR >> 1) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> 7) & 1;

	if (temp1 && temp2) {
		// Handle TXE
		spi_txe_handle(pSPIHandle);
		return;
	}

	// check for RXE
	temp1 = (pSPIHandle->pSPIx->SR >> 0) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> 6) & 1;

	if (temp1 && temp2) {
		// Handle TXE
		spi_rxe_handle(pSPIHandle);
		return;
	}

	// check for ERR [OVR]
	temp1 = (pSPIHandle->pSPIx->SR >> 6) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> 4) & 1;

	if (temp1 && temp2) {
		// Handle TXE
		spi_ovr_handle(pSPIHandle);
		return;
	}

}

void SPI_ClearOVR(SPI_RegDef_t *pSPIx) {
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << 7 );
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->args = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << 6 );
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}




