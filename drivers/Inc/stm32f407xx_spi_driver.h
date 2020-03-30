/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Mar 26, 2020
 *      Author: vladi
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT 		1
#define SPI_EVENT_RX_CMPLT 		2
#define SPI_EVENT_OVR_ERR	 	3
#define SPI_EVENT_CRR_ERR	 	4


typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

} SPI_Config_t;


typedef struct {
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer;
	uint8_t			**args;
	uint8_t 	 	*pRxBuffer;
	uint32_t	 	TxLen;
	uint32_t 	 	RxLen;
	uint8_t 	 	TxState;
	uint8_t 	 	RxState;

} SPI_Handle_t;

typedef struct {
	uint8_t code;
	char *args[2];

} command;

/*
 * @SPI_DEVICEMODE
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BUSCONFIG
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_Clkspeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/**********************************************************************
 * 						APIs Supported by this driver				  *
 **********************************************************************/

/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

/*
 * Data send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_SendArgs(SPI_RegDef_t *pSPIx, uint8_t **args, uint32_t len);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_SendArgsIT(SPI_Handle_t *pSPIHandle, command *commando, uint32_t len);
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetStatus(SPI_RegDef_t *pSPIx, uint8_t flag);
void SPI_ClearOVR(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
