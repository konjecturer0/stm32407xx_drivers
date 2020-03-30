/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Mar 23, 2020
 *      Author: vladi
 */


#include "stm32f407xx_gpio_driver.h"

/***********************************************************************************
 * 				APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 ***********************************************************************************/

/*
 * Peripheral Clock Setup
 */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if (EnorDi == 1) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}

}

/*
 * Init and Di-
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, 1);

	// 1. Configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	} else {
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

			// 1. Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_RT) {

			// 1. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {

			// 1. Configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] =  portcode << ( 4 * temp2 );

		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp = 0;

	// 2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// 5. Configure the alternate functionality if required
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {

		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 );
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if (pGPIOx == GPIOA) {
		GPIOA_RCC_DI();
	} else if (pGPIOx == GPIOB) {
		GPIOB_RCC_DI();
	} else if (pGPIOx == GPIOC) {
		GPIOC_RCC_DI();
	} else if (pGPIOx == GPIOD) {
		GPIOD_RCC_DI();
	} else if (pGPIOx == GPIOE) {
		GPIOE_RCC_DI();
	} else if (pGPIOx == GPIOF) {
		GPIOF_RCC_DI();
	} else if (pGPIOx == GPIOG) {
		GPIOG_RCC_DI();
	} else if (pGPIOx == GPIOH) {
		GPIOH_RCC_DI();
	} else if (pGPIOx == GPIOI) {
		GPIOI_RCC_DI();
	}

}

/*
 *  Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= ( 1 << PinNumber ); 	 // write 1
	} else {
		pGPIOx->ODR &= ~( 1 << PinNumber );	 // write 0
	}
}
void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Config and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber){

	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}

}


#if 0
struct Task;
typedef (*TaskCallback)(struct Task *);

typedef struct Task {
	uint8_t runNextAt;
	uint8_t timeBetweenRuns;
	TaskCallback callback;
	uint8_t enabled;
} Taskit ;

Taskit ex;

TaskCallback makeit(ex) {

}
#endif













