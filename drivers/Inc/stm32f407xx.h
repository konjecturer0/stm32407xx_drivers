/*
 * stm32f407xx.h
 *
 *  Created on: Mar 22, 2020
 *      Author: vladi
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vol volatile
#define __weak __attribute__((weak))

/****************** PRECCESSOR DETAILS ********************?
 *
 */

#define NVIC_ISER0				( (__vol uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (__vol uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (__vol uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (__vol uint32_t*)0xE000E10C )

#define NVIC_ICER0				( (__vol uint32_t*)0xE000E180 )
#define NVIC_ICER1				( (__vol uint32_t*)0xE000E184 )
#define NVIC_ICER2				( (__vol uint32_t*)0xE000E188 )
#define NVIC_ICER3				( (__vol uint32_t*)0xE000E18C )

#define NVIC_PR_BASE_ADDR		( (__vol uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED	4

/*
 * Base addresses of Flash and SRAM memories
 */


#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U // 112KB
#define SRAM2_BASEADDR			0x2001C000U	// 16KB
#define ROM_BASEADDR			0x1FFF0000U // a.k.a System Memory
#define SRAM 					SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PH_BASEADDR			PERIPH_BASEADDR
#define APB2PH_BASEADDR			0x40010000U
#define AHB1PH_BASEADDR			0x40020000U
#define AHB2PH_BASEADDR			0x50000000U
#define AHB3PH_BASEADDR			0x60000000U

/*
 * Define AHB1 Base peripheral addresses
 */

#define GPIOA_BASEADDR			( AHB1PH_BASEADDR + 0x0000 )
#define GPIOB_BASEADDR			( AHB1PH_BASEADDR + 0x0400 )
#define GPIOC_BASEADDR			( AHB1PH_BASEADDR + 0x0800 )
#define GPIOD_BASEADDR			( AHB1PH_BASEADDR + 0x0C00 )
#define GPIOE_BASEADDR			( AHB1PH_BASEADDR + 0x1000 )
#define GPIOF_BASEADDR			( AHB1PH_BASEADDR + 0x1400 )
#define GPIOG_BASEADDR			( AHB1PH_BASEADDR + 0x1800 )
#define GPIOH_BASEADDR			( AHB1PH_BASEADDR + 0x1C00 )
#define GPIOI_BASEADDR			( AHB1PH_BASEADDR + 0x2000 )
#define GPIOJ_BASEADDR			( AHB1PH_BASEADDR + 0x2400 )
#define GPIOK_BASEADDR			( AHB1PH_BASEADDR + 0x2800 )
#define	CRC_BASEADDR			( AHB1PH_BASEADDR + 0x3000 )
#define	RCC_BASEADDR			( AHB1PH_BASEADDR + 0x3800 )
#define	FLASH_RG_BASEADDR		( AHB1PH_BASEADDR + 0x3C00 )
#define	BKPSRAM_BASEADDR		( AHB1PH_BASEADDR + 0x4000 )
#define	DMA1_BASEADDR			( AHB1PH_BASEADDR + 0x6000 )
#define	DMA2_BASEADDR			( AHB1PH_BASEADDR + 0x6400 )
#define ETHMAC_BASEADDR			( AHB1PH_BASEADDR + 0x8000 )
#define DMA2D_BASEADDR			( AHB1PH_BASEADDR + 0xB000 )
#define USB_OTG_HS_BASEADDR		( AHB1PH_BASEADDR + 0x00040000 )

/*
 * Define AHB2 Base peripheral addresses
 */

#define USB_OTG_FS_BASEADDR		( AHB2PH_BASEADDR + 0x0000 )
#define DCMI_BASEADDR			( AHB2PH_BASEADDR + 0x00050000 )
#define CRYP_BASEADDR			( AHB2PH_BASEADDR + 0x00060000 )
#define HASH_BASEADDR			( AHB2PH_BASEADDR + 0x00060400 )
#define RNG_BASEADDR			( AHB2PH_BASEADDR + 0x00060800 )

/*
 * Define APB1 Base peripheral addresses
 */

#define TIM2_BASEADDR			( APB1PH_BASEADDR + 0x0000 )
#define TIM3_BASEADDR			( APB1PH_BASEADDR + 0x0400 )
#define TIM4_BASEADDR			( APB1PH_BASEADDR + 0x0800 )
#define TIM5_BASEADDR			( APB1PH_BASEADDR + 0x0C00 )
#define TIM6_BASEADDR			( APB1PH_BASEADDR + 0x1000 )
#define TIM7_BASEADDR			( APB1PH_BASEADDR + 0x1400 )
#define TIM12_BASEADDR			( APB1PH_BASEADDR + 0x1800 )
#define TIM13_BASEADDR			( APB1PH_BASEADDR + 0x1C00 )
#define TIM14_BASEADDR			( APB1PH_BASEADDR + 0x2000 )
#define RTC_BASEADDR			( APB1PH_BASEADDR + 0x2800 )
#define WWDG_BASEADDR			( APB1PH_BASEADDR + 0x2C00 )
#define IWDG_BASEADDR			( APB1PH_BASEADDR + 0x3000 )
#define IS2Sext_BASEADDR		( APB1PH_BASEADDR + 0x3400 )
#define SPI2_BASEADDR			( APB1PH_BASEADDR + 0x3800 )
#define SPI3_BASEADDR			( APB1PH_BASEADDR + 0x3C00 )
#define I2S3ext_BASEADDR		( APB1PH_BASEADDR + 0x4000 )
#define USART2_BASEADDR			( APB1PH_BASEADDR + 0x4400 )
#define USART3_BASEADDR			( APB1PH_BASEADDR + 0x4800 )
#define UART4_BASEADDR			( APB1PH_BASEADDR + 0x4C00 )
#define UART5_BASEADDR			( APB1PH_BASEADDR + 0x5000 )
#define I2C1_BASEADDR			( APB1PH_BASEADDR + 0x5400 )
#define I2C2_BASEADDR			( APB1PH_BASEADDR + 0x5800 )
#define I2C3_BASEADDR			( APB1PH_BASEADDR + 0x5C00 )
#define CAN1_BASEADDR			( APB1PH_BASEADDR + 0x6400 )
#define CAN2_BASEADDR			( APB1PH_BASEADDR + 0x6800 )
#define PWR_BASEADDR			( APB1PH_BASEADDR + 0x7000 )
#define DAC_BASEADDR			( APB1PH_BASEADDR + 0x7400 )
#define UART7_BASEADDR			( APB1PH_BASEADDR + 0x7800 )
#define UART8_BASEADDR			( APB1PH_BASEADDR + 0x7C00 )

/*
 * Define APB2 Base peripheral addresses
 */

#define TIM1_BASEADDR			( APB2PH_BASEADDR + 0x0000 )
#define TIM8_BASEADDR			( APB2PH_BASEADDR + 0x0400 )
#define USART1_BASEADDR			( APB2PH_BASEADDR + 0x1000 )
#define USART6_BASEADDR			( APB2PH_BASEADDR + 0x1400 )
#define ADCx_BASEADDR			( APB2PH_BASEADDR + 0x2000 )
#define SDIO_BASEADDR			( APB2PH_BASEADDR + 0x2C00 )
#define SPI1_BASEADDR			( APB2PH_BASEADDR + 0x3000 )
// #define SPI4_BASEADDR			( APB2PH_BASEADDR + 0x3400 ) not included
#define SYSCFG_BASEADDR			( APB2PH_BASEADDR + 0x3800 )
#define EXTI_BASEADDR			( APB2PH_BASEADDR + 0x3C00 )
#define TIM9_BASEADDR			( APB2PH_BASEADDR + 0x4000 )
#define TIM10_BASEADDR			( APB2PH_BASEADDR + 0x4400 )
#define TIM11_BASEADDR			( APB2PH_BASEADDR + 0x4800 )
// #define SPI5_BASEADDR			( APB2PH_BASEADDR + 0x5000 ) not included
// #define SPI6_BASEADDR			( APB2PH_BASEADDR + 0x5400 ) not included
// #define SAI1_BASEADDR			( APB2PH_BASEADDR + 0x5800 ) not included
// #define LCDTFT_BASEADDR			( APB2PH_BASEADDR + 0x6000 ) not included


/*
 * TODO: Define Specific Register Structures
 *
typedef struct {
	__vol uint32_t MODER0 	:2;
	__vol uint32_t MODER1 	:2;
	__vol uint32_t MODER2 	:2;
	__vol uint32_t MODER3 	:2;
	__vol uint32_t MODER4 	:2;
	__vol uint32_t MODER5 	:2;
	__vol uint32_t MODER6 	:2;
	__vol uint32_t MODER7 	:2;
	__vol uint32_t MODER8 	:2;
	__vol uint32_t MODER9 	:2;
	__vol uint32_t MODER10 	:2;
	__vol uint32_t MODER11	:2;
	__vol uint32_t MODER12 	:2;
	__vol uint32_t MODER13 	:2;
	__vol uint32_t MODER14 	:2;
	__vol uint32_t MODER15	:2;

} GPIOx_MODE_t;

#define GPIOx_MODE				((GPIOx_MODE_t*)GPIOx_MODE_BASEADDR)
 */


/* Register structures */

typedef struct {
	__vol uint32_t MODER;
	__vol uint32_t OTYPER;
	__vol uint32_t OSPEEDR;
	__vol uint32_t PUPDR;
	__vol uint32_t IDR;
	__vol uint32_t ODR;
	__vol uint32_t BSRR;
	__vol uint32_t LCKR;
	__vol uint32_t AFR[2];

} GPIO_RegDef_t ;


typedef struct {
	__vol uint32_t CR;
	__vol uint32_t PLLCFGR;
	__vol uint32_t CFGR;
	__vol uint32_t CIR;
	__vol uint32_t AHB1RSTR;
	__vol uint32_t AHB2RSTR;
	__vol uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vol uint32_t APB1RSTR;
	__vol uint32_t APB2RSTR;
	__vol uint32_t RESERVED1[2];
	__vol uint32_t AHB1ENR;
	__vol uint32_t AHB2ENR;
	__vol uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vol uint32_t APB1ENR;
	__vol uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vol uint32_t AHB1LPENR;
	__vol uint32_t AHB2LPENR;
	__vol uint32_t AHB3LPENR;
	__vol uint32_t RESERVED4;
	__vol uint32_t APB1LPENR;
	__vol uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vol uint32_t BDCR;
	__vol uint32_t CSR;
	uint32_t RESERVED6[2];
	__vol uint32_t SSCGR;
	__vol uint32_t PLLI2SCFGR;

} RCC_RegDef_t ;


/*
 * Peripheral structure for EXTI
 */

typedef struct {
	__vol uint32_t IMR;
	__vol uint32_t EMR;
	__vol uint32_t RTSR;
	__vol uint32_t FTSR;
	__vol uint32_t SWIER;
	__vol uint32_t PR;

} EXTI_RegDef_t ;

/*
 * Peripheral structure for SYSCFG
 */

typedef struct {
	__vol uint32_t MEMRMP;
	__vol uint32_t PMC;
	__vol uint32_t EXTICR[4];
	__vol uint32_t CMPCR;

} SYSCFG_RegDef_t;

/*
 * Peripheral structure for SPI
 */

typedef struct {
	__vol uint32_t CR1;
	__vol uint32_t CR2;
	__vol uint32_t SR;
	__vol uint32_t DR;
	__vol uint32_t CRCPR;
	__vol uint32_t RXCRCR;
	__vol uint32_t TXCRCR;
	__vol uint32_t I2SCFGR;
	__vol uint32_t I2SPR;

} SPI_RegDef_t;

/*
 * Peripheral definitions
 */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)

// ENABLE ----------------------------------------------------
/*
 * Clock enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()			( RCC->AHB1ENR |= (1 << 8) )


/*
 * Returns GPIO port code
 */

#define GPIO_BASEADDR_TO(x)		( ( x == GPIOA) ? 0 : \
								  ( x == GPIOB) ? 1 : \
								  ( x == GPIOC) ? 2 : \
								  ( x == GPIOD) ? 3 : \
								  ( x == GPIOE) ? 4 : \
								  ( x == GPIOF) ? 5 : \
								  ( x == GPIOG) ? 6 : \
								  ( x == GPIOH) ? 7 : \
								  ( x == GPIOI) ? 8:  0 )

/*
 * Clock enable Macros for I2Cx peripherals
 */

#define IC21_PCLK_EN()			( RCC->APB1ENR |= (1 << 21) )
#define IC22_PCLK_EN()			( RCC->APB1ENR |= (1 << 22) )
#define IC23_PCLK_EN()			( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()			( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()			( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()			( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()			( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()			( RCC->APB2ENR |= (1 << 14) )



// DISABLE -----------------------------------------------------
/*
 * Clock disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 8) )


/*
 * GPIO Port restart (disable)
 */

#define GPIOA_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 0) ); ( RCC->AHB1ENR &= ~(1 << 0) );}while(0)
#define GPIOB_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 1) ); ( RCC->AHB1ENR &= ~(1 << 1) );}while(0)
#define GPIOC_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 2) ); ( RCC->AHB1ENR &= ~(1 << 2) );}while(0)
#define GPIOD_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 3) ); ( RCC->AHB1ENR &= ~(1 << 3) );}while(0)
#define GPIOE_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 4) ); ( RCC->AHB1ENR &= ~(1 << 4) );}while(0)
#define GPIOF_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 5) ); ( RCC->AHB1ENR &= ~(1 << 5) );}while(0)
#define GPIOG_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 6) ); ( RCC->AHB1ENR &= ~(1 << 6) );}while(0)
#define GPIOH_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 7) ); ( RCC->AHB1ENR &= ~(1 << 7) );}while(0)
#define GPIOI_RCC_DI()			do{ ( RCC->AHB1ENR |= (1 << 8) ); ( RCC->AHB1ENR &= ~(1 << 8) );}while(0)

/*
 * Clock disable Macros for I2Cx peripherals
 */

#define IC21_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21) )
#define IC22_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22) )
#define IC23_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 15) )

/*
 * SPI port disable
 */
#define SPI1_RCC_DI()			do{ ( RCC->APB2ENR |= (1 << 12) ); ( RCC->APB2ENR &= ~(1 << 12) );}while(0)
#define SPI2_RCC_DI()			do{ ( RCC->APB1ENR |= (1 << 14) ); ( RCC->APB1ENR &= ~(1 << 14) );}while(0)
#define SPI3_RCC_DI()			do{ ( RCC->APB1ENR |= (1 << 15) ); ( RCC->APB1ENR &= ~(1 << 15) );}while(0)

/*
 * Clock disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 5) )

/*
 * IRQ (Interrupt Request) Numbers of EXTI for STM32F407xx MCU
 */

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

/*
 * IRQ (Interrupt Request) Numbers of SPI for STM32F407xx MCU
 */
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51

/*
 * Clock disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 14) )


// Generic Macros
#define THISSET 					1
#define THISRESET					0
#define GPIO_PIN_SET 			THISSET
#define GPIO_PIN_RESET			THISRESET

#if 0
/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __vol  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __vol uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __vol uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __vol uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __vol uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __vol uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __vol uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __vol uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __vol uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __vol uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __vol uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __vol uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __vol uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __vol uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __vol  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __vol  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __vol  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __vol  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __vol  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __vol uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

#define SCS_BASE            (0xE000E000UL)							  /*!< System Control Space Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#endif


#if 0
/*
 * BIT DEFINITIONS OF SPI Peripheral
 */

typedef struct {
	uint16_t CPHA		:1;
	uint16_t CPOL		:1;
	uint16_t MSTR		:1;
	uint16_t BR			:3;
	uint16_t SPE		:1;
	uint16_t LSB_FIRST	:1;
	uint16_t SSI		:1;
	uint16_t SSM		:1;
	uint16_t RX_ONLY	:1;
	uint16_t DFF		:1;
	uint16_t CRC_NEXT	:1;
	uint16_t CRC_EN		:1;
	uint16_t BIDI_OE	:1;
	uint16_t BIDI_MODE	:1;

} SPI_CR1_RegDef_t ;

#define SPI_CR1			(SPI_CR1_RegDef_t*)SPI_RegDef_t
#endif



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"


#endif /* INC_STM32F407XX_H_ */
