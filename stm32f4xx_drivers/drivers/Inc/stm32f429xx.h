/*
 * stm32f429xx.h
 *
 *  Created on: Sep 14, 2022
 *      Author: omark
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo						volatile
#define __weak						__attribute__((weak))

/*
 * Processor-specific details
 * */

/* ARM Cortex M4 NVIC unit ISERs registers addresses */
#define NVIC_ISER0					((uint32_t*)0xE000E100UL)
#define NVIC_ISER1					((uint32_t*)0xE000E104UL)
#define NVIC_ISER2					((uint32_t*)0xE000E108UL)
#define NVIC_ISER3					((uint32_t*)0xE000E10CUL)
#define NVIC_ISER4					((uint32_t*)0xE000E110UL)
#define NVIC_ISER5					((uint32_t*)0xE000E114UL)
#define NVIC_ISER6					((uint32_t*)0xE000E118UL)
#define NVIC_ISER7					((uint32_t*)0xE000E11CUL)

/* ARM Cortex M4 NVIC unit ICERs registers addresses */
#define NVIC_ICER0					((uint32_t*)0xE000E180UL)
#define NVIC_ICER1					((uint32_t*)0xE000E184UL)
#define NVIC_ICER2					((uint32_t*)0xE000E188UL)
#define NVIC_ICER3					((uint32_t*)0xE000E18CUL)
#define NVIC_ICER4					((uint32_t*)0xE000E190UL)
#define NVIC_ICER5					((uint32_t*)0xE000E194UL)
#define NVIC_ICER6					((uint32_t*)0xE000E198UL)
#define NVIC_ICER7					((uint32_t*)0xE000E19CUL)

/* ARM Cortex M4 NVIC unit IPR registers addresses */
#define NVIC_IPR_BASEADDR			((uint32_t*)0xE000E400UL)

#define NO_IMPLEMENTED_PRIO_BITS	4

/* MEMORY ORGANIZATION RELATED MACROS */
#define FLASH_BASEADDR				0x08000000UL
#define FLASH_SIZE					(2048U * 1024U)
#define SYSMEM_BASEADDR				0x1FFF0000UL
#define SYSMEM_SIZE					(30U * 1024U)
#define OTP_BASEADDR				0x1FFF7800UL
#define SRAM1_BASEADDR				0x20000000UL
#define SRAM1_SIZE					(128U * 1024U)
#define SRAM2_BASEADDR				0x2001C000U
#define SRAM2_SIZE					(16U * 1024U)
#define SRAM3_BASEADDR				0x20020000U
#define SRAM3_SIZE					(64U * 1024U)
#define SRAM_BASEADDR				SRAM1_BASEADDR

/* BUS DOMAINS */
#define PERIPH_BASE					0x40000000UL
#define APB1PERIPH_BASE				0x40000000UL
#define APB2PERIPH_BASE				0x40010000UL
#define AHB1PERIPH_BASE				0x40020000UL
#define AHB2PERIPH_BASE				0x50000000UL
#define AHB3PERIPH_BASE				0xA0000000UL

/* Peripherals hanging on AHB1 bus */
#define GPIOA_BASEADDR				0x40020000UL
#define GPIOB_BASEADDR				0x40020400UL
#define GPIOC_BASEADDR				0x40020800UL
#define GPIOD_BASEADDR				0x40020C00UL
#define GPIOE_BASEADDR				0x40021000UL
#define GPIOF_BASEADDR				0x40021400UL
#define GPIOG_BASEADDR				0x40021800UL
#define GPIOH_BASEADDR				0x40021C00UL
#define GPIOI_BASEADDR				0x40022000UL
#define GPIOJ_BASEADDR				0x40022400UL
#define GPIOK_BASEADDR				0x40022800UL
#define CRC_BASEADDR				0x40023000UL
#define RCC_BASEADDR				0x40023800UL
#define FLASH_IR_BASEADDR			0x40023C00UL
#define BKPSRAM_BASEADDR			0x40024000UL
#define DMA1_BASEADDR				0x40026000UL
#define DMA2_BASEADDR				0x40026400UL
#define ETHERNET_MAC_BASEADDR		0x40028000UL
#define DMA2D_BASEADDR				0x4002B000UL
#define USBOTGHS_BASEADDR			0x40040000UL

/* Peripherals hanging on APB1 bus */
#define TIM2_BASEADDR				0x40000000UL
#define TIM3_BASEADDR				0x40000400UL
#define TIM4_BASEADDR				0x40000800UL
#define TIM5_BASEADDR				0x40000C00UL
#define TIM6_BASEADDR				0x40001000UL
#define TIM7_BASEADDR				0x40001400UL
#define TIM12_BASEADDR				0x40001800UL
#define TIM13_BASEADDR				0x40001C00UL
#define TIM14_BASEADDR				0x40002000UL
#define RTCBKP_BASEADDR				0x40002800UL
#define WWGD_BASEADDR				0x40002C00UL
#define IWDG_BASEADDR				0x40003000UL
#define I2S2EXT_BASEADDR			0x40003400UL
#define SPI2_BASEADDR				0x40003800UL
#define SPI3_BASEADDR				0x40003C00UL
#define I2S3EXT_BASEADDR			0x40004000UL
#define USART2_BASEADDR				0x40004400UL
#define USART3_BASEADDR				0x40004800UL
#define UART4_BASEADDR				0x40004C00UL
#define UART5_BASEADDR				0x40005000UL
#define I2C1_BASEADDR				0x40005400UL
#define I2C2_BASEADDR				0x40005800UL
#define I2C3_BASEADDR				0x40005C00UL
#define CAN1_BASADDR				0x40006400UL
#define CAN2_BASEADDR				0x40006800UL
#define PWR_BASEADDR				0x40007000UL
#define DAC_BASEADDR				0x40007400UL
#define UART7_BASEADDR				0x40007800UL
#define UART8_BASEADDR				0x40007C00UL

/* Peripherals hanging on APB2 bus */
#define TIM1_BASEADDR				0x40010000UL
#define TIM8_BASEADDR				0x40010400UL
#define USART1_BASEADDR				0x40011000UL
#define USART6_BASEADDR				0x40011400UL
#define ADC123_BASEADDR				0x40012000UL
#define SDIO_BASEADDR				0x40012C00UL
#define SPI1_BASEADDR				0x40013000UL
#define SPI4_BASEADDR				0x40013400UL
#define SYSCFG_BASEADDR				0x40013800UL
#define EXTI_BASEADDR				0x40013C00UL
#define TIM9_BASEADDR				0x40014000UL
#define TIM10_BASEADDR				0x40014400UL
#define TIM11_BASEADDR				0x40014800UL
#define SPI5_BASEADDR				0x40015000UL
#define SPI6_BASEADDR				0x40015400UL
#define SAI1_BASEADDR				0x40015800UL
#define LCDTFT_BASEADDR				0x40016800UL

typedef struct
{
	__vo uint32_t MODER;				/*GPIO port mode register*/
	__vo uint32_t OTYPER;				/*GPIO port output type register*/
	__vo uint32_t OSPEEDR;				/*GPIO port output speed register*/
	__vo uint32_t PUPDR;				/*GPIO port pull-up/pull-down register*/
	__vo uint32_t IDR;					/*GPIO port input data register*/
	__vo uint32_t ODR;					/*GPIO port output data register*/
	__vo uint32_t BSRR;					/*GPIO port bit set/reset register*/
	__vo uint32_t LCKR;					/*GPIO port configuration lock register*/
	__vo uint32_t AFR[2];				/*AFR[0]: GPIO alternate function low register,
	 	 	 	 	 	 	 	 	 	  AFR[1]: GPIO alternate function high register*/
} GPIOx_t;

/* GPIOx_t peripheral definition */
#define GPIOA						((GPIOx_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIOx_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIOx_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIOx_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIOx_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIOx_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIOx_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIOx_t*)GPIOH_BASEADDR)
#define GPIOI						((GPIOx_t*)GPIOI_BASEADDR)
#define GPIOJ						((GPIOx_t*)GPIOJ_BASEADDR)
#define GPIOK						((GPIOx_t*)GPIOK_BASEADDR)

typedef struct
{
	__vo uint32_t CR;					/*!<RCC clock control register>*/
	__vo uint32_t PLLCFGR;				/*!<RCC PLL configuration register>*/
	__vo uint32_t CFGR;					/*!<RCC clock configuration register>*/
	__vo uint32_t CIR;					/*!<RCC clock interrupt register>*/
	__vo uint32_t AHB1RSTR;				/*!<RCC AHB1 peripheral reset register>*/
	__vo uint32_t AHB2RSTR;				/*!<RCC AHB2 peripheral reset register>*/
	__vo uint32_t AHB3RSTR;				/*!<RCC AHB3 peripheral reset register>*/
		 uint32_t RES0;					/*!<RESERVED>*/
	__vo uint32_t APB1RSTR;				/*!<RCC APB1 peripheral reset register>*/
	__vo uint32_t APB2RSTR;				/*!<RCC APB2 peripheral reset register>*/
	 	 uint32_t RES1;					/*!<RESERVED>*/
	 	 uint32_t RES2;					/*!<RESERVED>*/
	__vo uint32_t AHB1ENR;				/*!<RCC AHB1 peripheral clock enable register>*/
	__vo uint32_t AHB2ENR;				/*!<RCC AHB2 peripheral clock enable register>*/
	__vo uint32_t AHB3ENR;				/*!<RCC AHB3 peripheral clock enable register>*/
		 uint32_t RES3;					/*!<RESERVED>*/
	__vo uint32_t APB1ENR;				/*!<RCC APB1 peripheral clock enable register>*/
	__vo uint32_t APB2ENR;				/*!<RCC APB2 peripheral clock enable register>*/
		 uint32_t RES4;					/*!<RESERVED>*/
		 uint32_t RES5;					/*!<RESERVED>*/
	__vo uint32_t AHB1LP;				/*!<RCC AHB1 peripheral clock enable in low power mode register>*/
	__vo uint32_t AHB2LP;				/*!<RCC AHB2 peripheral clock enable in low power mode register>*/
	__vo uint32_t AHB3LP;				/*!<RCC AHB3 peripheral clock enable in low power mode register>*/
		 uint32_t RES6;					/*!<RESERVED>*/
	__vo uint32_t APB1LP;				/*!<RCC APB1 peripheral clock enable in low power mode register>*/
	__vo uint32_t APB2LP;				/*!<RCC APB2 peripheral clock enable in low power mode register>*/
		 uint32_t RES7;					/*!<RESERVED>*/
		 uint32_t RES8;					/*!<RESERVED>*/
	__vo uint32_t BDCR;					/*!<RCC Backup domain control register>*/
	__vo uint32_t CSR;					/*!<RCC clock control & status register>*/
		 uint32_t RES9;					/*!<RESERVED>*/
		 uint32_t RES10;				/*!<RESERVED>*/
	__vo uint32_t SSCGR;				/*!<RCC spread spectrum clock generation register>*/
	__vo uint32_t PLLI2SC;				/*!<RCC PLLI2S configuration register>*/

}RCC_xR;

#define RCC							((RCC_xR*)RCC_BASEADDR)

/*
 * peripheral register definition structure for EXTI
 * */
typedef struct
{
	uint32_t EXTI_IMR;
	uint32_t EXTI_EMR;
	uint32_t EXTI_RTSR;
	uint32_t EXTI_FTSR;
	uint32_t EXTI_SWIER;
	uint32_t EXTI_PR;

} EXTI_t;

#define EXTI						((EXTI_t*)EXTI_BASEADDR)

/*
 * SYSCFG registers structure definition
 * */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
		 uint32_t RES[2];
	__vo uint32_t CMPCR;

} SYSCFG_t;

#define SYSCFG						((SYSCFG_t*)SYSCFG_BASEADDR)

typedef struct
{
	__vo uint32_t CR[2];
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

} SPIx_t;

/*
 * stm32f429xx different SPI peripherals
 * */
#define SPI1						((SPIx_t*) SPI1_BASEADDR)
#define SPI2						((SPIx_t*) SPI2_BASEADDR)
#define SPI3						((SPIx_t*) SPI3_BASEADDR)
#define SPI4						((SPIx_t*) SPI4_BASEADDR)
#define SPI5						((SPIx_t*) SPI5_BASEADDR)
#define SPI6						((SPIx_t*) SPI6_BASEADDR)

typedef struct
{
	__vo uint32_t CR[2];
	__vo uint32_t OAR[2];
	__vo uint32_t DR;
	__vo uint32_t SR[2];
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

} I2Cx_t;

/*
 * I2C peripherals
 * */
#define I2C1						((I2Cx_t*) I2C1_BASEADDR)
#define I2C2						((I2Cx_t*) I2C2_BASEADDR)
#define I2C3						((I2Cx_t*) I2C3_BASEADDR)

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR[3];
	__vo uint32_t GTPR;

} USARTx_t;

/*
 * USART peripherals
 * */
#define USART1						((USARTx_t*) USART1_BASEADDR)
#define USART2						((USARTx_t*) USART2_BASEADDR)
#define USART3						((USARTx_t*) USART3_BASEADDR)
#define UART4						((USARTx_t*) UART4_BASEADDR)
#define UART5						((USARTx_t*) UART5_BASEADDR)
#define USART6						((USARTx_t*) USART6_BASEADDR)
#define UART7						((USARTx_t*) UART7_BASEADDR)
#define UART8						((USARTx_t*) UART8_BASEADDR)


/* GPIOx_t peripherals clock enable MACROS */
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()				(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()				(RCC->AHB1ENR |= (1 << 10))

/* GPIOx peripherals clock disable MACROS */
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 10))

/* GPIOx peripheral registers reset */
#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8));}while(0)
#define GPIOJ_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 9));	(RCC->AHB1RSTR &= ~(1 << 9));}while(0)
#define GPIOK_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 10));(RCC->AHB1RSTR &= ~(1 << 10));}while(0)

/* I2Cx peripheral clock enable MACROS */
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))

/* I2Cx peripheral clock disable MACROS */
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))

/* I2Cx peripheral register reset MACROS */
#define I2C1_REG_RESET()			(RCC->APB1RSTR |= (1 << 21))
#define I2C2_REG_RESET()			(RCC->APB1RSTR |= (1 << 22))
#define I2C3_REG_RESET()			(RCC->APB1RSTR |= (1 << 23))

/* SPIx peripherals clock enable MACROS */
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()				(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()				(RCC->APB2ENR |= (1 << 21))

/* SPIx peripherals clock disable MACROS */
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 21))

/* SPIx peripheral register reset MACROS */
#define SPI1_REG_RESET()			(RCC->APB2RSTR |= (1 << 12))
#define SPI2_REG_RESET()			(RCC->APB1RSTR |= (1 << 14))
#define SPI3_REG_RESET()			(RCC->APB1RSTR |= (1 << 15))
#define SPI4_REG_RESET()			(RCC->APB2RSTR |= (1 << 13))
#define SPI5_REG_RESET()			(RCC->APB2RSTR |= (1 << 20))
#define SPI6_REG_RESET()			(RCC->APB2RSTR |= (1 << 21))

/* SPIx peripheral enable MACROS */
#define SPI1_ENABLE()				(SPI1->CR[0] |= (1 << SPI_CR1_SPE))
#define SPI2_ENABLE()				(SPI2->CR[0] |= (1 << SPI_CR1_SPE))
#define SPI3_ENABLE()				(SPI3->CR[0] |= (1 << SPI_CR1_SPE))
#define SPI4_ENABLE()				(SPI4->CR[0] |= (1 << SPI_CR1_SPE))
#define SPI5_ENABLE()				(SPI5->CR[0] |= (1 << SPI_CR1_SPE))
#define SPI6_ENABLE()				(SPI6->CR[0] |= (1 << SPI_CR1_SPE))

/* SPIx peripheral disable MACROS */
#define SPI1_DISABLE()				(SPI1->CR[0] &= ~(1 << SPI_CR1_SPE))
#define SPI2_DISABLE()				(SPI2->CR[0] &= ~(1 << SPI_CR1_SPE))
#define SPI3_DISABLE()				(SPI3->CR[0] &= ~(1 << SPI_CR1_SPE))
#define SPI4_DISABLE()				(SPI4->CR[0] &= ~(1 << SPI_CR1_SPE))
#define SPI5_DISABLE()				(SPI5->CR[0] &= ~(1 << SPI_CR1_SPE))
#define SPI6_DISABLE()				(SPI6->CR[0] &= ~(1 << SPI_CR1_SPE))


/* USARTx/UARTx peripherals clock enable MACROS */
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()				(RCC->APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()				(RCC->APB1ENR |= (1 << 31))

/* USARTx/UARTx peripherals clock disable MACROS */
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 31))

/* SYSCFG peripheral clock enable/disable MACROS */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))


/*
 * GPIOs interrupt configuration/handling/priority MACROS**************************************
 * */

/* return port code to given GPIOx address (used while configuring Interrupt Priority Registers) */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOF) ? 5 :\
									 (x == GPIOG) ? 6 :\
									 (x == GPIOH) ? 7 :\
									 (x == GPIOI) ? 8 :\
									 (x == GPIOJ) ? 9 :\
									 (x == GPIOK) ? 10:0)

/* IRQ numbers for MCU: stm32f429xx */

// GPIOx IRQ numbers
#define IRQ_NO_EXTI0				6U
#define IRQ_NO_EXTI1				7U
#define IRQ_NO_EXTI2				8U
#define IRQ_NO_EXTI3				9U
#define IRQ_NO_EXTI4				10U
#define IRQ_NO_EXTI9_5				23U
#define IRQ_NO_EXTI15_10			40U

// SPI IRQ numbers
#define NVIC_IRQ_NO_SPI1			35U
#define NVIC_IRQ_NO_SPI2			36U
#define NVIC_IRQ_NO_SPI3			51U
#define NVIC_IRQ_NO_SPI4			84U
#define NVIC_IRQ_NO_SPI5			85U
#define NVIC_IRQ_NO_SPI6			86U

// I2C IRQ numbers
#define IRQ_NO_I2C1_EV				31U
#define IRQ_NO_I2C1_ER				32U
#define IRQ_NO_I2C2_EV				33U
#define IRQ_NO_I2C2_ER				34U
#define IRQ_NO_I2C3_EV				72U
#define IRQ_NO_I2C3_ER				73U


/* priority levels */
#define NVIC_IRQ_PRI0				0x0U
#define NVIC_IRQ_PRI1				0x1U
#define NVIC_IRQ_PRI2				0x2U
#define NVIC_IRQ_PRI3				0x3U
#define NVIC_IRQ_PRI4				0x4U
#define NVIC_IRQ_PRI5				0x5U
#define NVIC_IRQ_PRI6				0x6U
#define NVIC_IRQ_PRI7				0x7U
#define NVIC_IRQ_PRI8				0x8U
#define NVIC_IRQ_PRI9				0x9U
#define NVIC_IRQ_PRI10				0xAU
#define NVIC_IRQ_PRI11				0xBU
#define NVIC_IRQ_PRI12				0xCU
#define NVIC_IRQ_PRI13				0xDU
#define NVIC_IRQ_PRI14				0xEU
#define NVIC_IRQ_PRI15				0xFU

/***************************************************************************************/



/* Generic MACROS */
#define ENABLE 						1U
#define DISABLE 					0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET

/* MACROS for SPI peripherals */

/* Bit Position Definitions for register CR1 */
#define SPI_CR1_BIDI_MODE			15U
#define SPI_CR1_BIDI_OEN			14U
#define SPI_CR1_DFF					11U
#define SPI_CR1_RXONLY				10U
#define SPI_CR1_SSM					9U
#define SPI_CR1_MSTR				2U
#define SPI_CR1_CPOL				1U
#define SPI_CR1_CPHA				0U
#define SPI_CR1_BR					3U
#define SPI_CR1_SPE					6U
#define SPI_CR1_SSI					8U

/* Bit Position Definitions for register CR2 */
#define SPI_CR2_TXEIE				7U
#define SPI_CR2_RXNEIE				6U
#define SPI_CR2_ERRIE				5U
#define SPI_CR2_FRF					4U
#define SPI_CR2_SSOE				2U
#define SPI_CR2_TXDMAEN				1U
#define SPI_CR2_RXDMAEN				0U

/* Bit Position Definitions for register SR */
#define SPI_SR_FRE					8U
#define SPI_SR_BSY					7U
#define SPI_SR_OVR					6U
#define SPI_SR_MODF					5U
#define SPI_SR_CRCERR				4U
#define SPI_SR_UDR					3U
#define SPI_SR_CHISDE				2U
#define SPI_SR_TXE					1U
#define SPI_SR_RXNE					0U

/* MACROS for I2Cx peripherals */

/* CR1 register bit positions */
#define I2Cx_CR1_PE					0U
#define I2Cx_CR1_SMBUS				1U
#define I2Cx_CR1_SMBTYPE			3U
#define I2Cx_CR1_ENARP				4U
#define I2Cx_CR1_ENPEC				5U
#define I2Cx_CR1_ENGC				6U
#define I2Cx_CR1_NOSTRETCH			7U
#define I2Cx_CR1_START				8U
#define I2Cx_CR1_STOP				9U
#define I2Cx_CR1_ACK				10U
#define I2Cx_CR1_POS				11U
#define I2Cx_CR1_PEC				12U
#define I2Cx_CR1_ALERT				13U
#define I2Cx_CR1_SWRST				15U

/* CR2 register bit positions */
#define I2Cx_CR2_ITERREN			8U
#define I2Cx_CR2_ITEVTEN			9U
#define I2Cx_CR2_ITBUFEN			10U
#define I2Cx_CR2_DMAEN				11U
#define I2Cx_CR2_LAST				12U

/* OAR1 register bit positions */
#define I2Cx_OAR1_ADD0				0U
#define I2Cx_OAR1_ADDMODE			15

/* OAR2 register bit positions */
#define I2Cx_OAR2_ENDUAL			0U

/* SR1 register bit positions */
#define I2Cx_SR1_SB					0U
#define I2Cx_SR1_ADDR				1U
#define I2Cx_SR1_BTF				2U
#define I2Cx_SR1_ADD10				3U
#define I2Cx_SR1_STOPF				4U
#define I2Cx_SR1_RxNE				6U
#define I2Cx_SR1_TxE				7U
#define I2Cx_SR1_BERR				8U
#define I2Cx_SR1_ARLO				9U
#define I2Cx_SR1_AF					10U
#define I2Cx_SR1_OVR				11U
#define I2Cx_SR1_PECERR				12U
#define I2Cx_SR1_TIMEOUT			14U
#define I2Cx_SR1_SMBALERT			15U

/* SR2 register bit positions */
#define I2Cx_SR2_MSL				0U
#define I2Cx_SR2_BUSY				1U
#define I2Cx_SR2_TRA				2U
#define I2Cx_SR2_GENCALL			4U
#define I2Cx_SR2_SMBDEFAULT			5U
#define I2Cx_SR2_SMBHOST			6U
#define I2Cx_SR2_DUALF				7U

/* CCR register bit positions */
#define I2Cx_CCR_DUTY				14U
#define I2Cx_CCR_FS					15U

/* FLTR register bit positions */
#define I2Cx_FLTR_ANOFF				4U
/******************************************************************************************/

/* MACROS for USARTx peripherals */

/* USARTx bit positions */
#define USART_SR_PE					0U
#define USART_SR_FE					1U
#define USART_SR_NF					2U
#define USART_SR_ORE				3U
#define USART_SR_IDLE				4U
#define USART_SR_RXNE				5U
#define USART_SR_TC					6U
#define USART_SR_TXE				7U
#define USART_SR_LBD				8U
#define USART_SR_CTS				9U

#define USART_CR1_SBK				0U
#define USART_CR1_RWU				1U
#define USART_CR1_RE				2U
#define USART_CR1_TE				3U
#define USART_CR1_IDLEIE			4U
#define USART_CR1_RXNEIE			5U
#define USART_CR1_TCIE				6U
#define USART_CR1_TXEIE				7U
#define USART_CR1_PEIE				8U
#define USART_CR1_PS				9U
#define USART_CR1_PCE				10U
#define USART_CR1_WAKE				11U
#define USART_CR1_M					12U
#define USART_CR1_UE				13U
#define USART_CR1_OVER8				15U

#define USART_CR2_LBDL				5U
#define USART_CR2_LBDIE				6U
#define USART_CR2_LBCL				8U
#define USART_CR2_CPHA				9U
#define USART_CR2_CPOL				10U
#define USART_CR2_CLKEN				11U
#define USART_CR2_LINEN				14U
#define USART_CR2_STOP				12U

#define USART_CR3_EIE				0U
#define USART_CR3_IREN				1U
#define USART_CR3_IRLP				2U
#define USART_CR3_HDSEL				3U
#define USART_CR3_NACK				4U
#define USART_CR3_SCEN				5U
#define USART_CR3_DMAR				6U
#define USART_CR3_DMAT				7U
#define USART_CR3_RTSE				8U
#define USART_CR3_CTSE				9U
#define USART_CR3_CTSIE				10U
#define USART_CR3_ONEBIT			11U


/* USART NVIC IRQ numbers */
#define USART1_IRQ_NO				37U
#define USART2_IRQ_NO				38U
#define USART3_IRQ_NO				39U
#define UART4_IRQ_NO				52U
#define UART5_IRQ_NO				53U
#define USART6_IRQ_NO				71U
#define UART7_IRQ_NO				82U
#define UART8_IRQ_NO				83U

/******************************************************************************************/

#include "stm32f429xx_gpio_driver.h"
#include "stm32f42xx_spi_driver.h"
#include "stm32f429xx_i2c_driver.h"
#include "stm32f429xx_usart_driver.h"
#include "stm32f429xx_rcc_driver.h"

#endif /* INC_STM32F429XX_H_ */
