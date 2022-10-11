/*
 * stm32f407xx.h
 *
 *  Created on: Sep 28, 2022
 *      Author: user1
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo 	volatile
#define __weak 	__attribute__((weak))

/****************************************START: Processor Specific Details******************************
 *
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4			((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5			((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6			((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7			((__vo uint32_t*)0xE000E11C)

 /*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)
#define NVIC_ICER4			((__vo uint32_t*)0XE000E190)
#define NVIC_ICER5			((__vo uint32_t*)0XE000E194)
#define NVIC_ICER6			((__vo uint32_t*)0XE000E198)
#define NVIC_ICER7			((__vo uint32_t*)0XE000E19C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U //unsigned value  define FLASH base address
#define SRAM1_BASEADDR		0x20000000U //112kb
#define SRAM2_BASEADDR		0x2001C000U //The result of convert 112*1024 DEC to HEX 1C000
#define ROM_BASEADDR		0x1FFF0000U	//system memory Table 5 reference manual
#define SRAM 				SRAM1_BASEADDR
/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR			0x40000000U	//Base address of Peripheral
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U //Offset 0x00010000
#define AHB1PERIPH_BASEADDR		0x40020000U //Offset 0x00020000
#define AHB2PERIPH_BASEADDR		0x50000000U //Offset 0x10000000
/*
 * Base address of peripheral which are hanging on AHB1 Bus
 * TODO: Complete for all other peripherals
 */
//Define base address of all GPIOs on AHB1 (GPIOA to GPIOI)
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000) //0x0000 is offset, we will find it in table 1
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800) //define RCC base address on AHB1
/*
 * Base address of peripheral which are hanging on APB1 Bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00) //UART doesn't support synchronous communication
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000) //That reason why it's name UART instead of USART
/*
 * Base address of peripheral which are hanging on APB2 Bus
 */
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

/******************************peripheral register definition structures***********************
 * Note: Registers of a peripheral are specific to MCU STM32F407
 */

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		//GPIO port mode register 							/Offset: 0x00
	__vo uint32_t OTYPER;  		//GPIO port output type register 					/Offset: 0x04
	__vo uint32_t OSPEEDR;		//GPIO port output speed register 					/Offset: 0x08
	__vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register 				/Offset: 0x0C
	__vo uint32_t IDR;			//GPIO port input data register 					/Offset: 0x10
	__vo uint32_t ODR;			//GPIO port output data register 					/Offset: 0x14
	__vo uint32_t BSRR;			//GPIO port bit set/reset register 					/Offset: 0x18
	__vo uint32_t LCKR;			//GPIO port configuration lock register 			/Offset: 0x1C
	__vo uint32_t AFR[2];		//AFR[0] -> GPIO alternate function low register 	/Offset: 0x20
								//AFR[1] -> GPIO alternate function high register 	/Offset: 0x24
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;			//RCC clock control register						/Offset: 0x00
	__vo uint32_t PLLCFGR;		//RCC PLL configuration register 					/Offset: 0x04
	__vo uint32_t CFGR;			//RCC clock configuration register					/Offset: 0x08
	__vo uint32_t CIR;			//RCC clock interrupt register						/Offset: 0x0C
	__vo uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register				/Offset: 0x10
	__vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register				/Offset: 0x14
	__vo uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register				/Offset: 0x18
	uint32_t	  RESERVED0;	//Reserved											/Offset: 0x1C
	__vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register				/Offset: 0x20
	__vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register				/Offset: 0x24
	uint32_t	  RESERVED1[2];	//Reserved											/Offset: 0x28->0x2C
	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register 		/Offset: 0x30
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register 		/Offset: 0x34
	__vo uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable register 		/Offset: 0x38
	uint32_t	  RESERVED2;	//Reserved											/Offset: 0x3C
	__vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register			/Offset: 0x40
	__vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register			/Offset: 0x44
	uint32_t	  RESERVED3[2];	//Reserved											/Offset: 0x48->0x4C
	__vo uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register /Offset: 0x50
	__vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register /Offset: 0x54
	__vo uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register /Offset: 0x58
	uint32_t	  RESERVED4;	//Reserved											/Offset: 0x5C
	__vo uint32_t APB1LPENR; 	//RCC APB1 peripheral clock enable in low power mode register /Offset: 0x60
	__vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enable in low power mode register /Offset: 0x64
	uint32_t	  RESERVED5[2];	//Reserved											/Offset: 0x68->0x6C
	__vo uint32_t BDCR;			//RCC Backup domain control register				/Offset: 0x70
	__vo uint32_t CSR;			//RCC clock control & status register				/Offset: 0x74
	uint32_t	  RESERVED6[2];	//Reserved											/Offset: 0x78->0x7C
	__vo uint32_t SSCGR;		//RCC spread spectrum clock generation register		/Offset: 0x80
	__vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register 				/Offset: 0x84
	//Open register
	__vo uint32_t PLLSAICFGR;	//RCC PLLSAI configuration register 				/Offset: 0x88
	__vo uint32_t DCKCFGR;		//RCC DCK configuration register					/Offset: 0x8C
	__vo uint32_t CKGATENR;		//													/Offset: 0x90
	__vo uint32_t DCKCFGR2;		//RCC DCK2 configuration register					/Offset: 0x94
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR; 			//Interrupt mask register							/Offset: 0x00
	__vo uint32_t EMR;			//Event mask register								/Offset: 0x04
	__vo uint32_t RTSR;			//Rising trigger selection register					/Offset: 0x08
	__vo uint32_t FTSR;			//Falling trigger selection register				/Offset: 0x0C
	__vo uint32_t SWIER;		//Software interrupt event register					/Offset: 0x10
	__vo uint32_t PR;			//Pending register									/Offset: 0x14
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP; 		//SYSCFG memory remap register							/Offset: 0x00
	__vo uint32_t PMC;			//SYSCFG peripheral mode configuration register			/Offset: 0x04
	__vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register 1->4	/Offset: 0x08->0x14
	uint32_t RESERVED1[2];		//Reserved												/Offset: 0x18, 0x1C
	__vo uint32_t CMPCR;		//Compensation cell control register					/Offset: 0x20
	uint32_t RESERVED2[2];		//Reserved												/Offset: 0x24, 0x28
	__vo uint32_t CFGR;			//Configuration register								/Offset: 0x2C
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;			//SPI control register 1								/Offset: 0x00
	__vo uint32_t CR2;			//SPI control register 2								/Offset: 0x04
	__vo uint32_t SR;			//SPI status register									/Offset: 0x08
	__vo uint32_t DR;			//SPI status register									/Offset: 0x0C
	__vo uint32_t CRCPR;		//SPI CRC polynomial register							/Offset: 0x10
	__vo uint32_t RXCRCR;		//SPI RX CRC register									/Offset: 0x14
	__vo uint32_t TXCRCR;		//SPI TX CRC register									/Offset: 0x18
	__vo uint32_t I2SCFGR;		//SPI_I2S configuration register						/Offset: 0x1C
	__vo uint32_t I2SPR; 		//SPI_I2S prescaler register							/Offset: 0x20
}SPI_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)
/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOD) ? 3 :\
									  (x == GPIOE) ? 4 :\
									  (x == GPIOF) ? 5 :\
									  (x == GPIOG) ? 6 :\
									  (x == GPIOH) ? 7 :\
									  (x == GPIOI) ? 8 : 0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB1RSTR &= ~(1 << 13)); }while(0)

/*
 * IRQ(Interrupt request number) of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: you may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

/*
 * Macros for all the possible priority levels
 */
#define	NVIC_IRQ_PRI0			0
#define	NVIC_IRQ_PRI15			15

//Some generic macros
#define ENABLE 			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
