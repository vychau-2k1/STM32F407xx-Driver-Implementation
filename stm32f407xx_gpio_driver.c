/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 29, 2022
 *      Author: user1
 */
#include "stm32f407xx_gpio_driver.h"


/***************************************************
 * @functionName	- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @parameter[in]	- Base address of the GPIO peripheral
 * @parameter[in]	- ENABLE or DISABLE macros
 * @parameter[in] 	-
 *
 * @return 			- none
 *
 * @note			- none
 *
 **************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/***************************************************
 * @functionName	- GPIO_Init
 *
 * @brief			- This function Initialization
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			- none
 *
 * @note			- none
 *
 **************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temporary register

	//Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConFig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConFig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}
	else
	{
		//This part is Interrupt mode
		if(pGPIOHandle->GPIO_PinConFig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure Falling trigger selection register (FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConFig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure Rising trigger selection register (RTSR)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConFig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1/ Configure Both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN(); //Enable peripheral clock before using this
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enable the exti interrupt  delivery by using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
	}
	temp = 0; //Reset temporary register
	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConFig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;
	//3. Configure the pull up - pull down settings
	temp = (pGPIOHandle->GPIO_PinConFig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	temp = 0;
	//4. Configure the output types
	temp = (pGPIOHandle->GPIO_PinConFig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	temp = 0;
	//5. Configure the alternative functionality
	if(pGPIOHandle->GPIO_PinConFig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//Configure the alt function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConFig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConFig.GPIO_PinAltFunMode << (4 * temp2)); //setting
	}
}

/***************************************************
 * @functionName	- GPIO_DeInit
 *
 * @brief			- This function De-initialization GPIO port
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			- none
 *
 * @note			- none
 *
 **************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
}

/***************************************************
 * @functionName	- GPIO_ReadFromInputPin
 *
 * @brief			-
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/***************************************************
 * @functionName	- GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);

	return value;
}

/***************************************************
 * @functionName	- GPIO_WriteFromOutputPin
 *
 * @brief			-
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the corresponding to the pinNumber
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***************************************************
 * @functionName	- GPIO_WriteFromOutputPort
 *
 * @brief			-
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***************************************************
 * @functionName	- GPIO_ToggleOutputPin
 *
 * @brief			-
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/***************************************************
 * @functionName	- GPIO_IRQInterruptConfig
 *
 * @brief			- IRQ Interrupt configuration
{
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/***************************************************
 * @functionName	- GPIO_IRQPriorityConfig
 *
 * @brief			- IRQ Priority configuration
{
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First lets find out the IRQPriority register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/***************************************************
 * @functionName	- GPIO_IRQHandling
 *
 * @brief			- Whenever the interrupt triggers the user application can call this function to process that interrupt
{
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding  to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
