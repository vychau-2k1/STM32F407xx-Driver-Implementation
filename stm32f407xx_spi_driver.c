/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 3, 2022
 *      Author: user1
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/***************************************************
 * @functionName	- SPI_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given SPIx
 *
 * @parameter[in]	- Base address of the SPI peripheral
 * @parameter[in]	- ENABLE or DISABLE macros
 * @parameter[in] 	-
 *
 * @return 			- none
 *
 * @note			- none
 *
 **************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/***************************************************
 * @functionName	- SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First lets configure the SPI_CR1 register

	uint32_t tempReg = 0;

	//Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configure the SSM
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempReg;
}

/***************************************************
 * @functionName	- SPI_DeInit
 *
 * @brief			- This function De-Initialization
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			- none
 *
 * @note			- none
 *
 **************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//1. Disable RCC
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
	//2. Set SPE in CR1 register to 0
	// SPI_CR1_SPE = 0
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/***************************************************
 * @functionName	- SPI_SendData
 *
 * @brief			-
 *
 * @parameter[in]	- Base address of the SPI peripheral
 * @parameter[in]	-
 * @parameter[in] 	- Length of the Buffer
 *
 * @return 			- None
 *
 * @note			- This is Blocking call
 *
 **************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bits
			//1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			//2. Decrement length of TxBuffer
			Len--;
			Len--;
			//3. Increment TxBuffer
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bits
			//1. Load the data into the DR
			pSPIx->DR = *pTxBuffer;
			//2. Decrement length of TxBuffer
			Len--;
			//3. Increment TxBuffer
			pTxBuffer++;
		}
	}
}

/***************************************************
 * @functionName	- SPI_ReceiveData
 *
 * @brief			-
 *
 * @parameter[in]	- Base address of the SPI peripheral
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bits
			//1. Load the data from the DR to RXBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			//2. Decrement length of TxBuffer
			Len--;
			Len--;
			//3. Increment TxBuffer
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 bits
			//1. Load the data from the DR to RXBuffer address
			*pRxBuffer = pSPIx->DR;
			//2. Decrement length of TxBuffer
			Len--;
			//3. Increment TxBuffer
			pRxBuffer++;
		}
	}
}

/***************************************************
 * @functionName	- SPI_SendDataIT
 *
 * @brief			- Send data with interrupt mode
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	- Length of the Buffer
 *
 * @return 			- State
 *
 * @note			- This is Blocking call
 *
 **************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the TX buffer address and length information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same  SPI peripheral until the transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}
	return state;
}

/***************************************************
 * @functionName	- SPI_ReceiveDataIT
 *
 * @brief			- Receive data with interrupt mode
 *
 * @parameter[in]	-
 * @parameter[in]	-
 * @parameter[in] 	- Length of the Buffer
 *
 * @return 			- State
 *
 * @note			- This is Blocking call
 *
 **************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the TX buffer address and length information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same  SPI peripheral until the transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled by the ISR code
	}
	return state;
}

/***************************************************
 * @functionName	- SPI_PeripheralControl
 *
 * @brief			-
 *
 * @parameter[in]	- Base address of the SPI peripheral
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***************************************************
 * @functionName	- SPI_SSIConfig
 *
 * @brief			- This function make NSS signal internally high and avoids MODF error
 *
 * @parameter[in]	- Base address of the SPI peripheral
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/***************************************************
 * @functionName	- SPI_SSOEConfig
 *
 * @brief			- This function make NSS signal internally high and avoids MODF error
 *
 * @parameter[in]	- Base address of the SPI peripheral
 * @parameter[in]	-
 * @parameter[in] 	-
 *
 * @return 			-
 *
 * @note			-
 *
 **************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
/***************************************************
 * @functionName	- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}

/***************************************************
 * @functionName	- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/***************************************************
 * @functionName	- SPI_IRQHandling
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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	//First let check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Let check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//Check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//Handle OVR error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/*
 * Some helping function implementation
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bits
		//1. Load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		//2. Decrement length of TxBuffer
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		//3. Increment TxBuffer
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bits
		//1. Load the data into the DR
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		//2. Decrement length of TxBuffer
		pSPIHandle->TxLen--;
		//3. Increment TxBuffer
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the SPI transmission and inform the application that TX is over
		//This prevent interrupts from setting up of TXE flag
		/*
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
		*/
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bits
		//1. Load the data from the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		//2. Decrement length of TxBuffer
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		//3. Decrement TxBuffer
		(uint16_t*)pSPIHandle->pRxBuffer--;
	}
	else
	{
		//8 bits
		//1. Load the data from the DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		//2. Decrement length of TxBuffer
		pSPIHandle->RxLen--;
		//3. Decrement TxBuffer
		pSPIHandle->pRxBuffer--;
	}
	if(!pSPIHandle->RxLen)
	{
		//RxLen is zero, so close the SPI reception and inform the application that RX is over
		//This prevent interrupts from setting up of TXE flag
		/*
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
		*/
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the over flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	/*
	 * This is a weak implementation, the application may override this function
	 */
}
