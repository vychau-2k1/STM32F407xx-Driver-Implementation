/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Oct 3, 2022
 *      Author: user1
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * This is a configuration structure for a SPIx pin
 */
typedef struct
{
	uint8_t SPI_DeviceMode;			//Possible values from @SPI_DeviceMode
	uint8_t SPI_BusConfig;			//Possible values from @SPI_BusConfig
	uint8_t SPI_SclkSpeed;			//Possible values from @SPI_SclkSpeed
	uint8_t SPI_DFF;				//Possible values from @SPI_DFF
	uint8_t SPI_CPOL;				//Possible values from @SPI_CPOL
	uint8_t SPI_CPHA;				//Possible values from @SPI_CPHA
	uint8_t SPI_SSM;				//Possible values from @SPI_SSM
}SPI_Config_t;

/*
 * This is a Handle structure for a SPIx pin
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;		//This holds the base address of SPIx (x = 1, 2, 3) peripherals
	SPI_Config_t SPIConfig;		//This holds SPI pin configuration settings
	uint8_t 	 *pTxBuffer;	//To store the application TX buffer address
	uint8_t		 *pRxBuffer;	//To store the application RX buffer address
	uint32_t	 TxLen;			//To store TX length
	uint32_t	 RxLen;			//To store RX length
	uint8_t		 TxState;		//To store TX state
	uint8_t 	 RxState;		//to store RX state
}SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH 					1
#define SPI_CPOL_LOW					0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH 					1
#define SPI_CPHA_LOW					0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0

/*
 * SPI related status flags definition
 */
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)

/**********************************************************************************************
 * 									APIs supported by this driver
 * 			For more  information about the APIs check the function definitions
 ***********************************************************************************************/

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Initialization - De-initialization GPIO
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Len);
/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi); //Interrupt configuration
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); //Whenever the interrupt triggers the user application can call this function to process that interrupt

/*
 * Other Peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
