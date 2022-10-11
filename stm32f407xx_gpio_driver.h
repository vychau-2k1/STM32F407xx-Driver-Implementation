/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 29, 2022
 *      Author: user1
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t 	GPIO_PinNumber;			//Possible values from @GPIO_PIN_NUMBER
	uint8_t		GPIO_PinMode;			//Possible values from @GPIO_PIN_MODE
	uint8_t 	GPIO_PinSpeed;			//Possible values from @GPIO_PIN_SPEED
	uint8_t		GPIO_PinPuPdControl;	//Possible values from @GPIO_PIN_PUPD
	uint8_t		GPIO_PinOPType;			//Possible values from @GPIO_PIN_OP_TYPE
	uint8_t		GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t 		*pGPIOx; 		//This holds the base address of the GPIO port to which the pin belong
	GPIO_PinConfig_t 	GPIO_PinConFig; //This holds GPIO pin configuration settings

}GPIO_Handle_t;
/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODE
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4	//GPIO interrupt type Falling Trigger
#define GPIO_MODE_IT_RT		5	//GPIO interrupt type Rising Trigger
#define GPIO_MODE_IT_RFT	6	//GPIO interrupt type Rising Falling Trigger

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0 	//Output push-pull (reset state)
#define GPIO_OP_TYPE_OD		1	//Output open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
/**********************************************************************************************
 * 									APIs supported by this driver
 * 			For more  information about the APIs check the function definitions
 ***********************************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Initialization - De-initialization GPIO
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi); //Interrupt configuration
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber); //Whenever the interrupt triggers the user application can call this function to process that interrupt



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
