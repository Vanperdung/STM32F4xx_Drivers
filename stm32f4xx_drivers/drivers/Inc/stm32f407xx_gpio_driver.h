/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 27, 2021
 *      Author: dung
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;					// @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode; 					// @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;					// @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;			// @GPIO_PIN_PUPD
	uint8_t	GPIO_PinOPType;					// @GPIO_PIN_OP_TYPES
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/*
 * This is a handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;			// This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
	
}GPIO_Handle_t;

/*
 * @GPIO_PIN_ALTFUNMODE
 */
#define AF0 					0
#define AF1 					1
#define AF2 					2
#define AF3 					3
#define AF4 					4
#define AF5 					5
#define AF6 					6
#define AF7 					7
#define AF8 					8
#define AF9 					9
#define AF10 					10
#define AF11					11
#define AF12					12
#define AF13					13
#define AF14					14
#define AF15					15

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible mode
 */

#define GPIO_MODE_INPUT 		0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2



/*************************************************************************************************
 * 								APIs supported by this driver
 * 	For more information about the APIs check the function definitions
 ************************************************************************************************/


//Init, De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ configuration, IRQ priority configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
