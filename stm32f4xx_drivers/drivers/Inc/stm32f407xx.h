/*
 * stm32f407xx.h
 *
 *  Created on: Oct 26, 2021
 *      Author: dung
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))


/**********************************START: Processor Specific Details***********************************/

/*
 *  ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0							((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE000E10C)

/*
 *  ARM Cortex Mx Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0							((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0xE000E18C)

/*
 * 	ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASEADDR					((__vo uint32_t*)0xE000E400)



#define DISABLE 			0
#define ENABLE 				1
#define SET 				1
#define RESET				0
#define GPIO_PIN_SET		1
#define GPIO_PIN_RESET		0

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000UL
#define SRAM1_BASEADDR						0x20000000UL
#define SRAM2_BASEADDR						0x2001C000UL
#define ROM_BASEADDR						0x1FFF0000UL
#define SRAM_BASEADDR						SRAM1_BASEADDR

/*
 * Peripheral base addresses of different bus domains
 */

#define APB1PERIPH_BASE						0x40000000UL
#define APB2PERIPH_BASE						0x40010000UL
#define AHB1PERIPH_BASE						0x40020000UL
#define AHB2PERIPH_BASE						0x50000000UL
#define AHB3PERIPH_BASE						0x60000000UL

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						0x40020000UL
#define GPIOB_BASEADDR						0x40020400UL
#define GPIOC_BASEADDR						0x40020800UL
#define GPIOD_BASEADDR						0x40020C00UL
#define GPIOE_BASEADDR						0x40021000UL

#define RCC_BASEADDR 						0x40023800UL

#define EXTI_BASEADDR						0x40013C00UL

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR						0x40000000UL
#define TIM3_BASEADDR						0x40000400UL
#define TIM4_BASEADDR						0x40000800UL
#define TIM5_BASEADDR						0x40000C00UL
#define TIM6_BASEADDR						0x40001000UL
#define TIM7_BASEADDR						0x40001400UL
#define TIM12_BASEADDR						0x40001800UL
#define TIM13_BASEADDR						0x40001C00UL
#define TIM14_ASEADDR						0x40002000UL
#define I2C1_BASEADDR						0x40005400UL
#define I2C2_BASEADDR  						0x40005800UL
#define I2C3_BASEADDR						0x40005C00UL
#define USART2_BASEADDR						0x40004400UL
#define USART3_BASEADDR						0x40004800UL
#define UART4_BASEADDR						0x40004C00UL
#define UART5_BASEADDR						0x40005000UL
#define SPI2_BASEADDR						0x40003800UL
#define SPI3_BASEADDR						0x40003C00UL

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR						0x40013C00UL
#define USART1_BASEADDR						0x40011000UL
#define USART6_BASEADDR						0x40011400UL
#define SPI1_BASEADDR						0x40013000UL
#define SYSCFG_BASEADDR						0x40013800UL
#define TIM1_BASEADDR						0x40010000UL
#define TIM8_BASEADDR						0x40010400UL
#define TIM9_BASEADDR						0x40014000UL
#define TIM10_BASEADDR						0x40014400UL
#define TIM11_BASEADDR						0x40014800UL
#define ADC_BASEADDR						0x40012000UL

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 					6
#define IRQ_NO_EXTI1 					7
#define IRQ_NO_EXTI2 					8
#define IRQ_NO_EXTI3 					9
#define IRQ_NO_EXTI4 					10
#define IRQ_NO_EXTI9_5 					23
#define IRQ_NO_EXTI15_10 				40
#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2       				36
#define IRQ_NO_SPI3         			51
#define IRQ_NO_I2C1_EV     				31
#define IRQ_NO_I2C1_ER     				32
#define IRQ_NO_USART1	    			37
#define IRQ_NO_USART2	    			38
#define IRQ_NO_USART3	    			39
#define IRQ_NO_UART4	    			52
#define IRQ_NO_UART5	    			53
#define IRQ_NO_USART6	    			71




/***********************Peripheral register definition structures**********************************/
/*
 * Note: Registers of a peripheral are specific to MCU
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;			// GPIO port mode register 						Address offset: 0x00
	__vo uint32_t OTYPER;			// GPIO port output type register				Address offset: 0x04
	__vo uint32_t OSPEEDR;			// GPIO port output speed register 				Address offset: 0x08
	__vo uint32_t PUPDR;			// GPIO port pull-up/pull-down register 		Address offset: 0x0C
	__vo uint32_t IDR;				// GPIO port input data register 				Address offset: 0x10
	__vo uint32_t ODR;				// GPIO port output data register				Address offset: 0x14
	__vo uint32_t BSRR;				// GPIO port bit set/reset register  			Address offset: 0x18
	__vo uint32_t LCKR;				// GPIO port configuration lock register 		Address offset: 0x1C
	__vo uint32_t AFR[2];			// GPIO alternate function low register AFR[0]	Address offset: 0x20
									// GPIO alternate function high register AFR[1]	Address offset: 0x24
} GPIO_RegDef_t;

/*
 * Peripheral definitions base address
 */

#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*)GPIOE_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define USART1  				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  					((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  					((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  				((USART_RegDef_t*)USART6_BASEADDR)

#define I2C1  					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  					((I2C_RegDef_t*)I2C3_BASEADDR)

#define SPI2 					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)

/*
 * Peripheral register definition structures for RCC
 */

typedef struct
{
	  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
	  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
	  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR; 				/*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t EMR;				/*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t RTSR;				/*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t FTSR;				/*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t SWIER;			/*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t PR;				/*!< TODO,     										Address offset: 0x14 */

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP; 			/*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t PMC;				/*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t EXTICR[4];		/*!< TODO,     										Address offset: 0x08 - 0x14 */
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;			/*!< TODO,     										Address offset: 0x20 */
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;				/*!< TODO,     										Address offset: 0x2C */

}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */

typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
}USART_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;


/*
 * Peripheral register definition structure for I2C
 */

typedef struct stm32f407xx
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0)) // (*RCC_BASEADDR).AHB1ENR |= (1 << 0)
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		 	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PLCK_EN() 		(RCC->APB2ENR |= (1 << 4))
#define USART2_PLCK_EN() 		(RCC->APB1ENR |= (1 << 17))
#define USART3_PLCK_EN() 		(RCC->APB1ENR |= (1 << 18))
#define UART4_PLCK_EN()  		(RCC->APB1ENR |= (1 << 19))
#define UART5_PLCK_EN()  		(RCC->APB1ENR |= (1 << 20))
#define USART6_PLCK_EN() 		(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PLCK_DI() 		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PLCK_DI() 		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PLCK_DI() 		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PLCK_DI()  		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PLCK_DI()  		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PLCK_DI() 		(RCC->APB2ENR &= ~(1 << 5))



/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() 		(RCC->APB2ENR |= (1 << 14))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)

/*
 *  Macros to reset USARTx peripherals
 */

#define USART1_REG_RESET() 			do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0);
#define USART2_REG_RESET() 			do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0);
#define USART3_REG_RESET() 			do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0);
#define UART4_REG_RESET()  			do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0);
#define UART5_REG_RESET()  			do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0);
#define USART6_REG_RESET() 			do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0);


/*
 * This macro returns a code (between 0 to 4)for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x) 		((x == GPIOA) ? 0 : \
										 (x == GPIOB) ? 1 : \
										 (x == GPIOC) ? 2 : \
										 (x == GPIOD) ? 3 : \
										 (x == GPIOE) ? 4 : 0)


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F407XX_H_ */


