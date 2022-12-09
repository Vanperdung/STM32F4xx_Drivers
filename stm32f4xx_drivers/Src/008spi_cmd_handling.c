/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Dec 18, 2021
 *      Author: dung
 */


#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

// Command code
#define COMMAND_LED_CTRL                0x50
#define COMMAND_SENSOR_READ             0x51
#define COMMAND_LED_READ                0x52
#define COMMAND_PRINT                   0x53
#define COMMAND_ID_READ                 0x54

#define LED_ON                          1
#define LED_OFF                         0

// Arduino analog pins
#define ANALOG_PIN0                     0
#define ANALOG_PIN1                     1
#define ANALOG_PIN2                     2
#define ANALOG_PIN3                     3
#define ANALOG_PIN4                     4

// Arduino led pin
#define LED_PIN                         9

GPIO_Handle_t gpiob, button;
SPI_Handle_t spi2;

void delay(void)
{
    for(uint32_t i = 0; i < 50000; i++);
}

void SPIgpiob_Init(void)
{
    gpiob.pGPIOx = GPIOB;
    
    gpiob.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    gpiob.GPIO_PinConfig.GPIO_PinAltFunMode = 5; // AF5
    gpiob.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpiob.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpiob.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
    GPIO_Init(&gpiob);
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GPIO_Init(&gpiob);
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
    GPIO_Init(&gpiob);
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
    GPIO_Init(&gpiob);
}

void Button_Init(void)
{
    button.pGPIOx = GPIOB;
    button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_Init(&button);
}

void SPI2_Init(void)
{
    spi2.pSPIx = SPI2;
    spi2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
    spi2.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enable for NSS pin

    SPI_Init(&spi2);
}

uint8_t SPI_VerifyResponse(uint8_t ackData)
{
    if(ackData == 0xF5)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int main(void)
{
    uint8_t dummySend = 0xFF;
    uint8_t dummyReceive;
    uint8_t ackData;
    uint8_t args[2];
    SPIgpiob_Init();
    Button_Init();
    SPI2_Init();

    SPI_SSOEConfig(SPI2, ENABLE);

    while(1)
    {
        if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
        {
            delay();
            if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
            {
            	SPI_PeriControl(SPI2, ENABLE);

                // 1. CMD_LED_CTRL <pin number> <value>
                uint8_t cmdCode = COMMAND_LED_CTRL;
                SPI_SendData(SPI2, &cmdCode, 1); // In SPI communication when Master or Slave sends 1 byte, it also receives 1 byte in return.
                // Receive the dummy byte from Arduino for clearing RXNE flag
                SPI_ReceiveData(SPI2, &dummyReceive, 1);
                // Send some dummy bits to fetch the response from slave
                SPI_SendData(SPI2, &dummySend, 1);
                // Receive the data from Arduino
                SPI_ReceiveData(SPI2, &ackData, 1);
                if(SPI_VerifyResponse(ackData))
                {
                    args[0] = LED_PIN;
                    args[1] = LED_ON;
                    // Send arguments
                    SPI_SendData(SPI2, args, 2);
                    SPI_ReceiveData(SPI2, &dummyReceive, 2);
                }
                while(spi2.pSPIx->SR & (1 << SPI_SR_BSY));
                SPI_PeriControl(SPI2, DISABLE);
            }
            while(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)));
        }
    }
}





