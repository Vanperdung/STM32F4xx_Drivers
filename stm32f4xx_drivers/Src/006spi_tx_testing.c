/*
 * 006spi_tx_testing.c
 *
 *  Created on: Dec 16, 2021
 *      Author: dung
 */

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

GPIO_Handle_t gpiob;
SPI_Handle_t spi2;

uint8_t TxBuff[] = "Hello World!";

void GPIOB_Init(void)
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

void SPI2_Init(void)
{
    spi2.pSPIx = SPI2;
    spi2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    spi2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
    spi2.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2.SPIConfig.SPI_SSM = SPI_SSM_EN;
    SPI_Init(&spi2);
}

int main(void)
{
    GPIOB_Init();
    SPI2_Init();
    SPI_SSIConfig(SPI2, ENABLE);
    SPI_PeriControl(SPI2, ENABLE);
    SPI_SendData(SPI2, TxBuff, strlen((char*)TxBuff));
    SPI_PeriControl(SPI2, DISABLE);
    while(1)
    {

    }
}




