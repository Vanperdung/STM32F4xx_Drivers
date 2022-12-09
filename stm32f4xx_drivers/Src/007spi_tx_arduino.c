
#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

GPIO_Handle_t gpiob, button;
SPI_Handle_t spi2;

uint8_t TxBuff[] = "Hello world!";

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

int main(void)
{
    SPIgpiob_Init();
    Button_Init();
    SPI2_Init();

    SPI_SSOEConfig(SPI2, ENABLE);
    uint8_t lengthTxBuff = strlen((char*)TxBuff);
    

    while(1)
    {
        if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
        {
            delay();
            if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
            {
            	SPI_PeriControl(SPI2, ENABLE);
                SPI_SendData(SPI2, &lengthTxBuff, 1);
                SPI_SendData(SPI2, TxBuff, strlen((char*)TxBuff));
                while(spi2.pSPIx->SR & (1 << SPI_SR_BSY));
                SPI_PeriControl(SPI2, DISABLE);
            }
            while(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)));
        }
    }
}




