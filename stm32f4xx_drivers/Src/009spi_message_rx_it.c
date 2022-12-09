#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>

#define MAX_LEN 500

SPI_Handle_t spi2handle;

char rcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0;

// This flag will be set in the interrupt handler of the Arduino interrupt GPIO
volatile uint8_t dataAvailable = 0;

void delay(void)
{
    for(uint32_t i = 0; i < 250000; i++);
}

void SPI2_GPIOInit(void)
{

    GPIO_Handle_t gpiob;

    gpiob.pGPIOx = GPIOB;
    gpiob.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    gpiob.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    gpiob.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpiob.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpiob.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GPIO_Init(&gpiob);

    // MOSI
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
    GPIO_Init(&gpiob);

    // MISO 
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
    GPIO_Init(&gpiob);

    // NSS
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
    GPIO_Init(&gpiob);

}
void SPI2_Init(void)
{
    spi2handle.pSPIx = SPI2;
    spi2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
    spi2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    spi2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    spi2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    spi2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    spi2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    spi2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&spi2handle);
}

// This function configures the gpio pin over which SPI peripheral issues data available interrupt 
void Slave_IT_Init(void)
{
    GPIO_Handle_t spiITpin;

    // void *memset(void *ptr, int value, size_t num) (set num byte giá trị vùng nhớ tính từ vị trí ptr trỏ tới bằng giá trị value)
    memset(&spiITpin, 0, sizeof(spiITpin));
    spiITpin.pGPIOx = GPIOD;
    spiITpin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    spiITpin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    spiITpin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    spiITpin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spiITpin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&spiITpin);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

}

int main(void)
{
    uint8_t dummy = 0xFF;
    Slave_IT_Init();
    SPI2_GPIOInit();
    SPI2_Init();
    SPI_SSOEConfig(SPI2, ENABLE);
    SPI_IRQInterruptConfig(36, ENABLE);
    while(1)
    {
        rcvStop = 0;
        while(dataAvailable == 0); // Wail until data available interrupt from transmitter device (slave)

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

        SPI_PeriControl(SPI2, ENABLE);

        while(rcvStop == 0)
        {
            // Fetch the data from the SPI peripheral byte by byte in interrupt mode
            while(SPI_SendDataIT(&spi2handle, &dummy, 1) == SPI_BUSY_IN_TX);
            while(SPI_ReceiveDataIT(&spi2handle, (uint8_t*)&ReadByte, 1) == SPI_BUSY_IN_RX);
        }

        // Confirm SPI is not busy
        while(spi2handle.pSPIx->SR & (1 << SPI_SR_BSY));

        SPI_PeriControl(SPI2, DISABLE);

        dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    }


}

// Slave data available interrupt handler
void EXTI9_5_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_6);
    dataAvailable = 1;
}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&spi2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    static uint32_t i = 0;
    // In the RX complete event, copy data in to rcv buffer
    // '\0' indicates end of message (rcvStop = 1)
    if(AppEv == SPI_EVENT_RX_CMPLT)
    {
        rcvBuff[i++] = ReadByte;
        if(ReadByte == '\0' || (i == MAX_LEN))
        {
            rcvStop = 1;
            rcvBuff[i - 1] = '\0';
            i = 0;
        }
    }
}