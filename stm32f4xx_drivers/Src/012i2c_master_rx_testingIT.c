#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>

#define LENG_MAX 100

GPIO_Handle_t gpiob, button;
I2C_Handle_t i2c1;

uint8_t leng, data[LENG_MAX], leng_send = 0x51, data_send = 0x52;
uint8_t SlaveAddr = 0x68;

void Button_GPIOInit(void)
{
    button.pGPIOx = GPIOB;
    
    button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&button);
}

void I2C1_GPIOBInit(void)
{
    gpiob.pGPIOx = GPIOB;

    gpiob.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    gpiob.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
    gpiob.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    gpiob.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    gpiob.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCL
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
    GPIO_Init(&gpiob);
    // SDA
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&gpiob);
}

void I2C1_Init(void)
{
    i2c1.pI2Cx = I2C1;

    i2c1.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    i2c1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&i2c1);
}

void delay(void)
{
    for(uint32_t i = 0; i < 25000; i++);
}

int main(void)
{
    I2C1_GPIOBInit();
    
    Button_GPIOInit();

    I2C_PeripheralControl(i2c1.pI2Cx, ENABLE);
    I2C1_Init();
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    while(1)
    {
        if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
        {
            delay();
            if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
            {
                while(I2C_MasterSendDataIT(&i2c1, &leng_send, 1, SlaveAddr, I2C_ENABLE_SR) == I2C_BUSY_IN_TX);
                while(I2C_MasterReceiveDataIT(&i2c1, &leng, 1, SlaveAddr, I2C_ENABLE_SR) == I2C_BUSY_IN_RX);
                while(I2C_MasterSendDataIT(&i2c1, &data_send, (uint32_t)leng, SlaveAddr, I2C_ENABLE_SR) == I2C_BUSY_IN_TX);
                while(I2C_MasterReceiveDataIT(&i2c1, &data, (uint32_t)leng, SlaveAddr, I2C_ENABLE_SR) == I2C_BUSY_IN_RX); 
                data[leng + 1] = '\0'; 
            }
            while(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)));
        }
        
    }
}

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&i2c1);
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&i2c1);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    if(AppEv == I2C_EV_TX_CMPLT)
    {

    }
    else if(AppEv == I2C_EV_RX_CMPLT)
    {

    }
    else if(AppEv == I2C_ERROR_AF)
    {
        I2C_CloseSendData(pI2CHandle);
        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
        while(1);
    }
}